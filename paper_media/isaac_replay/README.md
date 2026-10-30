# isaac_replay

Re-render a recorded session in Isaac Sim 5.1 from its rosbag, with physics
deciding nothing, and write the frames in the frame recorder's format so the
replay compositor treats them as one more camera.

```text
<session>/bag  --extract_bag.py-->  <session>/replay/replay_state.npz  --render_replay.py-->  <session>/frames/replay_<camera>/
 (paper_media venv, rosbags)                                             (inside Isaac Sim)      000000.png ... + frames.jsonl
```

The work is split in two because Isaac's Python does not have `rosbags`. The
renderer needs only numpy and PyYAML, which Isaac ships.

## Step 1: extract

```bash
python -m paper_media.isaac_replay.extract_bag <session_dir> --config paper_media/isaac_replay/replay_config.yaml --check
python -m paper_media.isaac_replay.extract_bag <session_dir> --config paper_media/isaac_replay/replay_config.yaml --fps 30
```

- **Time.** Every tick carries the bag's message log time: the recorder's receive time, wall clock ns. It is written as `log_time_ns` and as a UTC `wall_time_iso`. `header.stamp` is simulation time and is never read.
- **Joints.** Every name on `/joint_states`, linearly interpolated, with the edge values held.
- **Poses.** `/tf` and `/tf_static`, composed to the world frame. Translation is linear, rotation is slerp, edges are held. Roots named in `world_frame` and `world_aliases` count as the world origin.
- **Quaternion order in the npz is wxyz,** which is also `isaacsim.core`'s order.
- **`--check`** writes nothing. It prints the bag span, the joints with their sample counts, every TF frame with its parent, coverage and gaps over 0.5 s, and whether each enabled registry part has a TF frame.

## Step 2: render

From the Script Editor, using the running app. **This opens the configured stage
in place of whatever is open, so never run it while a live session uses the app.**
Bag seconds (`t_rel`) are seconds from the first bag message; session seconds in a
story are about half a second later (the summary prints both t0 values).

```python
import sys, importlib, asyncio
sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1"); importlib.invalidate_caches()
import paper_media.isaac_replay.render_replay as rr; importlib.reload(rr)
S = "/home/mohammad/watchdog/WatchDog_V1/agents/transcripts/sessions/<id>"
CFG = "/home/mohammad/watchdog/WatchDog_V1/paper_media/isaac_replay/replay_config.yaml"

# 1. check: a few chosen bag seconds, every camera, into frames/replay_<camera>_check/
asyncio.ensure_future(rr.run_async(S + "/replay/replay_state.npz", CFG, check_at=[100.0, 300.0], overwrite=True))

# 2. a window for a video: bag seconds 99 to 330 at 15 fps, cell camera only,
#    into frames/replay_cell_99-330s/   (use `camera: replay_cell_99-330s` in the story)
asyncio.ensure_future(rr.run_async(S + "/replay/replay_state.npz", CFG, start=99.0, end=330.0, fps=15,
                                   cameras=["cell"], overwrite=True))
```

Standalone, with Isaac's Python (same options as flags):

```bash
~/isaacsim/python.sh paper_media/isaac_replay/render_replay.py \
    --state <session>/replay/replay_state.npz --config paper_media/isaac_replay/replay_config.yaml \
    --check-at 100 300 --overwrite
~/isaacsim/python.sh paper_media/isaac_replay/render_replay.py \
    --state <session>/replay/replay_state.npz --config paper_media/isaac_replay/replay_config.yaml \
    --start 99 --end 330 --fps 15 --cameras cell --overwrite
```

- **Output folders.** `frames/replay_<camera>_check/` for checks, `frames/replay_<camera>_<start>-<end>s/` for a window, `frames/replay_<camera>/` for a whole run, or `frames/replay_<camera>_<label>/` with `label=` / `--label`.
- **Overwrite.** A render refuses to replace an existing `frames.jsonl` unless you pass `--overwrite` (`overwrite=True`).
- **Each `frames.jsonl` line** has `index`, `file`, `wall_time_iso`, `log_time_ns`, `t_rel`, `camera` and `tick`. The compositor reads `file` and `wall_time_iso`, so a story can use any replay folder as its `camera`.
- **Speed.** About 3 s per 1080p frame on the shared A4500 while two other Isaac apps were running (2026-10-29), so a 230 s window at 15 fps is roughly 3 hours. Check and keyframe renders take minutes.
- **Warm-up.** The real-time renderer accumulates over frames, so a jump between distant ticks leaves faint ghosts of where parts were. Ticks after a jump of more than 1 s get 24 render passes instead of `warmup_subframes`.

## The physics-free method, and why

The original stage is opened, every layer it uses is marked not savable, and
the edit target is moved to the in-memory session layer. The replay never
calls Save. Then:

1. **Nothing can push anything.**
   - Gravity is set to 0 on every physics scene.
   - Every collider is disabled.
   - Robot links stay dynamic rigid bodies. Making them kinematic turns every robot joint into a joint between static bodies, and PhysX then drops the articulation (the 2026-10-29 failure: "cannot create a joint between static bodies").
   - Every other rigid body (the parts) is disabled with `part_physics: disable`, or made kinematic with `part_physics: kinematic`.
   - The ROS bridge graphs are deactivated, so no `/joint_command` subscriber drives the arm and nothing is published.
2. **The robot state is overwritten in full every frame.** `SingleArticulation.set_joint_positions(q)`, `set_joint_velocities(0)`, and `apply_action(ArticulationAction(joint_positions=q))`, so the drives' targets equal the state they are given.
3. **One `world.step(render=False)` commits the state.** With no gravity, no contacts, zero velocity and drives already at target, that step cannot move anything. It only makes PhysX publish the teleported link transforms and pick up the kinematic part poses.
4. **Replicator renders without advancing time.** `rep.orchestrator.step(rt_subframes=warmup_subframes, delta_time=0.0, pause_timeline=False)` renders the warm-up subframes with the timeline held, then the RGB annotator is read.

I chose this over the alternatives for these reasons:

- **Writing joint state while the timeline is paused** is unreliable. A paused articulation does not publish teleported link poses to Fabric, so the render can lag a frame or more.
- **Posing link prims from TF** would need every robot link in `/tf`, plus a TF-to-prim map for each link. It also bypasses the mimic gripper rather than reproducing it.
- **Setting joints, then one zero-effect step,** uses the authoritative `/joint_states` stream directly and keeps the RG2 mimic structure intact. Because the step changes nothing, every frame depends only on the npz tick: the result is deterministic.

**Gripper.**
- Every articulation DOF found in `/joint_states` is set from the bag.
- Missing mimic joints are computed from `finger_joint` using the `PhysxMimicJointAPI` attributes read from the stage, or from the URDF `<mimic>` tags with `mimic.source: urdf`.
- When the bag also contains a mimic joint, the renderer prints how well the relation predicts it under both sign conventions. That settles `mimic.convention` from data.
- Any DOF that is set by neither is held at its initial value, with a warning.

**Parts.** Each `part_map` prim gets its TF frame's world pose every frame through `SingleXFormPrim.set_world_pose`.

## Known limits

- **Injected objects come from ground truth, not the bag.** Objects under `/World/WatchdogIntruders` are deliberately outside the TF publisher, so the replay rebuilds them under `/Replay/Intruders` from `ground_truth_injections.jsonl`, with the injector's geometry and colors. A placed object holds each logged pose (place, then the pose tracker's moved and settled records). A visitor moves linearly between its logged phase points (enter_start, in_view, dwell_start, leave_start, out_of_view, removed), which is how the injector moves it: straight lines at constant speed. On 20261030T003243Z the rebuilt cube and visitor match the recorder frames at the same wall time.
- **Stage drift.** `assets/usd/assembly_room.usd` was re-saved between batches (layout changes are recorded in `batch.yaml`). Parts and robot come from the bag, so they are exact. Static scenery comes from whatever stage you open: for an older session, export the stage from that batch's `git_sha` and point `stage` at the copy.
- **Wall time.** It is the bag recorder's receive time, the same axis as every other session log.
