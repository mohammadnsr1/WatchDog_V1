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

From the Script Editor, using the running app:

```python
import sys, importlib, asyncio
sys.path.insert(0, "/home/mohammad/watchdog/WatchDog_V1"); importlib.invalidate_caches()
import paper_media.isaac_replay.render_replay as rr; importlib.reload(rr)
asyncio.ensure_future(rr.run_async(
    "/home/mohammad/watchdog/WatchDog_V1/agents/transcripts/sessions/<id>/replay/replay_state.npz",
    "/home/mohammad/watchdog/WatchDog_V1/paper_media/isaac_replay/replay_config.yaml",
    check=True))
```

Standalone, with Isaac's Python:

```bash
~/isaacsim/python.sh paper_media/isaac_replay/render_replay.py \
    --state <session>/replay/replay_state.npz --config paper_media/isaac_replay/replay_config.yaml --check
```

- **Check mode** renders the first, middle and last ticks into `frames/replay_<camera>_check/`, so a check never mixes with a real render.
- **A full render** writes into `frames/replay_<camera>/` and refuses to overwrite an existing `frames.jsonl` unless you pass `--overwrite` (`overwrite=True` in the Script Editor).
- **Each `frames.jsonl` line** has `index`, `file`, `wall_time_iso`, `log_time_ns`, `t_rel`, `camera` and `tick`. The compositor reads `file` and `wall_time_iso`, so in a story file `camera: replay_main` works unchanged.

## The physics-free method, and why

The original stage is opened, every layer it uses is marked not savable, and
the edit target is moved to the in-memory session layer. The replay never
calls Save. Then:

1. **Nothing can push anything.**
   - Gravity is set to 0 on every physics scene.
   - Every collider is disabled.
   - Every rigid body is made kinematic with zero velocity, or disabled entirely with `part_physics: disable`.
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

- **Injected scene events do not appear.** Intruder cubes and visiting hands live under `/World/WatchdogIntruders`, which is deliberately outside the object TF publisher's target list, so they are not in TF. Reconstructing them would need a separate pass over `ground_truth_injections.jsonl`.
- **Stage drift.** `assets/usd/assembly_room.usd` was re-saved between batches (layout changes are recorded in `batch.yaml`). Parts and robot come from the bag, so they are exact. Static scenery comes from whatever stage you open: for an older session, export the stage from that batch's `git_sha` and point `stage` at the copy.
- **Wall time.** It is the bag recorder's receive time, the same axis as every other session log.
