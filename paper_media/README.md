# paper_media

Tools for paper and website media. Additive: nothing in the executor, the
WatchDog, the orchestrator, the MCP servers, or the registry imports this
directory, and nothing here writes outside a session's `frames/<camera>/`
folder (recorder) or the paths you pass on the command line (compositor).

```text
paper_media/
  recorder/frame_recorder.py   passive ROS 2 node: camera frames + wall clock stamps
  replay/                      compositor: session -> MP4, PDF/SVG figure, web assets
  stories/                     hand-curated story files, one per rendered run
```

## Frame recorder

A standalone rclpy node. It is not an MCP tool and is in no `--mcp-config`
file, so neither agent can see or call it. It subscribes best effort, depth 1,
so it never slows the simulator.

- Default topic `/recorder_rgb`, a recording-only camera on the stage (1280 x 720,
  intrinsics on `/recorder_camera_info`). No agent subscribes to it. Frames go to
  `frames/recorder/` at JPEG quality 95.
- To record the agents' own 640 x 360 view instead, pass
  `-p image_topic:=/camera/depth/rgb -p camera_name:=main`.
- Writes `<session>/frames/<camera_name>/<index:06d>.jpg` and one line per
  frame to `frames.jsonl`: `index`, `file`, `wall_time_iso` (wall clock at
  receipt, UTC), `header_stamp` (simulation time, reference only), `topic`.
- Session directory: `out_dir` parameter, else `$WATCHDOG_SESSION_DIR`, else
  `~/.watchdog/current_session`. If none resolves it logs an error and exits.
- Run it with the ROS 2 Python. Raw topics need Pillow there
  (`sudo apt install python3-pil`); a `.../compressed` topic is written as is.
- Both views at once: run a second instance with `-p image_topic:=/camera/depth/rgb -p camera_name:=main`.

## Replay compositor

```bash
python -m paper_media.replay.cli summary <session_dir>
python -m paper_media.replay.cli preview <story.yaml> --frames 20 --out <dir>
python -m paper_media.replay.cli video   <story.yaml> --out <file.mp4>
python -m paper_media.replay.cli figure  <story.yaml> --out <file.pdf> [--width double|single]
python -m paper_media.replay.cli web     <story.yaml> [...] --site <SITE_REPO>
```

Run `summary` first on any new session: it prints event counts per lane and
kind, the time span, the frame sources, and every record it could not parse.

### Clock rule

Cross-process ordering uses the wall clock only: `wall_time_iso` where a
record has it, and fields that are wall clock epoch nanoseconds (orchestrator
`wall_ns`, look `wall_ns`, `capture.json` frame `wall_ns`, the bag's receive
time) converted to the same form. The envelope `timestamp_ns` is monotonic and
is never read. Message header stamps are simulation time and are never read.

### What the logs contain, and what they do not

Verified against the writers and the session directories on 2026-10-29.

| Wanted | Where it comes from |
|---|---|
| Step start, claimed outcome | `begin_step`, `end_step` envelopes in `events_scene.jsonl` |
| Executor tool calls | `mcp_calls_executor.jsonl` (start = `wall_time_iso`, end = start + `elapsed_sec`) |
| Bus items | every `/watchdog/events` envelope, plus orchestrator gate records (`gate_open`, `gate_timeout`) |
| Monitor arm / fire / disarm | `monitor_armed`, `monitor_fire`, `monitor_disarmed` in `events_watchdog.jsonl` (S1 to S3) or `events_executor_monitor.jsonl` (S4) |
| WatchDog wake reason | the return of each `wait_for_event` call: `result.wake` (`begin_step` is shown as arming, `monitor_fire` as fire, `end_step`, `timeout`) |
| `look()` and its frames | `look` envelopes, image at `payload.paths.rgb`, resolved to `frames/<name>` when the absolute path is from another machine |
| Verdicts | `verdict` and `run_verdict` envelopes; `conformant` shown as PASS, `non_conformant` as FAIL, `inconclusive` as INCONCLUSIVE |
| Gripper effort | bag `/joint_states`, joint `finger_joint`, `effort` |
| True gripper width | bag `/joint_states` `finger_joint` position, converted with the controller's own `rg2_calibration_table.csv` |
| TF distances | bag `/tf`, `/tf_static`, as `tf_dist:<frame_a>:<frame_b>` |

Not present in the logs, so not shown:

- **Halt.** Every recorded `monitor_fire` has `action_taken: flag` and
  `halt_result: null`, and no orchestrator log has an `abort`. The halt lane
  event and the FAIL to halt arrow are implemented and appear only if a future
  session records one.
- **Escalation flag.** There is no escalation event. The closest field is the
  fire's `action_taken`, which is always `flag`.
- **Session manifest.** `robot_backend/watchdog/session.py` can write
  `manifest.json`, but no session directory in this campaign has one. Scenario,
  fault class, assembly and trial therefore come from the story file, and the
  WatchDog registry for thresholds comes from the story's `fallback_value`
  (with a warning) unless `registry_paths` is given. The executor registry is
  read from `registry_provenance_executor.json` when present and unchanged
  (sha checked).
- **Continuous camera video** for sessions recorded before this recorder
  existed. The compositor then falls back to the sparse `look()` and
  `capture_frames()` stills and labels the frame panel accordingly.

### Story files

See `stories/example_class4.yaml`. Keys: `session_dir`, `run_id`, `scenario`,
`fault_class`, `assembly`, `trial`, `title`, `subtitle`, `camera`, `window`,
`speed`, `key_events`, `executor_tools_hide`, `prompt_excerpts`, `captions`,
`signals`, `thresholds` (`value_from`, `object`, `field`, `fallback_value`,
`label`), `registry_paths`, `figure_frames_at`, `ground_truth`.

### Ground truth overlay (video only)

`ground_truth` is a list of `{name, label, outcome, detected_at}`. `name` is an
injection in `ground_truth_injections.jsonl`; its presence runs from the first
`place` or `in_view` record to the first later `out_of_view` or `removed`
(or the end). `outcome` is `detected`, `late` or `missed` and `detected_at` is
the session second of the WatchDog record. Both are the author's labels, read
off the session; the compositor does not decide detection. With the key set,
the video gets a fifth swimlane ("Ground truth (hidden from agents)") and a
GROUND TRUTH tag on the camera frame while the object is present. Arrivals and
detections hold like key events, and a transient visit plays at `speed.event`
for its whole duration. Figures and the web JSON are unchanged.

### Hand-over card (video only)

Every video opens with a card (`style.BRIEFING_VIDEO_S`, 16 s, then a short
cross fade) that shows what each agent was handed for that run: the task and
part order read from the executor task YAML named in `agent_config_executor.json`,
the executor's and the WatchDog's duties, how the WatchDog arms monitors in that
scenario, the step gate, and the models. The wording lives in
`replay/briefing.py` (`TASK_BRIEFS`, `SCENARIO_BRIEFS`) and paraphrases
`agents/tasks/<task>.{executor,watchdog}.yaml`; keep it in step with those files.
The story key `briefing: {show: false}` drops the card, `briefing: {seconds: N}`
changes its length. The web JSON's `t_video` includes the card (`briefing_s`).

### Lane names

The swimlanes are named for a manufacturing reader (`style.LANE_TITLES`,
`LANE_SUBTITLES`): "Executor Agent", "Event Stream" (the announcements and the
orchestrator's step gate the agents talk through), "Armed Monitors" (the sensor
checks the WatchDog arms) and "WatchDog Agent". The lane keys in stories and
the web JSON are unchanged (`executor`, `bus`, `monitors`, `watchdog`).

### Task hand-over figure

`python -m paper_media.figures.task_handover` draws the static figure that sits
between the architecture and the data flow figure: one task specification, the
two different briefs, the part registry as the only source of thresholds, and
the run cycle. Output in `paper_media/figures/out/` (SVG with editable text,
PNG, and the box text for the caption).

### Time compression is always visible

The video maps session time to video time piecewise: `speed.idle` between
events, `speed.event` within `event_pad_s` of each key event, and a
`hold_s` freeze at each key event. A badge on the camera panel always shows
the current speed (for example 8x, or "paused" during a hold) and the real
session time.

## Install

```bash
python3 -m venv .venv-media
. .venv-media/bin/activate
pip install -r paper_media/requirements.txt
```
