# WatchDog Monitoring Layer

WatchDog has two tiers. Tier 1 is a deterministic recorder and monitor engine
running inside one dedicated ROS node, `watchdog_node`, spun continuously by a
background `MultiThreadedExecutor`. Tier 2 is `watchdog_mcp_server`, a thin MCP
tool surface that reads locked recorder/engine state or waits on thread events.

## Independence Rules

WatchDog owns its own `tf2_ros.Buffer` and `TransformListener`. It does not call
executor pose services and does not import executor skills, backends, or
interfaces. Monitor thresholds come from contract clauses or registry specs as
agent-supplied parameters at arm time. The halt path can only cancel MoveGroup
goals; it never sends motion, trajectory, gripper, or scene mutation commands.

## Wall-Clock Rule

Executor `EventEnvelope.timestamp_ns` is monotonic in the executor process and
is opaque to WatchDog. Recorder samples, monitor fires, and evidence windows are
stamped with `wall_ns() == time.time_ns()`. Executor events are aligned by
parsing `wall_time_iso` into wall-clock nanoseconds while preserving the
original envelope for provenance.

## Provenance Example

1. `get_contract("green_cube", "pick_green_cube")` returns a clause such as
   `grasp_effort_instantaneous <= spec.limits.max_clamp_effort`.
2. The agent arms an `effort_event` monitor with params copied from that
   clause unchanged, for example `joint_name="finger_joint"`,
   `comparison="le"`, and `threshold=4.0` -- the comparison the effort must
   satisfy for conformance, not its inverse. The monitor fires when the
   condition stops holding.
3. A violating `/joint_states` sample creates a `monitor_fire` event containing
   the monitor id, clause, triggering value, threshold, comparison, and
   `trigger_wall_ns`.
4. `get_evidence(start_wall_ns, end_wall_ns, ["joint_states", "events"])`
   returns the pre-trigger and post-trigger recorder window.
5. `record_verdict(...)` publishes the final conformance verdict with free-form
   clause results that cite the observed evidence.

## Known Expressiveness Gaps

**One-sided positional bounds (e.g. a minimum height above a surface).** The
current primitive set has no primitive for a single-axis threshold on a frame
or object position. The workaround, documented in the WatchDog system prompt's
arming discipline section, is to express it with `region` as a box that is
generous on the unconstrained axes and places the constrained face at the
bound. This works -- `region`'s half-extents just need to be positive on every
axis, chosen to comfortably contain the reachable workspace on the axes the
clause doesn't actually constrain -- but it is awkward: the armed monitor
carries geometry on axes the clause never mentioned, and getting the generous
axes' extents wrong (too tight) risks a spurious fire unrelated to the clause
being watched.

If a future contract carries several one-sided bounds, a fifth primitive
expressing a single-axis threshold directly (no incidental geometry on the
other axes) would be worth adding. Not added now -- this note exists so that
decision is made deliberately later, against real contract needs, rather than
being rediscovered from scratch or skipped silently.

## Session Recording

One cell bringup produces one session directory, selected by
`WATCHDOG_SESSION_DIR` (set by `sim_moveit_bringup.launch.py` and exported for
child processes). Everything a run persists is rooted there: the per-process
`events_*.jsonl` logs, the orchestrator log, the rosbag2 recording under
`bag/`, `manifest.json`, and any `frames/` captured by `capture_frames`. When
the variable is unset each process falls back to a per-process timestamped path
under `~/.watchdog/sessions/`. An explicitly set `WATCHDOG_EVENT_LOG` /
`WATCHDOG_MONITOR_LOG` still wins, so existing tooling is unaffected.

**Agent transcripts stay under `agents/transcripts/`, not in the session
directory.** The agents launch separately from a virtualenv and correlate to a
session by `wall_ns`, which is sufficient; coupling them to a ROS launch
argument would be more trouble than it solves.

### Clock and correlation (read before aligning a bag against anything)

`use_sim_time` is true for the cell's nodes, and Isaac Sim resets simulation
time to zero between runs. rosbag2 nonetheless stamps `recv_timestamp` from the
system clock -- a real Unix epoch, directly comparable to `wall_ns`. The bag
recorder is deliberately left on system time (`use_sim_time: false`) while the
rest of the graph runs on sim time; no correction term is needed.

Correlation uses the bag's `recv_timestamp` only. Message `header.stamp` values
(`sensor_msgs/JointState`, `sensor_msgs/Image`, `tf2_msgs/TFMessage`) are
simulation time, reset to zero at the start of each Isaac Sim run, and are not
comparable across sessions or against `wall_ns`. Using them will produce
plausible-looking but wrong alignments. This warning is also embedded in each
session's `manifest.json` under `clock`.

### The manifest embeds the registry

`manifest.json` embeds the full `model_registry.yaml` as loaded, plus its
sha256 and resolved path. In fault-injection runs the registry *is* the injected
fault: without it captured alongside the recording there is no way to
reconstruct what the executor was told versus what physically happened, and a
corrupted run cannot be told from a clean one. The hash makes that separation
mechanical across a batch.

## Buffers vs. the bag: a deliberate boundary

`get_evidence` and `capture_frames` read the in-memory ring buffers, never the
bag. The buffers are the only *current* source -- they serve live monitoring.
The bag is the only *complete* source -- it serves post-hoc analysis. These
roles do not cross:

- `get_evidence` must never read from the bag. rosbag2 cannot reliably read a
  bag it is still writing: the current chunk is unflushed, so the most recent
  seconds -- precisely the window a live verdict needs -- would be missing, and
  the tool would work for old data while silently returning nothing for the data
  that matters.
- The in-memory ceilings (`WATCHDOG_MAX_JOINT_SAMPLES`,
  `WATCHDOG_MAX_TF_SAMPLES`, defaults ~1 hour) are raised well past the old
  ten-minute bound so a long run can still serve a live query for an early
  window. Memory cost is tens of MB, negligible beside Isaac Sim.
- `/joint_states` was measured at ~51 Hz average but with gaps up to 0.333 s
  under heavy simulation. The evidence stream has real holes; a verdict citing a
  narrow window should know that.

## Visual evidence capture (`capture_frames`)

The WatchDog MCP server holds a small image ring buffer (RGB, throttled to
`WATCHDOG_IMAGE_BUFFER_HZ`, default 5 Hz), bounded by both a frame count and a
byte budget (`WATCHDOG_IMAGE_BUFFER_BYTES`, default 512 MB), evicting
oldest-first. Frames are stamped with `wall_ns()` at receipt -- never
`header.stamp`. The subscriptions run in a callback group separate from the
recorder's `/joint_states` and TF callbacks, so slow image handling never
delays monitor evaluation or halt latency.

`capture_frames(start, end, ...)` writes a window of buffered frames to
`<session>/frames/` retrospectively -- the design is a ring buffer, not a
per-step recording, because when a monitor fires the frames worth having are the
ones from *before* the fire, which cannot be recorded on demand after the fact.

**This tool preserves evidence; it does not perceive.** The agent harness sends
text only, so WatchDog cannot see the frames it captures. WatchDog's verdicts
rest on proprioceptive and TF evidence, independent of any vision pipeline;
capturing frames does not change what a verdict is based on. It exists so visual
evidence for a moment the monitor identified as significant survives for later
analysis.
