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

1. `get_contract("green_cube")` returns a clause such as
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
