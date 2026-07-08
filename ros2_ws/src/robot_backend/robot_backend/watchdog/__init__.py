"""WatchDog deterministic monitoring for robot_backend.

The package has two tiers: Tier 1 is an always-on deterministic recorder and
monitor engine hosted on a dedicated background-spun ROS node, and Tier 2 is
a thin MCP tool surface over that engine.

Independence rules: WatchDog owns its own TF buffer/listener, does not call
executor pose services, does not import executor skills/backends/interfaces,
and only stops execution by cancelling MoveGroup goals. Monitors compare live
physical quantities against agent-supplied contract or registry-spec bounds;
they do not learn baselines or perform statistical anomaly detection.
"""
