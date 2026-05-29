# `robot_backend.nodes`

Current nodes:
- `scene_tf_sync_node.py`: reads `config/model_registry.yaml`, looks up object TF poses,
  applies `pose_offset`, and updates the MoveIt planning scene through `robot_backend`.

Manual usage later:
- Prerequisites:
  - Isaac Sim publishing TF frames such as `World -> red_cube` and `World -> green_cube`
  - bringup publishing the static identity transform `world -> World`
  - MoveIt services `/apply_planning_scene` and `/get_planning_scene` available
- Then run `scene_tf_sync_node` once or on a timer.
- After syncing, run the scene backend smoke test `list` mode to confirm objects appear.

Future phase:
- add `skill_server_node.py` to expose stable skills as ROS services/actions
- agents, MCP, or LangGraph workflows can call that node remotely
- direct Python import remains useful for early testing and development
