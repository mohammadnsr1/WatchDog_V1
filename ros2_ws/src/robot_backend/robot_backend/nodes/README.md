# `robot_backend.nodes`

Current phase:
- No running ROS node is implemented yet.
- Backend classes and skills are reusable Python modules.
- Agent code should import backend and skill modules directly during this phase.

Future phase:
- Add a `skill_server_node.py` that exposes stable backend/skill APIs as ROS services and/or actions.
- Agents, MCP tools, or LangGraph workflows can call that node remotely instead of touching ROS details directly.
- Direct Python import will still remain useful for early testing, integration, and development.
