# `robot_backend.nodes`

Current phase:
- no running ROS node is implemented
- backend and skills are reusable Python modules

Future phase:
- add `skill_server_node.py` to expose stable skills as ROS services/actions
- agents, MCP, or LangGraph workflows can call that node remotely
- direct Python import remains useful for early testing and development
