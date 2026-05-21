"""Backend abstractions and backend-agnostic robot skills.

`interfaces` defines the contracts that skills depend on.
`ros_moveit_backend` implements those contracts using ROS 2, MoveIt,
and RG2 services.
`skills` defines reusable robot behaviors that stay backend-agnostic.
`nodes` is reserved for future ROS wrapper nodes that expose stable APIs.
"""
