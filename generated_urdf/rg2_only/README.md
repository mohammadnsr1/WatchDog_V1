# RG2-Only URDF

This RG2-only URDF was extracted from `robot_description/urdf/ur5e_ros.urdf`.

`rg2_only.urdf` is ROS and RViz friendly.

`rg2_only_isaac.urdf` is Isaac-import friendly and uses absolute `file://` mesh paths.

These files are intended for Isaac Sim URDF import and USD generation so the RG2 link and joint naming stays aligned with the ROS combined URDF.

It intentionally excludes `Gripper_joint`.

Isaac Sim should attach the generated RG2 USD to the UR5e `wrist_3_link` using a fixed joint named `Gripper_joint`.

The ROS combined URDF should also attach `onrobot_rg2_base_link` to `wrist_3_link` the same way.
