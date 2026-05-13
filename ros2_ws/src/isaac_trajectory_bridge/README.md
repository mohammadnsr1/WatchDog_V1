# isaac_trajectory_bridge

This package provides a ROS 2 Jazzy `FollowJointTrajectory` action server that lets MoveIt execute trajectories in Isaac Sim without modifying Isaac itself.

It connects the stack like this:

`MoveIt -> /arm_controller/follow_joint_trajectory -> isaac_trajectory_bridge -> /joint_command -> Isaac`

The bridge:

- accepts `control_msgs/action/FollowJointTrajectory` goals
- validates and reorders UR5e arm joints into the expected command order
- reads `/joint_states` for feedback and final tolerance checks
- publishes `sensor_msgs/msg/JointState` commands to `/joint_command`

## Run

Build the package in the workspace:

```bash
cd /home/mohammadnsr1/watchdog/WatchDog_V1/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isaac_trajectory_bridge
source install/setup.bash
```

Launch the bridge:

```bash
ros2 launch isaac_trajectory_bridge trajectory_bridge.launch.py
```

After the bridge is running, MoveIt execution should flow through `/arm_controller/follow_joint_trajectory`, and the bridge will stream joint targets to Isaac on `/joint_command`.
