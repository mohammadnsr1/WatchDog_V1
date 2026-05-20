# RG2 Controller

This package includes a simple ROS 2 RG2 gripper controller for Isaac Sim. The custom `SetGripperPosition` service comes from the companion `isaac_trajectory_bridge_interfaces` package.

## Services

- `/rg2_controller/open` using `std_srvs/srv/Trigger`
- `/rg2_controller/close` using `std_srvs/srv/Trigger`
- `/rg2_controller/set_position` using `isaac_trajectory_bridge_interfaces/srv/SetGripperPosition`

## Joint Names

The controller commands only these RG2 joints on `/joint_command`:

- `finger_joint`
- `left_inner_knuckle_joint`
- `right_inner_knuckle_joint`
- `right_outer_knuckle_joint`
- `left_inner_finger_joint`
- `right_inner_finger_joint`

It never publishes the UR5e arm joints.

## `set_position`

`set_position.position` is interpreted as the master RG2 joint target in radians for `finger_joint`.
The default joint multipliers and offsets match the current RG2 URDF mimic relationships, and can be tuned with parameters if Isaac needs a different mapping.

## Parameters

- `joint_state_topic`: default `/joint_states`
- `joint_command_topic`: default `/joint_command`
- `service_namespace`: default `/rg2_controller`
- `master_joint_name`: default `finger_joint`
- `master_joint_min_position`: default `-0.558505`
- `master_joint_max_position`: default `0.785398`
- `open_position`: default `-0.558505`
- `close_position`: default `0.785398`
- `command_duration_sec`: default `1.0`
- `command_rate_hz`: default `50.0`
- `wait_for_joint_state_timeout_sec`: default `2.0`
- `gripper_joint_names`: default current Isaac RG2 joint list
- `gripper_joint_multipliers`: default `[1.0, -1.0, -1.0, -1.0, 1.0, 1.0]`
- `gripper_joint_offsets`: default all zeros

## Manual Verification

```bash
cd /home/mohammadnsr1/watchdog/WatchDog_V1/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isaac_trajectory_bridge_interfaces isaac_trajectory_bridge sim_moveit_bringup
source install/setup.bash
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py
```

Then test the services in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/mohammadnsr1/watchdog/WatchDog_V1/ros2_ws/install/setup.bash
ros2 service call /rg2_controller/open std_srvs/srv/Trigger {}
ros2 service call /rg2_controller/close std_srvs/srv/Trigger {}
ros2 service call /rg2_controller/set_position isaac_trajectory_bridge_interfaces/srv/SetGripperPosition "{position: 0.3}"
```

## Note

Reset Isaac simulation time so `/clock`, `/joint_states`, TF, and camera stamps stay synchronized before doing full MoveIt perception work.
