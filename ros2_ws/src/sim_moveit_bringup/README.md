# sim_moveit_bringup

This package launches the UR5e MoveIt stack for simulation together with the Isaac trajectory bridge.

Camera frames are not part of `robot_description`. Camera TFs are scene and sensor extrinsics, so they live in `sim_moveit_bringup/config/cameras.yaml`.

ROS `world` is defined to correspond to Isaac `/World`.

It brings up:
- `robot_state_publisher` through `ur5e_moveit_config/launch/rsp.launch.py`
- `move_group` through `ur5e_moveit_config/launch/move_group.launch.py`
- `rviz2` through `ur5e_moveit_config/launch/moveit_rviz.launch.py` when `launch_rviz:=true`
- `isaac_trajectory_bridge/launch/trajectory_bridge.launch.py`
- fixed camera TF publishing through `sim_moveit_bringup/launch/camera_tf.launch.py` when `publish_camera_tf:=true`

Isaac Sim stays external. Start Isaac separately and make sure the simulation is already playing before launching this package.

Expected Isaac topics:
- `/joint_states`
- `/clock`
- a point cloud topic, by default `/camera/depth/points`

Each incoming point cloud `frame_id` must match the `child_frame` for the relevant camera entry in `config/cameras.yaml`.

Run:

```bash
cd /home/mohammadnsr1/watchdog/WatchDog_V1/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sim_moveit_bringup ur5e_moveit_config
source install/setup.bash
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py
```

Set a custom point cloud topic:

```bash
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py pointcloud_topic:=/your/isaac/points
```

Choose a specific camera config file or disable camera TF publishing:

```bash
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py publish_camera_tf:=true
ros2 launch sim_moveit_bringup sim_moveit_bringup.launch.py camera_config:=/absolute/path/to/cameras.yaml
```

The default MoveIt sensor configuration uses `/camera/depth/points`. The bringup launch remaps that default subscription to the `pointcloud_topic` launch argument for `move_group`. If a future plugin path does not honor the remap, edit `ur5e_moveit_config/config/sensors_3d.yaml` directly.

Add another fixed camera by creating a new entry under `cameras:` in `sim_moveit_bringup/config/cameras.yaml` with:
- `enabled`
- `parent_frame`
- `child_frame`
- `xyz`
- `rpy`
- optional sensor topics and notes

Verify simulation time and joint states:

```bash
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once
```

Verify TF:

```bash
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo world camera_depth_optical_frame
```

Verify the point cloud topic:

```bash
ros2 topic info /camera/depth/points
ros2 topic echo /camera/depth/points --once
```

If you launch with a different topic, replace `/camera/depth/points` with your configured topic.

Verify MoveIt receives planning scene updates:
- check that `move_group` starts without occupancy map monitor errors
- confirm TF connects the point cloud frame to the MoveIt planning frame
- inspect the monitored scene in RViz and look for occupancy/collision geometry updates as point cloud data arrives

Known limitation:
- point clouds provide occupancy and collision geometry, not object identity
- later, object-specific perception such as FoundationPose can publish `CollisionObject` or attached-object updates on top of this
