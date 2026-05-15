cmake_minimum_required(VERSION 3.8)
project(sim_moveit_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS sim_moveit_bringup/static_camera_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS sim_moveit_bringup/pointcloud_workspace_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS sim_moveit_bringup/pointcloud_workspace_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_workspace_filter
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
