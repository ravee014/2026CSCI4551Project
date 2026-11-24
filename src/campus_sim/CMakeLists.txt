cmake_minimum_required(VERSION 3.8)
project(campus_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models config launch data rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
