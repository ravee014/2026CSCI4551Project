cmake_minimum_required(VERSION 3.8)
project(campus_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  data
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
