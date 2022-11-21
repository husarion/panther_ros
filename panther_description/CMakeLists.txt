cmake_minimum_required(VERSION 3.0.2)
project(panther_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)
