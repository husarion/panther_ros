cmake_minimum_required(VERSION 3.10.2)
project(panther_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes rviz urdf DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_package()
