cmake_minimum_required(VERSION 3.0.2)
project(panther_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes rviz urdf config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
