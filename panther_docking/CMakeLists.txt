cmake_minimum_required(VERSION 3.8)
project(panther_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    rclcpp
    panther_msgs
    geometry_msgs
    sensor_msgs
    std_srvs
    realtime_tools
    opennav_docking_core
    opennav_docking
    pluginlib
    tf2_ros
    panther_utils)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(library_name ${PROJECT_NAME}_battery_state_dock)

add_library(${library_name} SHARED src/panther_charging_dock.cpp)

# TODO @delihus how to link the library what is not a name of a package
target_link_libraries(${library_name} /opt/ros/humble/lib/libpose_filter.so)

target_include_directories(${library_name}
                           PUBLIC ${CMAKE_INSTALL_PREFIX}/include)

pluginlib_export_plugin_description_file(opennav_docking_core plugin.xml)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_panther_charging_dock test/test_panther_charging_dock.cpp)
  target_link_libraries(${PROJECT_NAME}_test_panther_charging_dock ${library_name} )
  ament_target_dependencies(${PROJECT_NAME}_test_panther_charging_dock   ${PACKAGE_DEPENDENCIES})
endif()

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(panther_charging_dock ${library_name})
ament_target_dependencies(${library_name} ${PACKAGE_DEPENDENCIES})
ament_package()
