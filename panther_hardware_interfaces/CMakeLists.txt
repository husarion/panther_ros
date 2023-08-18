cmake_minimum_required(VERSION 3.5)
project(panther_hardware_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  PkgConfig
  panther_msgs
  realtime_tools
)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/panther_system.cpp
  src/panther_wheels_controller.cpp
  src/roboteq_driver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  rclcpp_lifecycle
  rclcpp
  pluginlib
  panther_msgs
  realtime_tools
)
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP gpiodcxx)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PANTHER_HARDWARE_INTERFACES_BUILDING_DLL")

# target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"s)
pluginlib_export_plugin_description_file(hardware_interface panther_hardware_interfaces.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/test_load_panther_system.cpp test/src/mock_roboteq.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    hardware_interface
    rclcpp
  )
  target_link_libraries(${PROJECT_NAME}_test PkgConfig::LIBLELY_COAPP)


  # add_executable(mock_roboteq test/src/mock_roboteq.cpp)
  # target_include_directories(mock_roboteq PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
  # )
  # # not the best solution, but rclcpp adds filesystem
  # ament_target_dependencies(mock_roboteq ament_index_cpp rclcpp)
  # target_link_libraries(mock_roboteq PkgConfig::LIBLELY_COAPP)

  # install(
  #   TARGETS mock_roboteq
  #   RUNTIME DESTINATION lib/${PROJECT_NAME}
  # )

  install(DIRECTORY
    test/config
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  controller_interface
  realtime_tools
  rclcpp_lifecycle
  rclcpp
  std_msgs
)

ament_package()
