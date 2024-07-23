# panther_bringup

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

## Launch Files

This package contains:

- `bringup.launch.py` - is responsible for activating whole robot system.

### bringup.launch.py - Nodes

| Node name                 | *Type*                                                                                          |
| ------------------------- | ----------------------------------------------------------------------------------------------- |
| `battery_driver`          | [*panther_battery/battery_node*](../panther_battery)                                            |
| `controller_manager`      | [*controller_manager/ros2_control_node*](https://github.com/ros-controls/ros2_control)          |
| `ekf_filter`              | [*robot_localization/ekf_node*](https://github.com/cra-ros-pkg/robot_localization)              |
| `imu_broadcaster`         | [*ros2_controllers/imu_broadcaster*](https://github.com/ros-controls/ros2_controllers)          |
| `joint_state_broadcaster` | [*ros2_controllers/joint_state_broadcaster*](https://github.com/ros-controls/ros2_controllers)  |
| `lights_controller`       | [*panther_lights/lights_controller_node*](../panther_lights)                                    |
| `lights_driver`           | [*panther_lights/lights_driver_node*](../panther_lights)                                        |
| `lights_manager`          | [*panther_lights/lights_manager_node*](../panther_lights)                                       |
| `navsat_transform`        | [*robot_localization/navsat_transform_node*](https://github.com/cra-ros-pkg/robot_localization) |
| `panther_system`          | [*panther_hardware_interfaces/PantherSystem*](../panther_hardware_interfaces)                   |
| `robot_state_publisher`   | [*robot_state_publisher/robot_state_publisher*](https://github.com/ros/robot_state_publisher)   |
| `safety_manager`          | [*panther_manager/safety_manager_node*](../panther_manager)                                     |
| `system_status`           | [*panther_diagnostics/system_status_node*](../panther_diagnostics)                              |

## ROS Nodes

This package only runs external nodes. If you want to learn how to configure individual nodes, please check the appropriate package.
