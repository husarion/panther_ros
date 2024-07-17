# ROS API

> [!WARNING] Initial Release Warning
>
> Please be advised that the software you are about to use is the first publicly available version of ROS 2 Driver for Panther. It is functional and architecture will not change in a significant way. It was tested by Husarion team, however some stability issues and bugs might still occur. Also, the ROS 2 API may face some minor changes in the following releases.
>
> We would be grateful for your feedback related to Panther ROS 2 driver. You can reach us the following ways:
>
> - by email at [support@husarion.com](mailto:support@husarion.com)
> - via our community forum: [https://community.husarion.com](https://community.husarion.com)
> - using issue request on GitHub: https://github.com/husarion/panther_ros/issues

## ROS 2 System Design

This section describes the ROS packages in the Panther ROS system. These packages are located in the [panther_ros](https://github.com/husarion/panther_ros/tree/ros2-devel) GitHub repository.

> [!NOTE] Differences in ROS System
>
> ROS 2 nodes differs slightly between **Panther v1.06** and **Panther v1.2+**. This is caused by internal hardware differences. Despite that, the ROS API was kept as closely matched between those revisions as possible and should be transparent in most of the use cases.

<!-- TODO: add this differences -->

The default way to communicate with Panther's hardware is via the Robot Operating System (ROS). All of the drivers were written in ROS 2 framework. The ROS API is provided by ROS packages found in the GitHub repository [husarion/panther_ros](https://github.com/husarion/panther_ros). These packages are responsible for accessing the hardware components of the robot.

The graph below represents Panther's ROS system. Some topics and services have been excluded from the graph for the sake of clarity.

![Panther ROS 2 API Diagram](.docs/panther_ros2_api_diagram.png)

## ROS Interfaces

Below is information about the physical robot API. For the simulation, topics and services are identical to the physical robot, but due to certain limitations, not all interfaces are present in the simulation.

| Symbol | Meaning                         |
| ------ | ------------------------------- |
| 🤖      | Available for physical robot    |
| 🖥️      | Available in simulated robot    |
| ⚙️      | Requires specific configuration |

### Topics

|     | Topic                            | Description                                                                                                                                                                                                                                                                                                                                                               |
| --- | -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 🤖🖥️  | `battery`                        | Mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published.<br/> [`sensor_msgs/BatteryState`](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html)                                                                                                                                           |
| 🤖   | `charging_status`                | Battery charging status value.<br/> [`panther_msgs/ChargingStatus`](https://github.com/husarion/panther_msgs)                                                                                                                                                                                                                                                             |
| 🤖🖥️  | `cmd_vel`                        | Command velocity value.<br/> [`geometry_msgs/Twist`](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)                                                                                                                                                                                                                                                       |
| 🤖🖥️  | `diagnostics`                    | Diagnostic data.<br/> [`diagnostic_msgs/DiagnosticArray`](https://docs.ros2.org/latest/api/diagnostic_msgs/msg/DiagnosticArray.html)                                                                                                                                                                                                                                      |
| 🤖🖥️  | `driver/motor_controllers_state` | Current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data.<br/> [`panther_msgs/DriverState`](https://github.com/husarion/panther_msgs)                                                                                                                                                                                         |
| 🤖🖥️  | `dynamic_joint_states`           | Dynamic joint state information.<br/> [`control_msgs/DynamicJointState`](https://github.com/ros-controls/control_msgs/blob/master/control_msgs/msg/DynamicJointState.msg)                                                                                                                                                                                                 |
| 🤖🖥️  | `ekf_node/set_pose`              | Set the pose of the EKF node.<br/> [`geometry_msgs/PoseWithCovarianceStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseWithCovarianceStamped.html)                                                                                                                                                                                                         |
| 🤖🖥️⚙️ | `gps/fix`                        | Raw GPS data.<br/> [`sensor_msgs/NavSatFix`](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)                                                                                                                                                                                                                                                             |
| 🤖🖥️⚙️ | `gps/filtered`                   | Filtered GPS position after fusing odometry data.<br/> [`sensor_msgs/NavSatFix`](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html)                                                                                                                                                                                                                         |
| 🤖   | `hardware/e_stop`                | Current E-stop state.<br/> [`std_msgs/Bool`](https://docs.ros.org/en/latest/api/std_msgs/html/msg/Bool.html)                                                                                                                                                                                                                                                              |
| 🤖   | `hardware/io_state`              | Current IO state.<br/> [`panther_msgs/IOState`](https://github.com/husarion/panther_msgs)                                                                                                                                                                                                                                                                                 |
| 🤖🖥️  | `imu/data`                       | Filtered IMU data.<br/> [`sensor_msgs/Imu`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html)                                                                                                                                                                                                                                                                    |
| 🤖🖥️  | `joint_states`                   | <br/> [`sensor_msgs/JointState`](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)                                                                                                                                                                                                                                                                        |
| 🤖🖥️  | `lights/driver/channel_1_frame`  | <br/> [`sensor_msgs/Image`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)                                                                                                                                                                                                                                                                                  |
| 🤖🖥️  | `lights/driver/channel_2_frame`  | <br/> [`sensor_msgs/Image`](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)                                                                                                                                                                                                                                                                                  |
| 🤖🖥️  | `odometry/filtered`              | Contains information about the filtered position and orientation. When `localization_mode` is `relative`, the position and orientation are relative to the starting point. When `localization_mode` is `enu`, the orientation is relative to the east-north-up (ENU) coordinates.<br/> [`nav_msgs/Odometry`](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html) |
| 🤖🖥️  | `odometry/wheels`                | Robot odometry calculated from wheels.<br/> [`nav_msgs/Odometry`](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)                                                                                                                                                                                                                                            |
| 🤖🖥️  | `robot_description`              | <br/> [`std_msgs/String`](https://docs.ros2.org/latest/api/std_msgs/msg/String.html)                                                                                                                                                                                                                                                                                      |
| 🤖   | `system_status`                  | <br/> [`panther_msgs/SystemStatus`](https://github.com/husarion/panther_msgs)                                                                                                                                                                                                                                                                                             |
| 🤖🖥️  | `tf`                             | Transforms of robot system.<br/> [`tf2_msgs/TFMessage`](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html)                                                                                                                                                                                                                                                     |
| 🤖🖥️  | `tf_static`                      | <br/> [`tf2_msgs/TFMessage`](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html)                                                                                                                                                                                                                                                                                |

#### Hidden topics

|     | Topic            | Description                                                                                                                                                           |
| --- | ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 🤖   | `_battery_1_raw` | First battery raw state.<br/> [`sensor_msgs/BatteryState`](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html)                                        |
| 🤖   | `_battery_2_raw` | Second battery raw state. Published if second battery detected.<br/> [`sensor_msgs/BatteryState`](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html) |
| 🤖🖥️⚙️ | `_odometry/gps`  | Transformed raw GPS data to odometry format.<br/> [`nav_msgs/Odometry`](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)                                  |

### Services

| Service                                           | Description                                                                                                                                                 |
| ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `controller_manager/configure_controller`         | <br/> [controller_manager_msgs/srv/ConfigureController](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                   |
| `controller_manager/list_controller_types`        | <br/> [controller_manager_msgs/srv/ListControllerTypes](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                   |
| `controller_manager/list_controllers`             | <br/> [controller_manager_msgs/srv/ListControllers](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                       |
| `controller_manager/list_hardware_components`     | <br/> [controller_manager_msgs/srv/ListHardwareComponents](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                |
| `controller_manager/list_hardware_interfaces`     | <br/> [controller_manager_msgs/srv/ListHardwareInterfaces](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                |
| `controller_manager/load_controller`              | <br/> [controller_manager_msgs/srv/LoadController](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                        |
| `controller_manager/reload_controller_libraries`  | <br/> [controller_manager_msgs/srv/ReloadControllerLibraries](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)             |
| `controller_manager/set_hardware_component_state` | <br/> [controller_manager_msgs/srv/SetHardwareComponentState](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)             |
| `controller_manager/switch_controller`            | <br/> [controller_manager_msgs/srv/SwitchController](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                      |
| `controller_manager/unload_controller`            | <br/> [controller_manager_msgs/srv/UnloadController](https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs)                      |
| `ekf_node/enable`                                 | Enable EKF node.<br/> [std_srvs/srv/Empty](https://docs.ros2.org/latest/api/std_srvs/srv/Empty.html)                                                        |
| `ekf_node/set_pose`                               | Set pose of EKF node.<br/> [robot_localization/srv/SetPose](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)                                    |
| `ekf_node/toggle`                                 | Toggle filter processing in the EKF node.<br/> [robot_localization/srv/ToggleFilterProcessing](https://github.com/cra-ros-pkg/robot_localization/tree/ros2) |
| `hardware/aux_power_enable`                       | Enables or disables AUX power.<br/> [std_srvs/srv/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html)                                      |
| `hardware/charger_enable`                         | Enables or disables charger.<br/> [std_srvs/srv/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html)                                        |
| `hardware/digital_power_enable`                   | Enables or disables digital power.<br/> [std_srvs/srv/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html)                                  |
| `hardware/e_stop_reset`                           | Resets E-stop.<br/> [std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)                                                      |
| `hardware/e_stop_trigger`                         | Triggers E-stop.<br/> [std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)                                                    |
| `hardware/fan_enable`                             | Enables or disables fan.<br/> [std_srvs/srv/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html)                                            |
| `hardware/motor_power_enable`                     | Enables or disables motor power.<br/> [std_srvs/srv/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html)                                    |
| `lights/controller/set/animation`                 | Sets LED animation.<br/> [panther_msgs/srv/SetLEDAnimation](https://github.com/husarion/panther_msgs)                                                       |
| `lights/driver/set/brightness`                    | Sets LED brightness.<br/> [panther_msgs/srv/SetLEDBrightness](https://github.com/husarion/panther_msgs)                                                     |
