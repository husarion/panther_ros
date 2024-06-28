# ROS API

| Color | Description                                              |
| ----- | -------------------------------------------------------- |
| 🟩     | Available both for hardware and simulation               |
| 🟧     | Available only in specific launch argument configuration |
| 🟦     | Available only in simulation mode                        |

| Node               | *Type*                                                            |
| ------------------ | ----------------------------------------------------------------- |
| `ekf_node`         | [**](https://github.com/cra-ros-pkg/robot_localization/tree/ros2) |
| `navsat_transform` | [](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)   |

## Running Nodes

| Node                      | Type                                       | Description                                                                                                                                                 |
| ------------------------- | ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `battery_node`            | *panther_batter/battery_node*              | Publishes battery state read from ADC unit for Panther version 1.2 and above, or based on Roboteq motor controllers' data for earlier versions of the robot |
| `controller_manager`      |                                            |                                                                                                                                                             |
| `ekf_node`                | *robot_localization/ekf_node*              | The Extended Kalman Filter node is designed to fuse odometry data from various sources, including wheel encoders, IMU, and GPS.                             |
| `imu_broadcaster`         |                                            |                                                                                                                                                             |
| `joint_state_broadcaster` |                                            |                                                                                                                                                             |
| `lights_controller_node`  |                                            |                                                                                                                                                             |
| `lights_driver_node`      |                                            |                                                                                                                                                             |
| `lights_manager_node`     |                                            |                                                                                                                                                             |
| 🟧 `navsat_transform`      | *robot_localization/navsat_transform_node* | It converts raw GPS data into odometry data and publishes corrected GPS positions based on sensor data at a higher frequency.                               |
| `panther_base_controller` |                                            |                                                                                                                                                             |
| `panther_system_node`     |                                            |                                                                                                                                                             |
| `robot_state_publisher`   |                                            |                                                                                                                                                             |
| `safety_manager_node`     |                                            |                                                                                                                                                             |
| `system_status_node`      |                                            |                                                                                                                                                             |

## Topics

| Topic                                      | *Type*                                        | Description                                                                                                                                                                                                                                                                       |
| ------------------------------------------ | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `battery`                                  | *sensor_msgs/msg/BatteryState*                | mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published                                                                                                                                                          |
| `battery_1_raw`                            | *sensor_msgs/msg/BatteryState*                | first battery raw state                                                                                                                                                                                                                                                           |
| `battery_2_raw`                            | *sensor_msgs/msg/BatteryState*                | second battery raw state. Published if second battery detected.                                                                                                                                                                                                                   |
| `charging_status`                          | *panther_msgs/ChargingStatus*                 | battery charging status value                                                                                                                                                                                                                                                     |
| `cmd_vel`                                  | *geometry_msgs/msg/Twist*                     | command velocity value                                                                                                                                                                                                                                                            |
| `diagnostics`                              | *diagnostic_msgs/msg/DiagnosticArray*         | diagnostic data                                                                                                                                                                                                                                                                   |
| `driver/motor_controllers_state`           | *panther_msgs/msg/DriverState*                | current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data                                                                                                                                                                              |
| `dynamic_joint_states`                     | *control_msgs/msg/DynamicJointState*          |                                                                                                                                                                                                                                                                                   |
| `ekf_node/set_pose`                        | *geometry_msgs/msg/PoseWithCovarianceStamped* |                                                                                                                                                                                                                                                                                   |
| 🟧 `gps/fix`                                | *sensor_msgs/msg/NavSatFix*                   | raw GPS data                                                                                                                                                                                                                                                                      |
| 🟧 `gps/filtered`                           | *sensor_msgs/msg/NavSatFix*                   | GPS position after including sensor data from `ekf_node`                                                                                                                                                                                                                          |
| `hardware/e_stop`                          | *std_msgs/msg/Bool*                           |                                                                                                                                                                                                                                                                                   |
| `hardware/io_state`                        | *panther_msgs/msg/IOState*                    |                                                                                                                                                                                                                                                                                   |
| `imu/data`                                 | *sensor_msgs/msg/Imu*                         | filtered IMU data                                                                                                                                                                                                                                                                 |
| `imu_broadcaster/imu`                      | *sensor_msgs/msg/Imu*                         |                                                                                                                                                                                                                                                                                   |
| `imu_broadcaster/transition_event`         | *lifecycle_msgs/msg/TransitionEvent*          |                                                                                                                                                                                                                                                                                   |
| `joint_state_broadcaster/transition_event` | *lifecycle_msgs/msg/TransitionEvent*          |                                                                                                                                                                                                                                                                                   |
| `joint_states`                             | *sensor_msgs/msg/JointState*                  |                                                                                                                                                                                                                                                                                   |
| `lights/driver/channel_1_frame`            | *sensor_msgs/msg/Image*                       |                                                                                                                                                                                                                                                                                   |
| `lights/driver/channel_2_frame`            | *sensor_msgs/msg/Image*                       |                                                                                                                                                                                                                                                                                   |
| `odometry/filtered`                        | *nav_msgs/msg/Odometry*                       | contains information about the filtered position and orientation. When `localization_mode` is `relative`, the position and orientation are relative to the starting point. When `localization_mode` is `enu`, the orientation is relative to the east-north-up (ENU) coordinates. |
| `odometry/wheels`                          | *nav_msgs/msg/Odometry*                       | robot odometry calculated from wheels.                                                                                                                                                                                                                                            |
| `panther_base_controller/transition_event` | *lifecycle_msgs/msg/TransitionEvent*          |                                                                                                                                                                                                                                                                                   |
| `robot_description`                        | *std_msgs/msg/String*                         |                                                                                                                                                                                                                                                                                   |
| `system_status`                            | *panther_msgs/msg/SystemStatus*               |                                                                                                                                                                                                                                                                                   |
| `tf`                                       | *tf2_msgs/msg/TFMessage*                      | transforms of robot system                                                                                                                                                                                                                                                        |
| `tf_static`                                | *tf2_msgs/msg/TFMessage*                      |                                                                                                                                                                                                                                                                                   |

- 🟧 `_odometry/gps` [*nav_msgs/msg/Odometry*]: transformed raw GPS data to odometry format.

## Services

- `~/set_pose` [*robot_localization/srv/SetPose*]: upon request, users can manually set the robot's position and speed. This is useful for resetting positions, e.g. during tests.
- `~/imu/calibrate` [*std_srvs/Empty*]: run calibration on the gyroscope.

/controller_manager/configure_controller [controller_manager_msgs/srv/ConfigureController]
/controller_manager/list_controller_types [controller_manager_msgs/srv/ListControllerTypes]
/controller_manager/list_controllers [controller_manager_msgs/srv/ListControllers]
/controller_manager/list_hardware_components [controller_manager_msgs/srv/ListHardwareComponents]
/controller_manager/list_hardware_interfaces [controller_manager_msgs/srv/ListHardwareInterfaces]
/controller_manager/load_controller [controller_manager_msgs/srv/LoadController]
/controller_manager/reload_controller_libraries [controller_manager_msgs/srv/ReloadControllerLibraries]
/controller_manager/set_hardware_component_state [controller_manager_msgs/srv/SetHardwareComponentState]
/controller_manager/switch_controller [controller_manager_msgs/srv/SwitchController]
/controller_manager/unload_controller [controller_manager_msgs/srv/UnloadController]
/ekf_node/enable [std_srvs/srv/Empty]
/ekf_node/set_pose [robot_localization/srv/SetPose]
/ekf_node/toggle [robot_localization/srv/ToggleFilterProcessing]
/hardware/aux_power_enable [std_srvs/srv/SetBool]
/hardware/charger_enable [std_srvs/srv/SetBool]
/hardware/digital_power_enable [std_srvs/srv/SetBool]
/hardware/e_stop_reset [std_srvs/srv/Trigger]
/hardware/e_stop_trigger [std_srvs/srv/Trigger]
/hardware/fan_enable [std_srvs/srv/SetBool]
/hardware/motor_power_enable [std_srvs/srv/SetBool]
/lights/controller/set/animation [panther_msgs/srv/SetLEDAnimation]
/lights/driver/set/brightness [panther_msgs/srv/SetLEDBrightness]
