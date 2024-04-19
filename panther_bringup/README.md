[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_bringup

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## Default Nodes Launched

- `battery_node` [*[panther_battery/battery_node](https://github.com/husarion/panther_ros/panther_battery/src/main.cpp)*]: node responsible for monitoring and publishing the internal Battery state of the Husarion Panther robot. For more information, refer to [panther_battery](https://github.com/husarion/panther_ros/panther_battery/README.md).
- `ekf_node` [*[robot_localization/ekf_localization_node](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ekf_node.cpp)*]: Extended Kalman Filter node for more accurate odometry. For more information, refer to [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/noetic-devel). The default configuration is stored in [ekf_config.yaml](https://github.com/husarion/panther_ros/panther_gazebo/config/ekf_config.yaml).
- `imu_container` [*[phidgets_spatial/phidgets::SpatialRosI](https://github.com/ros-drivers/phidgets_drivers/blob/humble/phidgets_spatial/src/spatial_ros_i.cpp)*, *[imu_filter_madgwick/ImuFilterMadgwickRos](https://github.com/CCNYRoboticsLab/imu_tools/blob/humble/imu_filter_madgwick/src/imu_filter_node.cpp)*]: container responsible for running Phidget Spatial IMU ROS driver, filtering and fusing the IMU data. It composes the `phidgets_spatial_node` and `imu_filter_node`.

## Bringup Launch Arguments

- `battery_config_path` [*string*, default: **None**]: path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only.
- `controller_config_path` [*string*, default: **panther_controller/config/<wheel_type arg>_controller.yaml**]: path to controller configuration file. A path to custom configuration can be specified here.
- `disable_manager` [*bool*, default: **false**]: Enable or disable manager_bt_node.
- `ekf_config_path` [*string*, default: **panther_bringup/config/ekf.yaml**]: path to the EKF configuration file.
- `led_config_file` [*string*, default: **panther_lights/config/led_config.yaml**]: path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations.
- `namespace` [*string*, default: **None**] Add namespace to all launched nodes.
- `publish_robot_state` [*bool*, default: **true**]: whether to publish the default Panther robot description.
- `shutdown_hosts_config_path` = [*string*, default: **panther_bringup/config/shutdown_hosts.yaml**]: Path to file with list of hosts to request shutdown.
- `simulation_engine` [*string*, default: **ignition-gazebo**]: simulation engine to use when running Gazebo.
- `use_ekf` [*bool*, default: **true**]: enable or disable Extended Kalman Filter.
- `use_sim` [*bool*, default: **false**]: whether simulation is used.
- `user_led_animations_file` [*string*, default: **None**]: path to a YAML file with a description of the user defined animations.
- `wheel_config_path` [*string*, default: **$(find panther_description)/config/<wheel_type arg>.yaml**]: path to YAML file with wheel specification. Arguments become required if `wheel_type` is set to **custom**.
- `wheel_type` [*string*, default: **WH01**]: type of wheel, possible are: **WH01** - offroad, **WH02** - mecanum, **WH04** - small pneumatic, and **custom** - custom wheel types (requires setting `wheel_config_path` argument accordingly).

[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

## External ROS Nodes

[//]: # (ROS_API_PACKAGE_NAME_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### ekf_node

[//]: # (ROS_API_NODE_NAME_END)

[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type: *[robot_localization/ekf_node](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/ekf_node.cpp)*.

Extended Kalman Filter node for more accurate odometry. For more information, refer to [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/humble-devel). The default configuration is stored in `panther_bringup/config/ekf.yaml`.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `~/odom/wheels` [*nav_msgs/Odometry*]: robot odometry calculated from wheels.
- `~/imu/data` [*sensor_msgs/Imu*]: filtered IMU data.
- `/tf` [*tf2_msgs/TFMessage*]: transforms of robot system.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/odometry/filtered` [*nav_msgs/Odometry*]: provides filtered odometry information. This topic contains a fused and enhanced estimate of the robot's pose and velocity, incorporating data from various sensors and correcting for any errors in the estimated state.
- `/tf` [*tf2_msgs/TFMessage*]: publishes `odom` to `base_link` transform.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `~/ekf_node/set_pose` [*robot_localization/SetPose*]: by issuing a *geometry_msgs/PoseWithCovarianceStamped* message to the set_pose topic, users can manually set the state of the filter. This is useful for resetting the filter during testing and allows for interaction with RViz. Alternatively, the state estimation nodes advertise a SetPose service, whose type is *robot_localization/SetPose*.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### imu_filter_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type: *[imu_filter_madgwick/imu_filter_node](https://github.com/CCNYRoboticsLab/imu_tools/blob/humble/imu_filter_madgwick/src/imu_filter_node.cpp)*.

Node responsible for filtering and fusing raw data from the IMU.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `~/imu/data_raw` [*sensor_msgs/Imu*]: the raw accelerometer and gyroscope data.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/imu/data` [*sensor_msgs/Imu*]: the fused IMU messages, containing the orientation.

[//]: # (ROS_API_NODE_PUBLISHERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### phidgets_spatial_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type: *[phidgets_spatial/spatial_ros_i.cpp](https://github.com/ros-drivers/phidgets_drivers/blob/humble/phidgets_spatial/src/spatial_ros_i.cpp)*.

The ROS driver for Phidgets Spatial.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/imu/data_raw` [*sensor_msgs/Imu*]: the raw accelerometer and gyroscope data.
- `~/imu/is_calibrated` [*std_msgs/Bool*]: whether the gyroscope has been calibrated. This will be done automatically at startup time but can also be re-done at any time by calling the `imu/calibrate` service.
- `~/imu/mag` [*sensor_msgs/MagneticField*]: the raw magnetometer data.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `~/imu/calibrate` [*std_srvs/Empty*]: run calibration on the gyroscope.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_PACKAGE_END)
