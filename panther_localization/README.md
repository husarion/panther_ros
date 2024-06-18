# panther_localization

The package is responsible for activating mods responsible for fusion of data related to the robot's location.

## Launch files

This package contains:

- `ekf.launch.py` - is responsible for activating EKF filtration along with the necessary dependencies needed to operate GPS

### ekf.launch.py - Launch Arguments

- `ekf_config_path` [*string*, default: **panther_localization/config/ekf.yaml**]: path to the EKF config file.
- `ekf_configuration` [*string*, default: **local**]: set the EKF mode: `local` combines wheel odometer and IMU data. `global` adds GPS data to this fusion.
- `namespace` [*string*, default: **env(ROBOT_NAMESPACE)**]: add namespace to all launched nodes.
- `use_sim` [*bool*, default: **False**]: whether simulation is used.

### ekf.launch.py - Nodes Launched

- `ekf_local` [*[robot_localization/ekf_node](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)*]: Extended Kalman Filter node responsible for fusing odometry data related to starting position of the robot.
- `ekf_global` [*[robot_localization/ekf_node](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)*]: Extended Kalman Filter node responsible for data fusion from local fusion and global data (e.g. GPS). Runs when argument `ekf_configuration:=global`.
- `navsat_transform` [*[robot_localization/navsat_transform_node](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)*]:
It converts raw GPS data into odometry data and publishes corrected GPS positions based on sensor data at a higher frequency.

#### ekf_local - Subscriber

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command velocity value.
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: robot odometry calculated from wheels.
- `imu/data` [*sensor_msgs/msg/Imu*]: filtered IMU data.
- `/tf` [*tf2_msgs/msg/TFMessage*]: transforms of robot system.

#### ekf_local - Publishers

- `diagnostics` [*diagnostic_msgs/msg/DiagnosticArray*]: diagnostic data.
- `odometry/filtered/local` [*nav_msgs/msg/Odometry*]: contains information about the position and velocities in relation to the initial position of the robot.
- `/tf` [*tf2_msgs/msg/TFMessage*]: publishes `odom` to `base_link` transform.

#### ekf_local - Service Servers

- `~/set_pose` [*robot_localization/srv/SetPose*]: upon request, users can manually set the robot's position and speed. This is useful for resetting positions, e.g. during tests.

#### ekf_global - Subscriber

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command velocity value.
- `imu/data` [*sensor_msgs/msg/Imu*]: filtered IMU data.
- `_odometry/gps` [*nav_msgs/msg/Odometry*]: transformed GPS data.
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: robot odometry calculated from wheels.
- `/tf` [*tf2_msgs/msg/TFMessage*]: transforms of robot system.

#### ekf_global - Publishers

- `diagnostics` [*diagnostic_msgs/msg/DiagnosticArray*]: diagnostic data.
- `odometry/filtered/global`[*nav_msgs/msg/Odometry*]: contains information about the position and velocities in relation to the initial position and orientation based on geographic  ENU convention (x-east, y-north, z-up). [*nav_msgs/Odometry*]: robot odometry calculated from wheels.
- `/tf` [*tf2_msgs/msg/TFMessage*]: publishes `odom` to `base_link` transform.

#### ekf_global - Service Servers

- `~/set_pose` [*robot_localization/srv/SetPose*]:upon request, users can manually set the robot's position and speed. This is useful for resetting positions, e.g. during tests.

#### navsat_transform - Subscriber

- `gps/fix` [*sensor_msgs/msg/NavSatFix*]: raw GPS data.
- `odometry/filtered/global` [*nav_msgs/msg/Odometry*]: odometry to transform to GPS position.

#### navsat_transform - Publishers

- `gps/filtered` [*sensor_msgs/msg/NavSatFix*]: GPS position after including sensor data from `ekf_local`.
- `_odometry/gps` [*nav_msgs/msg/Odometry*]: transformed raw GPS data to odometry data.
