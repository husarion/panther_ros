# panther_localization

The package is responsible for activating mods responsible for fusion of data related to the robot's location.

## Launch files

This package contains:

- `localization.launch.py` - is responsible for activating EKF filtration along with the necessary dependencies needed to operate GPS

### localization.launch.py - Launch Arguments

- `fuse_gps` [*bool*, default: **False**]: Include GPS for data fusion.
- `localization_config_path` [*string*, default: **depends on `fuse_gps` and `localization_mode`**]: path to the EKF config file.
- `localization_mode` [*string*, default: **relative**]: Specifies the localization mode:
  - `relative` - `odometry/filtered` data is relative to the initial position and orientation.
  - `enu` - `odometry/filtered` data is relative to the initial position and ENU (East North Up) orientation.
- `namespace` [*string*, default: **env(ROBOT_NAMESPACE)**]: add namespace to all launched nodes.
- `use_sim` [*bool*, default: **False**]: whether simulation is used.

### localization.launch.py - Nodes Launched

- `ekf_filter` [*[robot_localization/ekf_node](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)*]: The Extended Kalman Filter node is designed to fuse odometry data from various sources, including wheel encoders, IMU, and GPS.
- `navsat_transform` [*[robot_localization/navsat_transform_node](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)*]:
It converts raw GPS data into odometry data and publishes corrected GPS positions based on sensor data at a higher frequency.

#### ekf_filter - Subscriber

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command velocity value.
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: robot odometry calculated from wheels.
- `imu/data` [*sensor_msgs/msg/Imu*]: filtered IMU data.
- `localization/set_pose` [*geometry_msgs/msg/PoseWithCovarianceStamped*]: allows manually set the state of the filter by sending the pose.
- `/tf` [*tf2_msgs/msg/TFMessage*]: transforms of robot system.

#### ekf_filter - Publishers

- `diagnostics` [*diagnostic_msgs/msg/DiagnosticArray*]: diagnostic data.
- `odometry/filtered` [*nav_msgs/msg/Odometry*]: contains information about the filtered position and orientation. When `localization_mode` is `relative`, the position and orientation are relative to the starting point. When `localization_mode` is `enu`, the orientation is relative to the east-north-up (ENU) coordinates.
- `/tf` [*tf2_msgs/msg/TFMessage*]: publishes `odom` to `base_link` transform.

#### ekf_filter - Service Servers

- `localization/set_pose` [*robot_localization/srv/SetPose*]: upon request, users can manually set the robot's position and speed. This is useful for resetting positions, e.g. during tests.
- `odometry/filtered/global`[*nav_msgs/msg/Odometry*]: contains information about the position and velocities in relation to the initial position and orientation based on geographic  ENU convention (x-east, y-north, z-up). [*nav_msgs/Odometry*]: robot odometry calculated from wheels.

#### navsat_transform - Subscriber

- `gps/fix` [*sensor_msgs/msg/NavSatFix*]: raw GPS data.
- `odometry/filtered/global` [*nav_msgs/msg/Odometry*]: odometry to transform to GPS position.

#### navsat_transform - Publishers

- `gps/filtered` [*sensor_msgs/msg/NavSatFix*]: GPS position after including sensor data from `ekf_filter`.
- `_odometry/gps` [*nav_msgs/msg/Odometry*]: transformed raw GPS data to odometry format.
