# panther_localization

The package is responsible for activating mods responsible for fusion of data related to the robot's location.

## Launch files

This package contains:

- `localization.launch.py` - is responsible for activating EKF filtration along with the necessary dependencies needed to operate GPS

## Configuration Files

- [`enu_localization.yaml`](./config/enu_localization.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders** and **IMU**. Orientation follows East-North-Up (ENU) coordinates.
- [`enu_localization_with_gps.yaml`](./config/enu_localization_with_gps.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**, and **GPS**. Orientation follows East-North-Up (ENU) coordinates.
- [`relative_localization.yaml`](./config/relative_localization.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**. The initial orientation is always 0 in relative mode.
- [`relative_localization_with_gps.yaml`](./config/relative_localization_with_gps.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**, and **GPS**. The initial orientation is always 0 in relative mode.

## ROS Nodes

| Node name          | Description <br/> *Type*                                                                                                                                                                                                       |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `ekf_filter`       | The Extended Kalman Filter node is designed to fuse odometry data from various sources, including wheel encoders, IMU, and GPS. <br/> *[robot_localization/ekf_filter](https://github.com/cra-ros-pkg/robot_localization)*     |
| `navsat_transform` | It converts raw GPS data into odometry data and publishes corrected GPS positions based on sensor data at a higher frequency. <br/> *[robot_localization/navsat_transform](https://github.com/cra-ros-pkg/robot_localization)* |

### ekf_filter

#### Subscribers

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command velocity value.
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: robot odometry calculated from wheels.
- `imu/data` [*sensor_msgs/msg/Imu*]: filtered IMU data.
- `localization/set_pose` [*geometry_msgs/msg/PoseWithCovarianceStamped*]: allows manually set the state of the filter by sending the pose.
- `/tf` [*tf2_msgs/msg/TFMessage*]: transforms of robot system.

#### Publishers

- `diagnostics` [*diagnostic_msgs/msg/DiagnosticArray*]: diagnostic data.
- `odometry/filtered` [*nav_msgs/msg/Odometry*]: contains information about the filtered position and orientation. When `localization_mode` is `relative`, the position and orientation are relative to the starting point. When `localization_mode` is `enu`, the orientation is relative to the east-north-up (ENU) coordinates.
- `/tf` [*tf2_msgs/msg/TFMessage*]: publishes `odom` to `base_link` transform.

#### Service Servers

- `localization/set_pose` [*robot_localization/srv/SetPose*]: upon request, users can manually set the robot's position and speed. This is useful for resetting positions, e.g. during tests.

#### Parameters

A detailed explanation of the parameters can be found in the robot localization package documentation for [state_estimation_nodes](http://docs.ros.org/en/api/robot_localization/html/state_estimation_nodes.html).

### navsat_transform

#### Subscriber

- `gps/fix` [*sensor_msgs/msg/NavSatFix*]: raw GPS data.
- `odometry/filtered/global` [*nav_msgs/msg/Odometry*]: odometry to transform to GPS position.

#### Publishers

- `gps/filtered` [*sensor_msgs/msg/NavSatFix*]: GPS position after including sensor data from `ekf_filter`.
- `_odometry/gps` [*nav_msgs/msg/Odometry*]: transformed raw GPS data to odometry format.

#### Parameters

A detailed explanation of the parameters can be found in the robot localization package documentation for [navsat_transform_node](http://docs.ros.org/en/api/robot_localization/html/navsat_transform_node.html).
