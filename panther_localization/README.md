# panther_localization

The package is responsible for activating mods responsible for fusion of data related to the robot's location.

## Launch files

This package contains:

- `localization.launch.py` - is responsible for activating EKF filtration along with the necessary dependencies needed to operate GPS

### localization.launch.py

| Argument                   | *Type*: `Default`                | Description                                                                                                                                                                                                                                         |
| -------------------------- | -------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `fuse_gps`                 | *bool*: `False`                  | Include GPS for data fusion.                                                                                                                                                                                                                        |
| `localization_config_path` | *string*: depends                | Path to the EKF config. Path depends on: `fuse_gps` and `localization_mode`file.                                                                                                                                                                    |
| `localization_mode`        | *string*: `relative`             | Specifies the localization mode:<br/> - `relative` - `odometry/filtered` data is relative to the initial position and orientation.<br/> - `enu` - `odometry/filtered` data is relative to the initial position and ENU (East North Up) orientation. |
| `namespace`                | *string*: `env(ROBOT_NAMESPACE)` | Add namespace to all launched nodes.                                                                                                                                                                                                                |
| `use_sim`                  | *bool*: `False`                  | Whether simulation is used.                                                                                                                                                                                                                         |

| Node               | *Type*                                                                                                    |
| ------------------ | --------------------------------------------------------------------------------------------------------- |
| `ekf_node`         | [*robot_localization/ekf_node*](https://github.com/cra-ros-pkg/robot_localization/tree/ros2)              |
| `navsat_transform` | [*robot_localization/navsat_transform_node*](https://github.com/cra-ros-pkg/robot_localization/tree/ros2) |

## Running Nodes

- `ekf_node`: The Extended Kalman Filter node is designed to fuse odometry data from various sources, including wheel encoders, IMU, and GPS.
- `navsat_transform`: It converts raw GPS data into odometry data and publishes corrected GPS positions based on sensor data at a higher frequency.

### ekf_node - Configuration

You can find the ekf_nod parameters in the config folder in the files: `enu_localization.yaml`, `enu_localization.yaml`, `relative_localization.yaml`, `relative_localization_with_gps.yaml`. Selection of the configuration file, depending on the launch arguments. A detailed explanation of the parameters described there can be found in the robot localization package documentation for the [state_estimation_nodes](http://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html).

### navsat_transform - Configuration

You can find the ekf_nod parameters in the config folder in the files: `enu_localization.yaml`, `enu_localization.yaml`, `relative_localization.yaml`, `relative_localization_with_gps.yaml`. Selection of the configuration file, depending on the launch arguments. A detailed explanation of the parameters described there can be found in the robot localization package documentation for the [navsat_transform_node](https://docs.ros.org/en/api/robot_localization/html/navsat_transform_node.html).
