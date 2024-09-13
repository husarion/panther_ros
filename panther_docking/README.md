# panther_docking

The package contains a `PantherChargingDock` plugin for the [opennav_docking](https://github.com/open-navigation/opennav_docking) project. Thanks to this package, Panther can dock to a charging station.

## Launch Files

- `docking.launch.py`: Launch a node that creates `docking_server` and runs a `PantherChargingDock` plugin. Also it launches `station.launch.py`.
- `station.launch.py`: Launch a node that creates a charging station description with generated apriltag.

## Configuration Files

- [`panther_docking_server.yaml`](./config/panther_docking_server.yaml): Defines parameters for a `docking_server` and a `PantherChargingDock` plugin.

## ROS Nodes

- `PantherChargingDock`:  A plugin for a Panther robot what is responsible for a charger service.

### PantherChargingDock

#### Publishes

- `docking/dock_pose` [*geometry_msgs/PoseStamped*]: An offset dock pose.
- `docking/staging_pose` [*geometry_msgs/PoseStamped*]: An offset staging pose next to a charging station.

#### Subscribers

- `battery/charging_status` [*panther_msgs/ChargingStatus*]: A charging status of Panther robot.
- `hardware/io_state` [*panther_msgs/IOState*]: States of GPIOs of Panther robot. The `PantherChargingDock` subscribes this topic to check if a robot charges.

#### Service Clients

- `hardware/charger_enable` [*std_srvs/SetBoot*]: This service client enables charging in a robot.

#### Parameters

- `~panther_version` [*double*, default: **1.21**]: A version of Panther robot.
- `~<dock_name>.base_frame` [*string*, default: **base_link**]: A base frame id of a robot.
- `~<dock_name>.external_detection_timeout` [*double*, default: **0.2**]: A timeout in seconds for looking up a transformation from an april tag of a dock to a base frame id.
- `~<dock_name>.external_detection_translation_x` [*double*, default: **0.0**]: A translation over an X axis between a detected frame and a dock pose.
- `~<dock_name>.external_detection_translation_y` [*double*, default: **0.0**]: A translation over an Y axis between a detected frame and a dock pose.
- `~<dock_name>.external_detection_translation_x` [*double*, default: **0.0**]: A translation over a Z axis between a detected frame and a dock pose.
- `~<dock_name>.external_detection_rotation_roll` [*double*, default: **0.0**]: A rotation over an X axis between a detected frame and a dock pose.
- `~<dock_name>.external_detection_rotation_pitch` [*double*, default: **0.0**]: A rotation over an Y axis between a detected frame and a dock pose.
- `~<dock_name>.external_detection_rotation_yaw` [*double*, default: **0.0**]: A rotation over a Z axis between a detected frame and a dock pose.
- `~<dock_name>.filter_coef` [*double*, default: **0.1**]: A key parameter that influences the trade-off between the filter's responsiveness and its smoothness, balancing how quickly it reacts to new pose data pose how much it smooths out fluctuations.
- `~<dock_name>.docking_distance_threshold` [*double*, default: **0.05**]: A threshold of a distance between a robot pose and a dock pose to declare if docking succeed.
- `~<dock_name>.docking_yaw_threshold` [*double*, default: **0.3**]: A threshold of a difference of yaw angles between a robot pose and a dock pose to declare if docking succeed.
- `~<dock_name>.staging_x_offset` [*double*, default: **-0.7**]: A staging pose is defined by offsetting a dock pose in axis X.
- `~<dock_name>.staging_yaw_offset` [*double*, default: **0.0**]: A staging pose is defined by offsetting a yaw angle.
- `~<dock_name>.enable_charger_service_call_timeout` [*double*, default: **0.2**]: A timeout for calling enable charging service. A robot is unable to dock if excised.
