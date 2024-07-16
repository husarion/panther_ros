# panther_bringup

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

## Launch Files

This package contains:

- [`bringup.launch.py`](#bringuplaunchpy---arguments) - is responsible for activating whole robot system.

### bringup.launch.py - Arguments

| Argument                     | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                                                                        |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `battery_config_path`        | Path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only. <br/>  ***string:*** `None`                                                                                                                                                                                                           |
| `components_config_path`     | Additional components configuration file. Components described in this file are dynamically included in Panther's urdf. Panther options are described in [the manual](https://husarion.com/manuals/panther/panther-options).  <br/>  ***string:*** [`components.yaml`](../panther_description/config/components.yaml)                                          |
| `controller_config_path`     | Path to controller configuration file. A path to custom configuration can be specified here. <br/>  ***string:*** [`<wheel_type arg>_controller.yaml`](../panther_controller/config/)                                                                                                                                                                          |
| `disable_manager`            | Enable or disable manager_bt_node.  <br/>  ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                      |
| `fuse_gps`                   | Include GPS for data fusion.  <br/>  ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                            |
| `led_config_file`            | Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations. <br/>  ***string:*** [`led_config.yaml`](../panther_lights/config/led_config.yaml)                                                                                                                        |
| `lights_bt_project_path`     | Path to BehaviorTree project file, responsible for lights management. <br/>  ***string:*** [`PantherLightsBT.btproj`](../panther_manager/behavior_trees/PantherLightsBT.btproj)                                                                                                                                                                                |
| `localization_config_path`   | Specify the path to the localization configuration file. <br/>  ***string:*** [`PantherLocalization.yaml`](../panther_localization/config/relative_localization.yaml)                                                                                                                                                                                          |
| `localization_mode`          | Specifies the localization mode:  <br/>- 'relative' `odometry/filtered` data is relative to the initial position and orientation.  <br/>- 'enu' `odometry/filtered` data is relative to initial position and ENU (East North Up) orientation.  <br/>  ***string:*** `relative` (choices: `relative`, `enu`)                                                    |
| `namespace`                  | Add namespace to all launched nodes. <br/>  ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                                                                                                                               |
| `publish_robot_state`        | Whether to publish the default Panther robot description.  <br/>  ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                                                                                                                                |
| `safety_bt_project_path`     | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/>  ***string:*** [`PantherSafetyBT.btproj`](../panther_manager/behavior_trees/PantherSafetyBT.btproj)                                                                                                                                                                   |
| `shutdown_hosts_config_path` | Path to file with list of hosts to request shutdown. <br/>  ***string:*** [`shutdown_hosts.yaml`](../panther_manager/config/shutdown_hosts.yaml)                                                                                                                                                                                                               |
| `use_ekf`                    | Enable or disable EKF.  <br/>  ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                                                                                                                                                                   |
| `use_sim`                    | Whether simulation is used.  <br/>  ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                             |
| `user_led_animations_file`   | Path to a YAML file with a description of the user defined animations. <br/>  ***string:*** `None`                                                                                                                                                                                                                                                             |
| `wheel_config_path`          | Path to wheel configuration file. By default, it is located in 'panther_description/config/<wheel_type arg>.yaml'. You can also specify the path to your custom wheel configuration file here. <br/>  ***string:*** [`<wheel_type arg>.yaml`](../panther_description/config)                                                                                   |
| `wheel_type`                 | Type of wheel. If you choose a value from the preset options ('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path' parameters. For custom wheels, please define these parameters to point to files that accurately describe the custom wheels. <br/>  ***string:*** `WH01` (choices: `WH01`, `WH02`, `WH04`, `custom`) |

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
