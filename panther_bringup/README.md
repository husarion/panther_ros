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

### bringup.launch.py - Executables

| Executable                | *Type*                                             |
| ------------------------- | -------------------------------------------------- |
| `battery.launch.py`       | [*panther_batter/battery.launch.py*](.)            |
| `controller.launch.py`    | [*panther_controller/controller.launch.py*](.)     |
| `battery.launch.py`       | [*panther_batter/battery.launch.py*](.)            |
| `lights.launch.py`        | [*panther_lights/lights.launch.py*](.)             |
| `manager_bt.launch.py`    | [*panther_manager/manager_bt.launch.py*](.)        |
| `system_status.launch.py` | [*panther_diagnostics/system_status.launch.py*](.) |

## Running nodes

This package only runs the launch files. If you want to learn how to configure individual nodes, check the appropriate package.
