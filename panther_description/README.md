# panther_description

The package contains URDF files responsible for creating a representation of the robot by specifying the relationships and types of connections (joints) between individual links. It also contains information about the robot's mesh.

## Launch Files

- `load_urdf.launch.py` - loads the robot's URDF and creates simple bindings to display moving joints.

### load_urdf.launch.py - Arguments

| Argument                 | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                                                                        |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `add_wheel_joints`       | Flag enabling joint_state_publisher to publish information about the wheel position. Should be false when there is a controller that sends this information. <br/> ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                               |
| `battery_config_path`    | Path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only. <br/>  ***string:*** `''`                                                                                                                                                                                                             |
| `components_config_path` | Additional components configuration file. Components described in this file are dynamically included in Panther's urdf. Panther options are described in [the manual](https://husarion.com/manuals/panther/panther-options).  <br/>  ***string:*** [`components.yaml`](../panther_description/config/components.yaml)                                          |
| `controller_config_path` | Path to controller configuration file. A path to custom configuration can be specified here. <br/>  ***string:*** [`<wheel_type arg>_controller.yaml`](../panther_controller/config/)                                                                                                                                                                          |
| `namespace`              | Add namespace to all launched nodes. <br/>  ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                                                                                                                               |
| `use_sim`                | Whether simulation is used.  <br/>  ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                             |
| `wheel_config_path`      | Path to wheel configuration file.   <br/>  ***string:*** [`<wheel_type arg>.yaml`](../panther_description/config)                                                                                   |
| `wheel_type`             | Type of wheel. If you choose a value from the preset options ('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path' parameters. For custom wheels, please define these parameters to point to files that accurately describe the custom wheels. <br/>  ***string:*** `WH01` (choices: `WH01`, `WH02`, `WH04`, `custom`) |

### load_urdf.launch.py - Nodes

| Node name               | *Type*                                                                                        |
| ----------------------- | --------------------------------------------------------------------------------------------- |
| `joint_state_publisher` | *[joint_state_publisher/joint_state_publisher](https://github.com/ros/joint_state_publisher)* |
| `robot_state_publisher` | *[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)* |

## ROS Nodes

This package only runs external nodes. If you want to learn how to configure individual nodes, please check the appropriate package.
