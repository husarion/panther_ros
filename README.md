# panther_ros

ROS 2 packages for Panther autonomous mobile robot

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

## Quick start

### Create workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone -b ros2 https://github.com/husarion/panther_ros.git src/panther_ros
```

### Configure environment

The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

Real robot:

``` bash
export HUSARION_ROS_BUILD_TYPE=hardware
```

Simulation:

```bash
export HUSARION_ROS_BUILD_TYPE=simulation
```

### Build

``` bash
vcs import src < src/panther_ros/panther/panther_$HUSARION_ROS_BUILD_TYPE.repos

cp -r src/ros2_controllers/diff_drive_controller src
cp -r src/ros2_controllers/imu_sensor_broadcaster src
rm -rf src/ros2_controllers

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

>[!NOTE]
> To build code on a real robot you need to run above commands on the Panther Built-in Computer.

### Running

Real robot:

```bash
ros2 launch panther_bringup bringup.launch.py
```

Simulation:

```bash
ros2 launch panther_gazebo simulation.launch.py
```

### Launch Arguments

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulated robot |

    'battery_config_path':
        Path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only.
        (default: LocalVar('FindPackageShare(pkg='panther_gazebo') + 'config' + 'battery_plugin_config.yaml''))

    'components_config_path':
        Additional components configuration file. Components described in this file are dynamically included in Panther's urdf.Panther options are described here https://husarion.com/manuals/panther/panther-options/
        (default: LocalVar('FindPackageShare(pkg='panther_description') + 'config' + 'components.yaml''))

    'gz_bridge_config_path':
        Path to the parameter_bridge configuration file.
        (default: LocalVar('FindPackageShare(pkg='panther_gazebo') + 'config' + 'gz_bridge.yaml''))

    'namespace':
        Add namespace to all launched nodes.
        (default: EnvVar('ROBOT_NAMESPACE'))

    'use_ekf':
        Enable or disable EKF. Valid choices are: ['True', 'False']
        (default: 'True')

    'wheel_type':
        Type of wheel. If you choose a value from the preset options ('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path' parameters. For custom wheels, please define these parameters to point to files that accurately describe the custom wheels. Valid choices are: ['WH01', 'WH02', 'WH04', 'custom']
        (default: 'WH01')

    'controller_config_path':
        Path to controller configuration file. By default, it is located in 'panther_controller/config/{wheel_type}_controller.yaml'. You can also specify the path to your custom controller configuration file here.
        (default: LocalVar('FindPackageShare(pkg='panther_controller') + 'config' + PythonExpr(''' + LaunchConfig('wheel_type') + '_controller.yaml'')'))

    'use_sim':
        Whether simulation is used. Valid choices are: ['True', 'False']
        (default: 'False')

    'wheel_config_path':
        Path to wheel configuration file. By default, it is located in 'panther_description/config/{wheel_type}.yaml'. You can also specify the path to your custom wheel configuration file here.
        (default: LocalVar('FindPackageShare(pkg='panther_description') + 'config' + PythonExpr(''' + LaunchConfig('wheel_type') + '.yaml'')'))

    'led_config_file':
        Path to a YAML file with a description of led configuration.
        (default: LocalVar('FindPackageShare(pkg='panther_lights') + 'config' + 'led_config.yaml''))

    'user_led_animations_file':
        Path to a YAML file with a description of the user defined animations.
        (default: '')

    'lights_bt_project_path':
        Path to BehaviorTree project file, responsible for lights management.
        (default: LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherLightsBT.btproj''))

    'safety_bt_project_path':*
        Path to BehaviorTree project file, responsible for safety and shutdown management.
        (default: LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherSafetyBT.btproj''))

    'shutdown_hosts_config_path':
        Path to file with list of hosts to request shutdown.
        (default: LocalVar('FindPackageShare(pkg='panther_manager') + 'config' + 'shutdown_hosts.yaml''))

    'publish_robot_state':
        Whether to launch the robot_state_publisher node.When set to False, users should publish their own robot description. Valid choices are: ['True', 'False']
        (default: 'True')

    'fuse_gps':
        Include GPS for data fusion. Valid choices are: ['False', 'True']
        (default: 'False')

    'localization_mode':
        Specifies the localization mode:
        - 'relative' odometry/filtered data is relative to the initial position and orientation.
        - 'enu' odometry/filtered data is relative to initial position and ENU (East North Up) orientation. Valid choices are: ['relative', 'enu']
        (default: 'relative')

    'localization_config_path':
        Specify the path to the localization configuration file.
        (default: LocalVar('FindPackageShare(pkg='panther_localization') + 'config' + PythonExpr(''' + PythonExpr(''' + LaunchConfig('localization_mode') + '_'') + 'localization' + PythonExpr(''_with_gps' if ' + LaunchConfig('fuse_gps') + ' else ''') + '.yaml'')'))

|     | Argument                     | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                                                                        |
| --- | ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 🖥️   | `add_wheel_joints`           | Flag enabling joint_state_publisher to publish information about the wheel position. Should be false when there is a controller that sends this information. <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                              |
| 🖥️   | `add_world_transform`        | Adds a world frame that connects the tf trees of individual robots (useful when running multiple robots). <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                              |
| 🖥️   | `battery_config_path`        | Path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only. <br/> ***string:*** `None`                                                                                                                                                                                                           |
| 🤖🖥️  | `components_config_path`     | Additional components configuration file. Components described in this file are dynamically included in Panther's urdf. Panther options are described in [the manual](https://husarion.com/manuals/panther/panther-options). <br/> ***string:*** [`components.yaml`](../panther_description/config/components.yaml)                                          |
| 🤖🖥️  | `controller_config_path`     | Path to controller configuration file. A path to custom configuration can be specified here. <br/> ***string:*** [`{wheel_type}_controller.yaml`](../panther_controller/config/)                                                                                                                                                                              |
| 🤖   | `disable_manager`            | Enable or disable manager_bt_node. <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                      |
| 🤖🖥️  | `fuse_gps`                   | Include GPS for data fusion. <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                            |
| 🖥️   | `gz_bridge_config_path`      | Path to the parameter_bridge configuration file. <br/> ***string:*** [`gz_bridge.yaml`](./panther_gazebo/config/gz_bridge.yaml)                                                                                                                                                                                                                                 |
| 🖥️   | `gz_gui`                     | Run simulation with specific GUI layout. <br/> ***string:*** [`teleop.config`](https://github.com/husarion/husarion_gz_worlds/blob/main/config/teleop.config)                                                                                                                                                                                                                |
| 🖥️   | `gz_headless_mode`           | Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the amount of calculations. <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                             |
| 🖥️   | `gz_log_level`               | Adjust the level of console output. <br/> ***int:*** `1` (choices: `0`, `1`, `2`, `3`, `4`)                                                                                                                                                                                                                                                                               |
| 🖥️   | `gz_world`                   | Absolute path to SDF world file. <br/> ***string:*** [`husarion_world.sdf`](https://github.com/husarion/husarion_gz_worlds/blob/main/worlds/husarion_world.sdf)                                                                                                                                                                                                 |
| 🤖🖥️  | `led_config_file`            | Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations. <br/> ***string:*** [`led_config.yaml`](../panther_lights/config/led_config.yaml)                                                                                                                        |
| 🤖🖥️  | `lights_bt_project_path`     | Path to BehaviorTree project file, responsible for lights management. <br/> ***string:*** [`PantherLightsBT.btproj`](../panther_manager/behavior_trees/PantherLightsBT.btproj)                                                                                                                                                                                |
| 🤖🖥️  | `localization_config_path`   | Specify the path to the localization configuration file. <br/> ***string:*** [`PantherLocalization.yaml`](../panther_localization/config/relative_localization.yaml)                                                                                                                                                                                          |
| 🤖🖥️  | `localization_mode`          | Specifies the localization mode:  <br/>- 'relative' `odometry/filtered` data is relative to the initial position and orientation. <br/>- 'enu' `odometry/filtered` data is relative to initial position and ENU (East North Up) orientation. <br/> ***string:*** `relative` (choices: `relative`, `enu`)                                                    |
| 🤖🖥️  | `namespace`                  | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                                                                                                                               |
| 🤖🖥️  | `publish_robot_state`        | Whether to publish the default Panther robot description. <br/> ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                                                                                                                                |
| 🖥️   | `robots`                     | The list of the robots spawned in the simulation e. g. `robots:='robot1={x: 1.0, y: -2.0}; robot2={x: 1.0, y: -4.0}'` <br/> ***string:*** `''`                                                                                                                                                                                                                  |
| 🤖🖥️  | `safety_bt_project_path`     | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/> ***string:*** [`PantherSafetyBT.btproj`](../panther_manager/behavior_trees/PantherSafetyBT.btproj)                                                                                                                                                                   |
| 🤖🖥️  | `shutdown_hosts_config_path` | Path to file with list of hosts to request shutdown. <br/> ***string:*** [`shutdown_hosts.yaml`](../panther_manager/config/shutdown_hosts.yaml)                                                                                                                                                                                                               |
| 🤖🖥️  | `use_ekf`                    | Enable or disable EKF. <br/> ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                                                                                                                                                                   |
| 🤖🖥️  | `use_sim`                    | Whether simulation is used. <br/> ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                             |
| 🤖🖥️  | `user_led_animations_file`   | Path to a YAML file with a description of the user defined animations. <br/> ***string:*** `None`                                                                                                                                                                                                                                                             |
| 🤖🖥️  | `wheel_config_path`          | Path to wheel configuration file. <br/> ***string:*** [`{wheel_type}.yaml`](../panther_description/config)                                                                                                                                                                                                                                                   |
| 🤖🖥️  | `wheel_type`                 | Type of wheel. If you choose a value from the preset options ('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path' parameters. For custom wheels, please define these parameters to point to files that accurately describe the custom wheels. <br/> ***string:*** `WH01` (choices: `WH01`, `WH02`, `WH04`, `custom`) |
| 🖥️   | `x`                          | Initial robot position in the global 'x' axis. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                                                          |
| 🖥️   | `y`                          | Initial robot position in the global 'y' axis. <br/> ***float:***` -2.0`                                                                                                                                                                                                                                                                                         |
| 🖥️   | `z`                          | Initial robot position in the global 'z' axis. <br/> ***float:*** `0.2`                                                                                                                                                                                                                                                                                          |
| 🖥️   | `roll`                       | Initial robot 'roll' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                                                                       |
| 🖥️   | `pitch`                      | Initial robot 'pitch' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                                                                      |
| 🖥️   | `yaw`                        | Initial robot 'yaw' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                                                                        |

> [!TIP]
>
> Others packages also contains

## Developer Info

### Setup pre-commit

This project uses pre-commit to maintain high quality of the source code. Install precommit after downloading the repository to apply the changes.

```bash
pre-commit install
```
