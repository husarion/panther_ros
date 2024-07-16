# panther_gazebo

The package contains a launch file and source files used to run the robot simulation in Gazebo. The simulator tries to reproduce the behavior of a real robot as much as possible, including the provision of an analogous ROS_API.

## Launch Files

- `spawn_robot.launch.py` - is responsible for spawning the robot in the simulator
- `simulate_robot.launch.py` - is responsible for giving birth to the robot and simulating its physical behavior, such as driving, displaying data, etc.
- `simulate_multiple_robots.launch.py` - similar to the above with logic allowing you to quickly add a swarm of robots
- **`simulation.launch.py`** - a target file that runs the gazebo simulator and adds and simulates the robot's behavior in accordance with the given arguments.

### spawn_robot - Launch Arguments

| Argument                    | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                                                                                                                                                                                           |
|-----------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `fuse_gps`                  | Include GPS for data fusion. Valid choices are: ['False', 'True'] <br/> ***bool:*** `'False'`                                                                                                                                                                                                                                                                                                                                                                                      |
| `gz_bridge_config_path`     | Path to the parameter_bridge configuration file. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_gazebo') + 'config' + 'gz_bridge.yaml''`)                                                                                                                                                                                                                                                                                                                           |
| `led_config_file`           | Path to a YAML file with a description of led configuration. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_lights') + 'config' + 'led_config.yaml''`)                                                                                                                                                                                                                                                                                                               |
| `lights_bt_project_path`    | Path to BehaviorTree project file, responsible for lights management. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherLightsBT.btproj''`)                                                                                                                                                                                                                                                                                      |
| `localization_config_path`  | Specify the path to the localization configuration file. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_localization') + 'config' + PythonExpr(''' + PythonExpr(''' + LaunchConfig('localization_mode') + '_'') + 'localization' + PythonExpr(''_with_gps' if ' + LaunchConfig('fuse_gps') + ' else ''') + '.yaml'')`)                                                                                                                                                |
| `localization_mode`         | Specifies the localization mode: <br/> - 'relative' odometry/filtered data is relative to the initial position and orientation. <br/> - 'enu' odometry/filtered data is relative to initial position and ENU (East North Up) orientation. Valid choices are: ['relative', 'enu'] <br/> ***string:*** `'relative'`                                                                                                                                                                |
| `publish_robot_state`       | Whether to launch the robot_state_publisher node. When set to False, users should publish their own robot description. Valid choices are: ['True', 'False'] <br/> ***bool:*** `'True'`                                                                                                                                                                                                                                                                                           |
| `safety_bt_project_path`    | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherSafetyBT.btproj''`)                                                                                                                                                                                                                                                                        |
| `shutdown_hosts_config_path`| Path to file with list of hosts to request shutdown. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'config' + 'shutdown_hosts.yaml''`)                                                                                                                                                                                                                                                                                                                  |
| `use_ekf`                   | Enable or disable EKF. Valid choices are: ['True', 'False'] <br/> ***bool:*** `'True'`                                                                                                                                                                                                                                                                                                                                                                                            |
| `user_led_animations_file`  | Path to a YAML file with a description of the user defined animations. <br/> ***string:*** `''`                                                                                                                                                                                                                                                                                                                                                                                   |
| All `spawn_robot.launch.py` arguments              | All the previously mentioned arguments are also available for this launch file.                                                                                                                                                                                                                                                                                                                                                                                                   |

## Configuration

### Linear Battery Plugin

It is possible to simulate the battery operation of the Panther robot. By default, this feature is disabled, but you can enable it by setting the `simulate_discharging` parameter to `true` in the `battery_plugin_config.yaml` file or in the file pointed to by the `battery_config_path` parameter. Below, you will find the plugin parameters that enable battery simulation.

- `simulate_discharging` [*bool*, default: **false**]: Enables battery simulation. If set to `true`, the battery will discharge **at a constant rate** (regardless of joint torque), and if it depletes completely, the robot will stop moving. When set to `false`, the battery will not discharge, but the battery status information will still be published on the `battery/battery_status` topic.
- `initial_charge_percentage` [*float*, default: **70.0**]: Sets the initial charge percentage of the battery.
- `charging_time` [*float*, default: **6.0**]: Specifies the charging time for the battery in hours.
- `power_load` [*float*, default: **120.0**]: Represents the average power load during normal operation **[W]** and is initially set to 120.0 W. With the default power load of 120.0 W, the robot can operate for up to 8 hours. When the `simulate_discharging` parameter is set to `false`, this value defaults to 0.0 W. Users are encouraged to customize this value to match their specific requirements. For more information on Panther power consumption, please refer to [Panther Battery & Charging Documentation](https://husarion.com/manuals/panther/#battery--charging).

> **Note**
>
> The `percentage` field has a range of 0.0-100.0. This differs from the real functioning of Panther, where, in accordance with the BatteryState message definition, the value is within the range of 0.0-1.0.

#### Charging Process

Unfortunately, there is no straightforward way to exchange `LinearBatteryPlugin` services between ROS and Gazebo Transport, so you need to use the `ign` commands. As a result, the method of charging differs between the real and simulated robot.

You can start the charging process by calling the Ignition service:

```bash
ign service --service /model/panther/battery/panther_battery/recharge/start --reqtype ignition.msgs.Boolean --reptype ignition.msgs.Empty --req '' --timeout 0
```

and stop it using:

```bash
ign service --service /model/panther/battery/panther_battery/recharge/stop --reqtype ignition.msgs.Boolean --reptype ignition.msgs.Empty --req '' --timeout 0
```

### Use of GPS in Simulation

The NavSat system is utilized to publish the Panther robot's position within the Gazebo world. It manages navigation satellite sensors, such as GPS, which report position and velocity in spherical coordinates (latitude/longitude) through Ignition Transport.

The NavSat sensors requires the spherical coordinates of the world origin to be configured. This configuration can be accomplished, for instance, by employing the `<spherical_coordinates>` tag within the world's SDF or by utilizing the Ignition `/world/world_name/set_spherical_coordinates` service.

To obtain GPS data in Ignition, follow these steps:

- Include the [ANT02](https://github.com/husarion/ros_components_description/blob/ros2/urdf/external_antenna.urdf.xacro) by adding the following lines to your [components.yaml](https://github.com/husarion/panther_ros/blob/ros2/panther_description/config/components.yaml) file inside the `components` list:

```yaml
  - type: ANT02
    parent_link: cover_link
    xyz: 0.185 -0.12 0.0
    rpy: 0.0 0.0 3.14
    device_namespace: gps
```

- Add the following tag to your world's SDF file and specify this file using the `world` parameter (the default `husarion_world.sdf` file already includes this tag):

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>53.1978</latitude_deg>
  <longitude_deg>18.3732</longitude_deg>
  <elevation>0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```
