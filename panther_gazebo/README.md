# panther_gazebo

A package containing the launch files and dependencies needed to run the simulation in Ignition Fortress.

## Usage

The recommended method for launching the simulation is by utilizing the [simulation.launch.py](./launch/simulation.launch.py) file. Below, you will find launch arguments that enable simulation configuration.

### Launch Arguments

- `battery_config_path` [*string*, default: **<panther_gazebo share directory>/config/battery_plugin_config.yaml**]: Path to the Ignition `LinearBatteryPlugin` configuration file. This configuration is intended for use in simulations only. For more information on how to configure this plugin, please refer to this [tab](#linear-battery-plugin).
- `controller_config_path` [*string*, default: **<panther_controller share directory>/config/<wheel_type arg>_controller.yaml**]: Path to the controller configuration file. If you want to use a custom configuration, you can specify the path to your custom controller configuration file here.
- `gz_bridge_config_path` [*string*, default: **<panther_gazebo share directory>/config/gz_bridge.yaml**]: Path to the `parameter_bridge` configuration file. For detailed information on configuring the `parameter_bridge`, please refer to this [example](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge#example-5-configuring-the-bridge-via-yaml).
- `pos_x` [*float*, default: **5.0**]: spawn position **[m]** of the robot in the world in **X** direction.
- `pos_y` [*float*, default: **-5.0**]: spawn position **[m]** of the robot in the world in **Y** direction.
- `pos_z` [*float*, default: **0.2**]: spawn position **[m]** of the robot in the world in **Z** direction.
- `rot_yaw` [*float*, default: **0.0**]: spawn yaw angle **[rad]** of the robot in the world.
- `publish_robot_state` [*bool*, default: **true**]: Whether to launch the robot_state_publisher node. When set to `false`, users should publish their own robot description.
- `wheel_config_path` [*string*, default: **<panther_description share directory>/config/<wheel_type arg>.yaml**]: Path to the wheel configuration file. If you want to use a custom configuration, you can specify the path to your custom wheel configuration file here. Please refer to the `wheel_type` parameter description for more information.
- `wheel_type` [*string*, default: **WH01**]: Specify the type of wheel. If you select a value from the provided options (`WH01`, `WH02`, `WH04`), you can disregard the `wheel_config_path` and `controller_config_path` parameters. If you have custom wheels, set this parameter to `CUSTOM` and provide the necessary configurations.
- `world` [*string*, default: **-r <husarion_office_gz share directory>/worlds/husarion_world.sdf**]: path to Gazebo world file used for simulation.

### Changing Wheel Type

It is possible to change Panther wheels model in simulation. All you need to do is to point to new wheel and controller configuration files using `wheel_config_path` and `controller_config_path` parameters. These files should be based on the default ones, i.e., [WH01_controller.yaml](../panther_controller/config/WH01_controller.yaml) and [WH01.yaml](../panther_description/config/WH01.yaml).

### Linear Battery Plugin

It is possible to simulate the battery operation of the Panther robot. By default, this feature is disabled, but you can enable it by setting the `simulate_discharging` parameter to `true` in the `battery_plugin_config.yaml` file or in the file pointed to by the `battery_config_path` parameter. Below, you will find the plugin parameters that enable battery simulation.

- `simulate_discharging` [*bool*, default: **false**]: Enables battery simulation. If set to `true`, the battery will discharge **at a constant rate** (regardless of joint torque), and if it depletes completely, the robot will stop moving. When set to `false`, the battery will not discharge, but the battery status information will still be published on the `/battery` topic.
- `initial_charge_percentage` [*float*, default: **70.0**]: Sets the initial charge percentage of the battery.
- `charging_time` [*float*, default: **6.0**]: Specifies the charging time for the battery in hours.
- `power_load` [*float*, default: **120.0**]: Represents the average power load during normal operation **[W]** and is initially set to 120.0 W. With the default power load of 120.0 W, the robot can operate for up to 8 hours. When the `simulate_discharging` parameter is set to `false`, this value defaults to 0.0 W. Users are encouraged to customize this value to match their specific requirements. For more information on Panther power consumption, please refer to [Panther Battery & Charging Documentation](https://husarion.com/manuals/panther/#battery--charging).

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

The NavSat sensors depend on the spherical coordinates of the world origin being configured. This configuration can be accomplished, for instance, by employing the `<spherical_coordinates>` tag within the world's SDF or by utilizing the Ignition `/world/world_name/set_spherical_coordinates` service.

To obtain GPS data in Ignition, all you need to do is include the [external_antenna](../panther_description/urdf/components/external_antenna.urdf.xacro) macro to your robot model and add the following tag to your world's SDF file and point this file with `world` param:

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
