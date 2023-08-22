# panther_manager

A package containing nodes responsible for high-level control of Husarion Panther robot.

## ROS Nodes

### manager_bt_node

Node responsible for managing the Husarion Panther robot. Composes control of three behavior trees responsible for handling Bumper Lights animation scheduling, safety features, and software shutdown of components.

#### Subscribers

- `/panther/battery` [*sensor_msgs/BatteryState*]: state of the internal Battery.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: state of motor controllers.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: state of emergency stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*]: state of IO pins.
- `/panther/system_status` [*panther_msgs/SystemStatus*]: state of the system, including Built-in Computer's CPU temperature and load.

#### Service Clients (for Default Trees)

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enables Aux Power output.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: triggers E-stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enables fan.
- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `~battery_percent_window_len` [*int*, default: **6**]: moving average window length used to smooth out Battery percentage readings.
- `~battery_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of the Battery.
- `~bt_project_file` [*string*, default: **$(find panther_manager)/config/PantherBT.btproj**]: path to a BehaviorTree project.
- `~cpu_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of the Built-in Computer's CPU.
- `~driver_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out the temperature readings of each driver.
- `~launch_lights_tree` [*bool*, default: **true**]: launch behavior tree responsible for scheduling animations on Panther Bumper Lights.
- `~launch_safety_tree` [*bool*, default: **true**]: launch behavior tree responsible for managing Panther safety measures.
- `~launch_shutdown_tree` [*bool*, default: **true**]: launch behavior tree responsible for the gentle shutdown of robot components.
- `~lights/battery_state_anim_period` [*float*, default: **120.0**]: time in **[s]** to wait before repeating animation representing the current Battery percentage.
- `~lights/critical_battery_anim_period` [*float*, default: **15.0**]: time in **[s]** to wait before repeating animation, indicating a critical Battery state.
- `~lights/critical_battery_threshold_percent` [*float*, default: **0.1**]: if the Battery percentage drops below this value, an animation indicating a critical Battery state will start being displayed.
- `~lights/low_battery_anim_period` [*float*, default: **30.0**]: time in **[s]** to wait before repeating the animation, indicating a low Battery state.
- `~lights/low_battery_threshold_percent` [*float*, default: **0.4**]: if the Battery percentage drops below this value, the animation indicating a low Battery state will start being displayed.
- `~lights/update_charging_anim_step` [*float*, default: **0.1**]: percentage representing how discretized the Battery state animation should be.
- `~plugin_libs` [*list*, default: **Empty list**]: list with names of plugins that are used in the BT project.
- `~ros_plugin_libs` [*list*, default: **Empty list**]: list with names of ROS plugins that are used in a BT project. 
- `~safety/cpu_fan_off_temp` [*float*, default: **60.0**]: temperature in **[&deg;C]** of the Built-in Computer's CPU, below which the fan is turned off.
- `~safety/cpu_fan_on_temp` [*float*, default: **70.0**]: temperature in **[&deg;C]** of the Built-in Computer's CPU, above which the fan is turned on.
- `~safety/critical_bat_temp` [*float*, default: **59.0**]: extends `safety/high_bat_temp` by turning off Aux Power.
- `~safety/driver_fan_off_temp` [*float*, default: **35.0**]: temperature in **[&deg;C]** of any drivers below which the fan is turned off.
- `~safety/driver_fan_on_temp` [*float*, default: **45.0**]: temperature in **[&deg;C]** of any drivers above which the fan is turned on.
- `~safety/high_bat_temp` [*float*, default: **55.0**]: the Battery temperature in **[&deg;C]** above which the robot starts displaying a warning log and E-stop is triggered.
- `~shutdown_hosts_file` [*string*, default: **None**]: path to a YAML file containing a list of hosts to request shutdown. To correctly format the YAML file, include a **hosts** field consisting of a list with the following fields:
  - `command` [*string*, default: **sudo shutdown now**]: command executed on shutdown of given device.
  - `ip` [*string*, default: **None**]: IP of a host to shutdown over SSH.
  - `ping_for_success` [*bool*, default: **true**]: ping host until it is not available or timeout is reached.
  - `port` [*string*, default: **22**]: SSH communication port.
  - `timeout` [*string*, default: **5.0**]: time in **[s]** to wait for the host to shutdown. The built-in computer will turn off after all computers are shutdown or reached timeout. Keep in mind that hardware will cut power off after a given time after pressing the power button. Refer to the hardware manual for more information. 
  - `username` [*string*, default: **None**]: username used to log in to over SSH.

#### Shutdown Behavior

For more information regarding shutdown behavior, refer to `ShutdownSingleHost` BT node in the [Actions](#actions) section. An example of a shutdown hosts YAML file can be found below.
``` yaml
# My shutdown_hosts.yaml
hosts:
  # Intel NUC, user computer
  - ip: 10.15.20.3
    username: husarion
  # Universal robots UR5
  - ip: 10.15.20.4
    username: root
  # My Raspberry pi that requires very long shutdown sequence
  - ip: 10.15.20.12
    timeout: 40
    username: pi
    command: /home/pi/my_long_shutdown_sequence.sh
```
To set up a connection with a new User Computer and allow execution of commands, login to the Built-in Computer with `ssh husarion@10.15.20.2`.
Add Built-in Computer's public key to **known_hosts** of a computer you want to shutdown automatically:
``` bash
ssh-copy-id username@10.15.20.XX
```

> **Warning**
> To allow your computer to be shutdown without the sudo password, ssh into it and execute
> (if needed replace **husarion** with username of your choice):
> ``` bash
> sudo su
> echo husarion 'ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown' | EDITOR='tee -a' visudo
> ```

#### Faults Handle

After receiving a message on the `/panther/battery` topic, the `panther_manager` node makes decisions regarding safety measures. For more information regarding the power supply state, please refer to the [adc_node](/panther_battery/README.md#battery-statuses) documentation.

| Power Supply Health | Procedure                                                                                                                                                                                                          |
| ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| GOOD                | -                                                                                                                                                                                                                  |
| UNKNOWN             | -                                                                                                                                                                                                                  |
| OVERHEAT            | 1. Turn on the fan. <br> 2. If the Battery temperature is higher than 55.0 **[&deg;C]**, trigger an emergency stop and turn off AUX. <br> 3. If the Battery temperature is higher than 62.0 **[&deg;C]**, shutdown the robot. |
| DEAD                | Shutdown the robot.                                                                                                                                                                                               |
| OVERVOLTAGE         | 1. Initiate an emergency stop. <br> 2. Display an error animation if the charger is connected.                                                                                                                     |
| COLD                | -                                                                                                                                                                                                                  |

> **NOTE**
> 1. The fan exhibits a form of hysteresis, allowing it to be turned off after a delay of at least 60 seconds.
> 2. Once the Panther ROS stack initializes, the fan activates and operates for a duration of approximately 60 seconds.


### system_status_node.py

Publishes stats and status of the Built-in Computer. Stats include CPU utilization and temperature, as well as disk and RAM usage.

#### Publishers

- `/panther/system_status` [*panther_msgs/SystemStatus*]: information about Built-in Computer CPU temperature, utilization, disk, and RAM usage.

---

## BehaviorTree

For a BehaviorTree project to work correctly, it must contain three trees with names as described below. However, if any of the parameters (`~launch_lights_tree`, `~launch_safety_tree`, `~launch_shutdown_tree`) is set to false, the corresponding tree is disabled and is no longer required in the project. Files with trees XML descriptions can be shared between projects. Each tree is provided with a set of default blackboard entries (described below), which can be used to specify the behavior of a given tree.

### Nodes

#### Actions

- `CallSetBoolService` - allows calling the standard **std_srvs/SetBool** ROS service. Provided ports are:
  - `data` [*input*, *bool*, default: **None**]: service data - **true** or **false** value.
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
  - `timeout` [*input*, *unsigned*, default: **100**]: time in **[s]** to wait for service to become available.
- `CallSetLedAnimationService` - allows calling custom type **panther_msgs/SetLEDAnimation** ROS service. The provided ports are:
  - `id` [*input*, *unsigned*, default: **None**]: animation ID.
  - `param` [*input*, *string*, default: **None**]: optional parameter passed to animation.
  - `repeating` [*input*, *bool*, default: **false**]: indicates if the animation should repeat.
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
  - `timeout` [*input*, *unsigned*, default: **100**]: time in **[s]** to wait for service to become available.
- `CallTriggerService` - allows calling the standard **std_srvs/Trigger** ROS service. The provided ports are:
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
  - `timeout` [*input*, *unsigned*, default: **100**]: time in **[s]** to wait for service to become available.
- `ShutdownHostsFromFile` - allows to shutdown devices based on a YAML file. Returns `SUCCESS` only when a YAML file is valid and the shutdown of all defined hosts was successful. Nodes are processed in a semi-parallel fashion. Every tick of the tree updates the state of a host. This allows some hosts to wait for a SSH response, while others are already pinged and awaiting a full shutdown. If a host is shutdown it is no longer processed. In the case of a long timeout is used for a given host, other hosts will be processed simultaneously. The provided ports are:
  - `shutdown_host_file` [*input*, *string*, default: **None**]: global path to YAML file with hosts to shutdown.
- `ShutdownSingleHost` - allows to shutdown a single device. Will return `SUCCESS` only when the device has been successfully shutdown. The provided ports are:
  - `command` [*input*, *string*, default: **sudo shutdown now**]: command to execute on shutdown.
  - `ip` [*input*, *string*, default: **None**]: IP of the host to shutdown.
  - `ping_for_success` [*input*, *bool*, default: **true**]: ping host until it is not available or timeout is reached.
  - `port` [*input*, *string*, default: **22**]: SSH communication port.
  - `timeout` [*input*, *string*, default: **5.0**]: time in **[s]** to wait for the host to shutdown. Keep in mind that hardware will cut power off after a given time after pressing the power button. Refer to the hardware manual for more information. 
  - `user` [*input*, *string*, default: **None**]: user to log into while executing the shutdown command.
- `SignalShutdown` - signals shutdown of the robot. The provided ports are:
  - `message` [*input*, *string*, default: **None**]: message with reason for robot to shutdown.

#### Decorators

- `TickAfterTimeout` - will skip a child until the specified time has passed. It can be used to specify the frequency at which a node or subtree is triggered. The provided ports are:
  - `timeout` [*input*, *unsigned*, default: **None**]: time in **[s]** to wait before ticking child again.

### Trees

#### Lights

A tree responsible for scheduling animations displayed on the Bumper Lights based on the Husarion Panther robot's system state.

<p align="center">
  <img align="center" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/lights_tree.svg">
</p>

Default blackboard entries:
- `battery_percent` [*float*, default: **None**]: moving average of the Battery percentage.
- `battery_percent_round` [*string*, default: **None**] Battery percentage rounded to a value specified with `~lights/update_charging_anim_step` parameter and cast to string.
- `battery_status` [*unsigned*, default: **None**]: the current Battery status.
- `charging_anim_percent` [*string*, default: **None**]: the charging animation Battery percentage value, cast to a string.
- `current_anim_id` [*int*, default: **-1**]: ID of currently displayed animation.
- `e_stop_state` [*bool*, default: **None**]: state of E-stop.

Default constant blackboard entries:
- `BATTERY_STATE_ANIM_PERIOD` [*float*, default: **120.0**]: refers to `battery_state_anim_period` ROS parameter.
- `CRITICAL_BATTERY_ANIM_PERIOD` [*float*, default: **15.0**]: refers to `critical_battery_anim_period` ROS parameter.
- `CRITICAL_BATTERY_THRESHOLD_PERCENT` [*float*, default: **0.1**]: refers to `critical_battery_threshold_percent` ROS parameter.
- `LOW_BATTERY_ANIM_PERIOD` [*float*, default: **30.0**]: refers to `low_battery_anim_period` ROS parameter.
- `LOW_BATTERY_THRESHOLD_PERCENT` [*float*, default: **0.4**]: refers to `low_battery_threshold_percent` ROS parameter.
- `E_STOP_ANIM_ID` [*unsigned*, value: **0**]: animation ID constant obtained from `panther_msgs::LEDAnimation::E_STOP`.
- `READY_ANIM_ID` [*unsigned*, value: **1**]: animation ID constant obtained from `panther_msgs::LEDAnimation::READY`.
- `ERROR_ANIM_ID` [*unsigned*, value: **2**]: animation ID constant obtained from `panther_msgs::LEDAnimation::ERROR`.
- `MANUAL_ACTION_ANIM_ID` [*unsigned*, value: **3**]: animation ID constant obtained from `panther_msgs::LEDAnimation::MANUAL_ACTION`.
- `AUTONOMOUS_ACTION_ANIM_ID` [*unsigned*, value: **4**]: animation ID constant obtained from `panther_msgs::LEDAnimation::AUTONOMOUS_ACTION`.
- `GOAL_ACHIEVED_ANIM_ID` [*unsigned*, value: **5**]: animation ID constant obtained from `panther_msgs::LEDAnimation::GOAL_ACHIEVED`.
- `LOW_BATTERY_ANIM_ID` [*unsigned*, value: **6**]: animation ID constant obtained from `panther_msgs::LEDAnimation::LOW_BATTERY`.
- `CRITICAL_BATTERY_ANIM_ID` [*unsigned*, value: **7**]: animation ID constant obtained from `panther_msgs::LEDAnimation::CRITICAL_BATTERY`.
- `BATTERY_STATE_ANIM_ID` [*unsigned*, value: **8**]: animation ID constant obtained from `panther_msgs::LEDAnimation::BATTERY_STATE`.
- `CHARGING_BATTERY_ANIM_ID` [*unsigned*, value: **9**]: animation ID constant obtained from `panther_msgs::LEDAnimation::CHARGING_BATTERY`.
- `POWER_SUPPLY_STATUS_UNKNOWN` [*unsigned*, value: **0**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN`.
- `POWER_SUPPLY_STATUS_CHARGING` [*unsigned*, value: **1**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING`.
- `POWER_SUPPLY_STATUS_DISCHARGING` [*unsigned*, value: **2**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING`.
- `POWER_SUPPLY_STATUS_NOT_CHARGING` [*unsigned*, value: **3**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING`.
- `POWER_SUPPLY_STATUS_FULL` [*unsigned*, value: **4**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL`.

### Safety

A tree responsible for monitoring the Panther robot's state and handling safety measures, such as cooling the robot in case of high Built-in Computer's CPU or Battery temperatures.

<p align="center">
  <img align="center" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/safety_tree.svg">
</p>

Default blackboard entries:
- `aux_state` [*bool*, default: **None**]: state of AUX Power.
- `bat_temp` [*double*, default: **None**]: moving average of the Battery temperature.
- `cpu_temp` [*double*, default: **None**]: moving average of the Built-in Computer's CPU temperature
- `driver_temp` [*double*, default: **None**]: moving average of driver temperature. Out of the two drivers, the one with the higher temperature is taken into account.
- `e_stop_state` [*bool*, default: **None**]: state of the E-stop.
- `fan_state` [*bool*, default: **None**]: state of the fan.

Default constant blackboard entries:
- `CPU_FAN_OFF_TEMP` [*float*, default: **60.0**]: refers to the`cpu_fan_off_temp` ROS parameter.
- `CPU_FAN_ON_TEMP` [*float*, default: **70.0**]: refers to the `cpu_fan_on_temp` ROS parameter.
- `CRITICAL_BAT_TEMP` [*float*, default: **59.0**]: refers to the `critical_bat_temp` ROS parameter.
- `DRIVER_FAN_OFF_TEMP` [*float*, default: **35.0**]: refers to the `driver_fan_off_temp` ROS parameter.
- `DRIVER_FAN_ON_TEMP` [*float*, default: **45.0**]: refers to the `driver_fan_on_temp` ROS parameter.
- `HIGH_BAT_TEMP` [*float*, default: **55.0**]: refers to the `high_bat_temp` ROS parameter.
- `POWER_SUPPLY_HEALTH_UNKNOWN` [*unsigned*, value: **0**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN`.
- `POWER_SUPPLY_HEALTH_GOOD` [*unsigned*, value: **1**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD`.
- `POWER_SUPPLY_HEALTH_OVERHEAT` [*unsigned*, value: **2**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT`.
- `POWER_SUPPLY_HEALTH_DEAD` [*unsigned*, value: **3**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD`.
- `POWER_SUPPLY_HEALTH_OVERVOLTAGE` [*unsigned*, value: **4**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE`.
- `POWER_SUPPLY_HEALTH_UNSPEC_FAILURE` [*unsigned*, value: **5**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE`.
- `POWER_SUPPLY_HEALTH_COLD` [*unsigned*, value: **6**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD`.
- `POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE` [*unsigned*, value: **7**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE`.
- `POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE` [*unsigned*, value: **8**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE`.

### Shutdown 
 
A tree responsible for the graceful shutdown of robot components, user computers, and the built-in computer. By default, it will proceed to shutdown all computers defined in a YAML file with a path defined by the `~shutdown_host_file` ROS parameter.

<p align="center">
  <img src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/shutdown_tree.svg">
</p>

Default constant blackboard entries:
  - `SHUTDOWN_HOSTS_FILE` [*string*, default: **None**]: refers to `shutdown_hosts_file` ROS parameter.

Expected blackboard entries:
  - `signal_shutdown` [*pair(bool, string)*, default: **(false, '')**]: flag to shutdown robot with information to display while shutting down.

### Modifying Behavior Trees

Each behavior tree can be easily customized to enhance its functions and capabilities. To achieve this, we recommend using Groot2, a powerful tool for developing and modifying behavior trees. To install Groot2 and learn how to use it, please refer to the [official guidelines](https://www.behaviortree.dev/groot).

When creating a new BehaviorTree project, it is advised to use an existing project as a guideline and leverage it for reference. You can study the structure and implementation of the behavior trees in the existing project to inform your own development process. The project should consist of three behavior trees: `Lights`, `Safety`, `Shutdown`. Additionally, you have the option to incorporate some of the files used in the existing project into your own project. By utilizing these files, you can benefit from the work already done and save time and effort in developing certain aspects of the behavior trees.

> **Note**:
> It is essential to exercise caution when modifying the trees responsible for safety or shutdown and ensure that default behaviors are not removed.
>
> Remember to use the files from the existing project in a way that avoids conflicts, such as by saving them under new names to ensure they don't overwrite any existing files.

When modifying behavior trees, you have the flexibility to use standard BehaviorTree.CPP nodes or leverage nodes created specifically for Panther, as detailed in the [Nodes](#nodes) section. Additionally, if you have more specific requirements, you can even create your own custom Behavior Tree nodes. However, this will involve modifying the package and rebuilding the project accordingly.

To use your customized project, you need to provide the `bt_project_file` launch argument when running `panther_bringup.launch` file. Here's an example of how to launch the project with the specified BehaviorTree project file:

```bash
roslaunch --wait panther_bringup bringup.launch bt_project_file:=/path/to/bt/project/file
```

