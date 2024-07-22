# panther_manager

A package containing nodes responsible for high-level control of Husarion Panther robot.

## Launch Files

This package contains:

- [`manager_bt.launch.py`](#manager_btlaunchpy---arguments) - is responsible for activating whole robot system.

### manager_bt.launch.py - Arguments

| Argument                     | Description <br/> ***Type:*** `Default`                                                                                                                                                                    |
| ---------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `lights_bt_project_path`     | Path to BehaviorTree project file, responsible for lights management. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherLightsBT.btproj'')`              |
| `safety_bt_project_path`*    | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'behavior_trees' + 'PantherSafetyBT.btproj'')` |
| `namespace`                  | Add namespace to all launched nodes. <br/> ***string:*** `EnvVar('ROBOT_NAMESPACE')`                                                                                                                       |
| `shutdown_hosts_config_path` | Path to file with list of hosts to request shutdown. <br/> ***string:*** `LocalVar('FindPackageShare(pkg='panther_manager') + 'config' + 'shutdown_hosts.yaml'')`                                          |
| `use_sim`                    | Whether simulation is used. <br/> ***boolean:*** `'False'`                                                                                                                                                 |

### manager_bt.launch.py - Nodes

| Node name        | *Type*                                     |
| ---------------- | ------------------------------------------ |
| `lights_manager` | [*panther_manager/battery_node*](.)        |
| `safety_manager` | [*panther_manager/safety_manager_node*](.) |

## ROS Nodes

### lights_manager

Node responsible for managing Bumper Lights animation scheduling.

#### Subscribers

- `battery/battery_status` [*sensor_msgs/BatteryState*]: state of the internal Battery.
- `hardware/e_stop` [*std_msgs/Bool*]: state of emergency stop.

#### Service Clients (for Default Trees)

- `~/lights/set_animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `battery.anim_period.critical` [*float*, default: **15.0**]: time in **[s]** to wait before repeating animation, indicating a critical Battery state.
- `battery.anim_period.low` [*float*, default: **30.0**]: time in **[s]** to wait before repeating the animation, indicating a low Battery state.
- `battery.charging_anim_step` [*float*, default: **0.1**]: this parameter defines the minimum change in battery percentage required to trigger an update in the battery charging animation.
- `battery.percent.threshold.critical` [*float*, default: **0.1**]: if the Battery percentage drops below this value, an animation indicating a critical Battery state will start being displayed.
- `battery.percent.threshold.low` [*float*, default: **0.4**]: if the Battery percentage drops below this value, the animation indicating a low Battery state will start being displayed.
- `battery.percent.window_len` [*int*, default: **6**]: moving average window length used to smooth out Battery percentage readings.
- `bt_project_path` [*string*, default: **$(find panther_manager)/config/PantherBT.btproj**]: path to a BehaviorTree project.
- `plugin_libs` [*list*, default: **Empty list**]: list with names of plugins that are used in the BT project.
- `ros_plugin_libs` [*list*, default: **Empty list**]: list with names of ROS plugins that are used in a BT project.
- `timer_frequency` [*float*, default: **10.0**]: frequency **[Hz]** at which lights tree will be ticked.

### safety_manager

Node responsible for managing safety features, and software shutdown of components.

#### Subscribers

- `battery/battery_status` [*sensor_msgs/BatteryState*]: state of the internal Battery.
- `hardware/e_stop` [*std_msgs/Bool*]: state of emergency stop.
- `hardware/io_state` [*panther_msgs/IOState*]: state of IO pins.
- `hardware/motor_controllers_state` [*panther_msgs/DriverState*]: state of motor controllers.
- `system_status` [*panther_msgs/SystemStatus*]: state of the system, including Built-in Computer's CPU temperature and load.

#### Service Clients (for Default Trees)

- `~/hardware/aux_power_enable` [*std_srvs/SetBool*]: enables Aux Power output.
- `~/hardware/e_stop_trigger` [*std_srvs/Trigger*]: triggers E-stop.
- `~/hardware/fan_enable` [*std_srvs/SetBool*]: enables fan.

#### Parameters

- `battery.temp.window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of the Battery.
- `bt_project_path` [*string*, default: **$(find panther_manager)/config/PantherBT.btproj**]: path to a BehaviorTree project.
- `cpu.temp.fan_off` [*float*, default: **60.0**]: temperature in **[&deg;C]** of the Built-in Computer's CPU, below which the fan is turned off.
- `cpu.temp.fan_on` [*float*, default: **70.0**]: temperature in **[&deg;C]** of the Built-in Computer's CPU, above which the fan is turned on.
- `cpu.temp.window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of the Built-in Computer's CPU.
- `driver.temp.fan_off` [*float*, default: **35.0**]: temperature in **[&deg;C]** of any drivers below which the fan is turned off.
- `driver.temp.fan_on` [*float*, default: **45.0**]: temperature in **[&deg;C]** of any drivers above which the fan is turned on.
- `driver.temp.window_len` [*int*, default: **6**]: moving average window length used to smooth out the temperature readings of each driver.
- `fan_turn_off_timeout` [*float*, default: **60.0**]: minimal time in **[s]**, after which the fan may be turned off.
- `plugin_libs` [*list*, default: **Empty list**]: list with names of plugins that are used in the BT project.
- `ros_plugin_libs` [*list*, default: **Empty list**]: list with names of ROS plugins that are used in a BT project.
- `shutdown_hosts_path` [*string*, default: **None**]: path to a YAML file containing a list of hosts to request shutdown. To correctly format the YAML file, include a **hosts** field consisting of a list with the following fields:
  - `command` [*string*, default: **sudo shutdown now**]: command executed on shutdown of given device.
  - `ip` [*string*, default: **None**]: IP of a host to shutdown over SSH.
  - `ping_for_success` [*bool*, default: **true**]: ping host until it is not available or timeout is reached.
  - `port` [*string*, default: **22**]: SSH communication port.
  - `timeout` [*string*, default: **5.0**]: time in **[s]** to wait for the host to shutdown. The Built-in Computer will turn off after all computers are shutdown or reached timeout. Keep in mind that hardware will cut power off after a given time after pressing the power button. Refer to the hardware manual for more information.
  - `username` [*string*, default: **None**]: username used to log in to over SSH.
- `timer_frequency` [*float*, default: **10.0**]: frequency **[Hz]** at which safety tree will be ticked.
