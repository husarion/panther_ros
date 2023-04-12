# panther_manager

A package containing nodes responsible for high-level control of Husarion Panther robot.

## ROS Nodes

### manager_bt_node.cpp

Node responsible for managing the Husarion Panther robot. Composes control of three behavior trees responsible for handling LED panels, safety features and software shutdown of components.

To set up connection with a new user computer, login to the built-in computer with `ssh husarion@10.15.20.2`.
Add built-in computer's public key to **known_hosts** of a computer you want to shut down automatically:
``` bash
ssh-copy-id username@10.15.20.XX
```

To allow your computer to be shut down without sudo password, ssh into it and execute:
``` bash
echo $USERNAME 'ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown' | sudo EDITOR='tee -a' visudo
```

#### Subscribes

- `/panther/battery` [*sensor_msgs/BatteryState*]: state of internal battery.
- `/panther/driver/motor_controllers_state` [*panther_msgsDriverState*]: state of motor controllers.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: state of emergency stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*]: state of IO pins.
- `/panther/system_status` [*panther_msgs/SystemStatus*]: state of system including CPU temperature and load.

#### Services subscribed (default trees)

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enables aux power output.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: triggers e-stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enables fan.
- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.

#### Parameters

- `~battery_state_anim_period` [*float*, default: **120.0**]: time in seconds to wait before repeating animation representing current battery percentage.
- `~battery_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of battery.
- `~cpu_fan_off_temp` [*float*, default: **60.0**]: temperature in **deg C** of CPU, below which the fan is turned off.
- `~cpu_fan_on_temp` [*float*, default: **70.0**]: temperature in **deg C** of CPU, above which the fan is turned on.
- `~cpu_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of CPU.
- `~critical_bat_temp` [*float*, default: **59.0**]: extends `high_bat_temp` by turning off AUX power.
- `~critical_battery_anim_period` [*float*, default: **15.0**]: time in seconds to wait before repeating animation indicating a critical battery state.
- `~critical_battery_threshold_percent` [*float*, default: **0.1**]: if battery percentage drops below this value, animation indicating a critical battery state will start being displayed.
- `~driver_fan_off_temp` [*float*, default: **35.0**]: temperature in **deg C** of any drivers below which the fan is turned off.
- `~driver_fan_on_temp` [*float*, default: **45.0**]: temperature in **deg C** of any drivers above which the fan is turned on.
- `~driver_temp_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of each driver.
- `~fatal_bat_temp` [*float*, default: **62.0**]: temperature of battery above which robot is shutdown.
- `~high_bat_temp` [*float*, default: **55.0**]: temperature of battery above which robots starts displaying warning log and e-stop is triggered.
- `~low_battery_anim_period` [*float*, default: **30.0**]: time in seconds to wait before repeating animation indicating a low battery state.
- `~low_battery_threshold_percent` [*float*, default: **0.4**]: if the battery percentage drops below this value, animation indicating a low battery state will start being displayed.
- `~shutdown_hosts_file` [*string*, default: **None**]: path to a YAML file containing list of hosts to request shutdown. To correctly format the YAML file, include a **hosts** field consisting of a list with the following fields:
  - `command` [*string*, default: **sudo shutdown now**]: command executed on shutdown of given device.
  - `ip` [*string*, default: **None**]: IP of a host to shutdown over SSH.
  - `username` [*string*, default: **None**]: username used to log in to over SSH.
- `~shutdown_timeout` [*float*, default: **15.0**]: time in seconds to wait for graceful shutdown. After timeout power will be cut from all devices.
- `~update_charging_anim_step` [*float*, default: **0.1**]: percentage value representing a step for updating the charging battery animation.

### system_status_node.py

Publishes stats status of the built-in computer. Stats include CPU utilization and temperature, as well as disc and RAM usage.

#### Publishes

- `/panther/system_status` [*panther_msgs/SystemStatus*]: information about internal computer CPU temperature, utilization, disc and RAM usage.

## BehaviorTree 

### Nodes

#### Actions

- `CallSetBoolService` - allows calling standard **std_srvs/SetBool** ROS service.
- `CallSetLedAnimationService` - allows calling custom type **panther_msgs/SetLEDAnimation** ROS service.
- `CallTriggerService` - allows calling standard **std_srvs/Trigger** ROS service.
- `ShutdownHostsFromFile` - allows to shutdown devices based on YAML file. Returns FAILURE only when YAML file is incorrect or a path to the file does not exists. If it fails to execute command in a remote host node will proceed tothe next device from the list.
- `ShutdownSingleHost` - allows to shutdown single device. Will return SUCCESS only when device can be reached and the command was executed.

#### Decorators

- `TickAfterTimeout` - will skip child before specified time has passed. Can be used to control frequency at witch given behavior in the tree is triggered.

### Trees

- `Lights` - this tree is responsible for scheduling animations displayed on LED panels based on the Husarion Panther robot's system state.
- `Safety` - this tree is responsible for monitoring the Panther robot's state and handling safety measures like cooling the robot in case of high CPU or battery temperature.
- `Shutdown` - this tree is responsible for gracefull shutdown of robot components and the built-in computer.