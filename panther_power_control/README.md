# panther_power_control

A package containing nodes responsible for power management of the Husarion Panther robot.

## ROS Nodes

### fan_controller_node.py

Observes internal temperatures of the robot and turns on and off the built-in cooling fan. The fan is turned on based on the internal computer CPU temperature and motor driver temperature. Available since Panther version 1.2.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: information about driver temperature.
- `/panther/hardware/fan_enabled` [*std_msgs/Bool*]: feedback if fan is currently turned on.
- `/panther/system_status` [*panther_msgs/SystemStatus*]: information about internal computer temperature.

#### Services subscribed

- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: turns on and off internal fan.

#### Parameters

- `~cpu_fan_on_temp` [*float*, default: **70.0**]: temperature in **deg C** of CPU, above which the fan is turned on.
- `~cpu_fan_off_temp` [*float*, default: **60.0**]: temperature in **deg C** of CPU, below which the fan is turned off.
- `~driver_fan_on_temp` [*float*, default: **45.0**]: temperature in **deg C** of any drivers above which the fan is turned on.
- `~driver_fan_off_temp` [*float*, default: **35.0**]: temperature in **deg C** of any drivers below which the fan is turned off.
- `~hysteresis` [*float*, default: **60.0**]: minimum time of work before turning off the fan.
- `~cpu_window_len` [*int*, default: **6**]: length of moving average used to smooth out temperature readings of CPU.
- `~driver_window_len` [*int*, default: **6**]: length of moving average used to smooth out temperature readings of each driver.

### power_board_node.py

Node responsible for management of the safety board and the power board. Available since Panther version 1.2.

In order to set up connection with new user computer, login to the built-in computer with `ssh husarion@10.15.20.2`.
Add built-in computer's public key to **known_hosts** of a computer you want to shut down automatically:
``` bash
ssh-copy-id username@10.15.20.XX
```

#### Publishes

- `/panther/hardware/charger_connected` [*std_msgs/Bool*]: informs if charger is connected.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: the current state of the emergency stop.

#### Services advertised

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable auxiliary power output, e.g. supply to robotic arms.
- `/panther/hardware/charger_enable` [*std_srvs/SetBool*]: enable or disable charger.
- `/panther/hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. NUC, Router, etc.
- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable internal fan.
- `/panther/hardware/motors_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.

#### Parameters

- `~shutdown_timeout` [*float*, default: **60.0**]: time in seconds to wait for graceful shutdown. After timeout power will be cut from all devices.
- `~self_ip` [*string*, default: **127.0.0.1**]: IP used to shut down device running this node.
- `~self_username` [*string*, default: **husarion**]: username used to shut down device running this node.
- `~default_identity_file` [*string*, default: **~/.ssh/id_rsa**]: default path used to find identity global file for SSH.
- `~self_identity_file` [*string*, default: **None**]: identity file global path to shut down device running this node. If not set defaults to `~default_identity_file`.
- `~hosts` [*list*, default: **None**]: list of hosts to request shutdown.
  - `~hosts/ip` [*string*, default: **None**]: IP of a host to shutdown over SSH.
  - `~hosts/username` [*string*, default: **None**]: username used to login to over SSH.
  - `~hosts/identity_file` [*string*, default: **None**]: SSH identity file global path. If not set defaults to `~default_identity_file`.


### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Publishes

- `/panther/hardware/e_stop` [*std_msgs/Bool*]: the current state of the emulated emergency stop.
- `/panther/hardware/motor_on` [*std_msgs/Bool*]: informs if motor controllers are on.

#### Services advertised

- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.