# panther_manager

A package containing nodes responsible for high-level control of Husarion Panther robot.

## ROS Nodes

### manager_node.py

Node responsible for managing the Husarion Panther robot. Controls built-in fan and controls software shutdown of components.

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

#### Services

- `/panther/manager/overwrite_fan_control` [*std_srvs/SetBool*]: overwrites fan control, setting it to always on.

#### Service clients

- `hardware/aux_power_enable` [*std_srvs/SetBool*]: enables aux power output.
- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: triggers e-stop.
- `hardware/fan_enable` [*std_srvs/SetBool*]: enables fan.

#### Parameters

- `~battery_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of battery.
- `~cpu_fan_off_temp` [*float*, default: **60.0**]: temperature in **deg C** of CPU, above which the fan is turned off.
- `~cpu_fan_on_temp` [*float*, default: **70.0**]: temperature in **deg C** of any drivers above which the fan is turned on.
- `~cpu_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of CPU.
- `~critical_bat_temp` [*float*, default: **59.0**]: temperature of battery above which robot enters e-stop state.
- `~default_identity_file` [*string*, default: **~/.ssh/id_rsa**]: 
 default path used to find identity global file for SSH. 
- `~driver_fan_off_temp` [*float*, default: **35.0**]: temperature in **deg C** of any drivers below which the fan is turned off.
- `~driver_fan_on_temp` [*float*, default: **45.0**]: temperature in **deg C** of any drivers above which the fan is turned on.
- `~driver_window_len` [*int*, default: **6**]: moving average window length used to smooth out temperature readings of each driver.
- `~fatal_bat_temp` [*float*, default: **62.0**]: temperature of battery above which robot shuts down.
- `~high_bat_temp` [*float*, default: **55.0**]: temperature of battery above which robots starts displaying warning log and e-stop is triggered.
- `~hosts` [*list*, default: **empty list**]: list of hosts to request shutdown.
  - `cmd` [*string*, default: **sudo shutdown now**]: command executed to on shutdown of given device.
  - `identity_file` [*string*, default: **None**]: SSH identity file global path. If not set, defaults to `~default_identity_file`.
  - `ip` [*string*, default: **None**]: IP of a host to shutdown over SSH.
  - `username` [*string*, default: **None**]: username used to log in to over SSH.
- `~hysteresis` [*float*, default: **60.0**]: minimum time of fan being turned on.
- `~overwrite_fan_control` [*bool*, default: **False**]: enable fan to be always on start of the node. It can be turned off later by `/panther/manager/overwrite_fan_control`.
- `~self_identity_file` [*float*, default: **~/.ssh/id_rsa**]: identity file global path to shutdown device running this node. If not, set defaults to `~default_identity_file`.
- `~self_ip` [*string*, default: **127.0.0.1**]: IP used to shutdown device running this node.
- `~self_username` [*string*, default: **husarion**]: username used to shutdown device running this node.
- `~shutdown_timeout` [*float*, default: **15.0**]: time in seconds to wait for graceful shutdown. After timeout power will be cut from all devices.

### system_status_node.py

Publishes stats status of the internal computer. Stats include CPU utilization and temperature, as well as disc and RAM usage.

#### Publishes

- `/panther/system_status` [*panther_msgs/SystemStatus*]: information about internal computer CPU temperature, utilization and disc and RAM usage.