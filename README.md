# ROS packages

# panther

ROS Metapackeage composing basic functionalities of Panther robot with links to dependency repositories.

---

# panther_bringup

Package containing default config and launch files, necessary to  start all base functionalities of the robot.

---

# panther_battery

Package containing nodes monitoring and publishing internal battery state.

## Nodes

### adc_node.py

  Publishes battery state read from ADC unit. Available from Panther version 1.2

#### Publishes

- `battery` [*sensor_msgs/BatteryState*]: average values of both batteries if panther has two batteries. In case of single battery values only for the single one. **TODO FIX SINGLE BATTERY**
- `battery_1` [*sensor_msgs/BatteryState*]: first battery state.
- `battery_2` [*sensor_msgs/BatteryState*]: second battery state.

#### Subscribes

- `motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

#### Parameters

- `~loop_rate` [*float*, default: **20**]: rate in Hz at which battery state will be computed and published.

### roboteq_republisher_node.py

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below.

#### Publishes

- `battery` [*sensor_msgs/BatteryState*]: battery state.

#### Subscribes

- `motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

---

# panther_power_control

A package containing nodes responsible for power management of the Panther robot.

## Nodes

### power_board_node.py

Node is responsible for management of a safety board and power board. Available from Panther version 1.2.

#### Publishes

- `/panther_hardware/e_stop` [*std_msgs/Bool*]: current state of emergency stop.
- `/panther_hardware/charger_connected` [*std_msgs/Bool*]: informs if charger is connected.

#### Services

- `/panther_hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable auxiliary power output, e.g. supply to robotic arms etc. or disable it
- `/panther_hardware/charger_enable` [*std_srvs/SetBool*]: enable or disable charger.
- `/panther_hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable internal the digital power used to power on, e.g. NUC, Router etc.
- `/panther_hardware/motors_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.
- `/panther_hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable internal fan.
- `/panther_hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther_hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.

### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Publishes

- `/panther_hardware/motor_on` [*std_msgs/Bool*]: informs if motor controllers are on.
