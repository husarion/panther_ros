# panther_ros
Repository with ROS packages for Panther autonomous mobile robot

# ROS packages

---

# `panther`

Metapackeage that contains dependencies to other repositories.

---

# `panther_bringup`

Package that contains default robot config and launch, which starts all base functionalities and parameters.

---

# `panther_battery`

This package contains nodes monitoring and publishing battery state

## Nodes

### `adc_node.py`

  This node publishes battery state read from ADC unit available from Panther version 1.2

#### Topics published

- `battery` [*sensor_msgs/BatteryState*]: Panther batteries state
- `battery_1` [*sensor_msgs/BatteryState*]: Panther first battery state
- `battery_2` [*sensor_msgs/BatteryState*]: Panther second battery state

#### Topics subscribed

- `motor_controllers_state` [*panther_msgs/DriverState*]: used to get motor controllers state

#### Parameters

- `~loop_rate` [*float*, default: **20**]: rate in Hz at which battery state will be calculated and published

### `roboteq_republisher_node.py`

This node publishes Panther battery state read from motor controllers. Used in Panther versions 1.05 and 1.06

#### Topics published

- `battery` [*sensor_msgs/BatteryState*]: Panther battery state

#### Topics subscribed

- `motor_controllers_state` [*panther_msgs/DriverState*]: used to get motor controllers state

---

# `panther_power_control`

A package that contains nodes responsible for power management in Panther robot

## Nodes

### `power_board_node.py`

This node is responsible for power management using safety board available from Panther version 1.2

#### Topics published

- `/panther_hardware/e_stop` [*std_msgs/Bool*]: Panther emergency stop state
- `/panther_hardware/charger_connected` [*std_msgs/Bool*]: informs if charger is connected

#### Services

- `/panther_hardware/aux_power_enable` [*std_srvs/SetBool*]: this service allows to enable auxiliary power, eg. supply to robotic arms etc. or disable it
- `/panther_hardware/charger_enable` [*std_srvs/SetBool*]: this service allows to enable or disable charger
- `/panther_hardware/digital_power_enable` [*std_srvs/SetBool*]: this service allows to enable the digital power eg. NUC, Router etc. or disable it
- `/panther_hardware/motors_enable` [*std_srvs/SetBool*]: this service allows to enable or disable motor drivers
- `/panther_hardware/fan_enable` [*std_srvs/SetBool*]: this service allows to enable or disable fan
- `/panther_hardware/e_stop_reset` [*std_srvs/Trigger*]: this service allows to reset emergency stop
- `/panther_hardware/e_stop_trigger` [*std_srvs/Trigger*]: this service allows to trigger emergency stop

### `relays_node.py`

This node is responsible for power management using relays available in Panther versions 1.05 and 1.06

#### Topics published

- `/panther_hardware/motor_on` [*std_msgs/Bool*]: informs if motor controllers are on
