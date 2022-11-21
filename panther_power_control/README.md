# panther_power_control

A package containing nodes responsible for power management of the Husarion Panther robot.

## ROS Nodes

### power_board_node.py

Node responsible for management of the safety board and the power board. Available from Panther version 1.2.

#### Publishes

- `/panther_hardware/e_stop` [*std_msgs/Bool*]: the current state of the emergency stop.
- `/panther_hardware/charger_connected` [*std_msgs/Bool*]: informs if charger is connected.

#### Services

- `/panther_hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable auxiliary power output, e.g. supply to robotic arms, etc.
- `/panther_hardware/charger_enable` [*std_srvs/SetBool*]: enable or disable charger.
- `/panther_hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. NUC, Router, etc.
- `/panther_hardware/motors_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.
- `/panther_hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable internal fan.
- `/panther_hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther_hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.

### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Publishes

- `/panther_hardware/motor_on` [*std_msgs/Bool*]: informs if motor controllers are on.
- `/panther_hardware/e_stop` [*std_msgs/Bool*]: the current state of the emulated emergency stop.

#### Services

- `/panther_hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther_hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.