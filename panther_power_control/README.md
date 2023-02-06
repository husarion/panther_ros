# panther_power_control

A package containing nodes responsible for power management of the Husarion Panther robot.

## ROS Nodes

### power_board_node.py

Node responsible for management of the safety board and the power board. Available since Panther version 1.2.

#### Publishes

- `/panther/hardware/e_stop` [*std_msgs/Bool*, *latched*]: the current state of the emergency stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, *latched*]: publishes state of panther IO pins.

#### Subscribes

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling e-stop if published.

#### Services advertised

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable auxiliary power output, e.g. supply to robotic arms.
- `/panther/hardware/charger_enable` [*std_srvs/SetBool*]: enable or disable charger.
- `/panther/hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. NUC, Router, etc.
- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable internal fan.
- `/panther/hardware/motors_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.

### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Publishes

- `/panther/hardware/e_stop` [*std_msgs/Bool*]: the current state of the emulated emergency stop.
- `/panther/hardware/motor_on` [*std_msgs/Bool*]: informs if motor controllers are on.

#### Services advertised

- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.

