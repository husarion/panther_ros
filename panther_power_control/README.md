# panther_power_control

A package containing nodes responsible for power management of the Husarion Panther robot.

## ROS Nodes

### power_board_node.py

Node responsible for management of the safety board and the power board. Available since Panther version 1.2.

#### Subscribers

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling E-stop if published.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

#### Publishers

- `/panther/hardware/e_stop` [*std_msgs/Bool*, latched: **true**]: the current state of the E-stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, latched: **true**]: publishes state of panther IO pins. Message fields are related to:
  - `aux_power`: related to service `aux_power_enable`,
  - `charger_connected`: indicates if standard charger is connected,
  - `charger_enabled`: related to service `charger_enable`,
  - `digital_power`: related to service `digital_power_enable`,
  - `fan`: related to service `fan_enable`,
  - `motor_power`: related to service `motor_power_enable`,
  - `power_button`: indicates if the Power Button is pressed.

#### Service Servers

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable AUX Power output, e.g., supply to robotic arms.
- `/panther/hardware/charger_enable` [*std_srvs/SetBool*]: if a non-standard charger is available, this service allows enabling and disabling it.
- `/panther/hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. User Computer, Router, etc.
- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset E-stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger E-stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable the internal fan. Calling the service from the terminal when it is enabled might yield unintuitive behavior. This is because `manager_node` overwrites control. It is advisable to use `manager_node` when implementing fan behaviors. For more information, refer to [panther_manager](../panther_manager/README.md).
- `/panther/hardware/motor_power_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.

#### Service Clients

- `/panther/driver/reset_roboteq_script` [*std_srvs/Trigger*]: used to reset the Roboteq drivers script when enabling motor drivers.

### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Subscribers

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling E-stop if published.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

#### Publishers

- `/panther/hardware/e_stop` [*std_msgs/Bool*, latched: **true**]: the current state of the emulated E-stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, latched: **true**]: publishes the state of panther IO pins. Used for driver compatibility with Panther version 1.06 and below. Message fields with real hardware representation are:
  - `motor_power` indicates if motor drivers are powered on.

#### Service Servers

- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset E-stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger E-stop.
- `/panther/hardware/motor_power_enable` [*std_srvs/SetBool*]: enable or disable motor drivers. Acts in conjunction with the three-position Main Switch. Motors can not be enabled if the switch is in Stage 1. In the case of a switch transitioning from Stage 1 to Stage 2, the state set by the service will be overwritten and motors will be enabled.
