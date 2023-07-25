# panther_power_control

A package containing nodes responsible for power management of the Husarion Panther robot.

## ROS Nodes

### power_board_node.py

Node responsible for management of the safety board and the power board. Available since Panther version 1.2.

#### Publishes

- `/panther/hardware/e_stop` [*std_msgs/Bool*, *latched*]: the current state of the emergency stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, *latched*]: publishes state of panther IO pins. Message fields are related to:
  - `aux_power` is related to service `aux_power_enable`,
  - `charger_connected` indicates if standard charger is connected,
  - `charger_enabled` is related to service `charger_enable`,
  - `digital_power` is related to service `digital_power_enable`,
  - `fan` is related to service `fan_enable`,
  - `motor_on` is related to service `motor_enable`,
  - `power_button` indicates if power button is pressed.

#### Subscribes

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling e-stop if published.

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

#### Services advertised

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable auxiliary power output, e.g. supply to robotic arms.
- `/panther/hardware/charger_enable` [*std_srvs/SetBool*]: if non standard charger is available this service allows to enable and disable it.
- `/panther/hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. NUC, Router, etc.
- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable internal fan.
- `/panther/hardware/motor_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.

#### Service clients

- `/panther/driver/reset_roboteq_script` [*std_srvs/Trigger*]: used to reset Roboteq drivers script when enabling motor drivers.

### relays_node.py

This node is responsible for power management using relays. Available in Panther versions 1.06 and below.

#### Publishes

- `/panther/hardware/e_stop` [*std_msgs/Bool*, *latched*]: the current state of the emulated emergency stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, *latched*]: publishes state of panther IO pins. Used for driver compatybility with Panther version 1.06 and below. Message fields with real hardware representation are:
  - `motor_on` indicates if motor drivers are powered on.

#### Subscribes

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling e-stop if published.

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

#### Services advertised

- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset emergency stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger emergency stop.
- `/panther/hardware/motor_enable` [*std_srvs/SetBool*]: enable or disable motor drivers. Acts in conjunction with the three-position Main switch. Motors can not be enabled if switch is in Stage 1. In case of switch transitioning from Stage 1 to Stage 2, state set by service will be overwritten and motors will be enabled.
