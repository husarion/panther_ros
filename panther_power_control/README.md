[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_power_control

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

A package containing nodes responsible for power management of the Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### power_board_node.py

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

Node responsible for management of the safety board and the power board.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

> **Note**
> Available since Panther version 1.2.

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling E-stop if published.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/panther/hardware/e_stop` [*std_msgs/Bool*, latched: **true**]: the current state of the E-stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, latched: **true**]: publishes state of panther IO pins. Message fields are related to:
  - `aux_power`: related to service `aux_power_enable`,
  - `charger_connected`: indicates if standard charger is connected,
  - `charger_enabled`: related to service `charger_enable`,
  - `digital_power`: related to service `digital_power_enable`,
  - `fan`: related to service `fan_enable`,
  - `motor_power`: related to service `motor_power_enable`,
  - `power_button`: indicates if the Power Button is pressed.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `/panther/hardware/aux_power_enable` [*std_srvs/SetBool*]: enable or disable AUX Power output, e.g., supply to robotic arms.
- `/panther/hardware/charger_enable` [*std_srvs/SetBool*]: if a non-standard charger is available, this service allows enabling and disabling it.
- `/panther/hardware/digital_power_enable` [*std_srvs/SetBool*]: enable or disable the internal digital power used to power on, e.g. User Computer, Router, etc.
- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset E-stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger E-stop.
- `/panther/hardware/fan_enable` [*std_srvs/SetBool*]: enable or disable the internal fan. Calling the service from the terminal when it is enabled might yield unintuitive behavior. This is because `manager_node` overwrites control. It is advisable to use `manager_node` when implementing fan behaviors. For more information, refer to [panther_manager](../panther_manager/README.md).
- `/panther/hardware/motor_power_enable` [*std_srvs/SetBool*]: enable or disable motor drivers.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)

#### Service Clients

[//]: # (ROS_API_NODE_SERVICE_CLIENTS_START)

- `/panther/driver/reset_roboteq_script` [*std_srvs/Trigger*]: used to reset the Roboteq drivers script when enabling motor drivers.

[//]: # (ROS_API_NODE_SERVICE_CLIENTS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_NAME_START)

### relays_node.py

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

This node is responsible for power management using relays.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

> **Note**
> This node is used in Panther versions 1.06 and below.

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `/cmd_vel` [*geometry_msgs/Twist*]: observes if velocity commands are sent to the robot. Prevents disabling E-stop if published.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: checks for errors on motor controllers.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/panther/hardware/e_stop` [*std_msgs/Bool*, latched: **true**]: the current state of the emulated E-stop.
- `/panther/hardware/io_state` [*panther_msgs/IOState*, latched: **true**]: publishes the state of panther IO pins. Used for driver compatibility with Panther version 1.06 and below. Message fields with real hardware representation are:
  - `motor_power` indicates if motor drivers are powered on.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `/panther/hardware/e_stop_reset` [*std_srvs/Trigger*]: reset E-stop.
- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: trigger E-stop.
- `/panther/hardware/motor_power_enable` [*std_srvs/SetBool*]: enable or disable motor drivers. Acts in conjunction with the three-position Main Switch. Motors can not be enabled if the switch is in Stage 1. In the case of a switch transitioning from Stage 1 to Stage 2, the state set by the service will be overwritten and motors will be enabled.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)
[//]: # (ROS_API_NODE_END)
[//]: # (ROS_API_PACKAGE_END)