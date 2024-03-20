[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_hardware_interfaces

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

Package that implements SystemInterface from ros2_control for Panther.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### PantherSystem

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

This package doesn't contain any standalone nodes - `PantherSystem` is a plugin loaded by the controller manager.
To use this hardware interface you have to add it to your URDF (you can check how to do it in [panther_description](../panther_description/)) and add a controller (example configuration provided in [panther_controller](../panther_controller/) package).
That said apart from the usual interface provided by the ros2_control, this plugin also provides additional published topics and services specific for Panther.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Panther system diagnostic messages.
- `/panther_system_node/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers state and error flags.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)

Parameters that are required, are defined when including interface in URDF (you can check out [panther_macro.urdf.xacro](../panther_description/urdf/panther_macro.urdf.xacro)).

Physical properties

- `encoder_resolution` [*int*, default: **1600**]: property of the encoder used, shouldn't be changed.
- `gear_ratio` [*float*, default: **30.08**]: property of the gearbox used, shouldn't be changed.
- `motor_torque_constant` [*float*, default: **0.11**]: same as set in the Roboteq driver (TNM parameter), also shouldn't be changed, as it is measured property of the motor.
- `max_rpm_motor_speed` [*float*, default: **3600.0**]: max RPM speed set in the Roboteq driver (MXRPM parameter).
- `gearbox_efficiency` [*float*, default: **0.75**]: measured efficiency, used for converting read current to torque, can vary depending on different factors such as temperature and wear.

CAN settings

- `can_interface_name` [*string*, default: **panther_can**]: name of the CAN interface.
- `master_can_id` [*int*, default: **3**]: CAN ID of the master device (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).
- `front_driver_can_id` [*int*, default: **1**]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).
- `rear_driver_can_id` [*int*, default: **2**]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).
- `sdo_operation_timeout_ms` [*int*, default: **100**]: timeout of the SDO operations, currently no SDO operation is required in RT operation, so this timeout can be set to a higher value.
- `pdo_motor_states_timeout_ms` [*int*, default: **15**]: depends on the frequency at which Roboteq is configured to send motor states (PDO 1 and 2) data. By default, there should be 10 **[ms]** between received data, if it takes more than `pdo_motor_states_timeout_ms`, a motor states read error is triggered. The default value is set to be expected period +50% margin.
- `pdo_driver_state_timeout_ms` [*int*, default: **75**]: depends on the frequency at which Roboteq is configured to send driver state (PDO 3 and 4) data. By default, there should be 50 **[ms]** between received data, if it takes more than `pdo_driver_state_timeout_ms`, a driver state read error is triggered. The default value is set to be expected period +50% margin.
- `driver_states_update_frequency` [*float*, default: **20.0**]: as by default, the driver state is published with lower frequency, it also shouldn't be updated with every controller loop iteration. The exact frequency at which driver state is published won't match this value - it will also depend on the frequency of the controller (the exact value of the period can be calculated with the following formula `controller_frequency / ceil(controller_frequency / driver_states_update_frequency)`).
- `max_roboteq_initialization_attempts` [*int*, default: **5**]: in some cases, an SDO error can happen during initialization, it is possible to configure more attempts, before escalating to a general error.
- `max_roboteq_activation_attempts` [*int*, default: **5**]: similar to initialization, it is possible to allow some SDO errors before escalating to error.
- `max_write_pdo_cmds_errors_count` [*int*, default: **2**]: how many consecutive errors can happen before escalating to general error.
- `max_read_pdo_motor_states_errors_count` [*int*, default: **2**]: how many consecutive errors can happen before escalating to general error.
- `max_read_pdo_driver_state_errors_count` [*int*, default: **2**]: how many consecutive errors can happen before escalating to general error.

> [!CAUTION]
> `max_write_pdo_cmds_errors_count`, `max_read_pdo_motor_states_errors_count`, `max_read_pdo_driver_state_errors_count`, `sdo_operation_timeout`, `pdo_motor_states_timeout_ms` and `pdo_driver_state_timeout_ms` are safety-critical parameters, they should be changed only in very specific cases, be sure that you know how they work and be really cautious when changing them.

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)

## Code structure

The code structure is described in more detail in a [separate file](./CODE_STRUCTURE.md).

## Generating CAN config

Adjust your configuration and generate a new `master.dcf` using:
`dcfgen canopen_configuration.yaml -r`

### RT

To configure RT check out the instructions provided in the [ros2_control docs](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism) (add group and change `/etc/security/limits.conf`).

## Testing

### Setup

First, it is necessary to set up a virtual CAN:

<!-- todo move setup somewhere so the test can be run more easily -->

```bash
sudo modprobe vcan
sudo ip link add dev panther_can type vcan
sudo ip link set up panther_can
sudo ip link set panther_can down
sudo ip link set panther_can txqueuelen 1000
sudo ip link set panther_can up
```

### Running tests

```bash
colcon build --packages-select panther_hardware_interfaces --symlink-install
colcon test --event-handlers console_direct+ --packages-select panther_hardware_interfaces --parallel-workers 1
colcon test-result --verbose --all
```

As some of the tests are accessing the virtual CAN interface, they can't be executed in parallel (that's why `--parallel-workers 1` flag).
