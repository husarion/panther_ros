# panther_hardware_interfaces

Package that implements SystemInterface from ros2_control for Panther.

## ROS Nodes

This package doesn't contain any nodes - it is used as a plugin within the controller manager. To use this hardware interface you have to add it to your URDF (you can check how to do it in `panther_description`) and add a controller (example configuration provided in `panther_controller` package).

### Publishers
- `/panther_system_node/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags

### Services
- `/panther_system_node/clear_errors` [*std_srvs/Trigger*]: clear current errors

## panther_hardware_interface parameters

Parameters that are required, are defined when including interface in URDF (you can check out [panther_macro.urdf.xacro](../panther_description/urdf/panther_macro.urdf.xacro)).

Physical properties
 - `encoder_resolution` [*int*, default: 1600]: property of the encoder used, shouldn't be changed
 - `gear_ratio` [*float*, default: 30.08]: property of the gearbox used, shouldn't be changed
 - `motor_torque_constant` [*float*, default: 0.11]: same as set in the Roboteq driver (TNM parameter), also shouldn't be changed, as it is measured property of the motor
 - `max_rpm_motor_speed` [*float*, default: 3600.0]: max RPM speed set in the Roboteq driver (MXRPM parameter)
 - `gearbox_efficiency` [*float*, default: 0.75]: measured efficiency, used for converting read current to torque, can vary depending on different factors such as temperature and wear

CAN settings
 - `can_interface_name` [*string*, default: panther_can]: name of the CAN interface
 - `master_can_id` [*int*, default: 3]: CAN ID of the master device (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml))
 - `front_driver_can_id` [*int*, default: 1]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml))
 - `rear_driver_can_id` [*int*, default: 2]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml))
 - `sdo_operation_timeout_ms` [*int*, default: 3 [ms]]: it is set so that the full controller loop takes up to the required time. Each controller loop contains five SDO operations (four writes and one read). For example, in a 100Hz loop, there is up to 10ms for all operations. This timeout should be set so that in the worst case everything takes 10ms.
 - `pdo_feedback_timeout_ms` [*int*, default: 30 [ms]]  - depends on the frequency at which Roboteq is configured to send PDO data. At 100Hz there should be 10ms between received data if it takes more than `pdo_feedback_timeout_ms`, a PDO read error is triggered
 - `max_roboteq_initialization_attempts` [*int*, default: 5]: in some cases, an SDO error can happen during initialization, it is possible to configure more attempts, before escalating to general error
 - `max_roboteq_activation_attempts` [*int*, default: 5]: similar to initialization, it is possible to allow some SDO errors before escalating to error
 - `max_safety_stop_attempts` [*int*, default: 20]: how many attempts to activate safety stop will be taken before failing
 - `max_write_sdo_errors_count` [*int*, default: 2]: how many consecutive errors can happen before escalating to general error
 - `max_read_sdo_errors_count` [*int*, default: 2]: how many consecutive errors can happen before escalating to general error
 - `max_read_pdo_errors_count` [*int*, default: 2]: how many consecutive errors can happen before escalating to general error

> [!CAUTION]
> `max_write_sdo_errors_count`, `max_read_sdo_errors_count`, `max_read_pdo_errors_count`, `max_safety_stop_attempts`, `sdo_operation_timeout_ms` and `pdo_feedback_timeout_ms` are safety-critical parameters, they should be changed only in very specific cases, be sure that you know how they work and be really cautious when changing them.

## Code structure

The code structure is described in more detail in a [separate file](./CODE_STRUCTURE.md).

## Generating CAN config

Adjust your configuration and generate a new `master.dcf` using:
`dcfgen canopen_configuration.yaml -r`

## Setup

<!-- todo: automate and move it to CMakeLists -->

### Lely CANopen installation

```bash
sudo apt-get update && \
sudo apt-get install -y software-properties-common && \
sudo add-apt-repository ppa:lely/ppa && \
sudo apt-get update && \
sudo apt-get install -y liblely-coapp-dev liblely-co-tools python3-dcf-tools
```

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

### Updating config
Copy eds file to config and run
```bash
dcfgen canopen_configuration.yaml -r
```
Remove master.dcf
