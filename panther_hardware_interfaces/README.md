# panther_hardware_interfaces

Package that implements SystemInterface from ros2_control for Panther.

## Structure

A brief introduction on code structure of panther system. 

### `roboteq_driver`

Low level CANopen driver implementing FiberDriver from [Lely](https://opensource.lely.com/canopen/) ([here](https://en.wikipedia.org/wiki/Fiber_%28computer_science%29) you can read more about fibers). It takes care of translating CANopen indexes into meaningful data. It handles PDO and SDO communication and provides methods for sending commands and reading all the useful parameters from the Roboteq drivers. It saves timestamp of last RPDO, which can be later used to detect timeout errors.

### `canopen_controller`
Takes care of CANopen communication - creates master controller and two Roboteq drivers (front and rear) - intializaiton. For handling CANopen communication separate thread is created.  

### `panther_wheels_controller`

It abstract usage of two Roboteq controllers:
* uses `canopen_controller` for communication with Roboteq controllers
* implements activate procedure for controllers - resets script and sends initial 0 command.
* provides methods to get data feedback and send commands. Data is converted between raw Roboteq formats and SI units using `roboteq_data_converters`

### `roboteq_data_converters`

Provides a few classes for converting data in raw Roboteq formats read from Roboteq drivers into appropriate units or message formats. It can be divided into two types, command and data feedback. Command provides one utlity function that converts command in rad/s into Roboteq command and returns it:
* `RoboteqCommandConverter`

Data feedback converters also store data (it is passed using Set methods, and later converted data can be read using Get data).
* `MotorState` - converts position, velocity and torque feedback
* `FaultFlag`, `ScriptFlag`, `RuntimeError` - converts flag error data into messages
* `DriverState` - temperature, voltage and current

Feedback converters are combined in the `RoboteqData` class to provide full state of one controller. It consists of 
* 2 `MotorState` (left and right)
* `FaultFlag`, `ScriptFlag`
* 2 `RuntimeError` (for left and right motors)
* `DriverState`

### `canopen_error_filter`

Class that keeps track of different types of errors. In some rare cases Roboteq controllers can miss for example the SDO response, or PDO can be received a bit later, which results in timeout. 
As it usually are rare and singular occurences, it is better to filter some of this errors, and escalate only when certain number of errors happen.

### `gpio_driver`

WIP - it will handle reading/writing pins of the RPi GPIO.

### `panther_system`

Main class that implements SystemInterface from ros2_control (for details refer to the [ros2_control documetation](https://control.ros.org/master/index.html)).

Apart from usual ros2_control interface, it also creates node to provide additional functionalities:
* publishing current drivers state
* service for clearing errors

<!-- TODO: when exception is thrown it is not RT safe -->

## ROS Nodes

This package doesn't contain any nodes - it is used as a plugin within controller manager. To use this hardware interface you have to add it to your URDF (you can check how to do it in `panther_description`) and add controller (example configuration provided in `panther_controller` package).

### Publishers
- `/panther_system_node/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags

### Services
- `/panther_system_node/clear_errors` [*std_srvs/Trigger*]: clear current errors

## panther_hardware_interface parameters

Parameters that are required, they are defined when including interface in URDF (you can check out panther_macro.urdf.xacro).

Physical properties
 - `encoder_resolution` [*int*, default: 1600] - property of the encoder used, shouldn't be changed
 - `gear_ratio` [*float*, default: 30.08] - property of the gearbox used, shouldn't be changed
 - `motor_torque_constant` [*float*, default: 0.11] - same as set in the Roboteq driver (TNM parameter), also shouldn't be changed, as it is measured property of the motor 
 - `max_rpm_motor_speed` [*float*, default: 3600.0] - max RPM speed set in the Roboteq driver (MXRPM parameter)
 - `gearbox_efficiency` [*float*, default: 0.75] - measured efficiency, used for converting read current to torque, can very depending on different factors such as temperature and wear

CAN settings
 - `master_can_id` [*int*, default: 3] - CAN ID of the master device (set as in `panther_can.yaml`)
 - `front_driver_can_id` [*int*, default: 1] - CAN ID defined in the properties of Roboteq (set as in `panther_can.yaml`)
 - `rear_driver_can_id` [*int*, default: 2] - CAN ID defined in the properties of Roboteq (set as in `panther_can.yaml`)
 - `sdo_operation_timeout` [*int*, default: 4 [ms]] - it is set so that full controller loop takes up to required time. Each controller loop contains of two SDO operations (one write and one read). For example in 100Hz loop there is up to 10ms for every operation. This timeout should be set so that in worst case everything takes 10ms.
 - `pdo_feedback_timeout` [*int*, default: 15 [ms]]  - depends on frequnecy at which Roboteq is configured to send PDO data. At 100Hz there should be 10ms between received data, if it takes more than `pdo_feedback_timeout`, PDO read error is triggered
 - `roboteq_initialization_attempts` [*int*, default: 5] - in some cases SDO error happen during initialization, it is possible to configure more attempts, before escaliting to error
 - `roboteq_activation_attempts` [*int*, default: 5] - similat to initilizaiton, it is possible to allow some SDO errors before escaliting to error
 - `max_write_sdo_errors_count` [*int*, default: 2] - how many consecutive errors can happen before escaliting to general error
 - `max_read_sdo_errors_count` [*int*, default: 2] - how many consecutive errors can happen before escaliting to general error
 - `max_read_pdo_errors_count` [*int*, default: 2] - how many consecutive errors can happen before escaliting to general error

## Generating CAN config

Adjust your configuration and generate new `master.dcf` using:
`dcfgen panther_can.yaml -r`

## Setup

<!-- TODO: automate and move it to CMakeLists -->

### Lely CANopen installation
sudo apt-get update && \
sudo apt-get install -y software-properties-common && \
sudo add-apt-repository ppa:lely/ppa && \
sudo apt-get update && \
sudo apt-get install -y liblely-coapp-dev liblely-co-tools python3-dcf-tools

### RT
(information from ros2 control)

If you have a realtime kernel installed, the main thread of Controller Manager attempts to configure ``SCHED_FIFO`` with a priority of ``50``.
By default, the user does not have permission to set such a high priority.
To give the user such permissions, add a group named realtime and add the user controlling your robot to this group:

.. code-block:: console

    $ sudo addgroup realtime
    $ sudo usermod -a -G realtime $(whoami)

Afterwards, add the following limits to the realtime group in ``/etc/security/limits.conf``:

.. code-block:: console

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

The limits will be applied after you log out and in again.

## Testing

### Setup

First it is necessary to set up virtual CAN:

<!-- TODO move setup somewhere so test can be run more easily -->

```
sudo modprobe vcan
sudo ip link add dev panther_can type vcan
sudo ip link set up panther_can
sudo ip link set panther_can down
sudo ip link set panther_can txqueuelen 1000
sudo ip link set panther_can up
```

### Runing tests

```
colcon build --packages-select panther_hardware_interfaces --symlink-install
colcon test --event-handlers console_direct+ --packages-select panther_hardware_interfaces --parallel-workers 1
colcon test-result --verbose --all
```

TODO comment
--parallel-workers 1

### Updating config
Copy eds file to config and run 
`dcfgen panther_can.yaml -r`
Remove master.dcf


<!-- TODO torque control, not used currently, move it to some other place -->
  <!-- // Converts desired wheel torque in Nm to Roboteq motor command. Steps:
  // 1. Convert desired wheel Nm torque to motor Nm ideal torque (multiplication by (1.0/gear_ratio))
  // 2. Convert motor Nm ideal torque to motor Nm real torque (multiplication by (1.0/gearbox_efficiency))
  // 3. Convert motor Nm real torque to motor A current (multiplication by (1.0/motor_torque_constant))
  // 4. Convert motor A current to Roboteq GO command - permille of the Amps limit current
  //    set in the roboteq driver (ALIM parameter) - multiplication by 1000.0/max_amps_motor_current
  newton_meter_to_roboteq_cmd_ = (1.0 / drivetrain_settings.gear_ratio) *
                                 (1.0 / drivetrain_settings.gearbox_efficiency) *
                                 (1.0 / drivetrain_settings.motor_torque_constant) *
                                 (1000.0 / drivetrain_settings.max_amps_motor_current); -->