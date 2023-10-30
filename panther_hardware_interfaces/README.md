# panther_hardware_interfaces

Package that implements SystemInterface from ros2_control for Panther.

## Structure

A brief introduction on code structure of panther system. 

### `roboteq_driver`

Low level CANopen driver implementing FiberDriver from [Lely](https://opensource.lely.com/canopen/) ([here](https://en.wikipedia.org/wiki/Fiber_%28computer_science%29) you can read more about fibers). It takes care of translating CANopen indexes into meaningful data. It handles PDO and SDO communication and provides methods for sending commands and reading all the useful parameters from the Roboteq drivers. It saves timestamp of last RPDO, which can be later used to detect timeout errors.

### `panther_wheels_controller`

It abstract usage of two Roboteq controllers:
* takes care of CANopen communication - creates master controller and two Roboteq drivers (front and rear) - intializaiton. For handling CANopen communication separate thread is created.  
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

### `panther_system_error_handler`

<!-- TODO more dateiled description -->
Class that keeps track of different types of errors. It is necessary because sometimes Roboteq controllers can miss the SDO response, which results in timeout. In this case it is better to filter some of this errors, and escalate only when certain number of errors happen.

### `gpio_driver`

WIP - it will handle reading/writing pins of the RPi GPIO.

### `panther_system`

Main class that implements SystemInterface from ros2_control (for details refer to the [ros2_control documetation](https://control.ros.org/master/index.html)).

Apart from usual ros2_control interface, it also creates node to provide additional functionalities:
* publishing current drivers state
* service for clearing errors


## ROS Nodes

This package doesn't contain any nodes - it is used as a plugin within controller manager. To use this hardware interface you have to add it to your URDF (you can check how to do it in `panther_description`) and add controller (example configuration provided in `panther_controller` package).

### Publishers
- `/panther_system_node/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags

### Services
- `/panther_system_node/clear_errors` [*std_srvs/Trigger*]: clear current errors

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
colcon test --event-handlers console_direct+ --packages-select panther_hardware_interfaces
colcon test-result --verbose --all
```

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