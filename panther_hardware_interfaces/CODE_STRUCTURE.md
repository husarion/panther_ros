# Structure

A brief introduction to the code structure of the panther system.

## RoboteqDriver

Low-level CANopen driver implementing LoopDriver from [Lely](https://opensource.lely.com/canopen/).
It takes care of translating CANopen indexes into meaningful data.
Provided methods can be used for sending commands and reading all the useful parameters from the Roboteq drivers (they abstract low level SDO and PDO communication).
Timestamp of all received PDO data is also saved, which can be later used for detecting timeout errors.

## CanopenController

Takes care of CANopen communication initialization - creates master controller and two Roboteq drivers (front and rear). For handling CANopen communication separate thread is created with configurable RT priority (additionally two threads for each driver is also created).

## MotorsController

It abstract usage of two Roboteq controllers:
* uses `canopen_controller` for communication with Roboteq controllers
* implements the activate procedure for controllers - resets script and sends initial 0 command.
* provides methods to get data feedback and send commands. Data is converted between raw Roboteq formats and SI units using `roboteq_data_converters`

## RoboteqDataConverters

Provides a few classes for converting data in raw Roboteq formats read from Roboteq drivers into appropriate units or message formats. It can be divided into two types, command and data feedback. The command provides one utility function that converts a command in rad/s into a Roboteq command and returns it:
* `RoboteqVeloctiyCommandConverter`

Data feedback converters also store data (it is passed using Set methods, and later converted data can be read using Get data).
* `MotorState` - converts position, velocity and torque feedback
* `FaultFlag`, `ScriptFlag`, `RuntimeError` - converts flag error data into messages
* `DriverState` - temperature, voltage, and current

Feedback converters are combined in the `RoboteqData` class to provide the full state of one controller. It consists of
* 2 `MotorState` (left and right)
* `FaultFlag`, `ScriptFlag`
* 2 `RuntimeError` (for left and right motors)
* `DriverState`

## RoboteqErrorFilter

A class that keeps track of different types of errors. In some rare cases, Roboteq controllers can miss for example the SDO response, or PDO can be received a bit later, which results in a timeout.
As they usually are rare and singular occurrences, it is better to filter some of these errors and escalate only when a certain number of errors happen.

## GPIODriver

WIP - it will handle reading/writing pins of the RPi GPIO.

## PantherSystemRosInterface

A class that takes care of additional ROS interface of panther system, such as publishing driver state and providing service for clearing errors.

## PantherSystem

Main class that implements SystemInterface from ros2_control (for details refer to the [ros2_control documentation](https://control.ros.org/master/index.html)).
Handles transitions (initialization, activation, shutdown, error, etc.), provides interfaces for feedback (position, velocity, effort) and commands (velocity).
In the main loop controller should call read and write functions to communicate with motor drivers.
