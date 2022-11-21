# Panther driver

Software for controlling the Husarion Panther robot motors via CAN interface.

## ROS API

### Publish
  - `/joint_states` *(sensor_msgs/JointState)* - robot joints states.
  - `/odom/wheel` *(nav_msgs/Odometry)* - robot odometry calculated from wheels.
  - `/pose` *(geometry_msgs/Pose)* - robot position.
  - `/tf` *(tf2_msgs/TFMessage)* - transform between `odom_frame` and `base_link_frame`.
  - `/motor_controller_state` *(panther_msgs/DriverState)* - motor controllers current, voltage, fault flags, script flags and runtime error flags

For a `/joint_states` message is crying given data:

- `position = [Front left, Front right, Rear left, Rear right]` - Encoder pulses.
- `velocity = [Front left, Front right, Rear left, Rear right]` - Encoder pulses per second.
- `effort = [Front left, Front right, Rear left, Rear right]` - Approximate motor torque in Nm.

### Subscribe
- `/cmd_vel` *(geometry_msgs/Twist)* - robot desired control velocity.
- `/panther_hardware/e_stop` *(std_msgs/Bool)* - robot emergency stop state.

### Service clients

- `/panther_hardware/e_stop_trigger` *(std_srvs/Trigger)* - allows to trigger robot emergency stop.

### Parameters

- `~eds_file` *(string)* - path to eds file containing CANopen configuration for Roboteq motor controllers.
- `~can_interface` *(string, default: 'panther_can')* - the name of the CAN interface.
- `~kinematics` *(string, default: 'differential')* - kinematics type, possible are: differential, mecanum.
- `~motor_torque_constant` *(float, default: 2.6149)* - motor torque constant.
- `~gear_ratio` *(float, default: 30.08)* - wheel gear ratio.
- `~encoder_resolution` *(int, default: 1600)* - resolution of motor encoder in **[PPR]**.
- `~odom_frame` *(string, default: 'odom')* - the name of the odom frame.
- `~base_link_frame` *(string, default: 'base_link')* - the name of the base link frame.
- `~publish_tf` *(boolean, default: True)* - whether to publish transform between `odom_frame` and `base_link_frame`.
- `~publish_odometry` *(boolean, default: True)* - whether to publish robot odometry.
- `~publish_pose` *(boolean, default: True)* - whether to publish robot pose.
- `~publish_joints` *(boolean, default: True)* - whether to publish robot joints states.
- `~wheel_separation` *(float, default: 0.697)* - separation of wheels alongside y axis in **[m]**.
- `~robot_length` *(float, default: 0.44)* - distance between wheels alongside x axis in **[m]**
- `~wheel_radius` *(float, default: 0.1825)* - wheel radius in **[m]**.
- `~power_factor` *(float, default: 0.04166667)* - motor controllers power factor.

### Kinematics type

The Panther robot can be configured with different wheels to match your needs, we provide 2 different kinematics types `differential`/`mecanum`. You can change the wheel type by providing an appropriate launch parameter with a path to the wheel configuration file - `wheel_config_file`. Basic wheel configuration files *(WH01.yaml, WH04.yaml)* are located in `panther_description` package.

## CAN bus

Documentation for USB-CAN converter:
https://ucandevices.github.io/uccb.html#!#socketCAN

### CAN bitrate
Slcan tool take `-sX` argument to set the CAN bitrate. The below table contains valid values.

| ASCII Command | CAN Bitrate |
| ---           | ---         |
| s0            | 10 Kbit/s   |
| s1            | 20 Kbit/s   |
| s2            | 50 Kbit/s   |
| s3            | 100 Kbit/s  |
| s4            | 125 Kbit/s  |
| s5            | 250 Kbit/s  |
| s6            | 500 Kbit/s  |
| s7            | 800 Kbit/s  |
| s8            | 1000 Kbit/s |
