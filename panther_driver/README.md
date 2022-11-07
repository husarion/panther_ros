# Panther driver

Software for controlling Panther robot motors via CAN interface.

## ROS API

### Publish
  - `/battery` *(sensor_msgs/BatteryState)*
  - `/joint_states` *(sensor_msgs/JointState)*
  - `/pose` *(geometry_msgs/Pose)*
  - `/tf` *(tf2_msgs/TFMessage)*
  - `/odom/wheel` *(nav_msgs/Odometry)*

For a `/joint_states` message is crying given data:

- `position = [Front left, Front right, Rear left, Rear right]` - Encoder pulses

- `velocity = [Front left, Front right, Rear left, Rear right]` - Encoder pulses per second

- `effort = [Front left, Front right, Rear left, Rear right]` - Approximate motor torque in Nm


### Subscribe
- `/cmd_vel` *(geometry_msgs/Twist)*

### Parameters

- `~use_kalman` *(boolean, default: "false")* - Enables and disables kalman filter.

- `~wheel_type` *(string, default: "WH01")* - specifies kinematics model used for calculating inverse and forward kinematics. Possible wheel types: WH01, WH02 and WH04. More about wheel types can be found in manual.


### Kinematics type

Panther can be configured with different wheels to match your needs, we provide 3 different kinematics types `classic`/`mecanum`/`mix` you can change type by selecting appropriate parameter in launch file - `wheel_type`. Mix type means mecanum wheels at the front and classic at the back.

For kalman filter setup please refer to [panther_ekf](https://github.com/adamkrawczyk/panther_ekf)

## CAN bus

Documentation for USB-CAN converter:
https://ucandevices.github.io/uccb.html#!#socketCAN

### CAN bitrate
Slcan tool take `-sX` argument to set CAN bitrate. Below table contains valid values.

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
