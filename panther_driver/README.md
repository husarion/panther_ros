# Panther driver

Software for controlling the Husarion Panther robot motors via CAN interface.

## ROS API

### Publishes

- `/joint_states` [*sensor_msgs/JointState*]: robot joints states.
- `/motor_controller_state` [*panther_msgs/DriverState*]: motor controllers current, voltage, fault flags, script flags and runtime error flags.
- `/odom/wheel` [*nav_msgs/Odometry*]: robot odometry calculated from wheels.
- `/pose` [*geometry_msgs/Pose*]: robot position.
- `/tf` [*tf2_msgs/TFMessage*]: transform between `odom_frame` and `base_link_frame`.

For a `/joint_states` message is crying given data:

- `position = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Encoder pulses.
- `velocity = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Encoder pulses per second.
- `effort = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Approximate motor torque in Nm.

### Subscribe
- `/cmd_vel` [*geometry_msgs/Twist*]: robot desired control velocity.
- `/panther_hardware/e_stop` [*std_msgs/Bool*]: robot emergency stop state.

### Service clients

- `/panther_hardware/e_stop_trigger` [*std_srvs/Trigger*]: allows to trigger robot emergency stop.

### Parameters

- `~base_link_frame` [*string*, default: **'base_link'**]: the name of the base link frame.
- `~can_interface` [*string, default: **'panther_can'**]: the name of the socket CAN interface.
- `~eds_file` [*string*]: path to eds file containing CANopen configuration for Roboteq motor controllers.
- `~encoder_resolution` [*int*, default: **1600**]: resolution of motor encoder in **[PPR]**.
- `~gear_ratio` [*float*, default: **30.08**]: wheel gear ratio.
- `~kinematics` [*string*, default: **'differential'**]: kinematics type, possible are: differential, mecanum.
- `~motor_torque_constant` [*float*, default: **2.6149**]: constant used to estimate torque.
- `~odom_frame` [*string*, default: **'odom'**]: the name of the odom frame.
- `~publish_joints` [*boolean*, default: **True**]: whether to publish robot joints states.
- `~publish_odometry` [*boolean*, default: **True**]: whether to publish robot odometry.
- `~publish_pose` [*boolean*, default: **True**]: whether to publish robot pose.
- `~publish_tf` [*boolean*, default: **True**]: whether to publish transform between `odom_frame` and `base_link_frame`.
- `~robot_length` [*float*, default: **0.44**]: distance between wheels alongside x axis in **[m]**.
- `~use_pdo` [*boolean*, default: **false**]: whether to use Process Data Object protocol to acquire motors drivers data via CAN interface. Available since Panther version 1.2.
- `~wheel_radius` [*float*, default: **0.1825**]: wheel radius in **[m]**.
- `~wheel_separation` [*float*, default: **0.697**]: separation of wheels alongside y axis in **[m]**.

### Kinematics type

The Panther robot can be configured with different wheels to match your needs, we provide 2 different kinematics types `differential`/`mecanum`. You can change the wheel type by providing an appropriate launch parameter with a path to the wheel configuration file - `wheel_config_file`. Basic wheel configuration files (*WH01.yaml, WH04.yaml*): are located in `panther_description` package.
