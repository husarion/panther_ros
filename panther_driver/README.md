# Panther driver

Software for controlling the Husarion Panther robot motors via CAN interface.

## ROS Nodes

### driver_node.py

Node responsible for communication with motor controllers and computing inverse and forward kinematics of a robot.

#### Publishes

- `/joint_states` [*sensor_msgs/JointState*]: robot joints states.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: motor controllers current, voltage, fault flags, script flags and runtime error flags.
- `/panther/odom/wheels` [*nav_msgs/Odometry*]: robot odometry calculated from wheels.
- `/panther/pose` [*geometry_msgs/Pose*]: robot position.
- `/tf` [*tf2_msgs/TFMessage*]: transform between `odom_frame` and `base_link_frame`.

For a `/joint_states` message is carrying given data:
- `position = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Encoder pulses.
- `velocity = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Encoder pulses per second.
- `effort = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Approximate motor torque in Nm.

#### Subscribe

- `/cmd_vel` [*geometry_msgs/Twist*]: robot desired control velocity.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: robot emergency stop state.

#### Services subscribed

- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: allows to trigger robot emergency stop.

#### Services advertised

- `/panther/driver/reset_roboteq_script` [*std_srvs/Trigger*]: allows rest script running on motor drivers. Available since Panther version 1.2.

#### Parameters

- `~base_link_frame` [*string*, default: **base_link**]: the name of the base link frame.
- `~can_interface` [*string*, default: **panther_can**]: the name of the socket CAN interface.
- `~eds_file` [*string*, default: **None**]: required path to eds file containing CANopen configuration for Roboteq motor controllers.
- `~encoder_resolution` [*int*, default: **1600**]: resolution of motor encoder in **[PPR]**.
- `~gear_ratio` [*float*, default: **30.08**]: wheel gear ratio.
- `~kinematics` [*string*, default: **differential**]: kinematics type, possible are: differential, mecanum.
- `~motor_torque_constant` [*float*, default: **2.6149**]: constant used to estimate torque.
- `~odom_frame` [*string*, default: **odom**]: the name of the odom frame.
- `~odom_stderr/vel_x` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~odom_stderr/vel_y` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~odom_stderr/vel_yaw` [*float*, default: **8.5e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~publish_joints` [*bool*, default: **true**]: whether to publish robot joints states.
- `~publish_odometry` [*bool*, default: **true**]: whether to publish robot odometry.
- `~publish_pose` [*bool*, default: **true**]: whether to publish robot pose.
- `~publish_tf` [*bool*, default: **true**]: whether to publish transform between `odom_frame` and `base_link_frame`.
- `~robot_length` [*float*, default: **0.44**]: distance between wheels alongside x axis in **[m]**.
- `~use_pdo` [*bool*, default: **false**]: whether to use Process Data Object protocol to acquire motors drivers data via CAN interface. Available since Panther version 1.2.
- `~wheel_radius` [*float*, default: **0.1825**]: wheel radius in **[m]**.
- `~wheel_separation` [*float*, default: **0.697**]: separation of wheels alongside y axis in **[m]**.

#### Velocity smoother parameters

- `~max_vel_x` [*float*, default: **2.0**]: maximum linear vlelocity in **X** direction. 
- `~max_vel_y` [*float*, default: **2.0**]: maximum linear vlelocity in **Y** direction.
- `~max_vel_theta` [*float*, default: **4.0**]: maximum angular velocity.
- `~acc_lim_x` [*float*, default: **1.0**]: maximum linear acceleration in **X** direction.
- `~acc_lim_y` [*float*, default: **1.0**]: maximum linear acceleration in **Y** direction.
- `~acc_lim_theta` [*float*, default: **1.57**]: maximum angular acceleration.
- `~decel_lim_x` [*float*, default: **1.5**]: maximum linear decelaration in **X** direction.
- `~decel_lim_y` [*float*, default: **1.5**]: maximum linear decelaration in **Y** direction.
- `~decel_lim_theta` [*float*, default: **2.3**]: maximum angular deceleration.
- `~emergency_decel_lim_x` [*float*, default: **2.7**]: maximum linear decelaration in **X** direction when a timeout is reached for velocity commands.
- `~emergency_decel_lim_y` [*float*, default: **2.7**]: maximum linear decelaration in **Y** direction when a timeout is reached for velocity commands.
- `~emergency_decel_lim_theta` [*float*, default: **5.74**]: maximum angular deceleration when a timeout is reached for velocity commands.

#### Kinematics type - explanation

The Panther robot can be configured with different wheels to match your needs, we provide 2 different kinematics types `differential`/`mecanum`. You can change the wheel type by providing an appropriate launch parameter with a path to the wheel configuration file - `wheel_config_file`. Basic wheel configuration files (*WH01.yaml, WH02.yaml, WH04.yaml*): are located in `panther_description` package.
