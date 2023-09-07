[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_driver

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

Software for controlling the Husarion Panther robot motors via CAN interface.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### driver_node.py

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

Node responsible for communication with motor controllers and computing inverse and forward kinematics of a robot. Commanded velocities are smoothed with acceleration constraints.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `/cmd_vel` [*geometry_msgs/Twist*]: robot desired control velocity.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: robot E-stop state.
- `/panther/hardware/io_state` [*panther_msgs/IOState*]: checks whether robot GPIO pins are powered on.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/joint_states` [*sensor_msgs/JointState*]: robot joints states.
- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: motor controllers current, voltage, fault flags, script flags and runtime error flags. If the robot's motors are disabled, no messages will be sent to this topic.
- `/panther/odom/wheels` [*nav_msgs/Odometry*]: robot odometry calculated from wheels.
- `/panther/pose` [*geometry_msgs/Pose*]: robot position.
- `/tf` [*tf2_msgs/TFMessage*]: transform between `odom` and `base_link` frames.

For a `/joint_states` topic, message carries given data:
- `position = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` wheel position in **[rad]**, ranging from **[-pi, pi]**.
- `velocity = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` wheels velocity in **[rad/s]**.
- `effort = [fl_wheel_joint, fr_wheel_joint, rl_wheel_joint, rr_wheel_joint]` Approximate motor torque in **[Nm]**.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `/panther/driver/reset_roboteq_script` [*std_srvs/Trigger*]: resets the script running on motor drivers. Available since Panther version 1.2.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)

#### Service Clients

[//]: # (ROS_API_NODE_SERVICE_CLIENTS_START)

- `/panther/hardware/e_stop_trigger` [*std_srvs/Trigger*]: allows to trigger robot E-stop.

[//]: # (ROS_API_NODE_SERVICE_CLIENTS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)

- `~base_link_frame` [*string*, default: **base_link**]: the name of the `base_link` frame.
- `~can_interface` [*string*, default: **panther_can**]: the name of the socket CAN interface.
- `~eds_file` [*string*, default: **None**]: required path to eds file containing CANopen configuration for the Roboteq motor controllers.
- `~encoder_resolution` [*int*, default: **1600**]: resolution of motor encoder in **[PPR]**.
- `~gear_ratio` [*float*, default: **30.08**]: wheel gear ratio.
- `~kinematics` [*string*, default: **differential**]: kinematics type, possible are **differential** and **mecanum**.
- `~motor_torque_constant` [*float*, default: **2.6149**]: constant used to estimate torque.
- `~odom_frame` [*string*, default: **odom**]: the name of the odom frame.
- `~odom_stderr/vel_x` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~odom_stderr/vel_y` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~odom_stderr/vel_yaw` [*float*, default: **8.5e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `~publish_joints` [*bool*, default: **true**]: whether to publish robot `joints_states`.
- `~publish_odometry` [*bool*, default: **true**]: whether to publish robot odometry.
- `~publish_pose` [*bool*, default: **true**]: whether to publish robot pose.
- `~publish_tf` [*bool*, default: **true**]: whether to publish transform between `odom` and `base_link` frames.
- `~robot_length` [*float*, default: **0.44**]: distance between wheels alongside **X** axis in **[m]**.
- `~use_pdo` [*bool*, default: **false**]: whether to use Process Data Object protocol to acquire motors drivers data via CAN interface. Available since Panther version 1.2.
- `~wheel_radius` [*float*, default: **0.1825**]: wheel radius in **[m]**.
- `~wheel_separation` [*float*, default: **0.697**]: separation of wheels alongside **Y** axis in **[m]**.
- `velocity_x_stderr` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `velocity_y_stderr` [*float*, default: **3.2e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
- `velocity_yaw_stderr` [*float*, default: **8.5e-3**]: standard error used to place in covariance matrix, after squaring, in odometry message.
<br/><br/>
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

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)
[//]: # (ROS_API_PACKAGE_END)

#### Kinematics Type - Explanation

The Panther robot can be configured with different wheels to match your needs, we provide 2 different kinematics types **differential** / **mecanum**. You can change the wheel type by providing an appropriate launch parameter with a path to the wheel configuration file - `wheel_config_file`. Basic wheel configuration files (*WH01.yaml, WH02.yaml, WH04.yaml*): are located in `panther_description` package.
