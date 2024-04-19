[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_controller

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## Default Nodes Launched

- `ros2_control_node` [*[controller_manager/controller_manager](https://github.com/ros-controls/ros2_control/blob/master/controller_manager)*]: Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. For more information, refer to  [controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html). This node manages the following controllers:
  - `imu_broadcaster` [*[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*]: The broadcaster to publish readings of IMU sensors.
  - `joint_state_broadcaster` [*[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)*]: The broadcaster reads all state interfaces and reports them on specific topics.
  - `panther_base_controller` [*[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)* ]: Controller which manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it.
- `robot_state_publisher` [*[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)*]: The Robot State Publisher broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics.

[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

## External ROS Nodes

[//]: # (ROS_API_PACKAGE_NAME_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### imu_broadcaster

[//]: # (ROS_API_NODE_NAME_END)

[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type:*[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*.

The broadcaster to publish readings of IMU sensors.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/imu_broadcaster/imu` [*sensor_msgs/msg/Imu*]: data from IMU sensor.

[//]: # (ROS_API_NODE_PUBLISHERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### joint_state_broadcaster

[//]: # (ROS_API_NODE_NAME_END)

[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type:*[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)*.

The broadcaster reads all state interfaces and reports them on specific topics.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/dynamic_joint_states` [*control_msgs/msg/DynamicJointState*] - provides information about the state of various movable joints in a robotic system.
- `~/joint_states` [*sensor_msgs/msg/JointState*] - provides information about the state of various joints in a robotic system.

[//]: # (ROS_API_NODE_PUBLISHERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### panther_base_controller

[//]: # (ROS_API_NODE_NAME_END)

[//]: # (ROS_API_NODE_DESCRIPTION_START)

External node type:*[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)*.

Controller which manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `~/cmd_vel` [*geometry_msgs/msg/Twist*]: command linear and angular velocity values.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~/panther_base_controller/odom` [*nav_msgs/msg/Odometry*]: odometry data from wheel encoders.

[//]: # (ROS_API_NODE_PUBLISHERS_END)
[//]: # (ROS_API_NODE_END)

[//]: # (ROS_API_PACKAGE_END)
