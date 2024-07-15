# panther_controller

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

## Default Nodes Launched

- `ros2_control_node` [*[controller_manager/controller_manager](https://github.com/ros-controls/ros2_control/blob/master/controller_manager)*]: Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. For more information, refer to  [controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html). This node manages the following controllers:
  - `imu_broadcaster` [*[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*]: The broadcaster to publish readings of IMU sensors.
  - `joint_state_broadcaster` [*[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)*]: The broadcaster reads all state interfaces and reports them on specific topics.
  - `drive_controller` [*[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)* ]: Controller which manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it.
- `robot_state_publisher` [*[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)*]: The Robot State Publisher broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics.

## External ROS Nodes

### imu_broadcaster

External node type:*[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*.

The broadcaster to publish readings of IMU sensors.

#### Publishers

- `imu/data` [*sensor_msgs/msg/Imu*]: data from IMU sensor.

### joint_state_broadcaster

External node type:*[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)*.

The broadcaster reads all state interfaces and reports them on specific topics.

#### Publishers

- `dynamic_joint_states` [*control_msgs/msg/DynamicJointState*] - provides information about the state of various movable joints in a robotic system.
- `joint_states` [*sensor_msgs/msg/JointState*] - provides information about the state of various joints in a robotic system.

### drive_controller

External node type:*[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)*.

Controller which manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it.

#### Subscribers

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command linear and angular velocity values.

#### Publishers

- `/tf` [*tf2_msgs/msg/TFMessage*]: tf tree. Published only if `enable_odom_tf=true`
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: odometry data from wheel encoders.
