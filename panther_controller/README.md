# panther_controller

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

## Launch Files

- `controller.launch.py` - establishes communication with the hardware by loading the robot's URDF with plugins and configures the controllers to exchange information between the engine driver and the IMU.

## Configuration Files

- [`WH01_controller.yaml`](./config/WH01_controller.yaml) - configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller`  controllers for WH01 wheels.
- [`WH02_controller.yaml`](./config/WH02_controller.yaml) - configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller`  controllers for mecanum WH02 wheels.
- [`WH04_controller.yaml`](./config/WH04_controller.yaml) - configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller`  controllers for small pneumatic WH04 wheels.

## ROS Nodes

| Node name                 | Description <br/> *Type*                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `controller_manager`       | Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. For more information, refer to  [controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html). This node manages the: `imu_broadcaster`, `joint_state_broadcaster`, `drive_controller` <br/> *[controller_manager/controller_manager](https://github.com/ros-controls/ros2_control/blob/master/controller_manager)* |
| `drive_controller`        | Manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it. <br/> *[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)*                                                                                                                                                                                                                                                                                        |
| `imu_broadcaster`         | Publishes readings of IMU sensors. <br/> *[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `joint_state_broadcaster` | Reads all state interfaces and reports them on specific topics. <br/> *[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)*                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `robot_state_publisher`   | Broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics. <br/> *[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)*                                                                                                                                                                                                                                                                                                                                                              |

### controller_manager

#### Parameters

A detailed explanation of the parameters can be found in the ros2 controllers package documentation for [controller_manager](https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html).

### imu_broadcaster

#### Publishers

- `imu/data` [*sensor_msgs/msg/Imu*]: data from IMU sensor.

#### Parameters

A detailed explanation of the parameters can be found in the ros2 controllers package documentation for [imu_sensor_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/imu_sensor_broadcaster/doc/userdoc.html).

### joint_state_broadcaster

#### Publishers

- `dynamic_joint_states` [*control_msgs/msg/DynamicJointState*] - provides information about the state of various movable joints in a robotic system.
- `joint_states` [*sensor_msgs/msg/JointState*] - provides information about the state of various joints in a robotic system.

#### Parameters

A detailed explanation of the parameters can be found in the ros2 controllers package documentation for [joint_state_broadcaster](https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html).

### drive_controller

#### Subscribers

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command linear and angular velocity values.

#### Publishers

- `/tf` [*tf2_msgs/msg/TFMessage*]: tf tree. Published only if `enable_odom_tf=true`
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: odometry data from wheel encoders.

#### Parameters

A detailed explanation of the parameters can be found in the ros2 controllers package documentation for [diff_drive_controller](https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).

### robot_state_publisher

#### Publishers

- `/robot_description` [*std_msgs/msg/String*]: contains information about robot description from URDF file.

#### Parameters

A detailed explanation of the parameters can be found in the [robot_state_publisher](https://github.com/ros/robot_state_publisher).
