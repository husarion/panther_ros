# panther_controller

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

## Launch Files

- [`controller.launch.py`](#controllerlaunchpy---arguments) - establishes communication with the hardware by loading the robot's URDF with plugins and configures the controllers to exchange information between the engine driver and the IMU.

### controller.launch.py - Arguments

| Argument                 | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                                                                        |
| ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `battery_config_path`    | Path to the Ignition LinearBatteryPlugin configuration file. This configuration is intended for use in simulations only. <br/> ***string:*** `''`                                                                                                                                                                                                              |
| `components_config_path` | Additional components configuration file. Components described in this file are dynamically included in Panther's urdf. Panther options are described [here](https://husarion.com/manuals/panther/panther-options/). <br/> ***string:*** [`components.yaml`](../panther_description/config/components.yaml)                                                    |
| `controller_config_path` | Path to controller configuration file. <br/> ***string:*** [`{wheel_type}_controller.yaml`](../panther_controller/config/)                                                     |
| `namespace`              | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                                                                                                                                |
| `publish_robot_state`    | Whether to publish the default Panther robot description.  <br/>  ***bool:*** `True` (choices: `True`, `False`)                                                                                                                                                                                                                                                |
| `use_sim`                | Whether simulation is used.  <br/>  ***bool:*** `False` (choices: `True`, `False`)                                                                                                                                                                                                                                                                             |
| `wheel_config_path`      | Path to wheel configuration file.   <br/>  ***string:*** [`{wheel_type}.yaml`](../panther_description/config)                                                                                   |
| `wheel_type`             | Type of wheel. If you choose a value from the preset options ('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path' parameters. For custom wheels, please define these parameters to point to files that accurately describe the custom wheels. <br/>  ***string:*** `WH01` (choices: `WH01`, `WH02`, `WH04`, `custom`) |

### controller.launch.py - Nodes

| Node name                 | *Type*                                                                                                                                    |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `ros2_control_node`       | *[controller_manager/controller_manager](https://github.com/ros-controls/ros2_control/blob/master/controller_manager)*                    |
| `imu_broadcaster`         | *[imu_sensor_broadcaster/imu_sensor_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/imu_sensor_broadcaster)*    |
| `joint_state_broadcaster` | *[joint_state_broadcaster/joint_state_broadcaster](https://github.com/ros-controls/ros2_controllers/tree/master/joint_state_broadcaster)* |
| `drive_controller`        | *[diff_drive_controller/diff_drive_controller](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller)*       |
| `robot_state_publisher`   | *[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)*                                             |

## ROS Nodes

- `ros2_control_node`: Controller Manager performs two main functions. First, it manages controllers and their required interfaces, handling tasks like loading, activating, deactivating, and unloading. Second, it interacts with hardware components, ensuring access to their interfaces. For more information, refer to  [controller_manager](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html). This node manages the following controllers:
  - `imu_broadcaster`: The broadcaster to publish readings of IMU sensors.
  - `joint_state_broadcaster`: The broadcaster reads all state interfaces and reports them on specific topics.
  - `drive_controller`: Controller which manages mobile robots with a differential drive. It converts velocity commands for the robot body into wheel commands for the base. It also calculates odometry from hardware feedback and shares it.
- `robot_state_publisher`: The Robot State Publisher broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics.

### imu_broadcaster

#### Publishers

- `imu/data` [*sensor_msgs/msg/Imu*]: data from IMU sensor.

### joint_state_broadcaster

#### Publishers

- `dynamic_joint_states` [*control_msgs/msg/DynamicJointState*] - provides information about the state of various movable joints in a robotic system.
- `joint_states` [*sensor_msgs/msg/JointState*] - provides information about the state of various joints in a robotic system.

### drive_controller

#### Subscribers

- `cmd_vel` [*geometry_msgs/msg/Twist*]: command linear and angular velocity values.

#### Publishers

- `/tf` [*tf2_msgs/msg/TFMessage*]: tf tree. Published only if `enable_odom_tf=true`
- `odometry/wheels` [*nav_msgs/msg/Odometry*]: odometry data from wheel encoders.

### robot_state_publisher

#### Publishers

- `/robot_description` [*std_msgs/msg/String*]: contains information about robot description from URDF file.

## Configuration

### Changing Wheel Type

It is possible to change Panther wheels model in simulation. All you need to do is to point to new wheel and controller configuration files using `wheel_config_path` and `controller_config_path` parameters. These files should be based on the default ones, i.e., [WH01_controller.yaml](https://github.com/husarion/panther_ros/panther_controller/config/WH01_controller.yaml) and [WH01.yaml](https://github.com/husarion/panther_ros/panther_description/config/WH01.yaml).
