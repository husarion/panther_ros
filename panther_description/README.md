# panther_description

The package contains URDF files responsible for creating a representation of the robot by specifying the relationships and types of connections (joints) between individual links. It also contains information about the robot's mesh.

## Launch Files

- `load_urdf.launch.py` - loads the robot's URDF and creates simple bindings to display moving joints.

### Configuration Files

- [`components.yaml`](./config/components.yaml): allows you to quickly add visualization of sensors, TF connections and simulate their behavior in the simulator.
- [`WH01.yaml`](./config/WH01.yaml): description of physical and visual parameters for the wheel WH01.
- [`WH02.yaml`](./config/WH02.yaml): description of physical and visual parameters for the wheel WH02.
- [`WH04.yaml`](./config/WH04.yaml): description of physical and visual parameters for the wheel WH04.

## ROS Nodes

| Node name               | Description <br/> *Type*                                                                                                                                                                                                                                                                                                                                                          |
| ----------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `joint_state_publisher` | Node responsible for continually publish values for all of the movable joints in the URDF to the `/joint_states` topic. In combination with robot_state_publisher, this ensures that there is a valid transform for all joints even when the joint doesn't have encoder data. <br/> *[joint_state_publisher/joint_state_publisher](https://github.com/ros/joint_state_publisher)* |
| `robot_state_publisher` | Broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics. *[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)*                                                                                                               |

### joint_state_publisher

#### Publishers

- `/joint_states` [*std_msgs/msg/String*]: contains information about robot description from URDF file.

#### Parameters

A detailed explanation of the parameters can be found in the [joint_state_publisher](https://github.com/ros/joint_state_publisher).

### robot_state_publisher

#### Publishers

- `/robot_description` [*std_msgs/msg/String*]: contains information about robot description from URDF file.

#### Parameters

A detailed explanation of the parameters can be found in the [robot_state_publisher](https://github.com/ros/robot_state_publisher).
