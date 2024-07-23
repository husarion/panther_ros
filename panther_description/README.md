# panther_description

The package contains URDF files responsible for creating a representation of the robot by specifying the relationships and types of connections (joints) between individual links. It also contains information about the robot's mesh.

## Launch Files

- `load_urdf.launch.py` - loads the robot's URDF and creates simple bindings to display moving joints.

### load_urdf.launch.py - Nodes

| Node name               | *Type*                                                                                        |
| ----------------------- | --------------------------------------------------------------------------------------------- |
| `joint_state_publisher` | *[joint_state_publisher/joint_state_publisher](https://github.com/ros/joint_state_publisher)* |
| `robot_state_publisher` | *[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher)* |

## ROS Nodes

- `joint_state_publisher` this node will continually publish values for all of the movable joints in the URDF to the `/joint_states` topic. In combination with robot_state_publisher, this ensures that there is a valid transform for all joints even when the joint doesn't have encoder data.
- `robot_state_publisher` broadcasts a robot's state to tf2 using a provided URDF model and joint states. It updates the model and broadcasts poses for fixed and movable joints to tf2 topics.

### joint_state_publisher

#### Publishers

- `/joint_states` [*std_msgs/msg/String*]: contains information about robot description from URDF file.

### robot_state_publisher

#### Publishers

- `/robot_description` [*std_msgs/msg/String*]: contains information about robot description from URDF file.
