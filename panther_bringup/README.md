[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_bringup

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## Default Nodes Launched

In cases where different nodes are used depending on the robot version, **<= 1.06** means a node is used in Husarion Panther 1.06 and older versions. **>= 1.2** means Husarion Panther 1.2 and newer is expected.

- `battery_node` [<= 1.06 *[panther_battery/roboteq_republisher_node.py](../panther_battery/src/roboteq_republisher_node.py)*, >= 1.2 *[panther_battery/adc_node.py](../panther_battery/src/adc_node.py)*]: node responsible for monitoring and publishing the internal Battery state of the Husarion Panther robot. For more information, refer to [panther_battery](../panther_battery/README.md).
- `driver_node` [*[panther_driver/driver_node.py](../panther_driver/src/driver_node.py)*]: node responsible for communication with motor controllers and computing the inverse and forward kinematics of a robot. For more information, refer to [panther_driver](../panther_driver/README.md).
- `ekf_node` [*[robot_localization/ekf_localization_node](https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/src/ekf_localization_node.cpp)*]: Extended Kalman Filter node for more accurate odomtery. For more information, refer to [robot_localization](https://github.com/cra-ros-pkg/robot_localization/tree/noetic-devel). The default configuration is stored in [ekf_config.yaml](./config/ekf_config.yaml).
- `imu_filter_node` [*[imu_filter_madgwick/imu_filter_node](https://github.com/CCNYRoboticsLab/imu_tools/blob/noetic/imu_filter_madgwick/src/imu_filter_node.cpp)*]: node responsible for filtering and fusing raw data from IMU.
- `lights_controller_node` [*[panther_lights/controller_node.py](../panther_lights/src/controller_node.py)*]: node responsible for processing animations and publishing frames to be displayed on the Husarion Panther robot LED panels. For more information, refer to [panther_lights](../panther_lights/README.md).
- `lights_driver_node` [*[panther_lights/driver_node](../panther_lights/src/driver_node.cpp)*]: node responsible for displaying frames on the Husarion Panther robot LED panels. For more information, refer to [panther_lights](../panther_lights/README.md).
- `manager_bt_node` [*[panther_manager/manager_bt_node](../panther_manager/src/manager_bt_node.cpp)*]: node responsible for managing the Husarion Panther robot. Incorporates a system for managing displayed LED panel animations, safety features, and software shutdown of components. For more information, refer to [panther_manager](../panther_manager/README.md).
- `phidgets_spatial_node` [*nodelet/nodelet*, type: *[phidgets_spatial/PhidgetsSpatialNodelet](https://github.com/ros-drivers/phidgets_drivers/blob/noetic/phidgets_spatial/src/phidgets_spatial_nodelet.cpp)*]: Phidget Spatial IMU ROS driver.
- `power_control_node` [<= 1.06 *[panther_power_control/relays_node.py](../panther_power_control/src/relays_node.py)*, >= 1.2 *[panther_power_control/power_board_node.py](../panther_power_control/src/power_board_node.py)*]: node responsible for power management of the Husarion Panther robot. For more information, refer to [panther_power_control](../panther_power_control/README.md).
- `robot_state_publisher` [*[robot_state_publisher/robot_state_publisher](https://github.com/ros/robot_state_publisher/blob/noetic-devel/src/robot_state_publisher_node.cpp)*]: node publishing description and transformations of the Husarion Panther robot. For more information, refer to [panther_description](../panther_description/README.md).
- `system_status_node` [*[panther_manager/system_status_node.py](../panther_manager/src/system_status_node.py)*]: node publishing information about the status of a Build-in Computer.
- `welcome_msg_node` [*[panther_bringup/welcome_msg_node.py](../panther_bringup/src/welcome_msg_node.py)*]: user-friendly welcome message with basic information about the given robot.

## Bringup Launch Arguments

- `bt_project_file` [*string*, default: **$(find panther_manager)/config/Panther106BT.btproj**]: path to BehaviorTree project used by `panther_manager`. The default value of this parameter depends on the `panther_version` argument. For version 1.2 and above, it is **(find panther_manager)/config/Panther12BT.btproj**. Otherwise: **(find panther_manager)/config/Panther106BT.btproj**
- `disable_manager` [*bool*, default: **false**]: allows disabling `panther_manager`, used for testing purposes.
- `exit_on_wrong_hw` [*bool*, default: **true**]: if set to **true** and incorrect hardware is detected, the entire roslaunch is killed. If set to **false** does not launch nodes and only spins the dummy `welcome_msg_node`.
- `namespace` [*string*, default: **panther**]: namespace to use with robot.
- `panther_common_config_file` [*string*, default: **$(find panther_bringup)/config/panther_common.yaml**]: path to YAML file with standard parameters used by the `panther_driver/driver_node`.
- `panther_version` [*float*, default: **1.0**]: robot version parsed using `PANTHER_ROBOT_VERSION` environmental variable.
- `publish_robot_state` [*bool*, default: **true**]: whether to publish the default Panther robot description.
- `robot_description` [*string*, default: **xacro $(find panther_description)/urdf/panther.urdf.xacro**]: robot description in URDF format. URDF itself contains its own arguments that are responsible for wheel selection and IMU position. If you use Xacro alongside URDF add the `xacro` command before the argument to parse the URDF file.
- `shutdown_hosts_config_file` [*string*, default: **$(find panther_bringup)/config/shutdown_hosts.yaml**]: path to YAML file with description of hosts to shutdown. For more information, see [panther_manager](../panther_manager/README.md).
- `test_animations` [*bool*, default: **false**]: enables service `/panther/lights/controller/set/image_animation` allowing to test animations based on provided images.
- `user_animations_file` [*string*, default: **None**]: optional parameter with a path to YAML file with user-defined animations. 
- `use_ekf` [*bool*, default: **true**]: enable or disable Extended Kalman Filter. Keep in mind that parameters in [panther_common.yaml](./config/panther_common.yaml) and in [ekf_config.yaml](./config/ekf_config.yaml) are separate and are not affected by this parameter. Especially, parameters such as `publish_tf` and TF frames are separate for both nodes and have to be changed independently.
- `wheel_config_file` [*string*, default: **$(find panther_description)/config/WH01.yaml**]: path to YAML file with wheel specification. Arguments become required if `wheel_type` is set to **custom**.
- `wheel_type` [*string*, default: **WH01**]: type of wheel, possible are: **WH01** - offroad, **WH02** - mecanum, **WH04** - small pneumatic, and **custom** - custom wheel types (requires setting `wheel_config_file` argument accordingly).

## Expected Launch Behavior

If the environment variable `PANTHER_HW_CONFIG_CORRECT` is set to **true** the robot will typically start. If it is set to **false** all ROS nodes will be prevented from starting. In such case, a welcome message will be displayed, followed by a warning:
```
[ERROR] [0.0]: [/welcome_msg_node] OS detected incorrect hardware. ROS nodes are prevented from starting!
[ERROR] [0:0]: [/welcome_msg_node] Refer to the instructions in the manual or those shown on the terminal login.
```

`welcome_msg_node` is a required node. Setting `exit_on_wrong_hw` will cause it to stop spinning, in the case of `PANTHER_HW_CONFIG_CORRECT` not being equal to **true**. This will result in the entire launch file being stopped. If stopping the whole roslaunch behavior is not desired (running within the Docker container), setting `exit_on_wrong_hw` to **false** will result in the expected behavior while preventing launching ROS nodes in a potentially harmful way. 

## ROS Nodes

[//]: # (ROS_API_NODE_START)

[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)

[//]: # (ROS_API_NODE_NAME_START)

### welcome_msg_node.py

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

Displays user-friendly welcome message:
```
 ____             _   _               
|  _ \ __ _ _ __ | |_| |__   ___ _ __ 
| |_) / _` | '_ \| __| '_ \ / _ \ '__|
|  __/ (_| | | | | |_| | | |  __/ |   
|_|   \__,_|_| |_|\__|_| |_|\___|_|   

Serial number: ----
Robot version: 1.0
ROS driver version: 1.1.0
Website: https://husarion.com
Support: https://community.husarion.com/
Bugtracker: https://github.com/husarion/panther_ros/issues
```

[//]: # (ROS_API_NODE_DESCRIPTION_END)

The serial number and robot version are changed according to the parameters of your real robot. If the `PANTHER_HW_CONFIG_CORRECT` is not set to **true** or does not exist, an error indicating incorrect hardware will be displayed after the welcome message.

#### Parameters

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `~exit_on_wrong_hw` [*bool*, default: **true**]: if set to **true** stops the node if incorrect hardware is detected. Otherwise, keeps spinning the node.
- `/panther/serial_no` [*string*, default: **----**]: serial number of a robot.
- `/panther/robot_version` [*string*, default: **1.0**]: robot hardware revision.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Environment Variables

- `PANTHER_HW_CONFIG_CORRECT` [*string*, default: **None**]: information whether hardware is correctly configured.

[//]: # (ROS_API_NODE_END)
[//]: # (ROS_API_PACKAGE_END)