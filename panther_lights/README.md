[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_lights

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

Package used to control the Husarion Panther Bumper Lights.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### driver_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

This node is responsible for displaying frames on the Husarion Panther robot's Bumper Lights.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishes

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/diagnostics` [*diagnostic_msgs/DiagnosticArray*]: lights diagnostic messages.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Subscribers

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `/panther/lights/driver/front_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Front Bumper Lights.
- `/panther/lights/driver/rear_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Rear Bumper Lights.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `/panther/lights/driver/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows setting global LED brightness, value ranges from **0.0** to **1.0**.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)

- `~frame_timeout` [*float*, default: **0.1**]: time in **[s]** after which an incoming frame will be considered too old.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness. The range between **[0.0, 1.0]**.
- `~num_led` [*int*, default: **46**]: number of LEDs in a single bumper.

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)
[//]: # (ROS_API_PACKAGE_END)
