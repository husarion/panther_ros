# panther_lights

Package used to control the Husarion Panther Bumper Lights.

## Launch files

This package contains:

- `lights.launch.py`: Responsible for launching the nodes required to control the Panther Bumper Lights.

## Configuration Files

- [`led_config.yaml`](./config/led_config.yaml): Defines and describes the appearance and parameters of the animations.

## ROS Nodes

### ControllerNode

This node is of type rclcpp_components is responsible for processing animations and publishing frames to be displayed on the Husarion Panther robot Bumper Lights.

#### Publishers

- `lights/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: An animation frame to be displayed on robot Front Bumper Lights.
- `lights/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: An animation frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `lights/set_animation` [*panther_msgs/SetLEDAnimation*]: Allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `~controller_frequency` [*float*, default: **50.0**]: Frequency [Hz] at which the lights controller node will process animations.
- `~led_config_file` [*string*, default: **$(find panther_lights)/panther_lights/config/led_config.yaml**]: Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations.
- `~user_led_animations_file` [*string*, default: **None**]: Path to a YAML file with a description of the user defined animations.

### DriverNode

This node is of type rclcpp_components is responsible for displaying frames on the Husarion Panther robot's Bumper Lights.

#### Publishers

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Lights diagnostic messages.

#### Subscribers

- `lights/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Front Bumper Lights.
- `lights/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `lights/set_brightness` [*panther_msgs/SetLEDBrightness*]: Allows setting global LED brightness, value ranges from **0.0** to **1.0**.

#### Service Clients

- `hardware/led_control_enable` [*std_srvs/SetBool*]: Makes SBC controlling LEDs.

#### Parameters

- `~frame_timeout` [*float*, default: **0.1**]: Time in **[s]** after which an incoming frame will be considered too old.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness. The range between **[0.0, 1.0]**.
- `~num_led` [*int*, default: **46**]: Number of LEDs in a single bumper.
