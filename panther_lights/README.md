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

- `/panther/lights/driver/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Front Bumper Lights.
- `/panther/lights/driver/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Rear Bumper Lights.

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

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### controller_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

This node is responsible for processing animations and publishing frames to be displayed on the Husarion Panther robot Bumper Lights.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishers

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/panther/lights/driver/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Front Bumper Lights.
- `/panther/lights/driver/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Rear Bumper Lights.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Service Servers

[//]: # (ROS_API_NODE_SERVICE_SERVERS_START)

- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on Bumper Lights based on animation ID.

[//]: # (ROS_API_NODE_SERVICE_SERVERS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)

- `~controller_frequency` [*float*, default: **50.0**]: frequency [Hz] at which the lights controller node will process animations.
- `~led_config_file` [*string*, default: **$(find panther_lights)/panther_lights/config/led_config.yaml**]: path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations.
- `~user_led_animations_file` [*string*, default: **None**]: path to a YAML file with a description of the user defined animations.

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)
[//]: # (ROS_API_PACKAGE_END)

## LED configuration

Basic led configuration is loaded from [`led_config.yaml`](config/led_config.yaml) file. It includes definition of robot panels, virtual segments and default animations.

### Panels

The `panels` section of the YAML file lists all the physical LED panels on the robot. Each panel has two attributes:

- `channel` [*int*, default: **None**] the identifier for the LED panel. It is used to differentiate between multiple panels.
- `number_of_leds`: defines the total number of LEDs present on the panel.

### Segments

The `segments` section is used to create virtual segments on the robot by dividing the LED panels into different parts. This allows for more precise control over which LEDs are lit up for different effects or indicators. Each segment has three attributes:

- `name`: the identifier for the segment, such as "front" or "rear". It is used to differentiate between multiple segments.
- `channel`: This specifies which LED panel the segment belongs to. It have to match one of the channels defined in the `panels` section.
- `led_range`: This defines the range of LEDs within the panel that the segment covers. The range is specified as a start-end pair (e.g. 0-45). The range can be specified in reverse order (e.g. 45-0), which may be useful for wiring or orientation reasons.

### Segments map

The `segments_map` section allows creating named groups of segments on which animations can be displayed. Each entry under `segments_map` consists of a key representing the group name and a list of segments included in the group. Segment names have to match one of the segments defined in the `segments` section. By default you can use provided mapping:

- `all` [*list*, default: **None**]: Grouping both `front` and `rear` segments together.
- `front` [*list*, default: **None**]: Containing only the `front` segment.
- `rear` [*list*, default: **None**]: Containing only the `rear` segment.

### Animations

The `led_animations` section contains list with definitions for various animations that can be displayed on the LED segments. Supported keys are:

- `animations` [*list*, default: **None**]: definition of animation for each Bumper Lights. Supported keys are:
  - `type` [*string*, default **None**]: Specifies the type of animation. Default animation types are: `panther_lights::ImageAnimation`, `panther_lights::ChargingAnimation`.
  - `segments` [*string*, default **None**]: Indicates which segment mapping this particular animation applies to (e.g., all, front, rear).
  - `animation` [*yaml*, default: **None**]: An animation to be displayed on segments. The keys for the configuration of different animation types are explained in detail under the [**Animation Types**](#animation-types) section.
- `id` [*int*, default: **None**]: unique ID of an animation.
- `name` [*string*, default: **ANIMATION_<ID>**]: name of an animation. If not provided will default to **ANIMATION_<ID>**, where `<ID>` is equal to `id` parameter of the given animation.
- `priority` [*int*, default: **3**]: priority at which animation will be placed in the queue. The list below shows the behavior when an animation with a given ID arrives:
    - **1** interrupts and removes animation with priorities **2** and **3**.
    - **2** interrupts animations with priority **3**.
    - **3** adds animation to the end of the queue.
- `timeout` [*float*, default: **120.0**]: time in **[s]**, after which animation will be removed from the queue.

Default animations can be found in the table below:

| ID  | NAME              | PRIORITY | DESCRIPTION                                                 |
| :-: | ----------------- | :------: | ----------------------------------------------------------- |
| 0   | E_STOP            | 3        | red expanding from the center to the edges                  |
| 1   | READY             | 3        | green expanding from center to the edges                    |
| 2   | ERROR             | 1        | red, whole bumper blinking twice                             |
| 3   | MANUAL_ACTION     | 3        | blue expanding from the center to the edges                 |
| 4   | AUTONOMOUS_ACTION | 3        | orange expanding from center to the edges                   |
| 5   | GOAL_ACHIEVED     | 2        | purple, whole bumper blinking three times                    |
| 6   | LOW_BATTERY       | 2        | two orange stripes moving towards the center, repeats twice |
| 7   | CRITICAL_BATTERY  | 2        | two red stripes moving towards the center, repeats twice    |
| 9   | CHARGING_BATTERY  | 3        | whole bumper blinks with a duty cycle proportional to the Battery percentage. Short blinking means low Battery, and no blinking means full Battery. The color changes from red to green |

### Animation Types

#### Animation

Basic animation definition. Keys are inherited from the basic **Animation** class by all animations. Supported keys are:

- `brightness` [*float*, default: **1.0**]: animation brightness relative to APA102C driver `global_brightness`. The range between **[0.0, 1.0]**.
- `duration` [*float*, default: **None**]: duration of the animation.
- `repeat` [*int*, default: **1**]: number of times the animation will be repeated.

> [!NOTE]
> Overall display duration of an animation is a product of a single image duration and repeat count. The result of `duration` x `repeat`  can't exceed 10 **[s]**. If animation fails to fulfill the requirement it will result in an error.

#### ImageAnimation

Animation of type `panther_lights::ImageAnimation`, returns frames to display based on a supplied image. Extends `Animation` with keys:

- `color` [*int*, default: **None**]: The image is turned into grayscale, and then the color is applied with brightness from the gray image. Values have to be in HEX format. This parameter is not required.
- `image` [*string*, default: **None**]: path to an image file. Only global paths are valid. Allows using `$(find ros_package)` syntax.

#### ChargingAnimation

Animation of type `panther_lights::ChargingAnimation`, returns frame to display based on `param` value representing Battery percentage. Displays a solid color with a duty cycle proportional to the Battery percentage. The color is changing from red (Battery discharged) to green (Battery fully charged).

### Defining Animations

Users can define their own LED animations using basic animation types. Similar to basic ones, user animations are parsed using YAML file and loaded on node start. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations.

Create a YAML file with an animation description list. Example file:

```yaml
# my_awesome_user_animations.yaml
user_animations:
  # animation with default image and custom color
  - id: 21
    name: ANIMATION_1
    priority: 2
    animations:
      - type: panther_lights::ImageAnimation
        segments: all
        animation:
          image: $(find panther_lights)/animations/strip01_red.png
          duration: 2
          repeat: 2
          color: 0xffff00

  # animation with custom image
  - id: 22
    name: ANIMATION_2
    priority: 3
    animation:
      - type: panther_lights::ImageAnimation
        segments: all
        animation:
          image: /animations/custom_image.png
          duration: 3
          repeat: 1

  # animation with a custom image from custom ROS package
  - id: 23
    name: ANIMATION_3
    priority: 3
    animation:
      - type: panther_lights::ImageAnimation
        segments: all
        animation:
          image: $(find my_custom_animation_package)/animations/custom_image.png
          duration: 3
          repeat: 1

  # different animations for Front and Rear Bumper Light
  - id: 24
    name: ANIMATION_4
    priority: 3
    animation:
      - type: panther_lights::ImageAnimation
        segments: front
        animation:
          image: $(find panther_lights)/animations/triangle01_blue.png
          duration: 2
          repeat: 2
      - type: panther_lights::ImageAnimation
        segments: rear
        animation:
          image: $(find panther_lights)/animations/triangle01_red.png
          duration: 3
          repeat: 1
```

> [!NOTE]
> ID numbers from 0 to 19 are reserved for system animations.

> [!NOTE]
> Priority **1** is reserved for crucial system animations. Users can only define animations with priority **2** and **3**.

Remember to modify launch command to use user animations:

``` bash
ros2 launch panther_bringup bringup.launch user_animations_file:=/my_awesome_user_animations.yaml
```

Test new animations:

```bash
ros2 service call /lights/controller/set/animation panther_msgs/srv/SetLEDAnimation "{animation: {id: 0, param: ''}, repeating: true}""
```

---
## Defining a Custom Animation Type

It is possible to define your own animation type with expected, new behavior. For more information, see: [**Animation API**](./LIGHTS_API.md).
