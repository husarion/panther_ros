# panther_lights

Package used to control the Husarion Panther Bumper Lights.

## ROS structure

![panther_lights ROS structure](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/panther_lights.drawio.svg)

## ROS Nodes

### controller_node.py

This node is responsible for processing animations and publishing frames to be displayed on the Husarion Panther robot Bumper Lights.

#### Publishers

- `/panther/lights/driver/front_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame pixels to be displayed on robot Front Bumper Lights.
- `/panther/lights/driver/rear_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame pixels to be displayed on robot Rear Bumper Lights.
- `/panther/lights/controller/queue` [*panther_msgs/LEDAnimationQueue*]: list of names of currently enqueued animations in the controller node, the first element of the list is the currently displayed animation.

#### Service Servers

- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on Bumper Lights based on animation ID.
- `/panther/lights/controller/set/image_animation` [*panther_msgs/SetLEDImageAnimation*]: allows setting animation based on provided images. Only available if `~test` parameter is set to **true**.
- `/panther/lights/controller/update_animations` [*std_srvs/Trigger*]: allows updating user-defined animations using `~user_animations` parameter.

#### Parameters

- `~animations` [*list*, default: **None**]:  required list of defined animations.
- `~controller_frequency` [*float*, default: **46.0**]: frequency **[Hz]** at which the lights controller node will process animations.
- `~num_led` [*int*, default: **46**]: number of LEDs in a single bumper. Must match driver `num_led` in *driver_node*.
- `~test` [*bool*, default: **false**]: enables `/panther/lights/controller/set/image_animation` service.
- `~user_animations` [*list*, default: **None**]: optional list of animations defined by the user.

### driver_node

This node is responsible for displaying frames on the Husarion Panther robot's Bumper Lights.

#### Subscribers

- `/panther/lights/driver/front_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Front Bumper Lights.
- `/panther/lights/driver/rear_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `/panther/lights/driver/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows setting global LED brightness, value ranges from **0.0** to **1.0**.

#### Parameters

- `~frame_timeout` [*float*, default: **0.1**]: time in **[s]** after which an incoming frame will be considered too old.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness. The range between **[0.0, 1.0]**.
- `~num_led` [*int*, default: **46**]: number of LEDs in a single bumper.

## Animations

Basic animations provided by Husarion are loaded upon node starting from [`panther_lights_animations.yaml`](config/panther_lights_animations.yaml) and parsed as a list using the ROS parameter. Supported keys are:

- `animation` [*list*, default: **Empty List**]: definition of animation for each Bumper Lights. Use the keys described below to display the same animation on both Bumper Lights or different ones on each. The keys for the configuration of different animations are explained in detail under the [**Animation Types**](#animation-types) section.
  - `both` animation for Front and Rear Bumper Lights.
  - `front` animation for the Front Bumper Lights.
  - `rear` animation for the Rear Bumper Lights.
- `id` [*int*, default: **None**]: ID of an animation.
- `name` [*string*, default: **ANIMATION_<ID>**]: name of an animation. If not provided will default to **ANIMATION_<ID>**, where `<ID>` is equal to `id` parameter of the given animation.
- `priority` [*int*, default: **3**]: priority at which animation will be placed in the queue. The list below shows the behavior when an animation with a given ID arrives:
    - **1** interrupts and removes animation with priorities **2** and **3**.
    - **2** interrupts animations with priority **3**.
    - **3** adds animation to the end of the queue.
- `timeout` [*float*, default: **120.0**]: time in **[s]**, after which animation will be removed from the queue.

> **Warning**
> If the `animation` key at the same time has values `both` and `front` or `rear` provided. It will default to `both`.

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
| 8   | BATTERY_STATE     | 3        | two stripes moving towards the edges stopping at a point representing Battery percentage and filling back to the center, color changes from red to green |
| 9   | CHARGING_BATTERY  | 3        | whole bumper blinks with a duty cycle proportional to the Battery percentage. Short blinking means low Battery, and no blinking means full Battery. The color changes from red to green |

### Animation types

#### Animation

Basic animation definition. Keys are inherited from the basic **Animation** class by all animations. Supported keys are:

- `brightness` [*float*, default: **1.0**]: animation brightness relative to APA102C driver `global_brightness`. The range between **[0.0, 1.0]**.
- `duration` [*float*, default: **None**]: duration of animation for a given bumper.
- `repeat` [*int*, default: **1**]: number of times the animation will be repeated.
- `type` [*string*, default: **None**]: animation type, default animation types are: `image_animation`, `battery_animation`, `charging_animation`.

> **Note**
> Overall display duration of an animation is a product of a single image duration and repeat count. The result of `duration` x `repeat`  can't exceed 10 **[s]**. If animation fails to fulfill the requirement it will result in an error.

#### ImageAnimation

Animation of type `image_animation`, returns frames to display based on a supplied image. Extends `Animation` with keys:

- `color` [*int*, default: **None**]: image is turned into grayscale, and then the color is applied with brightness from the gray image. Values have to be in HEX format. This parameter is not required.
- `image` [*string*, default: **None**]: path to an image file. Only global paths are valid. Allows using `$(find ros_package)` syntax.

#### BatteryAnimation

Animation of type `battery_animation` returning frame to display based on `param` value representing Battery percentage. The animation displays two stripes moving towards the edges, stopping at a point representing the Battery percentage, and filling back to the center. The color changes from red (Battery discharged) to green (Battery full).

### ChargingAnimation

Animation of type `charging_animation` returning frame to display based on `param` value representing Battery percentage. Displays a solid color with a duty cycle proportional to the Battery percentage. The color is changing from red (Battery discharged) to green (Battery fully charged).

### Defining Animations

Users can define their own animations using basic animation types. Similar to basic ones, user animations are parsed using the ROS parameter `/panther/lights/lights_controller_node/user_animations`. They can be loaded on node start or updated using the `/panther/lights/controller/update_animations` ROS service. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations. 

Create a YAML file with an animation description list. Example file: 

```yaml
# my_awesome_user_animations.yaml
user_animations:
  # animation with default image and custom color
  - id: 21
    name: ANIMATION_1
    priority: 2
    animation:
      both:
        type: image_animation
        image: $(find panther_lights)/animations/strip01_red.png
        duration: 2
        repeat: 2
        color: 0xffff00

  # animation with custom image
  - id: 22
    name: ANIMATION_2
    priority: 3
    animation:
      both:
        type: image_animation
        image: /animations/custom_image.png
        duration: 3
        repeat: 1

  # animation with a custom image from custom ROS package
  - id: 23
    name: ANIMATION_3
    priority: 3
    animation:
      both:
        type: image_animation
        image: $(find my_custom_animation_package)/animations/custom_image.png
        duration: 3
        repeat: 1

  # different animations for Front and Rear Bumper Light
  - id: 24
    name: ANIMATION_4
    priority: 3
    animation:
      front:
        type: image_animation
        image: $(find panther_lights)/animations/triangle01_blue.png
        duration: 2
        repeat: 2
      rear:
        type: image_animation
        image: $(find panther_lights)/animations/triangle01_red.png
        duration: 3
        repeat: 1
```

> **Note**
> ID numbers from 0 to 19 are reserved for system animations.

> **Note**
> Priority **1** is reserved for crucial system animations. Users can only define animations with priority **2** and **3**.

Add a Docker volume with a previously created animation description list. If using custom images for `ImageAnimation`, add a Docker volume with a folder containing custom images. On Built-in Computer modify `compose.yaml`:

``` yaml
volumes:
  - ./my_awesome_user_animations.yaml:/my_awesome_user_animations.yaml
  - ./animations:/animations
```

Remember to modify command to use user animations:

``` yaml
command: >
  roslaunch --wait
  panther_bringup bringup.launch
  user_animations_file:=/my_awesome_user_animations.yaml
```

> **Warning**:
> While using Docker you will only be able to find packages that are within that Docker. Only images from packages that were built inside that Docker image can be found using `$(find my_package)` syntax. Global paths work normally, but will only refere to paths inside Docker container.

Restart the Cocker container:

```
docker compose up -d --force-recreate
```

Test new animations:

```bash
rosservice call /panther/lights/controller/set/animation "{animation: {id: 21, param: ''}, repeating: false}"
```

### Updating the Animation List at a Runtime

User animations can also be updated at runtime.

Create a YAML file with an animation description list similar to the one in [Defining animations](#defining-animations). Then update user animations ROS parameter:

```bash
rosparam load ./my_awesome_user_animations.yaml /panther/lights_controller_node
```

Update animation list with ROS service:

```bash
rosservice call /panther/lights/controller/update_animations "{}"
```

Test new animations:

```bash
rosservice call /panther/lights/controller/set/animation "{animation: {id: 21, param: ''}, repeating: false}"
```

---
## Defining a Custom Animation Type

It is possible to define your own animation type with expected, new behavior. For more information, see: [**Animation API**](./LIGHTS_API.md).


