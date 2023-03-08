# panther_lights

Package used to control the Husarion Panther LED panels.

## ROS structure

![panther_lights ROS structure](https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/panther_lights.drawio.svg
)

## ROS Nodes

### controller_node.py

This node is responsible for processing animations and publishing frames to be displayed on the Husarion Panther robot LED panels.

#### Publishes

- `/panther/lights/driver/front_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame pixels to be displayed on robot front LED panel.
- `/panther/lights/driver/rear_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame pixels to be displayed on robot rear LED panel.
- `/panther/lights/controller/queue` [*panther_msgs/LEDAnimationQueue*]: list of names of currently enqueued animations in controller node, the first element of the list is the currently displayed animation.

#### Services advertised

- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.
- `/panther/lights/controller/set/image_animation` [*panther_msgs/SetLEDImageAnimation*]: allows setting animation based on provided images. Only available if `~test` parameter is set to **true**.
- `/panther/lights/controller/update_animations` [*std_srvs/Trigger*]: allows updating user defined animations using `~user_animations` parameter.

#### Parameters

- `~animations` [*list*, default: **None**]:  required list of defined animations.
- `~controller_frequency` [*float*, default: **46**]: frequency at which the lights controller node will process animations.
- `~num_led` [*int*, default: **46**]: number of LEDs in a single panel. Must match driver `num_led`.
- `~test` [*bool*, default: **false**]: enables testing mode, enabling extra functionalities.
- `~user_animations` [*list*, default: **None**]: optional list of animations defined by the user.

### driver_node.py

This node is responsible for displaying frames on the Husarion Panther robot LED panels.

#### Subscribes

- `/panther/lights/driver/front_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot front LED panel.
- `/panther/lights/driver/rear_panel_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: an animation frame to be displayed on robot rear LED panel.

#### Services advertised

- `/panther/lights/driver/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows to set global LED brightness, value ranges from 0 to 1.

#### Parameters

- `~frame_timeout` [*float*, default: **0.1**]: time in seconds, after which an incoming frame will be ignored if not processed.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness. Range between [0,1].
- `~num_led` [*int*, default: **46**]: number of LEDs in a single panel.

### scheduler_node.py

This node is responsible for scheduling animations displayed on LED panels based on the Husarion Panther robot's system state.

#### Subscribes

- `/panther/battery` [*sensor_msgs/BatteryState*]: robot battery state.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: informs if robot is in emergency stop state.

#### Services subscribed

- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.

#### Parameters

- `~battery_state_anim_period` [*float*, default: **120.0**]: time in seconds to wait before repeating animation representing current battery percentage.
- `~charging_battery_anim_period` [*float*, default: **20.0**]: time in seconds to wait before updating the charging battery animation if the battery percentage has changed by the value specified in the `update_charging_anim_step` param.
- `~critical_battery_anim_period` [*float*, default: **15.0**]: time in seconds to wait before repeating animation indicating a critical battery state.
- `~critical_battery_threshold_percent` [*float*, default: **0.1**]: if battery percentage drops below this value, animation indicating a critical battery state will start being displayed.
- `~low_battery_anim_period` [*float*, default: **30.0**]: time in seconds to wait before repeating animation indicating a low battery state.
- `~low_battery_threshold_percent` [*float*, default: **0.4**]: if the battery percentage drops below this value, animation indicating a low battery state will start being displayed.
- `~update_charging_anim_step` [*float*, default: **0.1**]: percentage value representing a step for updating the charging battery animation.

## Animations

Basic animations provided by Husarion are loaded upon node start from [`panther_lights_animations.yaml`](config/panther_lights_animations.yaml) and parsed as a list using the ROS parameter. Supported keys are:

- `animation` [*dict*]: definition of animation for each LED panel. Use keys described below to display the same animation on both panels or different ones on each. The keys for configuration of different animations are explained in detail under the [**Animation types**](#animation-types) section.
  - `both` animation for front and rear LED panel.
  - `front` animation for front LED panel.
  - `rear` animation for rear LED panel.
- `id` [*int*]: ID of an animation.
- `name` [*string*, default: **ANIMATION_<ID>**]: name of an animation. If not provided will default to `ANIMATION_<ID>`, where `<ID>` is this animation ID.
- `priority` [*int*, default: **3**]: priority at which animation will be placed in the queue. The list below shows behavior when an animation with a given ID arrives:
    - **1** interrupts and removes animation with priorities **2** and **3**.
    - **2** interrupts animations with priority **3**.
    - **3** adds animation to the end of queue.
- `timeout` [*float*, default: **120.0**]: time in seconds, after which animation will be removed from the queue.

> **Warning**: If `both` and `front/rear` keys are provided together, will default to `both`.

Default animations can be found in the table below:

| ID  | NAME              | PRIORITY | DESCRIPTION                                                 |
| :-: | ----------------- | :------: | ----------------------------------------------------------- |
| 0   | E_STOP            | 3        | red expanding from the center to the edges                  |
| 1   | READY             | 3        | green expanding from center to the edges                    |
| 2   | ERROR             | 1        | red, whole panel blinking twice                             |
| 3   | MANUAL_ACTION     | 3        | blue expanding from the center to the edges                 |
| 4   | AUTONOMOUS_ACTION | 3        | orange expanding from center to the edges                   |
| 5   | GOAL_ACHIEVED     | 2        | purple, whole panel blinking three times                    |
| 6   | LOW_BATTERY       | 2        | two orange stripes moving towards the center, repeats twice |
| 7   | CRITICAL_BATTERY  | 2        | two red stripes moving towards the center, repeats twice    |
| 8   | BATTERY_STATE     | 3        | two stripes moving towards the edges stopping at a point representing battery percentage and filling back to the center, color changes from red to green |
| 9   | CHARGING_BATTERY  | 3        | whole panel blinking with a duty cycle proportional to the battery percentage. Short blinking means low battery, no blinking means full battery. Color changes from red to green |

### Animation types

#### Animation

Basic animation definition. Keys are inherited from basic **Animation** class by all animations. Supported keys are:

- `brightness` [*float*, optional]: animation brightness relative to APA102 driver `global_brightness`. Range between [0,1].
- `duration` [*float*, required]: duration of an animation for a given panel.
- `repeat` [*int*, default: **1**]: number of times the animation will be repeated.
- `type` [*string*, required]: animation type, default animation types are: `image_animation`, `battery_animation`, `charging_animation`.

> **Note**: Overall display duration of an animation is a product of a single image duration and repeat count. Result of `duration` x `repeat`  can't exceed 10 seconds. If animation fails to fulfill the requirement it will result in an error.

#### ImageAnimation

Animation of type `image_animation`, returns frames to display based on an supplied image. Extends `Animation` with keys:

- `color` [*int*, optional]: image is turned into grayscale and then the color is applied with brightness from gray image. Values have to be in HEX format.
- `image` [*string*, required]: path to an image file. Only global paths are valid. Allows using `$(find ros_package)` syntax.

#### BatteryAnimation

Animation of type `battery_animation` returning frame to display based on `param` value representing battery percentage. The animation displays two stripes moving towards the edges, stopping at a point representing battery percentage and filling back to the center. Color changes from red (battery discharged) to green (battery full).

### ChargingAnimation

Animation of type `charging_animation` returning frame to display based on `param` value representing battery percentage. Displays solid color with a duty cycle proportional to the battery percentage. Color is changing from red (battery discharged) to green (battery fully charged).

### Defining animations

User can define own animations using basic animation types. Similar to basic ones, user animations are parsed using a ROS parameter `/panther/lights/lights_controller_node/user_animations`. They can be loaded on node start or updated using the `/panther/lights/controller/update_animations` ROS service. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations. 

Create a yaml file with an animation description list. Example file: 

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

  # animation with custom image from custom ROS package
  - id: 23
    name: ANIMATION_3
    priority: 3
    animation:
      both:
        type: image_animation
        image: $(find my_custom_animation_package)/animations/custom_image.png
        duration: 3
        repeat: 1

  # different animations for front and rear panel
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

> **Note**: ID numbers from 0 to 19 are reserved for system animations.

> **Note**: Priority **1** is reserved for crucial system animations. Users can only define animations with priority **2** and **3**.

Add docker volume with a previously created animation description list. If using custom images for `ImageAnimation`, add also a docker volume with a folder containing custom images. On built-in computer modify `compose.yaml`:

```yaml
volumes:
  - ./my_awesome_user_animations.yaml:/my_awesome_user_animations.yaml
  - ./animations:/animations
```

Remember to modify command to use user animations:

```yaml
command: >
  roslaunch --wait
  panther_bringup bringup.launch
  user_animations_file:=/my_awesome_user_animations.yaml
```

> **Warning**:
> While using docker you will only be able to find packages that are within that docker. Only images from packages that were built inside that docker image can be found using `$(find my_package)` syntax. Global paths work normally, but will only refere to paths inside docker container.

Restart the docker container:

```
docker compose up -d
```

Test new animations:

```bash
rosservice call /panther/lights/controller/set/animation "{animation: {id: 21, param: ''}, repeating: false}"
```

### Updating animation list at a runtime

User animations can also be updated at a runtime.

Create a yaml file with an animation description list similar to one in [Defining animations](#defining-animations). Then update user animations ROS parameter:

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
## Defining custom animation type

It is possible to define your own animation type with expected, new behavior. For more information see: [**Animation API**](./LIGHTS_API.md).
