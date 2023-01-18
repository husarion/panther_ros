# panther_lights

Package used to control the Husarion Panther LED panels.

## ROS Nodes

### controller_node.py

Node responsible for displaying animations on the Husarion Panther robot LED panels.

#### Services

- `panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.
- `panther/lights/controller/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows to set global LED brightness, value ranges from 0 to 1.
- `panther/lights/controller/set/image_animation` [*panther_msgs/SetLEDImageAnimation*]: allows setting animation based on provided images, available in testing mode.

#### Parameters

- `~animations` [*list*, default: **None**]:  required list of defined animations.
- `~controller_frequency` [*float*, default: **100**]: frequency at which the lights controller node will process animations.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness. Range between [0,1].
- `~num_led` [*int*, default: **46**]: number of LEDs in single panel.
- `~test` [*bool*, default: **false**]: enables testing mode with some extra functionalities.
- `~user_animations` [*list*, default: **None**]: optional list of animations defined by the user.

### scheduler_node.py

Node responsible for scheduling animations displayed on LED panels based on the Husarion Panther robot's system state.

#### Subscribe

- `/panther/battery` [*sensor_msgs/BatteryState*]: robot battery state.
- `/panther/hardware/charger_connected` [*std_msgs/Bool*]: informs if charger is connected.
- `/panther/hardware/e_stop` [*std_msgs/Bool*]: informs if robot is in emergency stop state.

#### Services subscribed

- `/panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.

#### Parameters

- `~critical_battery_anim_period` [*float*, default: **15.0**]: time in seconds to wait before repeating animation indicating a critical battery state.
- `~critical_battery_threshold_percent` [*float*, default: **0.1**]: if battery percentage drops below this value, animation indicating a critical battery state will start being displayed.
- `~low_battery_anim_period` [*float*, default: **30.0**]: time in seconds to wait before repeating animation indicating a low battery state.
- `~low_battery_threshold_percent` [*float*, default: **0.4**]: if the battery percentage drops below this value, animation indicating a low battery state will start being displayed.

## Animations

Basic animations are parsed as a list using the ROS parameter. Default animations supplied by Husarion are listed in table below.

| ID  | NAME              | DESCRIPTION                                                 |
| --- | ----------------- | ----------------------------------------------------------- |
| 0   | E_STOP            | red expanding from the center to the edges                  |
| 1   | READY             | green expanding from center to the edges                    |
| 2   | ERROR             | red blinking twice                                          |
| 3   | MANUAL_ACTION     | blue expanding from the center to the edges                 |
| 4   | AUTONOMOUS_ACTION | orange expanding from center to the edges                   |
| 5   | GOAL_ACHIEVED     | purple blinking three times                                 |
| 6   | LOW_BATTERY       | two orange stripes moving towards the center, repeats twice |
| 7   | CRITICAL_BATTERY  | two red stripes moving towards the center, repeats twice    |
| 8   | BATTERY_STATE      | two stripes moving towards the edges stopping at a point representing battery percentage and filling back to the center, color changes from red to green |
| 9   | CHARGING_BATTERY  | solid color with a duty cycle proportional to the battery percentage, color changes from red to green |

Default animations are described and loaded on the node start, directly from `config/panther_lights_animations.yaml`. Supported keys are:

- `animation` [*dict*]: definition of animation. See section below for more info.
- `id` [*int*]: ID of an animation.
- `name` [*string*, default: **NAME_NOT_DEFINED**]: name of an animation.
- `priority` [*int*, default: **3**]: priority at which animation will be placed in the queue. List below shows behaviour when ne animation with given ID arrives:
    - **1** intterupts and removes animation with priorites **2** and **3**.
    - **2** interrupts animations with priority **3**.
    - **3** add adnimation to the end of queue.
- `timeout` [*float*, default: **120.0**]: time in seconds after which animation will be removed from the queue.

### Animation types

#### Animation

Basic animation definition. Supported keys are:

- `brightness` [*float*, optional]: animation brightness. This will overwrite `global_brightness` for a given animation.
- `duration` [*float*]: duration of a single image animation.
- `repeat` [*int*, optional]: number of times the animation will be repeated, by default animation will run once.
- `type` [*string*]: required field specyfying animation type, currently suported animation types are: `image_animation`, `battery_animation`, `charging_animation`.

:bulb: **NOTE:** The overall display duration of an animation is a product of a single image duration and repeat count. It can't exceed 10 seconds.

#### ImageAnimation

Animation of type `image_animation` returning frame to display based on an image. Additional keys are:

- `color` [*int*, optional]: image will be turned into grayscale and then the color will be applied with brightness from grayscale. Values have to be in HEX format.
- `image` [*string*]: path to an image file. Only global paths are valid. Allows using `$(find ros_package)` syntax.

#### BatteryAnimation

Animation of type `battery_animation` returning frame to display based on `param` value representing battery percentage. Animation displays two stripes moving towards the edges stopping at a point representing battery percentage and filling back to the center. Color changes from red (battery discharged) to green (battery full).

### ChargingAnimation

Animation of type `charging_animation` returning frame to display based on `param` value representing battery percentage. Displays solid color with a duty cycle proportional to the battery percentage. Color is changing from red (battery discharged) to green (battery fully charged).

### Defining animations

User can define own animations using basic animation types. Similar to basic ones user animations are parsed using a ROS parameter `/panther/lights/lights_controller_node/user_animations`. They can be loaded on node start or updated using the `/panther/lights/controller/update_animations` ROS service. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations. 

#### 1. Create an animation description list.

Create a yaml file with an animation description list. Example file: 

:bulb: **NOTE:** ID numbers from 0 to 19 are reserved for system animations.

:bulb: **NOTE:** Priority **1** is reserved for crucial system animations. Users can only define animations with lower priority. 

```yaml
# user_animations.yaml
user_animations:
  # animation with default image and custom color
  - id: 21
    name: 'ANIMATION_1'
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
    name: 'ANIMATION_2'
    priority: 3
    animation:
      both:
        type: image_animation
        image: /animations/custom_image.png
        duration: 3
        repeat: 1

   # animation with custom image from custom ROS package
  - id: 23
    name: 'ANIMATION_3'
    priority: 3
    animation:
      both:
        type: image_animation
        image: $(find my_custom_animation_package)/animations/custom_image.png
        duration: 3
        repeat: 1
```

#### 2. Modify compose file.

Add docker volume with a previously created animation description list. If using custom images for `ImageAnimation`, add also a docker volume with a folder containing custom images. On Raspberry Pi modify `compose.yaml`:

```yaml
volumes:
  - ./user_animations.yaml:/user_animations.yaml
  - ./animations:/animations
```

Modify also `command` to use user animations:

```yaml
command: roslaunch panther_bringup bringup.launch user_animations_file:=/user_animations.yaml --wait
```

:warning: **Warning**
While using docker you will only be able to find packages that are within that docker. Only images from packages that were built inside that docker imge can be  found using `$(find my_package)` syntax. Global paths work normally, but will refere to paths inside docker container.

#### 3. Restart the docker container:

```
docker compose up -d
```

#### 4. Display new animations:

```bash
rosservice call /lights/controller/set/animation "{animation: {id: 21, name: 'ANIMATION_1'}, repeating: false}"
```

### Updating animation list at a runtime

User animations can be also updated at a runtime.

#### 1. Create animation description list

Create a yaml file with an animation description list similar to step 1 in [Defining animations](#defining-animations).

#### 2. Update user animations ROS parameter:

```bash
rosparam load /path_to_description_file /namespace
# eg. 
rosparam load ./user_animations.yaml /panther/lights_controller_node
```

#### 3. Update animation list with ROS service:

```bash
rosservice call /panther/lights/controller/update_animations "{}"
```

#### 4. Display new animations:

```bash
rosservice call /lights/controller/set/animation "{animation: {id: 21, name: 'ANIMATION_1'}, repeating: false}"
```

### Defining new animation type

It is possible to define your own animation type with expected, new behavior. All animation definitions are stored in `/src/animation` and inherit from the basic class `Animation`. This class consists of:

Arguments:

- `animation_description` [*dict*]: a dictionary containing animation description, `Animation` class will process:
  - `brightness` [*float*, optional]: will be assigned to the `self._brightness` variable as a value in range [0,100].
  - `duration` [*float*]: will be assigned to `self._duration` variable.
  - `repeat` [*int*, optional]: will be assigned to `self._loops` variable.
- `num_led` [*int*]: number of LEDs in a panel.
- `controller_freq` [*float*]: controller frequency at which animation frames will be processed.

Methods:

- `reset` - resets animation to initial state. If overwritten requires calling parent class implementation first.
- `_update_animation` - returns a list of length `num_led` with **RGB** values of colors to be displayed on the LED panel. Colors are described as a list of integers with respectively **R**, **G**, and **B** color values. By default not implemented.

Properties:

- `progress` [*float*]: returns animation execution progress from `self._progress`.

The new animation definition should contain `ANIMATION_NAME` used to identify it. Animation frames are processed in ticks with a frequency of `controller_freq`. It is required to overwrite the `_update_animation` method which must return a list representing the animation frame. The advised way is to use the `self._anim_iteration` variable  (current animation tick) to produce an animation frame. As an example see other animation definitions.

To add a new animation definition to basic animations edit the `__init__.py` file in `/src/animation`, and import the newly created animation class:

```
from .my_animation import MyAnimation
```

then add it to the `BASIC_ANIMATIONS` dictionary:

```
BASIC_ANIMATIONS = {
    ImageAnimation.ANIMATION_NAME: ImageAnimation,
    MyAnimation.ANIMATION_NAME : MyAnimation,
}