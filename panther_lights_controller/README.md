# panther_lights_controller

Package used to control the Husarion Panther LED panels.

## ROS Nodes

### controller_node.py

#### Services

- `panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.
- `panther/lights/controller/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows to set global LED brightness, value ranges from 0 to 1.
- `panther/lights/controller/set/image_animation` [*panther_msgs/SetLEDImageAnimation*]: allows setting animation based on provided images, available in testing mode.

#### Parameters

- `~animations` [*list*]: it is a required ROS parameter containing a list of defined animations.
- `~global_brightness` [*float*, default: **1.0**]: LED global brightness.
- `~num_led` [*int*, default: **46**]: number of LEDs in single panel.
- `~controller_frequency` [*float*, default: **100**]: frequency at which the lights controller node will process animations.
- `~test` [*bool*, default: **false**]: allows testing mode with some extra functionalities.

## Animations

Basic animations are parsed as a list using the ROS parameter. Currently, defined animations are:

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

Basic animations are described and loaded on the node start, directly from `config/panther_lights_animations.yaml`. Supported keys are:

- `id` [*int*]: ID of an animation.
- `animation` [*dict*]: description of animation.
- `name` *(optional)* [*string*]: name of an animation.
- `interrupting` *(optional)* [*bool*]: if *true* animation will interrupt currently displayed animation.

### ImageAnimation

Animation returning frame to display based on an image. The display duration which is a product of a single image duration and repeat count can't exceed 10 seconds. Supported keys are:

- `image` [*string*]: path to an image file. If a path is not global will look in the `animations` folder in the ROS package specified in `animations_package` for the given name.
- `duration` [*float*]: duration of an animation.
- `repeat` *(optional)* [*int*]: number of times the animation will be repeated, by default animation will run once.
- `brightness` *(optional)* [float]: animation brightness, this will be used instead of controller `global_brightness`.
- `color` *(optional)* [*int*]: if set the image color will be changed to the specified one, advised way is to use HEX format eg. `0xff0000`.
- `animations_package` *(optional)* [*string*]: ROS package containing `animations` folder with images. If not specified will default to this package. If using Docker given package has to be inside the same container as this package.

### Defining animations

User can define own animations using basic animation types. Similar to basic ones user animations are parsed using a ROS parameter `/panther/lights/lights_controller_node/user_animations`. They can be loaded on node start or updated using the `/panther/lights/controller/update_animations` ROS service. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations. 

1. Create a yaml file with an animation description list. Example file: 

NOTE: ID numbers from 0 to 20 are reserved for system animations.

```yaml
# user_animations.yaml
user_animations:
  - id: 21
    name: 'ANIMATION_1'
    animation:
      both:
        type: image_animation
        image: strip01_green.png
        duration: 2
        repeat: 2
        color: 0xffff00

  # animation with custom image
  - id: 22
    name: 'ANIMATION_2'
    animation:
      both:
        type: image_animation
        image: /animations/custom_image.png
        duration: 3
        repeat: 1
```

2. Add docker volume with a previously created animation description list. If using custom images for `ImageAnimation`, add also a docker volume with a folder containing custom images. On Raspberry Pi modify `compose.yaml`:

```yaml
volumes:
  - ./user_animations.yaml:/user_animations.yaml
  - ./animations:/animations
```

3. Modify `command` in `compose.yaml` to use user animations:

```yaml
command: roslaunch panther_bringup bringup.launch user_animations_file:=/user_animations.yaml --wait
```

4. Restart the docker container:

```
docker compose up -d
```

5. Display new animations:

```bash
rosservice call /lights/controller/set/animation "{animation: {id: 21, name: 'ANIMATION_1'}, repeating: false}"
```

#### Updating animation list at a runtime

User animations can be also updated at a runtime.

1. Create a yaml file with an animation description list similar to step 1 in [Defining animations](#defining-animations).

2. Update user animations ROS parameter:

```bash
rosparam load /path_to_description_file /namespace
# eg. 
rosparam load ./user_animations.yaml /panther/lights_controller_node
```

3. Update animation list with ROS service:

```bash
rosservice call /panther/lights/controller/update_animations "{}"
```

4. Display new animations:

```bash
rosservice call /lights/controller/set/animation "{animation: {id: 21, name: 'ANIMATION_1'}, repeating: false}"
```