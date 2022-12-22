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
- `~test` [*bool*, default: **false**]: allows testing mode with some extra funcionalities.

## Animations

Basic animations are parsed as a list using the ROS parameter. They are loaded on node start directly from `config/panther_lights_animations.yaml`. Supported keys are:

- `id` [*int*]: ID of an animation.
- `animation` [*dict*]: description of animation.
- `name` *(optional)* [*string*]: name of an animation.
- `interrupting` *(optional)* [*bool*]: if *true* animation will interrupt currently displayed animation.

### ImageAnimation

Animation returning frame to display based on an image. The display duration which is a product of a single image duration and repeat count can't exceed 10 seconds. Supported keys are:

- `image` [*string*]: path to an image file, if a path is not global will look in the `animations` folder for the given name.
- `duration` [*float*]: duration of an animation.
- `repeat` *(optional)* [*int*]: number of times the animtion will be repeated, by default animation will run once.
- `brightness` *(optional)* [float]: animation brightness, this will be used instead of controller `global_brightness`.
- `color` *(optional)* [*int*]: if set the image color will be changed to the specified one, advised way is to use HEX format eg. `0xff0000`.