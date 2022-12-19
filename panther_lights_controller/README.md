# panther_lights_controller

Package used to control the Husarion Panther LED panels.

## ROS Nodes

### controller_node.py

#### Services

- `panther/lights/controller/set/animation` [*panther_msgs/SetLEDAnimation*]: allows setting animation on LED panel based on animation ID.
- `panther/lights/controller/set/brightness` [*panther_msgs/SetLEDBrightness*]: allows to set global LED brightness, value ranges from 0 to 1.
- `panther/lights/controller/set/image_nimation` [*panther_msgs/SetLEDImageAnimation*]: allows setting animation based on provided images.

#### Parameters

- `~global_brightness` [*float*, default: **1.0**]: LED global brightness.
- `~num_led` [*int*, default: **46**]: number of LEDs in single panel.
- `~controller_frequency` [*float*, default: **100**]: frequency at which the lights controller node will process animations.

## Animations

Basic animation definitions are described in **YAML** format in `config/panther_lights_animations.yaml`. Supported keys are:

- `id` [*int*]: ID of an animation.
- `animation` [*YAML*]: description of animation.
- `name` *(optional)* [*string*]: name of an animation.
- `interrupting` *(optional)* [*bool*]: if *true* animation will interrupt currently displayed animation.

### ImageAnimation

Animation returning frame to display based on an image.

Supported keys are:

- `image` [*string*]: path to an image file, if a path is not global will look in the `animations` folder for the given name.
- `duration` [*float*]: duration of an animation.
- `repeat` *(optional)* [*int*]: number of times the animtion will be repeated, by default animation will run once.
- `brightness` *(optional)* [float]: animation brightness, this will be used instead of controller `global_brightness`.