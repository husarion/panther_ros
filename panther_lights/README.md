# Panther lights

Control Panther's APA102C lights using ROS.

## ROS API

### Service clients
- `/set_panther_lights` (*panther_lights/SetLights*) - allows to change LEDs animation.

    Change animation to `BLINIKER_LEFT`:

    ```bash
    rosservice call /set_panther_lights "{animation: 1, custom_color: ''}"
    ```

    Change animation to `BLINKER_LEFT` with custom colors (front is green, rear is red):

    ```bash
    rosservice call /set_panther_lights "{animation: 1, custom_color: '0x00FF00 0xFF0000'}"
    ```

    Possible values:
    ```
    BLINKER_RIGHT = 0
    BLINKER_LEFT = 1
    BRAKE_FRONT = 2
    BRAKE_REAR = 3
    BRAKE_BOTH = 4
    NORMAL_FORWARD = 5
    NORMAL_REVERSING = 6
    SKID_RIGHT = 7
    SKID_LEFT = 8
    ERROR = 9
    DEFAULT = 10
    DEFAULT_FAST = 11
    ```
- `/set_panther_lights_brightness` *(panther_lights/SetBrightness)* - set Panther lights brightness.

    Change lights brightness to half:

    ```bash
    $ rosservice call /set_panther_lights_brightness "brightness: 0.5"
    ```

### Animation configuration

You can customize your animation in file `panther_lights_animations.yaml`. You can find it in your installation directory (`<installation_path>install/share/config/panther_lights_animations.yaml`). 

Default configuration:
```yaml
# animations.yaml
# BLINKER_RIGHT = 0
# BLINKER_LEFT = 1
# BRAKE_FRONT = 2
# BRAKE_REAR = 3
# BRAKE_BOTH = 4
# NORMAL_FORWARD = 5
# NORMAL_REVERSING = 6
# SKID_RIGHT = 7
# SKID_LEFT = 8
# ERROR = 9
# TEST1 = 10
# TEST2 = 11
global_brightness: 20
num_leds: 73

animations:
  - id: 0 # BLINKER_RIGHT
    front: {type: 'COLOR_WIPE_HALF_DOWN_FAST', color: !!int 0xff6600}
    back: {type: 'COLOR_WIPE_HALF_UP_FAST', color: !!int 0xff6600}
  
  - id: 1 # BLINKER_LEFT
    front: {type: 'COLOR_WIPE_HALF_UP_FAST', color: !!int 0xff6600}
    back: {type: 'COLOR_WIPE_HALF_DOWN_FAST', color: !!int 0xff6600}
  
  - id: 2 # BRAKE_FRONT
    front: {type: 'ONE_DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
    back: {type: 'SOLID_COLOR', color: !!int 0xffffff}
  
  - id: 3 # BRAKE_REAR
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'ONE_DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
  
  - id: 4 # BRAKE_BOTH
    front: {type: 'ONE_DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
    back: {type: 'ONE_DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}

  - id: 5 # NORMAL_FORWARD
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'SOLID_COLOR', color: !!int 0xffffff}

  - id: 6 # NORMAL_REVERSING
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'FADE_COLOR', color: !!int 0xEEDD00}

  - id: 7 # SKID_RIGHT
    front: {type: 'COLOR_WIPE_DOWN_NORMAL', color: !!int 0xEEDD00}
    back: {type: 'COLOR_WIPE_UP_NORMAL', color: !!int 0xEEDD00}

  - id: 8 # SKID_LEFT
    front: {type: 'COLOR_WIPE_UP_NORMAL', color: !!int 0xEEDD00}
    back: {type: 'COLOR_WIPE_DOWN_NORMAL', color: !!int 0xEEDD00}

  - id: 9 # ERROR
    front: {type: 'FADE_COLOR', color: !!int 0xFF0000}
    back: {type: 'FADE_COLOR', color: !!int 0xFF0000}

  - id: 10 # TEST1
    front: {type: 'DOUBLE_COLOR_WIPE_NORMAL', color: !!int 0xff0000}
    back: {type: 'DOUBLE_COLOR_WIPE_NORMAL', color: !!int 0xff0000}

  - id: 11 # TEST2
    front: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
    back: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
```
