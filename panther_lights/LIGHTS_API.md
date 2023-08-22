# Animation API

## Animation Class

Basic animation type. This class consists of:

Arguments:

- `animation_description` [*dict*]: a dictionary with animation description. Contain following keys:
  - `brightness` [*float*, optional]: will be assigned to the `self._brightness` variable as a value in a range **[0, 255]**.
  - `duration` [*float*]: will be assigned to `self._duration` variable.
  - `repeat` [*int*, optional]: will be assigned to `self._loops` variable.
- `num_led` [*int*]: number of LEDs in a bumper.
- `controller_freq` [*float*]: controller frequency **[Hz]** at which animation frames will be processed.

Functions:

- `__call__` - returns a list of length `num_led` with **RGB** values of colors to be displayed on the Bumper Lights based on the `_update_animation` method. Handles looping over animation based on `repeating` parameter, iterating over animation, and updating its progress.
- `_update_animation` - returns a list of shape (`num_led`, 3) with **RGB** values of colors to be displayed on the Bumper Lights. Colors are described as a list of integers with respective **R**, **G**, and **B** color values. By default, not implemented.
- `reset` - resets animation to its initial state. If overwritten, it requires calling the parent class implementation first.
- `set_param` - allows processing an additional parameter passed to the animation when it is created. 

Properties:

- `brightness` [*float*]: returns animation brightness from `self._brightness`.
- `finished` [*bool*]: returns a value indicating if the animation execution process has finished from `self._finished`.
- `num_led` [*int*]: returns the number of LEDs from `self._num_led`.
- `progress` [*float*]: returns animation execution progress from `self._progress`.

## Defining a New Animation Type

It is possible to define your own animation type with expected, new behavior. All animation definitions are stored in `/src/animation` and inherit from the basic `Animation` class.

The new animation definition should contain `ANIMATION_NAME` used to identify it. Frames are processed in ticks with a frequency of `controller_freq`. It is required to overwrite the `_update_animation` method which must return a list representing the frame for a given tick. The advised way is to use the `self._anim_iteration` variable  (current animation tick) as a time base for animation frames. The length of the frame has to match `self._num_led`. Each element of the frame represents a color for a single LED in the Bumper Lights. Colors are described as a list of integers with respective **R**, **G**, and **B** color values. Additional parameters (e.g. image path) can be passed within `animaiton_description` and processed using the `self._animation_description` variable. See the example below or other animation definitions.

Create a New Animation Type:

``` python 
# my_cool_animation.py
from animation import Animation


class MyCoolAnimation(Animation):
    ANIMATION_NAME = 'my_cool_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def _update_frame(self) -> list:
        return [[self._param, self._param, self._param]] * self._num_led

    def set_param(self, value: str) -> None:
        self._param = float(value)
```

To make new animation available in the system, edit the `__init__.py` file in `/src/animation`, and import the newly created animation class:

``` python
from .my_cool_animation import MyCoolAnimation
```

then add it to the `BASIC_ANIMATIONS` dictionary:

``` python
BASIC_ANIMATIONS = {
    ImageAnimation.ANIMATION_NAME: ImageAnimation,
    MyCoolAnimation.ANIMATION_NAME: MyCoolAnimation,
}
```


