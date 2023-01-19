from colorsys import hsv_to_rgb
import numpy as np

from .animation import Animation


class ChargingAnimation(Animation):

    ANIMATION_NAME = 'charging_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        fade_factor = 0.1
        self._fade_duration = int(round(self._anim_len * fade_factor))
        self._frame = np.zeros((self._num_led, 3), dtype=np.uint8)
        self._h_min = 0.0
        self._h_max = 120.0
        self._color = None
        self._on_duration = None

    def _update_frame(self) -> list:

        if self._on_duration < 2 * self._fade_duration:
            self._on_duration = 2 * self._fade_duration
        if self._on_duration >= self._anim_len:
            self._fade_duration = 0

        if self._anim_iteration < self._fade_duration:
            self._frame[:] = self._color * np.sin(
                np.pi / 2.0 * (self._anim_iteration / self._fade_duration)
            )
        elif self._anim_iteration < self._on_duration - self._fade_duration:
            self._frame[:] = self._color
        elif self._anim_iteration < self._on_duration:
            self._frame[:] = self._color * np.sin(
                np.pi
                / (2.0 * self._fade_duration)
                * (self._anim_iteration - self._on_duration + 2.0 * self._fade_duration)
            )
        else:
            self._frame.fill(0)

        return self._frame.tolist()

    def set_param(self, value: float) -> None:
        battery_percent = np.clip(value, 0.0, 1.0)
        self._on_duration = int(round(self._anim_len * battery_percent))
        h = (self._h_max - self._h_min) * battery_percent + self._h_min
        s = 1.0
        v = 1.0
        self._color = np.array(hsv_to_rgb(h / 360.0, s, v)) * 255.0
