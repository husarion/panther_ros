from colorsys import hsv_to_rgb
import numpy as np
from PIL import Image, ImageFilter

from animation import Animation


class BatteryAnimation(Animation):

    ANIMATION_NAME = 'battery_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._anim = np.zeros((self._anim_len, self.num_led))
        self._h_min = 0.0  # red color
        self._h_max = 120.0 / 360.0  # green color
        self._resolution = 100
        self._end_anim = 0.75 * self._resolution
        self._start_fade = 0.80 * self._resolution
        self._end_fade = 0.95 * self._resolution
        self._gaussian_blur_radius = 1.5

    def _update_frame(self) -> list:
        return self._anim[self._anim_iteration, :].tolist()

    def set_param(self, value: str) -> None:
        try:
            battery_percent = float(value)
        except ValueError:
            raise ValueError('Can not cast param to float!')
        self._param = battery_percent
        self._create_anim(battery_percent)

    def _create_anim(self, battery_percent: float) -> None:
        battery_percent = np.clip(battery_percent, 0.0, 1.0)
        anim = np.zeros((self._resolution, self._num_led, 3), dtype=np.uint8)

        color_rising_duration = int(round(battery_percent * self._end_anim / 2.0))

        if color_rising_duration < 1:
            color_rising_duration = 1

        for i in range(1, self._resolution):
            if i <= color_rising_duration:
                progress = i / color_rising_duration
                frame = self._color_rising(battery_percent, progress)

            elif i <= 2 * color_rising_duration:
                progress = (i - color_rising_duration) / color_rising_duration
                frame = self._fill_frame(frame, battery_percent, progress)

            anim[i] = frame

            if i > self._start_fade:
                progress = (i - self._start_fade) / (self._end_fade - self._start_fade)
                progress = min(1.0, progress)
                anim[i] = (1 - progress) * frame

        # filter to smooth animation and resize to match duration
        img = Image.fromarray(anim)
        img = img.filter(ImageFilter.GaussianBlur(self._gaussian_blur_radius))
        img = img.resize((self._num_led, self._anim_len))
        self._anim = np.array(img)

    def _color_rising(self, battery_percent: float, progress: float) -> np.array:
        frame = np.zeros((self._num_led, 3), dtype=np.uint8)
        led_to_disp = battery_percent * self._num_led

        middle_led = (self._num_led - 1) / 2.0
        ind_1 = int(np.ceil(middle_led - progress * led_to_disp / 2.0))
        ind_2 = int(np.floor(middle_led + progress * led_to_disp / 2.0))

        rgb = self._calculate_color(battery_percent, progress)
        frame[ind_1] = rgb
        frame[ind_2] = rgb
        return frame

    def _fill_frame(self, frame: np.array, battery_percent: float, progress: float) -> np.array:
        led_to_disp = battery_percent * self._num_led

        middle_led = (self._num_led - 1) / 2.0
        ind_1 = int(np.ceil(middle_led - (1.0 - progress) * led_to_disp / 2.0))
        ind_2 = int(np.floor(middle_led + (1.0 - progress) * led_to_disp / 2.0))

        rgb = self._calculate_color(battery_percent)
        frame[ind_1] = rgb
        frame[ind_2] = rgb
        return frame

    def _calculate_color(self, battery_percent: float, progress: float = 1.0) -> tuple:
        h = (self._h_max - self._h_min) * np.sin(
            battery_percent * np.pi / 2.0
        ) * progress + self._h_min
        rgb = np.array(hsv_to_rgb(h, 1.0, 1.0))
        rgb = np.uint8(rgb * 255.0)
        return rgb
