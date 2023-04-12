from colorsys import hsv_to_rgb
import numpy as np
from PIL import Image, ImageFilter

from animation import Animation

class BatteryAnimation(Animation):

    ANIMATION_NAME = 'battery_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._anim = np.zeros((self._anim_len, self.num_led))
        self._h_min = 0.0       # red color
        self._h_max = 0.6       # green color
        self._resolution = 100
        self._start_fade_th = 0.85 * self._resolution
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
        frame = np.zeros((self._num_led, 3), dtype=np.uint8)
        anim = np.zeros((self._resolution, self._num_led, 3), dtype=np.uint8)

        if self._num_led % 2 == 0:
            percent_point = int(round(battery_percent * self._num_led / 2.0))
        else:
            percent_point = int(np.ceil(battery_percent * self._num_led / 2.0))

        # 0.9 was added to hold for some time the final frame of the animation when percentage is 1.0
        display_iterations = battery_percent * self._resolution * 0.9

        if percent_point < 1:
            percent_point = 1
            display_iterations = 1

        for i in range(self._resolution):
            if i <= display_iterations:
                anim_ind = round(i / self._resolution * self._num_led / 0.9)

                if anim_ind < percent_point:
                    frame.fill(0)
                    ind_1 = int(np.ceil(self._num_led / 2.0 - anim_ind - 1.0))
                    ind_2 = int(np.floor(self._num_led / 2.0 + anim_ind))
                    h = (self._h_max - self._h_min) * np.sin(battery_percent * np.pi / 2.0) * i / display_iterations + self._h_min
                    rgb = np.array(hsv_to_rgb(h, 1.0, 1.0))
                    frame[ind_1] = np.uint8(rgb * 255.0)
                    frame[ind_2] = np.uint8(rgb * 255.0)
                elif percent_point <= anim_ind < 2 * percent_point:
                    ind_1 = int(np.ceil(self._num_led / 2.0 - 2.0 * percent_point + anim_ind))
                    ind_2 = int(
                        np.floor(self._num_led / 2.0 + 2.0 * percent_point - anim_ind - 1.0)
                    )
                    frame[ind_1] = np.uint8(rgb * 255.0)
                    frame[ind_2] = np.uint8(rgb * 255.0)

            anim[i] = frame

            if i > self._start_fade_th:
                a = 1/(self._start_fade_th - self._resolution)
                b = -a*self._resolution
                anim[i] = frame * (a*i + b)

        # filter to smooth animation and resize to match duration
        img = Image.fromarray(anim)
        img = img.filter(ImageFilter.GaussianBlur(self._gaussian_blur_radius))
        img = img.resize((self._num_led, self._anim_len))
        self._anim = np.array(img)
