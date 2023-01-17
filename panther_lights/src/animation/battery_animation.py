from colorsys import hsv_to_rgb
import numpy as np
from PIL import Image, ImageFilter

from animation import Animation


class BatteryAnimation(Animation):

    ANIMATION_NAME = 'battery_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._anim = np.zeros((self._anim_len, self.num_led))
        self._h_min = 0
        self._h_max = 120

    def _update_frame(self) -> list:
        return self._anim[self._anim_iteration, :].tolist()

    def set_param(self, value: float) -> None:
        self._param = value
        self._create_anim(value)

    def _create_anim(self, battery_percent: float) -> None:
        resolution = 100
        # define basic HSV color
        h = self._h_min
        s = 1.0
        v = 1.0
        rgb = np.array(hsv_to_rgb(h / 360, s, v))
        frame = np.zeros((self._num_led, 3), dtype=np.uint8)
        anim = np.zeros((resolution, self._num_led, 3), dtype=np.uint8)

        if self._num_led % 2 == 0:
            percent_point = int(round(battery_percent * self._num_led / 2))
        else:
            percent_point = int(np.ceil(battery_percent * self._num_led / 2))

        # 0.9 was added to hold for some time the final frame of the animation when percentage is 1.0
        display_iterations = battery_percent * resolution * 0.9

        if percent_point < 1:
            percent_point = 1
            display_iterations = 0

        # create binary animation
        for i in range(resolution):
            if i <= display_iterations:
                anim_ind = round(i / resolution * self._num_led * 1 / 0.9)

                if anim_ind < percent_point:
                    frame = np.zeros((self._num_led, 3), dtype=np.uint8)
                    ind_1 = int(np.ceil(self._num_led / 2 - int(anim_ind) - 1))
                    ind_2 = int(np.floor(self._num_led / 2 + int(anim_ind)))
                    h = (self._h_max - self._h_min) * np.sin(
                        max(0, battery_percent) * np.pi / 2
                    ) * i / (display_iterations / 2) + self._h_min
                    rgb = np.array(hsv_to_rgb(h / 360, s, v))
                    frame[ind_1] = np.uint8(rgb * 255)
                    frame[ind_2] = np.uint8(rgb * 255)
                elif percent_point <= anim_ind < 2 * percent_point:
                    ind_1 = int(np.ceil(self._num_led / 2 - 2 * percent_point + int(anim_ind)))
                    ind_2 = int(np.floor(self._num_led / 2 + 2 * percent_point - int(anim_ind) - 1))
                    frame[ind_1] = np.uint8(rgb * 255)
                    frame[ind_2] = np.uint8(rgb * 255)

            anim[i] = frame

        img = Image.fromarray(anim)
        img = img.filter(ImageFilter.GaussianBlur(1.5))
        img = img.resize((self._num_led, self._anim_len))
        self._anim = np.array(img)
