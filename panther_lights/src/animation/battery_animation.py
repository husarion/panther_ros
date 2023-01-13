from colorsys import hsv_to_rgb
import numpy as np
from scipy.signal import convolve2d

from animation import Animation


class BatteryAnimation(Animation):

    ANIMATION_NAME = 'battery_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._anim = np.zeros((self._anim_len, self.num_led))
        self._h_min = 5
        self._h_max = 120

    def _update_frame(self) -> list:
        return self._anim[self._anim_iteration, :]

    def set_param(self, value: float) -> None:
        self._param = value
        self._create_anim(value)

    def _create_anim(self, battery_percent: float) -> None:
        anim = np.zeros((self._anim_len, self._num_led))
        frame = np.zeros(self._num_led)

        if self._num_led % 2 == 0:
            percent_point = int(round(battery_percent * self._num_led / 2))
        else:
            percent_point = int(np.ceil(battery_percent * self._num_led / 2))

        # 0.9 was added to fully display animation when percentage is 1.0
        display_iterations = battery_percent * self._anim_len * 0.9

        if percent_point < 1:
            percent_point = 1
            display_iterations = 0

        # create binary animation
        for i in range(self._anim_len):
            if i <= display_iterations:
                anim_ind = round(i / self._anim_len * self._num_led * 1 / 0.9)
                if anim_ind < percent_point:
                    frame = np.zeros(self._num_led)
                    ind_1 = int(np.ceil(self._num_led / 2 - int(anim_ind) - 1))
                    ind_2 = int(np.floor(self._num_led / 2 + int(anim_ind)))
                    frame[ind_1] = 1
                    frame[ind_2] = 1
                elif percent_point <= anim_ind < 2 * percent_point:
                    ind_1 = int(np.ceil(self._num_led / 2 - 2 * percent_point + int(anim_ind)))
                    ind_2 = int(np.floor(self._num_led / 2 + 2 * percent_point - int(anim_ind) - 1))
                    frame[ind_1] = 1
                    frame[ind_2] = 1
            anim[i] = frame

        # convolve animation for smoother display
        if not display_iterations < 1:
            conv_mat = np.ones((3, 3), np.float32) / 9
            anim = convolve2d(anim, conv_mat, mode='same', fillvalue=1, boundary='symm')

        # set final animation color
        h = (self._h_max - self._h_min) * pow(max(0, battery_percent), 0.75) + self._h_min
        s = 1.0
        v = 1.0

        (r, g, b) = hsv_to_rgb(h / 360, s, v)
        r = np.uint8(r * 255)
        g = np.uint8(g * 255)
        b = np.uint8(b * 255)

        anim_r = np.uint32(anim * r) << 16
        anim_g = np.uint32(anim * g) << 8
        anim_b = np.uint32(anim * b)

        # dynamically change color during first part of the animation
        for i in range(round(display_iterations / 2)):
            anim_iter = i / (display_iterations / 2)
            h = (self._h_max - self._h_min) * pow(battery_percent, 0.75) * anim_iter + self._h_min

            (r, g, b) = hsv_to_rgb(h / 360, s, v)
            r = np.uint8(r * 255)
            g = np.uint8(g * 255)
            b = np.uint8(b * 255)

            anim_r[i] = np.uint32(anim[i] * r) << 16
            anim_g[i] = np.uint32(anim[i] * g) << 8
            anim_b[i] = np.uint32(anim[i] * b)

        self._anim = anim_r + anim_g + anim_b
