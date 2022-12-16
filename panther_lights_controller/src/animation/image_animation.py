import imageio
import numpy as np
import os
from PIL import Image

from .animation import Animation

import rospkg


class ImageAnimation(Animation):

    ANIMATION_NAME = 'image_animation'

    def __init__(self, anim_yaml, num_led) -> None:
        super().__init__(anim_yaml, num_led)

        if not 'image' in anim_yaml.keys():
            raise Animation.AnimationYAMLError('no image in parameters YAML')

        img_name = anim_yaml['image']
        if not os.path.isabs(img_name):
            rospack = rospkg.RosPack()
            img_path = os.path.join(
                rospack.get_path('panther_lights_controller') + f'/animations/{img_name}'
            )
        else:
            img_path = img_name

        # resize image to match duration
        original_img = imageio.imread(img_path)
        main_timer_hz = 100
        resized_image = Image.fromarray(original_img).resize(
            (num_led, int(self._duration * main_timer_hz))
        )
        self._img = np.array(resized_image)

        (img_y, _, _) = np.shape(self._img)
        self._img_y = img_y

        # overwrite animation's color
        if 'color' in anim_yaml.keys():
            color = anim_yaml['color']
            # change from hex to RGB
            r = (np.uint32(color) >> 16) & (0x0000FF)
            g = (np.uint32(color) >> 8) & (0x0000FF)
            b = (np.uint32(color)) & (0x0000FF)

            # turn image to grayscale
            self._img = (
                0.2989 * self._img[:, :, 0]
                + 0.5870 * self._img[:, :, 1]
                + 0.1140 * self._img[:, :, 2]
            )
            # normalize brightness
            self._img = self._img / np.max(self._img) * 255
            img_r = (self._img.astype(np.uint32) * r / 255).astype(np.uint8)
            img_g = (self._img.astype(np.uint32) * g / 255).astype(np.uint8)
            img_b = (self._img.astype(np.uint32) * b / 255).astype(np.uint8)
            # reconstruct image
            self._img = np.dstack((img_r, img_g, img_b))

        # convert image from RGB to HEX
        self._img = self._img.astype(np.uint32)
        r = self._img[:, :, 0] << 16
        g = self._img[:, :, 1] << 8
        b = self._img[:, :, 2]
        self._img = r + g + b
        self._img.astype(np.uint8)

        self._i = 0
        self._frame_time = self._duration / self._img_y

    def __call__(self) -> np.ndarray:
        '''returns new frame'''
        if self._i < self._img_y:
            frame = self._img[self._i, :]
            self._i += 1
            if self._i == self._img_y:
                self._i = 0
                self._current_cycle += 1
            if self._current_cycle > self._loops:
                self._finished = True
            return frame
        raise Animation.AnimationFinished

    def sleep_time(self) -> float:
        return self._frame_time

    def reset(self) -> None:
        self._i = 0
        self._current_cycle = 1
        self._finished = False
