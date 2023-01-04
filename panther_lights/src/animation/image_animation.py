import imageio
import numpy as np
import os
from PIL import Image
import re

import rospkg

from .animation import Animation


class ImageAnimation(Animation):

    ANIMATION_NAME = 'image_animation'

    def __init__(self, animation_description: dict, num_led: int, controller_freq: float) -> None:
        super().__init__(animation_description, num_led, controller_freq)

        if not 'image' in animation_description:
            raise KeyError('No image in aniamtion description')

        img_name = animation_description['image']
        if not os.path.isabs(img_name):
            if img_name[0] == '$':
                if re.search('^\$\(find .*\)', img_name):
                    path_sub_strings = re.split('(?<=\))', img_name, 1)
                    animation_package = re.sub('^\$\(find |\)$', '', path_sub_strings[0])
                    img_name = path_sub_strings[1]
                else:
                    raise KeyError('Can\'t process substitution expression')

                rospack = rospkg.RosPack()
                try:
                    img_path = os.path.join(rospack.get_path(animation_package) + img_name)
                except rospkg.ResourceNotFound:
                    raise KeyError(f'Can\'t find ROS package: {animation_package}')
            else:
                raise KeyError('Invalid image path')
        else:
            img_path = img_name

        # resize image to match duration
        original_img = imageio.imread(img_path)
        resized_img = Image.fromarray(original_img).resize((num_led, self._anim_len))
        self._img = np.array(resized_img)

        # overwrite animation's color
        if 'color' in animation_description:
            self._set_image_color(animation_description['color'])

        # convert image from RGB to HEX
        self._img = self._img.astype(np.uint32)
        r = self._img[:, :, 0] << 16
        g = self._img[:, :, 1] << 8
        b = self._img[:, :, 2]
        self._img = r + g + b
        self._img.astype(np.uint8)

    def _set_image_color(self, color: int):
        # change from hex to RGB
        r = (np.uint32(color) >> 16) & (0x0000FF)
        g = (np.uint32(color) >> 8) & (0x0000FF)
        b = (np.uint32(color)) & (0x0000FF)

        # turn image to grayscale
        self._img = (
            0.2989 * self._img[:, :, 0] + 0.5870 * self._img[:, :, 1] + 0.1140 * self._img[:, :, 2]
        )
        # normalize brightness
        self._img = self._img / np.max(self._img) * 255
        img_r = (self._img.astype(np.uint32) * r / 255).astype(np.uint8)
        img_g = (self._img.astype(np.uint32) * g / 255).astype(np.uint8)
        img_b = (self._img.astype(np.uint32) * b / 255).astype(np.uint8)
        # reconstruct image
        self._img = np.dstack((img_r, img_g, img_b))

    def _update_frame(self) -> list:
        return self._img[self._anim_iteration, :]
