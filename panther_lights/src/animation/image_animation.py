import numpy as np
import os
from PIL import Image
import re

import rospkg

from .animation import Animation


class ImageAnimation(Animation):

    ANIMATION_NAME = 'image_animation'

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        if not 'image' in self._animation_description:
            raise KeyError('No image in aniamtion description')

        img_name = str(self._animation_description['image'])
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

        # convert image to RGB and resize to match duration
        original_img = Image.open(img_path).convert('RGB')
        resized_img = original_img.resize((self._num_led, self._anim_len))
        self._img = np.array(resized_img)

        # overwrite animation's color
        if 'color' in self._animation_description:
            self._set_image_color(self._animation_description['color'])

    def _set_image_color(self, color: int) -> None:
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
        return self._img[self._anim_iteration, :].tolist()
