from .animation import Animation

from .battery_animation import BatteryAnimation
from .image_animation import ImageAnimation

BASIC_ANIMATIONS = {
    BatteryAnimation.ANIMATION_NAME: BatteryAnimation,
    ImageAnimation.ANIMATION_NAME: ImageAnimation,
}
