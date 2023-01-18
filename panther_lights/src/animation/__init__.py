from .animation import Animation

from .battery_animation import BatteryAnimation
from .charging_animation import ChargingAnimation
from .image_animation import ImageAnimation

BASIC_ANIMATIONS = {
    BatteryAnimation.ANIMATION_NAME: BatteryAnimation,
    ChargingAnimation.ANIMATION_NAME: ChargingAnimation,
    ImageAnimation.ANIMATION_NAME: ImageAnimation,
}
