#!/usr/bin/env python3

from dataclasses import dataclass
from threading import Lock

from apa102_pi.driver import apa102
import RPi.GPIO as GPIO

import rospy

from sensor_msgs.msg import Image

from panther_msgs.srv import SetLEDBrightness, SetLEDBrightnessRequest, SetLEDBrightnessResponse


@dataclass
class LEDConstants:
    LED_SWITCH_FRONT_STATE = GPIO.HIGH
    LED_SWITCH_REAR_STATE = GPIO.LOW
    LED_POWER_ON_STATE = GPIO.LOW  # active LED with low state
    LED_SWITCH_PIN = 20
    LED_POWER_PIN = 26
    LED_MAX_BRIGHTNESS = 31
    PANEL_FRONT = 0
    PANEL_REAR = 1

class LightsDriverNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        global_brightness = rospy.get_param('~global_brightness', 1.0)
        self._frame_timeout = rospy.get_param('~frame_timeout', 0.1)
        self._num_led = rospy.get_param('~num_led', 46)  # has to match controller num_led

        try:
            apa_driver_brightness = self._percent_to_apa_driver_brightness(global_brightness)
        except ValueError as err:
            rospy.logwarn(f'[{rospy.get_name()}] Invalid brightness: {err}. Using default')
            apa_driver_brightness = LEDConstants.LED_MAX_BRIGHTNESS

        self._lock = Lock()
        self._pixels = apa102.APA102(
            num_led=self._num_led,
            order='rgb',
            mosi=10,
            sclk=11,
            global_brightness=apa_driver_brightness,
        )
        self._front_active = True
        self._rear_active = True
        self._last_time_stamp_front = rospy.Time.now()
        self._last_time_stamp_rear = rospy.Time.now()
        color_correction_rgb = [255, 200, 62]
        # rgb values normalized to avoid additional division
        self._color_correction = [value / 255 for value in color_correction_rgb]

        # setup and activate panels
        self._setup_panels()

        # clear panels
        self._clear_panel(LEDConstants.PANEL_FRONT)
        self._clear_panel(LEDConstants.PANEL_REAR)

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._front_frame_sub = rospy.Subscriber(
            'lights/driver/front_panel_frame', Image, self._front_frame_cb, queue_size=3
        )
        self._rear_frame_sub = rospy.Subscriber(
            'lights/driver/rear_panel_frame', Image, self._rear_frame_cb, queue_size=3
        )

        # -------------------------------
        #   Services
        # -------------------------------

        self._set_brightness_service = rospy.Service(
            'lights/driver/set/brightness', SetLEDBrightness, self._set_brightness_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _percent_to_apa_driver_brightness(self, brightness_percent: float) -> int:
        if 0.0 <= brightness_percent <= 1.0:
            brightness = int(brightness_percent * LEDConstants.LED_MAX_BRIGHTNESS)
            # set minimal brightness for small values
            if brightness == 0 and brightness_percent > 0:
                brightness = 1
            return brightness
        raise ValueError('Brightness out of range <0,1>')

    def _setup_panels(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEDConstants.LED_SWITCH_PIN, GPIO.OUT)
        GPIO.setup(LEDConstants.LED_POWER_PIN, GPIO.OUT)
        GPIO.output(
            (LEDConstants.LED_SWITCH_PIN, LEDConstants.LED_POWER_PIN),
            (
                LEDConstants.LED_SWITCH_FRONT_STATE,
                LEDConstants.LED_POWER_ON_STATE,
            ),
        )

    def _front_frame_cb(self, image: Image) -> None:
        if (rospy.Time.now() - image.header.stamp) > rospy.Duration(self._frame_timeout):
            rospy.logwarn(f'[{rospy.get_name()}] Front frame timeout exceeded, ignoring frame.')
            return
        if (image.header.stamp < self._last_time_stamp_front):
            rospy.logwarn(f'[{rospy.get_name()}] Dropping message from past for front panel.')
            return
        self._last_time_stamp_front = image.header.stamp
        rgb_frame, brightness = self._decode_img_msg(image)
        self._set_panel_frame(LEDConstants.PANEL_FRONT, rgb_frame, brightness)

    def _rear_frame_cb(self, image: Image) -> None:
        if (rospy.Time.now() - image.header.stamp) > rospy.Duration(self._frame_timeout):
            rospy.logwarn(f'[{rospy.get_name()}] Rear frame timeout exceeded, ignoring frame.')
            return
        if (image.header.stamp < self._last_time_stamp_rear):
            rospy.logwarn(f'[{rospy.get_name()}] Dropping message from past for rear panel.')
            return
        self._last_time_stamp_rear = image.header.stamp
        rgb_frame, brightness = self._decode_img_msg(image)
        self._set_panel_frame(LEDConstants.PANEL_REAR, rgb_frame, brightness)

    def _decode_img_msg(self, image: Image) -> tuple:
        rgb_frame = [[image.data[i + j] for j in range(3)] for i in range(0, len(image.data), 4)]
        brightness = image.data[3]
        return rgb_frame, brightness

    def _set_panel_frame(self, panel_num: int, panel_frame: list, brightness: int = 255) -> None:
        with self._lock:
            # select panel
            if panel_num == LEDConstants.PANEL_FRONT and self._front_active:
                GPIO.output(
                    LEDConstants.LED_SWITCH_PIN, LEDConstants.LED_SWITCH_FRONT_STATE
                )
            elif panel_num == LEDConstants.PANEL_REAR and self._rear_active:
                GPIO.output(LEDConstants.LED_SWITCH_PIN, LEDConstants.LED_SWITCH_REAR_STATE)
            else:
                raise ValueError('panther lights have only two panels')

            for i, pixel in enumerate(panel_frame):
                r = int(pixel[0] * self._color_correction[0])
                g = int(pixel[1] * self._color_correction[1])
                b = int(pixel[2] * self._color_correction[2])
                pixel_hex = (r << 16) + (g << 8) + b
                self._pixels.set_pixel_rgb(i, pixel_hex, int(brightness / 255 * 100))
            self._pixels.show()

    def _set_brightness_cb(self, req: SetLEDBrightnessRequest) -> SetLEDBrightnessResponse:
        try:
            brightness = self._percent_to_apa_driver_brightness(req.data)
            self._pixels.global_brightness = brightness
        except ValueError as err:
            return SetLEDBrightnessResponse(False, f'{err}')

        return SetLEDBrightnessResponse(True, f'Changed brightness to {req.data}')

    def _clear_panel(self, panel_num: int) -> None:
        self._set_panel_frame(panel_num, [[0, 0, 0]] * self._num_led)

    def __del__(self) -> None:
        self._clear_panel(LEDConstants.PANEL_FRONT)
        self._clear_panel(LEDConstants.PANEL_REAR)
        GPIO.output(LEDConstants.LED_POWER_PIN, not LEDConstants.LED_POWER_ON_STATE)


def main():
    lights_driver_node = LightsDriverNode('lights_driver_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
