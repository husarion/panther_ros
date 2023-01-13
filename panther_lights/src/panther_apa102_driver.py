#!/usr/bin/env python3

from threading import Lock

from apa102_pi.driver import apa102
import RPi.GPIO as GPIO


class PantherAPA102Driver:
    LED_SWITCH_FRONT_STATE = GPIO.HIGH
    LED_SWITCH_REAR_STATE = GPIO.LOW
    LED_POWER_ON_STATE = GPIO.LOW  # active LED with low state

    def __init__(
        self,
        num_led: int,
        panel_count: int = 2,
        brightness: int = 31,
        led_switch_pin: int = 20,
        led_power_pin: int = 26,
    ) -> None:

        self._num_led = num_led
        self._panel_count = panel_count

        self._is_running = True
        self._thread_finished = False
        self._lock = Lock()
        self._pixels = apa102.APA102(
            num_led=self._num_led, order='rgb', mosi=10, sclk=11, global_brightness=brightness
        )
        self._led_switch_pin = led_switch_pin
        self._led_power_pin = led_power_pin
        self._front_active = True
        self._rear_active = True
        color_correction_rgb = [255, 200, 62]
        # rgb values normalized to avoid additional division
        self._color_correction = [value / 255 for value in color_correction_rgb]

        # setup and activate panels
        self._setup_panels()

    def _setup_panels(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._led_switch_pin, GPIO.OUT)
        GPIO.setup(self._led_power_pin, GPIO.OUT)
        GPIO.output(
            (self._led_switch_pin, self._led_power_pin),
            (
                PantherAPA102Driver.LED_SWITCH_FRONT_STATE,
                PantherAPA102Driver.LED_POWER_ON_STATE,
            ),
        )

    def set_panel_frame(self, panel_num: int, panel_frame: list, brightness: int = 100) -> None:
        with self._lock:
            # select panel
            if panel_num == 0 and self._front_active:
                GPIO.output(self._led_switch_pin, PantherAPA102Driver.LED_SWITCH_FRONT_STATE)
            elif panel_num == 1 and self._rear_active:
                GPIO.output(self._led_switch_pin, PantherAPA102Driver.LED_SWITCH_REAR_STATE)
            else:
                raise ValueError('panther lights have only two panels')

            for i, pixel in enumerate(panel_frame):
                r = int(pixel[0] * self._color_correction[0])
                g = int(pixel[1] * self._color_correction[1])
                b = int(pixel[2] * self._color_correction[2])
                pixel_hex = (r << 16) + (g << 8) + b
                self._pixels.set_pixel_rgb(i, pixel_hex, brightness)
            self._pixels.show()

    def set_panel_state(self, panel_num: int, state: bool) -> None:
        if not state:
            self.clear_panel(panel_num)
        with self._lock:
            if panel_num == 0:
                self._front_active = state
            elif panel_num == 1:
                self._rear_active = state

    def set_brightness(self, brightness: int) -> None:
        with self._lock:
            self._pixels.global_brightness = brightness

    def clear_panel(self, panel_num: int) -> None:
        self.set_panel_frame(panel_num, [[0, 0, 0]] * self._num_led)

    def __del__(self) -> None:
        self.clear_panel(0)
        self.clear_panel(1)
        GPIO.output(self._led_power_pin, not PantherAPA102Driver.LED_POWER_ON_STATE)
