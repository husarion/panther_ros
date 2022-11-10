from apa102_pi.driver import apa102 
import RPi.GPIO as GPIO
import logging
from threading import Thread, Event, Lock
from enum import Enum
import sys
import time
from math import ceil
from queue import Queue

class Animation:
    class Type(Enum):
        NO_ANIMATION = 0
        FADE_COLOR = 1
        SOLID_COLOR = 2
        COLOR_WIPE_UP_FAST = 3
        COLOR_WIPE_UP_NORMAL = 4
        COLOR_WIPE_UP_SLOW = 5
        COLOR_WIPE_DOWN_FAST = 6
        COLOR_WIPE_DOWN_NORMAL = 7
        COLOR_WIPE_DOWN_SLOW = 8
        COLOR_WIPE_HALF_UP_FAST = 9
        COLOR_WIPE_HALF_DOWN_FAST = 10
        ONE_DOUBLE_COLOR_WIPE_NORMAL = 11
        ONE_DOUBLE_COLOR_WIPE_FAST = 12
        DOUBLE_COLOR_WIPE_NORMAL = 13
        DOUBLE_COLOR_WIPE_FAST = 14

    def __init__(self, num_pixel, default_animation=Type.NO_ANIMATION.value, default_color=0):
        self.num_pixel = num_pixel
        self.buffer = [0] * num_pixel
        self.frame_count = 0
        self.bright_percent = 100
        self.current_animation = default_animation
        self.animation_color = default_color

        self.even = True
        self.fade_steps_per_cycle = 30
        if self.num_pixel % 2 == 0:
            self.snake_middle_point = (int(ceil(self.num_pixel / 2.0) - 1), int(ceil(self.num_pixel / 2.0)))
        else:
            self.snake_middle_point = (int(ceil(self.num_pixel / 2.0) - 1), int(ceil(self.num_pixel / 2.0) - 1))
        self.snake_index = 0
        self.snake_wait = 6
        self.double_snake_index = [0, 0]

    def update(self):
        # compute new animation frame
        if self.current_animation is None:
            return False
        if self.current_animation == Animation.Type.FADE_COLOR.value:
            return self.fade_color()
        elif self.current_animation == Animation.Type.SOLID_COLOR.value:
            return self.solid_color()
        elif self.current_animation == Animation.Type.COLOR_WIPE_UP_SLOW.value:
            return self.snake(True, 3, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_UP_NORMAL.value:
            return self.snake(True, 1, True, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_UP_FAST.value:
            return self.snake(True, 3, True, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_DOWN_SLOW.value:
            return self.snake(False, 3, False, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_DOWN_NORMAL.value:
            return self.snake(False, 1, True, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_DOWN_FAST.value:
            return self.snake(False, 3, True, False)
        elif self.current_animation == Animation.Type.COLOR_WIPE_HALF_DOWN_FAST.value:
            return self.snake(False, 2, True, True)
        elif self.current_animation == Animation.Type.COLOR_WIPE_HALF_UP_FAST.value:
            return self.snake(True, 2, True, True)
        elif self.current_animation == Animation.Type.ONE_DOUBLE_COLOR_WIPE_NORMAL.value:
            return self.double_snake(2)
        elif self.current_animation == Animation.Type.ONE_DOUBLE_COLOR_WIPE_FAST.value:
            return self.double_snake(3)
        elif self.current_animation == Animation.Type.DOUBLE_COLOR_WIPE_NORMAL.value:
            return self.double_snake(2, False)
        elif self.current_animation == Animation.Type.DOUBLE_COLOR_WIPE_FAST.value:
            return self.double_snake(3, False)

    def fade_color(self):
        frame = self.frame_count % self.fade_steps_per_cycle
        if frame <= self.fade_steps_per_cycle // 2:
            fade = 2.0 / self.fade_steps_per_cycle * frame
        else:
            fade = 2.0 - (2.0 / self.fade_steps_per_cycle * frame)

        red = int(((self.animation_color >> 16) & 0xff) * fade)
        green = int(((self.animation_color >> 8) & 0xff) * fade)
        blue = int((self.animation_color & 0xff) * fade)

        for i in range(self.num_pixel):
            if self.even and i % 2 == 0:
                self.buffer[i] = 0
            else:
                self.buffer[i] = apa102.APA102.combine_color(red,green,blue)
        self.frame_count += 1
        return True

    def solid_color(self):
        if self.frame_count == 0:
            for i in range(self.num_pixel):
                if self.even:
                    self.buffer[i] = self.animation_color if i % 2 == 0 else 0x000000
                else:
                    self.buffer[i] = self.animation_color
            self.frame_count += 1
            return True
        else:
            return False

    def snake(self, direction, speed, mode, half=False):
        if self.frame_count == 0:
            if half:
                self.snake_index = self.snake_middle_point[1] if direction else self.snake_middle_point[0]  # delta
            else:
                self.snake_index = 0 if direction else self.num_pixel - 1  # delta
            for i in range(self.num_pixel):
                self.buffer[i] = 0

        if direction:
            if self.snake_index < self.num_pixel:
                if mode:
                    end_index = self.num_pixel if self.snake_index + speed > self.num_pixel else self.snake_index + speed
                    for i in range(self.snake_index, end_index):
                        self.buffer[i] = self.animation_color
                    self.snake_index = end_index
                elif self.frame_count % speed == 0:
                    self.buffer[self.snake_index] = self.animation_color
                    self.snake_index += 1
            elif self.snake_index - self.num_pixel < self.snake_wait:
                self.snake_index += 1
            elif self.snake_index - self.num_pixel == self.snake_wait:
                for i in range(self.num_pixel):
                    self.buffer[i] = 0
                self.snake_index += 1
            elif self.snake_index - self.num_pixel < 2 * self.snake_wait:
                self.snake_index += 1
            else:
                self.snake_index = self.snake_middle_point[1] if half else 0
        else:
            if self.snake_index >= 0:
                if mode:
                    end_index = -1 if self.snake_index - speed < 0 else self.snake_index - speed
                    for i in range(self.snake_index, end_index, -1):
                        self.buffer[i] = self.animation_color
                    self.snake_index = end_index
                elif self.frame_count % speed == 0:
                    self.buffer[self.snake_index] = self.animation_color
                    self.snake_index -= 1
            elif self.snake_index > - self.snake_wait - 1:
                self.snake_index -= 1
            elif self.snake_index == -self.snake_wait - 1:
                for i in range(self.num_pixel):
                    self.buffer[i] = 0
                self.snake_index -= 1
            elif self.snake_index > (-self.snake_wait) * 2 - 1:
                self.snake_index -= 1
            else:
                self.snake_index = self.snake_middle_point[0] if half else self.num_pixel - 1

        self.frame_count += 1
        return True

    def double_snake(self, speed, once=True):
        update_frame = False
        if (self.frame_count == 0) or \
           (not once and (self.double_snake_index[1] >= self.num_pixel + 16)):
            for i in range(self.num_pixel):
                self.buffer[i] = 0
            self.bright_percent = 70
            if self.num_pixel % 2 == 1:
                for i in range(2):
                    self.double_snake_index[i] = self.num_pixel // 2 - 1
                self.buffer[self.double_snake_index[0]] = self.animation_color
            else:
                self.double_snake_index[0] = self.num_pixel // 2 - 1
                self.double_snake_index[1] = self.num_pixel // 2
                for i in range(2):
                    self.buffer[self.double_snake_index[i]] = self.animation_color
            update_frame = True
        elif self.double_snake_index[1] < self.num_pixel:
            for i in range(self.double_snake_index[0], self.double_snake_index[1]):
                self.buffer[i] = self.animation_color
            tmp = self.double_snake_index[0] - speed
            self.double_snake_index[0] = tmp if tmp >= 0 else 0
            tmp = self.double_snake_index[1] + speed
            self.double_snake_index[1] = tmp if tmp < self.num_pixel else self.num_pixel
            update_frame = True
        elif self.double_snake_index[1] < self.num_pixel + 16:
            self.bright_percent += 10
            self.double_snake_index[1] += speed
            for i in range(0, self.num_pixel):
                self.buffer[i] = self.animation_color
            update_frame = True
        self.frame_count += 1
        return update_frame

class PantherLights(Thread):
    
    class AnimationMessage:
        def __init__(self, anim_type_front, anim_color_front, anim_type_rear, anim_color_rear):
            self.anim_type_front = anim_type_front
            self.anim_type_rear = anim_type_rear
            self.anim_color_front = anim_color_front
            self.anim_color_rear = anim_color_rear
   
    class BrightnessMessage:
        def __init__(self, brightness: float):
            self.brightness = brightness

    DELTA_TIME = 0.030
    LED_SWITCH_FRONT_STATE = True 
    LED_POWER_ON_STATE = False # active with low state 
    GLOBAL_MAX_BRIGHTNESS = 15 
    
    def __init__(self, event, queue, num_leds=73, led_switch_pin=20, led_power_pin = 26, brightness = GLOBAL_MAX_BRIGHTNESS, rear_enabled = True):
        '''Initialize PantherLights thread'''
        super().__init__(name="panther_lights_thread")
        self.__led_switch_pin = led_switch_pin
        self.__led_power_pin = led_power_pin
        # initialize apa102 module and SPI outputs
        self.__pixels = apa102.APA102(num_led=num_leds, order="rgb", mosi=10, sclk=11, global_brightness=brightness)
        # initialize gpio outputs 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__led_switch_pin, GPIO.OUT)
        GPIO.setup(self.__led_power_pin, GPIO.OUT)
        GPIO.output((self.__led_switch_pin, self.__led_power_pin), (PantherLights.LED_SWITCH_FRONT_STATE, PantherLights.LED_POWER_ON_STATE))
        self.__stop_event = event
        self.__queue = queue
        self.__animation_front = Animation(num_leds)
        self.__animation_rear = Animation(num_leds)
        self.__front_enabled = True
        self.__rear_enabled = rear_enabled 
        self.__lock = Lock()

    @property
    def front_enabled(self):
        return self.__front_enabled

    @property
    def rear_enabled(self):
        return self.__rear_enabled
     
    def enable_strip(self, front = True, rear = True):
        self.__lock.acquire()
        self.__front_enabled = front
        self.__rear_enabled = rear
        self.__lock.release()
   
    def run(self):
        logging.info("Thread %s: started!", self._name)
        while not self.__stop_event.is_set():
            new_msg = None

            # check the queue
            if not self.__queue.empty():
                new_msg = self.__queue.get(block=False)

                if isinstance(new_msg, PantherLights.AnimationMessage):

                    if new_msg.anim_type_front is not None and new_msg.anim_type_front in Animation.Type.__members__.keys():
                        self.__animation_front.current_animation = Animation.Type[new_msg.anim_type_front].value
                    else:
                        self.__animation_front.current_animation = None

                    if new_msg.anim_type_rear is not None and new_msg.anim_type_rear in Animation.Type.__members__.keys():
                        self.__animation_rear.current_animation = Animation.Type[new_msg.anim_type_rear].value
                    else:
                        self.__animation_rear.current_animation = None
    
                    self.__animation_front.animation_color = new_msg.anim_color_front
                    self.__animation_rear.animation_color = new_msg.anim_color_rear
                    self.__animation_front.frame_count = 0
                    self.__animation_rear.frame_count = 0

                elif isinstance(new_msg, PantherLights.BrightnessMessage):

                    self.__pixels.global_brightness = min(int(new_msg.brightness * 31), 31)

            if new_msg is not None:
                del new_msg

            # display new frame
            self.__lock.acquire()
            if self.__front_enabled and self.__animation_front.update():
                for i in range(self.__pixels.num_led):
                    self.__pixels.set_pixel_rgb(i,self.__animation_front.buffer[i],self.__animation_front.bright_percent)
                self.__pixels.show()
            GPIO.output(self.__led_switch_pin, not PantherLights.LED_SWITCH_FRONT_STATE)
            # time.sleep(0.01)
            if self.__rear_enabled and self.__animation_rear.update():
                for i in range(self.__pixels.num_led):
                    self.__pixels.set_pixel_rgb(i,self.__animation_rear.buffer[i],self.__animation_rear.bright_percent)
                self.__pixels.show()
            GPIO.output(self.__led_switch_pin, PantherLights.LED_SWITCH_FRONT_STATE)
            self.__lock.release()
            
            time.sleep(PantherLights.DELTA_TIME) 
        logging.info("Thread %s: closing!", self._name) 
        self.__pixels.cleanup()
        GPIO.output(self.__led_power_pin, not PantherLights.LED_POWER_ON_STATE) 
        # GPIO.cleanup()