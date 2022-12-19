#!/usr/bin/env python3

import os
import yaml
from queue import Queue
from threading import Lock

import rospkg
import rospy

from animation.image_animation import ImageAnimation
from panther_msgs.srv import SetLEDAnimation, SetLEDAnimationRequest, SetLEDAnimationResponse
from panther_msgs.srv import SetLEDBrightness, SetLEDBrightnessRequest, SetLEDBrightnessResponse
from panther_msgs.srv import (
    SetLEDImageAnimation,
    SetLEDImageAnimationRequest,
    SetLEDImageAnimationResponse,
)

import panther_apa102_driver


MAX_BRIGHTNESS = 31


class PantherAnimation:
    front: ImageAnimation
    rear: ImageAnimation
    interrupting = False


class LightsControllerNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        self._current_animation = None
        self._default_animation = None
        self._interrupt = False
        self._animation_finished = True
        self._anim_queue = Queue()
        self._lock = Lock()
        rospack = rospkg.RosPack()

        default_animation_description = os.path.join(
            rospack.get_path('panther_lights_controller') + '/config/panther_lights_animations.yaml'
        )

        self._animation_description = rospy.get_param(
            'animation_description', default_animation_description
        )

        global_brightness = rospy.get_param('global_brightness', 1.0)
        if 0.0 <= global_brightness <= 1.0:
            apa_driver_brightness = int(global_brightness * MAX_BRIGHTNESS)
        else:
            rospy.logwarn(f'{rospy.get_name()} Brightness out of range <0,1>. Using default.')
            apa_driver_brightness = MAX_BRIGHTNESS

        self._num_led = rospy.get_param('num_led', 46)
        self._controller_frequency = rospy.get_param('controller_frequency', 100)

        # define controller and clear all panels
        self._controller = panther_apa102_driver.PantherAPA102Driver(
            num_led=self._num_led, brightness=apa_driver_brightness
        )
        self._controller.clear_panel(0)
        self._controller.clear_panel(1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._set_animation_service = rospy.Service(
            'lights/controller/set/animation', SetLEDAnimation, self._set_animation_cb
        )
        self._set_image_animation_service = rospy.Service(
            'lights/controller/set/image_animation',
            SetLEDImageAnimation,
            self._set_image_animation_cb,
        )
        self._set_brightness_service = rospy.Service(
            'lights/controller/set/brightness', SetLEDBrightness, self._set_brightness_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._controller_timer = rospy.Timer(
            rospy.Duration(1 / self._controller_frequency), self._controller_timer_cb
        )

        rospy.loginfo(f'{rospy.get_name()} Node started')

    def _controller_timer_cb(self, *args) -> None:
        with self._lock:
            if self._animation_finished or self._interrupt:
                if self._anim_queue.empty():
                    self._current_animation = self._default_animation
                else:
                    self._current_animation = self._anim_queue.get(block=False)
                    self._interrupt = False

            if self._current_animation:
                if not self._current_animation.front.finished:
                    frame_front = self._current_animation.front()
                    brightness_front = self._current_animation.front.brightness
                    self._controller.set_panel_frame(0, frame_front, brightness_front)
                if not self._current_animation.rear.finished:
                    frame_rear = self._current_animation.rear()
                    brightness_rear = self._current_animation.rear.brightness
                    self._controller.set_panel_frame(1, frame_rear, brightness_rear)

                self._animation_finished = (
                    self._current_animation.front.finished and self._current_animation.rear.finished
                )

                if self._animation_finished:
                    self._current_animation.front.reset()
                    self._current_animation.rear.reset()
                    self._current_animation = None

    def _set_animation_cb(self, req: SetLEDAnimationRequest) -> SetLEDAnimationResponse:
        try:
            animation = self._get_animation_by_id(req.animation.id)
            self._interrupt = animation.interrupting
            self._anim_queue.put(animation)
            if req.repeat:
                self._default_animation = animation
        except ImageAnimation.AnimationYAMLError as err:
            rospy.logerr(err)
            return f'failure: {err}'

        return 'success'

    def _set_image_animation_cb(
        self, req: SetLEDImageAnimationRequest
    ) -> SetLEDImageAnimationResponse:
        pass

    def _set_brightness_cb(self, req: SetLEDBrightnessRequest) -> SetLEDBrightnessResponse:
        if 0 <= req.data <= 1:
            brightness = int(req.data * MAX_BRIGHTNESS)
            self._controller.set_brightness(brightness)
        else:
            return f'brightness out of range <0,1>'
        return f'brightness: {req.data}'

    def _get_animation_by_id(self, animation_id) -> ImageAnimation:

        with open(self._animation_description, 'r') as yaml_file:
            try:
                animation_description = yaml.safe_load(yaml_file)
            except yaml.YAMLError as err:
                rospy.logerr(err)

        animation = PantherAnimation()

        for anim in animation_description['animations']:
            if anim['id'] == animation_id:
                if 'both' in anim['animation']:
                    animation.front = ImageAnimation(
                        anim['animation']['both'], self._num_led, self._controller_frequency
                    )
                    animation.rear = ImageAnimation(
                        anim['animation']['both'], self._num_led, self._controller_frequency
                    )
                elif 'front' in anim['animation'] and 'rear' in anim['animation']:
                    animation.front = ImageAnimation(
                        anim['animation']['front'], self._num_led, self._controller_frequency
                    )
                    animation.rear = ImageAnimation(
                        anim['animation']['rear'], self._num_led, self._controller_frequency
                    )
                else:
                    raise ImageAnimation.AnimationYAMLError(
                        'Missing \'both\' or \'front\'/\'rear\' in animation description'
                    )

                if 'interrupting' in anim:
                    animation.interrupting = anim['interrupting']

                return animation

        raise ImageAnimation.AnimationYAMLError(f'No Animation with id: {animation_id}')


def main():
    lights_controller_node = LightsControllerNode('lights_controller_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
