#!/usr/bin/env python3

from queue import Queue
from threading import Lock

import rospy

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import LEDImageAnimation
from panther_msgs.srv import SetLEDAnimation, SetLEDAnimationRequest, SetLEDAnimationResponse
from panther_msgs.srv import SetLEDBrightness, SetLEDBrightnessRequest, SetLEDBrightnessResponse
from panther_msgs.srv import (
    SetLEDImageAnimation,
    SetLEDImageAnimationRequest,
    SetLEDImageAnimationResponse,
)

from animation import Animation, BASIC_ANIMATIONS
import panther_apa102_driver


class PantherAnimation:
    front: Animation
    rear: Animation
    interrupting = False
    repeating = False


class LightsControllerNode:
    MAX_BRIGHTNESS = 31
    PANEL_FRONT = 0
    PANEL_REAR = 1

    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        self._current_animation = None
        self._default_animation = None
        self._interrupt = False
        self._animation_finished = True
        self._anim_queue = Queue()
        self._lock = Lock()

        # check for required ROS parameters
        if not rospy.has_param('~animations'):
            rospy.logerr(
                f'{rospy.get_name()} Missing required parameter \'~animations\'. Shutting down'
            )
            rospy.signal_shutdown('Missing required parameter')
            return

        self._animations = rospy.get_param('~animations')
        self._controller_frequency = rospy.get_param('~controller_frequency', 100)
        self._num_led = rospy.get_param('~num_led', 46)
        global_brightness = rospy.get_param('~global_brightness', 1.0)
        test = rospy.get_param('~test', False)

        self._update_animations()

        try:
            apa_driver_brightness = self._percent_to_apa_driver_brightness(global_brightness)
        except ValueError as err:
            rospy.logwarn(f'{rospy.get_name()} Invalid brightness: {err}. Using default')
            apa_driver_brightness = LightsControllerNode.MAX_BRIGHTNESS

        # define controller and clear all panels
        self._controller = panther_apa102_driver.PantherAPA102Driver(
            num_led=self._num_led, brightness=apa_driver_brightness
        )
        self._controller.clear_panel(LightsControllerNode.PANEL_FRONT)
        self._controller.clear_panel(LightsControllerNode.PANEL_REAR)

        # -------------------------------
        #   Services
        # -------------------------------

        self._set_animation_service = rospy.Service(
            'lights/controller/set/animation', SetLEDAnimation, self._set_animation_cb
        )
        self._set_brightness_service = rospy.Service(
            'lights/controller/set/brightness', SetLEDBrightness, self._set_brightness_cb
        )
        if test:
            self._set_image_animation_service = rospy.Service(
                'lights/controller/set/image_animation',
                SetLEDImageAnimation,
                self._set_image_animation_cb,
            )
        self._update_animations_service = rospy.Service(
            'lights/controller/update_animations', Trigger, self._update_animations_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._controller_timer = rospy.Timer(
            rospy.Duration(1 / self._controller_frequency), self._controller_timer_cb
        )

        rospy.loginfo(f'{rospy.get_name()} Node started')

    def _percent_to_apa_driver_brightness(self, brightness_percent) -> int:
        if 0 <= brightness_percent <= 1:
            brightness = int(brightness_percent * LightsControllerNode.MAX_BRIGHTNESS)
            # set minimal brightness for small values
            if brightness == 0 and brightness_percent > 0:
                brightness = 1
            return brightness
        raise ValueError('Brightness out of range <0,1>')

    def _controller_timer_cb(self, *args) -> None:
        with self._lock:
            if self._animation_finished or self._interrupt:
                if self._anim_queue.empty():
                    self._current_animation = self._default_animation
                else:
                    self._current_animation = self._anim_queue.get(block=False)
                    self._interrupt = False
                    print("new animation")

            if self._current_animation:
                if not self._current_animation.front.finished:
                    frame_front = self._current_animation.front()
                    brightness_front = self._current_animation.front.brightness
                    # self._controller.set_panel_frame(
                    #     LightsControllerNode.PANEL_FRONT, frame_front, brightness_front
                    # )
                if not self._current_animation.rear.finished:
                    frame_rear = self._current_animation.rear()
                    brightness_rear = self._current_animation.rear.brightness
                    # self._controller.set_panel_frame(
                    #     LightsControllerNode.PANEL_REAR, frame_rear, brightness_rear
                    # )

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
            animation.repeating = req.repeating
            self._add_animation_to_queue(animation)
        except (KeyError, FileNotFoundError) as err:
            return SetLEDAnimationResponse(False, f'{err}')

        return SetLEDAnimationResponse(
            True, f'Successfully set an animation with id {req.animation.id}'
        )

    def _set_brightness_cb(self, req: SetLEDBrightnessRequest) -> SetLEDBrightnessResponse:
        try:
            brightness = self._percent_to_apa_driver_brightness(req.data)
            self._controller.set_brightness(brightness)
        except ValueError as err:
            return SetLEDBrightnessResponse(False, f'{err}')

        return SetLEDBrightnessResponse(True, f'Changed brightness to {req.data}')

    def _set_image_animation_cb(
        self, req: SetLEDImageAnimationRequest
    ) -> SetLEDImageAnimationResponse:

        animation = PantherAnimation()
        animation.interrupting = req.interrupting
        animation.repeating = req.repeating

        try:
            animation_description_front = self._get_image_animation_description(req.front)
            animation_description_rear = self._get_image_animation_description(req.rear)

            animation.front = BASIC_ANIMATIONS['image_animation'](
                animation_description_front, self._num_led, self._controller_frequency
            )
            animation.rear = BASIC_ANIMATIONS['image_animation'](
                animation_description_rear, self._num_led, self._controller_frequency
            )

            self._add_animation_to_queue(animation)
        except Exception as err:
            return SetLEDImageAnimationResponse(False, f'{err}')

        return SetLEDImageAnimationResponse(True, f'Successfully set custom animation')

    def _update_animations_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._update_animations()
        return TriggerResponse(True, 'Animations updated successfully')

    def _get_animation_by_id(self, animation_id: int) -> PantherAnimation:

        animation = PantherAnimation()

        for anim in self._animations:
            if anim['id'] == animation_id:
                for panel in anim['animation']:
                    anim_desc = anim['animation'][panel]

                    if not 'type' in anim_desc:
                        raise KeyError('Missing \'type\' in animation description')

                    try:
                        BASIC_ANIMATIONS[anim_desc['type']]
                    except KeyError as err:
                        raise KeyError(f'Undefined animation type: {err}')

                    if panel == 'both':
                        animation.front = BASIC_ANIMATIONS[anim_desc['type']](
                            anim_desc, self._num_led, self._controller_frequency
                        )
                        animation.rear = BASIC_ANIMATIONS[anim_desc['type']](
                            anim_desc, self._num_led, self._controller_frequency
                        )
                        break
                    elif panel == 'front':
                        animation.front = BASIC_ANIMATIONS[anim_desc['type']](
                            anim_desc, self._num_led, self._controller_frequency
                        )
                    elif panel == 'rear':
                        animation.rear = BASIC_ANIMATIONS[anim_desc['type']](
                            anim_desc, self._num_led, self._controller_frequency
                        )
                    else:
                        raise KeyError(f'Invalid panel type: {panel}')

                if not hasattr(animation, 'front') or not hasattr(animation, 'rear'):
                    raise KeyError(
                        'Missing \'both\' or \'front\'/\'rear\' in animation description'
                    )

                if 'interrupting' in anim:
                    animation.interrupting = anim['interrupting']

                return animation

        raise KeyError(f'No Animation with id: {animation_id}')

    def _add_animation_to_queue(self, animation: PantherAnimation):
        self._interrupt = animation.interrupting
        if self._interrupt:
            self._anim_queue.queue.insert(0, animation)
        else:
            self._anim_queue.put(animation)
        if animation.repeating:
            self._default_animation = animation

    def _get_image_animation_description(self, animation: LEDImageAnimation):
        if not animation.image:
            raise Exception('missing required field \'image\'')

        animation_description = {
            'image': animation.image,
            'duration': animation.duration,
            'repeat': animation.repeat,
            'brightness': animation.brightness,
        }

        if animation.color:
            animation_description.update({'color': animation.color})

        return animation_description

    def _update_animations(self):

        user_animations = rospy.get_param('~user_animations', '')

        for animation in user_animations:
            # ID numbers from 0 to 19 are reserved for system animations
            if animation['id'] > 19:
                rospy.loginfo(f'{rospy.get_name()} Adding user animation: {animation["name"]}')
                # remove old animation definition
                for anim in self._animations:
                    if anim['id'] == animation['id']:
                        self._animations.remove(anim)
                self._animations.append(animation)
            else:
                rospy.logwarn(
                    f'{rospy.get_name()} Ignoring user animation: {animation["name"]}. Animation ID must be greater than 19.'
                )


def main():
    lights_controller_node = LightsControllerNode('lights_controller_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
