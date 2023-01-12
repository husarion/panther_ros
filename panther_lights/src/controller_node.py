#!/usr/bin/env python3

from copy import deepcopy

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
from panther_apa102_driver import PantherAPA102Driver


class PantherAnimation:
    ANIMATION_DEFAULT_PRIORITY = 3
    ANIMATION_DEFAULT_TIMEOUT = 120

    front: Animation
    rear: Animation
    name: str = 'UNDEFINED'
    priority: int = ANIMATION_DEFAULT_PRIORITY
    timeout: float = ANIMATION_DEFAULT_TIMEOUT
    repeating: bool = False

    def __init__(self) -> None:
        self._init_time = rospy.get_time()

    def reset_time(self) -> None:
        self._init_time = rospy.get_time()

    @property
    def init_time(self) -> float:
        return self._init_time


class AnimationsQueue:
    def __init__(self, max_queue_size: int = 5) -> None:
        self._queue = []
        self._max_queue_size = max_queue_size

    def put(self, animation: PantherAnimation) -> None:
        if animation.priority == 1:
            self.clear()
        self.validate_queue()

        if len(self._queue) == self._max_queue_size:
            rospy.logwarn(f'{rospy.get_name()} Animation queue overloaded')
            return

        self._queue.append(animation)
        self._queue = sorted(self._queue, key=lambda x: (x.priority, x.init_time))

    def get(self) -> PantherAnimation:
        return self._queue.pop(0)

    def empty(self) -> bool:
        return len(self._queue) == 0

    def clear(self, priority: int = 2) -> None:
        # clears queue except items below specified priority
        self._queue = [animation for animation in self._queue if animation.priority < priority]

    def remove(self, animation: PantherAnimation) -> None:
        if self.has_animation(animation):
            self._queue.remove(animation)

    def has_animation(self, animation: PantherAnimation) -> bool:
        return animation in self._queue

    def validate_queue(self) -> None:
        time = rospy.get_time()
        remove_animation_list = [
            anim for anim in self._queue if time - anim.init_time > anim.timeout
        ]
        for anim in remove_animation_list:
            rospy.loginfo(
                f'{rospy.get_name()} Timeout for animation: {anim.name}. Romoving from the queue'
            )
            self._queue.remove(anim)

    @property
    def first_anim_priority(self) -> int:
        if not self.empty():
            return self._queue[0].priority
        else:
            return PantherAnimation.ANIMATION_DEFAULT_PRIORITY


class LightsControllerNode:
    MAX_BRIGHTNESS = 31
    PANEL_FRONT = 0
    PANEL_REAR = 1

    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._current_animation = None
        self._default_animation = None
        self._animation_finished = True
        self._anim_queue = AnimationsQueue(max_queue_size=10)
        self._animations = {}

        # check for required ROS parameters
        if not rospy.has_param('~animations'):
            rospy.logerr(
                f'{rospy.get_name()} Missing required parameter \'~animations\'. Shutting down'
            )
            rospy.signal_shutdown('Missing required parameter')
            return

        self._animations_description = rospy.get_param('~animations')
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
        self._controller = PantherAPA102Driver(
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

    def _percent_to_apa_driver_brightness(self, brightness_percent: float) -> int:
        if 0 <= brightness_percent <= 1:
            brightness = int(brightness_percent * LightsControllerNode.MAX_BRIGHTNESS)
            # set minimal brightness for small values
            if brightness == 0 and brightness_percent > 0:
                brightness = 1
            return brightness
        raise ValueError('Brightness out of range <0,1>')

    def _controller_timer_cb(self, *args) -> None:
        if self._animation_finished:
            self._anim_queue.validate_queue()

            if self._default_animation and not self._anim_queue.has_animation(
                self._default_animation
            ):
                self._default_animation.reset_time()
                self._anim_queue.put(self._default_animation)

            if not self._anim_queue.empty():
                self._current_animation = self._anim_queue.get()

        if self._current_animation:
            if self._current_animation.priority > self._anim_queue.first_anim_priority:
                if (
                    self._current_animation.front.progress < 0.9
                    and not self._current_animation.repeating
                ):
                    self._current_animation.front.reset()
                    self._current_animation.rear.reset()
                    self._anim_queue.put(self._current_animation)
                self._animation_finished = True
                self._current_animation = None
                return

            if not self._current_animation.front.finished:
                frame_front = self._current_animation.front()
                brightness_front = self._current_animation.front.brightness
                self._controller.set_panel_frame(
                    LightsControllerNode.PANEL_FRONT, frame_front, brightness_front
                )
            if not self._current_animation.rear.finished:
                frame_rear = self._current_animation.rear()
                brightness_rear = self._current_animation.rear.brightness
                self._controller.set_panel_frame(
                    LightsControllerNode.PANEL_REAR, frame_rear, brightness_rear
                )

            self._animation_finished = (
                self._current_animation.front.finished and self._current_animation.rear.finished
            )

            if self._animation_finished:
                self._current_animation.front.reset()
                self._current_animation.rear.reset()
                self._current_animation = None

    def _set_animation_cb(self, req: SetLEDAnimationRequest) -> SetLEDAnimationResponse:
        try:
            animation = deepcopy(self._animations[req.animation.id])
            animation.reset_time()
            animation.repeating = req.repeating
            self._add_animation_to_queue(animation)
        except ValueError:
            return SetLEDAnimationResponse(False, f'No Animation with id: {req.animation.id}')

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
        try:
            self._update_animations()
        except (KeyError, FileNotFoundError) as err:
            return TriggerResponse(False, f'Failed to update animations: {err}')
        return TriggerResponse(True, 'Animations updated successfully')

    def _add_animation_to_queue(self, animation: PantherAnimation) -> None:
        self._anim_queue.put(animation)
        if animation.repeating:
            self._anim_queue.remove(self._default_animation)
            self._default_animation = animation

    def _get_image_animation_description(self, animation: LEDImageAnimation) -> dict:
        if not animation.image:
            raise Exception('Missing required field \'image\'')

        animation_description = {
            'image': animation.image,
            'duration': animation.duration,
            'repeat': animation.repeat,
            'brightness': animation.brightness,
        }

        if animation.color:
            animation_description['color'] = animation.color

        return animation_description

    def _update_animations(self) -> None:

        user_animations = rospy.get_param('~user_animations', '')

        for animation in user_animations:
            # ID numbers from 0 to 19 are reserved for system animations
            if animation['id'] > 19:
                if 'priority' in animation:
                    if animation['priority'] == 1:
                        rospy.logwarn(
                            f'{rospy.get_name()} Ignoring user animation: {animation["name"]}. User animation can\'t have priority 1.'
                        )
                        continue

                rospy.loginfo(f'{rospy.get_name()} Adding user animation: {animation["name"]}')
                # remove old animation definition
                for anim in self._animations_description:
                    if anim['id'] == animation['id']:
                        self._animations_description.remove(anim)
                self._animations_description.append(animation)
            else:
                rospy.logwarn(
                    f'{rospy.get_name()} Ignoring user animation: {animation["name"]}. Animation ID must be greater than 19.'
                )

        self._update_animations_list()

    def _update_animations_list(self) -> None:
        for anim in self._animations_description:
            if 'id' in anim:

                animation = PantherAnimation()

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

                if 'name' in anim:
                    animation.name = anim['name']

                if 'priority' in anim:
                    priority = int(round(anim['priority']))
                    if not 0 < priority <= PantherAnimation.ANIMATION_DEFAULT_PRIORITY:
                        priority = PantherAnimation.ANIMATION_DEFAULT_PRIORITY
                        rospy.logwarn(
                            f'{rospy.get_name()} Invalid priority for animaiton: {animation.name}. Using default'
                        )
                    animation.priority = priority

                if 'timeout' in anim:
                    timeout = anim['timeout']
                    if timeout <= 0:
                        rospy.logwarn(
                            f'{rospy.get_name} Invalid timeout for animation: {animation.name}. Using default'
                        )
                        timeout = PantherAnimation.ANIMATION_DEFAULT_TIMEOUT
                    animation.timeout = timeout

                self._animations[anim['id']] = animation

            else:
                raise KeyError(f'Missing ID in animation description')


def main():
    lights_controller_node = LightsControllerNode('lights_controller_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
