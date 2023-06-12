#!/usr/bin/env python3

from copy import deepcopy
from threading import Lock

import rospy

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import LEDAnimationQueue, LEDImageAnimation
from panther_msgs.srv import SetLEDAnimation, SetLEDAnimationRequest, SetLEDAnimationResponse
from panther_msgs.srv import (
    SetLEDImageAnimation,
    SetLEDImageAnimationRequest,
    SetLEDImageAnimationResponse,
)

from animation import Animation, BASIC_ANIMATIONS


class PantherAnimation:
    ANIMATION_DEFAULT_PRIORITY: int = 3
    ANIMATION_DEFAULT_TIMEOUT: float = 120.0

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

    @init_time.setter
    def init_time(self, init_time: float) -> None:
        self._init_time = init_time


class AnimationsQueue:
    def __init__(self, max_queue_size: int = 5) -> None:
        self._queue = []
        self._max_queue_size = max_queue_size

    def put(self, animation: PantherAnimation) -> None:
        if animation.priority == 1:
            self.clear()
        self.validate_queue()

        if len(self._queue) == self._max_queue_size:
            rospy.logwarn(f'[{rospy.get_name()}] Animation queue overloaded')
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
                f'[{rospy.get_name()}] Timeout for animation: {anim.name}. Romoving from the queue'
            )
            self._queue.remove(anim)

    @property
    def first_anim_priority(self) -> int:
        if not self.empty():
            return self._queue[0].priority
        else:
            return PantherAnimation.ANIMATION_DEFAULT_PRIORITY

    @property
    def queue(self) -> list:
        return self._queue


class LightsControllerNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._lock = Lock()

        # check for required ROS parameters
        if not rospy.has_param('~animations'):
            rospy.logerr(
                f'[{rospy.get_name()}] Missing required parameter \'~animations\'. Shutting down'
            )
            rospy.signal_shutdown('Missing required parameter')
            return

        self._animations_description = rospy.get_param('~animations')
        self._controller_frequency = rospy.get_param('~controller_frequency', 46.0)
        self._num_led = rospy.get_param('~num_led', 46)
        test = rospy.get_param('~test', False)

        self._anim_queue = AnimationsQueue(max_queue_size=10)
        self._animation_finished = True
        self._animations = {}
        self._current_animation = None
        self._default_animation = None
        self._empty_frame = [[0, 0, 0]] * self._num_led

        self._update_default_animations()
        self._update_user_animations()

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._front_frame_pub = rospy.Publisher(
            'lights/driver/front_panel_frame', Image, queue_size=10
        )
        self._rear_frame_pub = rospy.Publisher(
            'lights/driver/rear_panel_frame', Image, queue_size=10
        )
        self._animation_queue_pub = rospy.Publisher(
            'lights/controller/queue', LEDAnimationQueue, queue_size=10
        )

        # -------------------------------
        #   Service servers
        # -------------------------------

        self._set_animation_server = rospy.Service(
            'lights/controller/set/animation', SetLEDAnimation, self._set_animation_cb
        )
        if test:
            self._set_image_animation_server = rospy.Service(
                'lights/controller/set/image_animation',
                SetLEDImageAnimation,
                self._set_image_animation_cb,
            )
        self._update_animations_server = rospy.Service(
            'lights/controller/update_animations', Trigger, self._update_animations_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._controller_timer = rospy.Timer(
            rospy.Duration(1 / self._controller_frequency), self._controller_timer_cb
        )
        # Running at 2Hz
        self._animation_queue_timer = rospy.Timer(
            rospy.Duration(0.5), self._animation_queue_timer_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _controller_timer_cb(self, *args) -> None:
        with self._lock:
            brightness_front = 255
            brightness_rear = 255
            frame_front = self._empty_frame
            frame_rear = self._empty_frame

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
                    if self._current_animation.repeating:
                        self._current_animation.front.reset()
                        self._current_animation.rear.reset()
                    elif self._current_animation.front.progress < 0.65:
                        self._current_animation.front.reset()
                        self._current_animation.rear.reset()
                        self._anim_queue.put(self._current_animation)
                    self._animation_finished = True
                    self._current_animation = None
                    return

                if not self._current_animation.front.finished:
                    frame_front = self._current_animation.front()
                    brightness_front = self._current_animation.front.brightness
                if not self._current_animation.rear.finished:
                    frame_rear = self._current_animation.rear()
                    brightness_rear = self._current_animation.rear.brightness
                self._animation_finished = (
                    self._current_animation.front.finished and self._current_animation.rear.finished
                )

                if self._animation_finished:
                    self._current_animation.front.reset()
                    self._current_animation.rear.reset()
                    self._current_animation = None

            self._front_frame_pub.publish(
                self._rgb_frame_to_img_msg(frame_front, brightness_front, 'front_light_link')
            )
            self._rear_frame_pub.publish(
                self._rgb_frame_to_img_msg(frame_rear, brightness_rear, 'rear_light_link')
            )

    def _animation_queue_timer_cb(self, *args) -> None:
        with self._lock:
            anim_queue_msg = LEDAnimationQueue()
            if not self._anim_queue.empty():
                anim_queue_msg.queue = [anim.name for anim in self._anim_queue.queue]
            if self._current_animation:
                anim_queue_msg.queue.insert(0, self._current_animation.name)
            self._animation_queue_pub.publish(anim_queue_msg)

    def _set_animation_cb(self, req: SetLEDAnimationRequest) -> SetLEDAnimationResponse:
        if not req.animation.id in self._animations:
            return SetLEDAnimationResponse(False, f'No Animation with id: {req.animation.id}')

        try:
            animation = deepcopy(self._animations[req.animation.id])
            animation.front.set_param(req.animation.param)
            animation.rear.set_param(req.animation.param)
            animation.reset_time()
            animation.repeating = req.repeating
            self._add_animation_to_queue(animation)
        except ValueError as err:
            return SetLEDAnimationResponse(False, f'Failed to add animation to queue: {err}')

        return SetLEDAnimationResponse(
            True, f'Successfully set an animation with id {req.animation.id}'
        )

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
            self._update_user_animations()
        except Exception as err:
            return TriggerResponse(False, f'Failed to update animations: {err}')
        return TriggerResponse(True, 'Animations updated successfully')

    def _rgb_frame_to_img_msg(self, rgb_frame: list, brightness: int, frame_id: str) -> Image:
        img_msg = Image()
        img_msg.header.frame_id = frame_id
        img_msg.header.stamp = rospy.Time.now()
        img_msg.encoding = 'rgba8'
        img_msg.height = 1
        img_msg.width = self._num_led
        img_msg.step = 4 * self._num_led

        rgba_array = [val for rgba in [rgb + [brightness] for rgb in rgb_frame] for val in rgba]
        img_msg.data = bytes(rgba_array)

        return img_msg

    def _add_animation_to_queue(self, animation: PantherAnimation) -> None:
        if animation.repeating:
            interupting_animation = deepcopy(animation)
            interupting_animation.init_time -= 0.0001
            if interupting_animation.priority > 2:
                interupting_animation.priority = 2
            self._anim_queue.put(interupting_animation)
            self._anim_queue.remove(self._default_animation)
            self._default_animation = animation
        self._anim_queue.put(animation)

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

    def _update_default_animations(self) -> None:
        for animation in self._animations_description:
            self._update_animations_dict(animation)

    def _update_user_animations(self) -> None:
        user_animations = rospy.get_param('~user_animations', '')

        for animation in user_animations:
            if not 'id' in animation:
                rospy.logwarn(f'[{rospy.get_name()}] Ignoring user animation with missing ID.')
                continue

            if not isinstance(animation['id'], int):
                rospy.logwarn(f'[{rospy.get_name()}] Ignoring user animation with invalid ID.')
                continue

            if not 'name' in animation:
                animation['name'] = 'ANIMATION_' + str(animation['id'])

            # ID numbers from 0 to 19 are reserved for system animations
            if animation['id'] > 19:
                if 'priority' in animation:
                    if animation['priority'] == 1:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] Ignoring user animation: {animation["name"]}. User animation can not have priority 1.'
                        )
                        continue

                try:
                    self._update_animations_dict(animation)
                except (FileNotFoundError, KeyError, ValueError, TypeError) as err:
                    rospy.logwarn(
                        f'[{rospy.get_name()}] Failed to add animation: {animation["name"]}. Reason: {err}'
                    )
                    continue

                rospy.loginfo(
                    f'[{rospy.get_name()}] Successfuly added user animation: {animation["name"]}'
                )
            else:
                rospy.logwarn(
                    f'[{rospy.get_name()}] Ignoring user animation: {animation["name"]}. Animation ID must be greater than 19.'
                )

    def _update_animations_dict(self, anim: dict) -> None:
        if 'id' in anim:
            if not isinstance(anim['id'], int):
                raise KeyError('Invalid animation ID')

            animation = PantherAnimation()

            for panel in anim['animation']:
                if panel not in ('both', 'front', 'rear'):
                    raise KeyError(f'Invalid panel type: {panel}')

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
                raise KeyError('Missing \'both\' or \'front\'/\'rear\' in animation description')

            if 'name' in anim:
                animation.name = str(anim['name'])
            else:
                animation.name = 'ANIMATION_' + str(anim['id'])

            if 'priority' in anim:
                priority = anim['priority']
                if not isinstance(priority, int):
                    priority = PantherAnimation.ANIMATION_DEFAULT_PRIORITY
                    rospy.logwarn(
                        f'[{rospy.get_name()}] Invalid priority for animaiton: {animation.name}. Using default: {priority}'
                    )
                elif not 0 < priority <= PantherAnimation.ANIMATION_DEFAULT_PRIORITY:
                    priority = PantherAnimation.ANIMATION_DEFAULT_PRIORITY
                    rospy.logwarn(
                        f'[{rospy.get_name()}] Invalid priority for animaiton: {animation.name}. Using default: {priority}'
                    )
                animation.priority = priority

            if 'timeout' in anim:
                timeout = float(anim['timeout'])
                if timeout <= 0:
                    timeout = PantherAnimation.ANIMATION_DEFAULT_TIMEOUT
                    rospy.logwarn(
                        f'[{rospy.get_name()}] Invalid timeout for animation: {animation.name}. Using default: {timeout}'
                    )
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
