#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from panther_msgs.msg import LEDAnimation
from panther_msgs.srv import SetLEDAnimation, SetLEDAnimationRequest


class LightsSchedulerNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        self._battery_percentage = 1.0
        self._charger_connected = False
        self._charging_percentage = -1.0
        self._e_stop_state = None
        self._led_e_stop_state = None

        self._critical_battery_anim_period = rospy.get_param('critical_battery_anim_period', 15.0)
        self._critical_battery_threshold_percent = rospy.get_param(
            'critical_battery_threshold_percent', 0.1
        )
        self._battery_state_anim_period = rospy.get_param('battery_state_anim_period', 120.0)
        self._low_battery_anim_period = rospy.get_param('low_battery_anim_period', 30.0)
        self._low_battery_threshold_percent = rospy.get_param('low_battery_threshold_percent', 0.4)
        self._update_charging_anim_step = rospy.get_param('update_charging_anim_step', 0.1)

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        rospy.Subscriber('battery', BatteryState, self._battery_cb, queue_size=1)
        rospy.Subscriber(
            'hardware/charger_connected', Bool, self._charger_connected_cb, queue_size=1
        )
        rospy.Subscriber('hardware/e_stop', Bool, self._e_stop_cb, queue_size=1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._set_led_animation = rospy.ServiceProxy(
            'lights/controller/set/animation', SetLEDAnimation
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._scheduler_timer = rospy.Timer(rospy.Duration(0.1), self._scheduler_timer_cb)
        self._critical_battery_timer = rospy.Timer(
            rospy.Duration(self._critical_battery_anim_period), self._critical_battery_timer_cb
        )
        self._battery_state_timer = rospy.Timer(
            rospy.Duration(self._battery_state_anim_period), self._battery_state_timer_cb
        )
        self._low_battery_timer = rospy.Timer(
            rospy.Duration(self._low_battery_anim_period), self._low_battery_timer_cb
        )

        rospy.loginfo(f'{rospy.get_name()} Node started')

    def _scheduler_timer_cb(self, *args) -> None:

        # call animation service only when e_stop state changes
        if self._led_e_stop_state != self._e_stop_state:
            req = SetLEDAnimationRequest()
            req.repeating = True
            if self._e_stop_state:
                req.animation.id = LEDAnimation.E_STOP
                success = self._call_led_animation_srv(req)
            else:
                req.animation.id = LEDAnimation.READY
                success = self._call_led_animation_srv(req)
            if success:
                self._led_e_stop_state = self._e_stop_state

        if self._charger_connected:
            if (
                abs(self._battery_percentage - self._charging_percentage)
                >= self._update_charging_anim_step
            ):
                self._charging_percentage = (
                    round(self._battery_percentage / self._update_charging_anim_step)
                    * self._update_charging_anim_step
                )
                req = SetLEDAnimationRequest()
                req.repeating = True
                req.animation.id = LEDAnimation.CHARGING_BATTERY
                req.animation.param = self._battery_percentage
                self._call_led_animation_srv(req)
        else:
            self._charging_percentage = -1.0

    def _critical_battery_timer_cb(self, *args) -> None:
        if (
            self._battery_percentage < self._critical_battery_threshold_percent
            and not self._charger_connected
        ):
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.CRITICAL_BATTERY
            self._call_led_animation_srv(req)

    def _battery_state_timer_cb(self, *args):
        req = SetLEDAnimationRequest()
        req.animation.id = LEDAnimation.BATTERY_STATE
        req.animation.param = self._battery_percentage
        self._call_led_animation_srv(req)

    def _low_battery_timer_cb(self, *args) -> None:
        if (
            self._critical_battery_threshold_percent
            <= self._battery_percentage
            < self._low_battery_threshold_percent
            and not self._charger_connected
        ):
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.LOW_BATTERY
            self._call_led_animation_srv(req)

    def _battery_cb(self, msg: BatteryState) -> None:
        self._battery_percentage = msg.percentage

    def _charger_connected_cb(self, msg: Bool) -> None:
        self._charger_connected = msg.data

    def _e_stop_cb(self, msg: Bool) -> None:
        self._e_stop_state = msg.data

    def _call_led_animation_srv(self, req: SetLEDAnimationRequest) -> bool:
        try:
            self._set_led_animation.wait_for_service(5.0)
            response = self._set_led_animation.call(req)
            rospy.logdebug(
                f'[{rospy.get_name()}] Setting animation with ID: {req.animation.id}. Response: ({response})'
            )
            return response.success
        except rospy.ServiceException as err:
            rospy.logerr(
                f'Calling {self._set_led_animation.resolved_name} service for message id {req.animation.id} failed. Error: {err}'
            )
            return False


def main():
    lights_scheduler_node = LightsSchedulerNode('lights_scheduler_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
