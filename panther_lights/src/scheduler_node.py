#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from panther_msgs.msg import LEDAnimation
from panther_msgs.srv import SetLEDAnimation, SetLEDAnimationRequest


class LightsSchedulerNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._battery_percentage = 1.0
        self._battery_status = None
        self._charging_percentage = -1.0  # -1.0 to trigger animation when charger gets connected
        self._e_stop_state = None
        self._led_e_stop_state = None
        self._charging_battery_timer = None
        self._charger_connected_states = [
            BatteryState.POWER_SUPPLY_STATUS_FULL,
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
        ]

        self._critical_battery_anim_period = rospy.get_param('~critical_battery_anim_period', 15.0)
        self._critical_battery_threshold_percent = rospy.get_param(
            '~critical_battery_threshold_percent', 0.1
        )
        self._battery_state_anim_period = rospy.get_param('~battery_state_anim_period', 120.0)
        self._low_battery_anim_period = rospy.get_param('~low_battery_anim_period', 30.0)
        self._low_battery_threshold_percent = rospy.get_param('~low_battery_threshold_percent', 0.4)
        self._update_charging_anim_step = rospy.get_param('~update_charging_anim_step', 0.1)

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        self._battery_sub = rospy.Subscriber(
            'battery', BatteryState, self._battery_cb, queue_size=1
        )
        self._e_stop_sub = rospy.Subscriber('hardware/e_stop', Bool, self._e_stop_cb, queue_size=1)

        # -------------------------------
        #   Service clients
        # -------------------------------

        self._set_led_client = rospy.ServiceProxy(
            'lights/controller/set/animation', SetLEDAnimation
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._scheduler_timer = rospy.Timer(rospy.Duration(0.2), self._scheduler_timer_cb)  # 5 Hz
        self._critical_battery_timer = rospy.Timer(
            rospy.Duration(self._critical_battery_anim_period), self._critical_battery_timer_cb
        )
        self._battery_state_timer = rospy.Timer(
            rospy.Duration(self._battery_state_anim_period), self._battery_state_timer_cb
        )
        self._low_battery_timer = rospy.Timer(
            rospy.Duration(self._low_battery_anim_period), self._low_battery_timer_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _scheduler_timer_cb(self, *args) -> None:
        # call animation service only when e_stop state changes
        if (
            self._led_e_stop_state != self._e_stop_state
            and not self._battery_status in self._charger_connected_states
        ):
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

        if (
            self._battery_status in self._charger_connected_states
            and not self._charging_battery_timer
        ):
            self._charging_battery_timer_cb()  # manually trigger timers callback
            self._charging_battery_timer = rospy.Timer(
                rospy.Duration(6.0),
                self._charging_battery_timer_cb,
            )
        elif (
            not self._battery_status in self._charger_connected_states
            and self._charging_battery_timer
        ):
            self._charging_battery_timer.shutdown()
            self._charging_battery_timer.join()
            del self._charging_battery_timer
            self._charging_battery_timer = None
            self._charging_percentage = -1.0
            self._led_e_stop_state = None

    def _charging_battery_timer_cb(self, *args) -> None:
        if self._e_stop_state:
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.E_STOP
            self._call_led_animation_srv(req)

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
            req.animation.param = str(self._battery_percentage)
            self._call_led_animation_srv(req)

    def _critical_battery_timer_cb(self, *args) -> None:
        if (
            self._battery_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            and self._battery_percentage < self._critical_battery_threshold_percent
        ):
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.CRITICAL_BATTERY
            self._call_led_animation_srv(req)

    def _battery_state_timer_cb(self, *args) -> None:
        if not self._battery_status in self._charger_connected_states:
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.BATTERY_STATE
            req.animation.param = str(self._battery_percentage)
            self._call_led_animation_srv(req)

    def _low_battery_timer_cb(self, *args) -> None:
        if (
            self._battery_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            and self._critical_battery_threshold_percent
            <= self._battery_percentage
            < self._low_battery_threshold_percent
        ):
            req = SetLEDAnimationRequest()
            req.animation.id = LEDAnimation.LOW_BATTERY
            self._call_led_animation_srv(req)

    def _battery_cb(self, battery_state: BatteryState) -> None:
        self._battery_percentage = battery_state.percentage
        self._battery_status = battery_state.power_supply_status

    def _e_stop_cb(self, e_stop_state: Bool) -> None:
        self._e_stop_state = e_stop_state.data

    def _call_led_animation_srv(self, req: SetLEDAnimationRequest) -> bool:
        try:
            self._set_led_client.wait_for_service(5.0)
            response = self._set_led_client.call(req)
            rospy.logdebug(
                f'[{rospy.get_name()}] Setting animation with ID: {req.animation.id}. Response: ({response})'
            )
            return response.success
        except rospy.ServiceException as err:
            rospy.logerr(
                f'Calling {self._set_led_client.resolved_name} service for message id {req.animation.id} failed. Error: {err}'
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
