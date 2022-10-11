#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep

import rospy

from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

STAGE2_INPUT = 22
MOTOR_ON = 6


class RelaysNode:
    def __init__(self, name) -> None:
        self._setup_gpio()

        rospy.init_node(name, anonymous=False)

        self._e_stop_state = False

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._motor_on_pub = rospy.Publisher('/panther_hardware/motor_on', Bool, queue_size=1)
        self._e_stop_state_pub = rospy.Publisher('/panther_hardware/e_stop', Bool, queue_size=1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._e_stop_reset_srv = rospy.Service(
            '/panther_hardware/e_stop_reset', Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_srv = rospy.Service(
            '/panther_hardware/e_stop_trigger', Trigger, self._e_stop_trigger_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._timer_set_motor = rospy.Timer(rospy.Duration(0.01), self._set_motor_state)
        self._timer_e_stop = rospy.Timer(rospy.Duration(0.1), self._publish_e_stop_state)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._e_stop_state = False
        return TriggerResponse(True, 'E-STOP reset')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._e_stop_state = True
        return TriggerResponse(True, 'E-SROP triggered')

    def _publish_e_stop_state(self, *args) -> None:
        self._e_stop_state_pub.publish(self._e_stop_state)

    def _set_motor_state(self, *args) -> None:
        try:
            GPIO.output(MOTOR_ON, GPIO.input(STAGE2_INPUT))
            self._motor_on_pub.publish(GPIO.input(STAGE2_INPUT))
            sleep(0.01)
        except:
            GPIO.cleanup()
            self._setup_gpio()

    @staticmethod
    def _setup_gpio() -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_ON, GPIO.OUT)
        GPIO.setup(STAGE2_INPUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def main():
    relays_node = RelaysNode('relays_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
