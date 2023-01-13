#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep, time

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

STAGE2_INPUT = 22
MOTOR_ON = 6


class RelaysNode:
    def __init__(self, name) -> None:
        self._setup_gpio()

        rospy.init_node(name, anonymous=False)

        self._e_stop_state = False
        self._cmd_vel_msg_last_time = time()

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._motor_on_pub = rospy.Publisher('hardware/motor_on', Bool, queue_size=1)
        self._e_stop_state_pub = rospy.Publisher('hardware/e_stop', Bool, queue_size=1)

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._e_stop_reset_srv = rospy.Service(
            'hardware/e_stop_reset', Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_srv = rospy.Service(
            'hardware/e_stop_trigger', Trigger, self._e_stop_trigger_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._timer_set_motor = rospy.Timer(rospy.Duration(0.01), self._set_motor_state)
        self._timer_e_stop = rospy.Timer(rospy.Duration(0.1), self._publish_e_stop_state)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _cmd_vel_cb(self, data) -> None:
        self._cmd_vel_msg_last_time = time()

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        if time() - self._cmd_vel_msg_last_time <= 2.0:
            return TriggerResponse(
                False,
                'E-STOP reset failed, some messages are published on the /cmd_vel topic',
            )
        self._e_stop_state = False
        return TriggerResponse(True, 'E-STOP reset successfully')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._e_stop_state = True
        return TriggerResponse(True, 'E-SROP triggered successfully')

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
