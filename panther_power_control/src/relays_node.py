#!/usr/bin/python3

import RPi.GPIO as GPIO
from time import sleep

import rospy

from std_msgs.msg import Bool

STAGE2_INPUT = 22
MOTOR_ON = 6


class RelaysNode:
    def __init__(self, name) -> None:
        self._setup_gpio()

        rospy.init_node(name, anonymous=False)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._motor_on_pub = rospy.Publisher('/panther_hardware/motor_on', Bool, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._timer_set_motor = rospy.Timer(rospy.Duration(0.01), self._set_motor_state)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    @staticmethod
    def _setup_gpio() -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_ON, GPIO.OUT)
        GPIO.setup(STAGE2_INPUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def _set_motor_state(self, *args) -> None:
        try:
            GPIO.output(MOTOR_ON, GPIO.input(STAGE2_INPUT))
            self._motor_on_pub.publish(GPIO.input(STAGE2_INPUT))
            sleep(0.01)
        except:
            GPIO.cleanup()
            self._setup_gpio()


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
