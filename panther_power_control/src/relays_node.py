#!/usr/bin/python3

import gpiod
from threading import Lock

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import DriverState, IOState


class RelaysNode:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        self._lock = Lock()

        line_names = {
            'MOTOR_ON': False,  # Used to enable motors
            'STAGE2_INPUT': False,  # Input from 2nd stage of rotary power switch
        }

        self._chip = gpiod.Chip('gpiochip0', gpiod.Chip.OPEN_BY_NAME)
        self._lines = {name: self._chip.find_line(name) for name in list(line_names.keys())}
        not_matched_pins = [name for name, line in self._lines.items() if line is None]
        if not_matched_pins:
            for pin in not_matched_pins:
                rospy.logerr(f'[{rospy.get_name()}] Failed to find pin: \'{pin}\'')
            rospy.signal_shutdown('Failed to find GPIO lines')
            return

        self._lines['MOTOR_ON'].request(
            self._node_name, type=gpiod.LINE_REQ_DIR_OUT, default_val=line_names['MOTOR_ON']
        )
        self._lines['STAGE2_INPUT'].request(
            self._node_name, type=gpiod.LINE_REQ_DIR_IN, default_val=line_names['STAGE2_INPUT']
        )

        self._e_stop_state = not self._lines['STAGE2_INPUT'].get_value()
        self._cmd_vel_msg_time = rospy.get_time()
        self._can_net_err = True

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._e_stop_state_pub = rospy.Publisher('hardware/e_stop', Bool, queue_size=1, latch=True)
        self._io_state_pub = rospy.Publisher('hardware/io_state', IOState, queue_size=1, latch=True)

        # init e-stop state
        self._e_stop_state_pub.publish(self._e_stop_state)

        self._io_state = IOState()
        self._io_state.motor_on = self._lines['STAGE2_INPUT'].get_value()
        self._io_state.aux_power = False
        self._io_state.charger_connected = False
        self._io_state.fan = False
        self._io_state.power_button = False
        self._io_state.digital_power = True
        self._io_state.charger_enabled = False
        self._io_state_pub.publish(self._io_state)

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=1)
        self._motor_controllers_state_sub = rospy.Subscriber(
            'driver/motor_controllers_state', DriverState, self._motor_controllers_state_cb
        )

        # -------------------------------
        #   Service servers
        # -------------------------------

        self._e_stop_reset_server = rospy.Service(
            'hardware/e_stop_reset', Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_server = rospy.Service(
            'hardware/e_stop_trigger', Trigger, self._e_stop_trigger_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        # check motor state at 10 Hz
        self._set_motor_state_timer = rospy.Timer(
            rospy.Duration(0.1), self._set_motor_state_timer_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def __del__(self):
        for line in self._lines.values():
            if line:
                line.release()
        if self._chip:
            self._chip.close()

    def _cmd_vel_cb(self, *args) -> None:
        with self._lock:
            self._cmd_vel_msg_time = rospy.get_time()

    def _motor_controllers_state_cb(self, msg: DriverState) -> None:
        with self._lock:
            self._can_net_err = any(
                {msg.rear.fault_flag.can_net_err, msg.front.fault_flag.can_net_err}
            )

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._lock:
            if rospy.get_time() - self._cmd_vel_msg_time <= 2.0:
                return TriggerResponse(
                    False,
                    'E-STOP reset failed, messages are still published on /cmd_vel topic!',
                )
            elif self._can_net_err:
                return TriggerResponse(
                    False,
                    'E-STOP reset failed, unable to communicate with motor controllers! '
                    'Please check connection with motor controllers.',
                )

            self._e_stop_state = False
            self._e_stop_state_pub.publish(self._e_stop_state)
            return TriggerResponse(True, 'E-STOP reset successful')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._lock:
            self._e_stop_state = True
            self._e_stop_state_pub.publish(self._e_stop_state)
            return TriggerResponse(True, 'E-SROP triggered successful')

    def _set_motor_state_timer_cb(self, *args) -> None:
        motor_state = self._lines['STAGE2_INPUT'].get_value()
        self._lines['MOTOR_ON'].set_value(motor_state)
        if self._io_state.motor_on != motor_state:
            self._io_state.motor_on = motor_state
            self._io_state_pub.publish(self._io_state)


def main():
    relays_node = RelaysNode('relays_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
