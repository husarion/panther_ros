#!/usr/bin/python3

from dataclasses import dataclass
import gpiod
from threading import Lock

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import DriverState, IOState


class RelaysNode:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        self._motors_lock = Lock()
        self._e_stop_lock = Lock()

        chip = gpiod.Chip('gpiochip0', gpiod.Chip.OPEN_BY_NAME)

        line_names = {'MOTOR_ON': 6, 'STAGE2_INPUT': 22}
        self._lines = {name: chip.get_line(line_names[name]) for name in line_names}
        self._lines['MOTOR_ON'].request(self._node_name, type=gpiod.LINE_REQ_DIR_OUT)
        self._lines['STAGE2_INPUT'].request(self._node_name, type=gpiod.LINE_REQ_DIR_IN)

        self._e_stop_state = not self._lines['STAGE2_INPUT'].get_value()
        self._cmd_vel_msg_time = rospy.get_time()
        self._can_net_err = True
        self._motor_enabled = True

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
        self._motor_enable_server = rospy.Service(
            'hardware/motor_enable', SetBool, self._motor_enable_cb
        )

        # -------------------------------
        #   Service clients
        # -------------------------------

        self._reset_roboteq_script_client = rospy.ServiceProxy(
            'driver/reset_roboteq_script', Trigger
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
            line.release()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        with self._e_stop_lock:
            self._cmd_vel_msg_time = rospy.get_time()

    def _motor_controllers_state_cb(self, msg: DriverState) -> None:
        with self._e_stop_lock:
            self._can_net_err = any(
                {msg.rear.fault_flag.can_net_err, msg.front.fault_flag.can_net_err}
            )

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._e_stop_lock:
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
        with self._e_stop_lock:
            self._e_stop_state = True
            self._e_stop_state_pub.publish(self._e_stop_state)
            return TriggerResponse(True, 'E-SROP triggered successful')

    def _motor_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        with self._motors_lock:
            stage2_value = self._lines['STAGE2_INPUT'].get_value()
            if not stage2_value:
                self._motor_enabled = req.data
                return SetBoolResponse(
                    not req.data,
                    f'Three-position Main switch is in Stage 1. '
                    f'Motors are {"already " if not req.data else ""}powered off',
                )

            if self._motor_enabled == req.data:
                return SetBoolResponse(True, f'Motor state already set to: {req.data}')

            self._motor_enabled = req.data
            self._lines['MOTOR_ON'].set_value(req.data)

            if req.data:
                # wait for motor drivers to power on
                rospy.sleep(rospy.Duration(2.0))
                try:
                    reset_script_res = self._reset_roboteq_script_client.call()
                    if not reset_script_res.success:
                        self._lines['MOTOR_ON'].set_value(False)
                        self._publish_io_state('motor_on', False)
                        return SetBoolResponse(reset_script_res.success, reset_script_res.message)
                except rospy.ServiceException as e:
                    self._lines['MOTOR_ON'].set_value(False)
                    self._publish_io_state('motor_on', False)
                    return SetBoolResponse(False, f'Failed to reset roboteq script: {e}')

            return SetBoolResponse(True, f'Motors {"enabled" if req.data else "disabled"}')

    def _set_motor_state_timer_cb(self, *args) -> None:
        with self._motors_lock:
            motor_state = self._lines['STAGE2_INPUT'].get_value()
            should_enable_motors = motor_state and self._motor_enabled
            self._lines['MOTOR_ON'].set_value(should_enable_motors)
            self._publish_io_state('motor_on', should_enable_motors)

    def _publish_io_state(self, attribute: str, val: bool) -> None:
        last_msg = self._io_state_pub.impl.latch
        if getattr(last_msg, attribute) != val:
            setattr(last_msg, attribute, val)
            self._io_state_pub.publish(last_msg)


def main():
    relays_node = RelaysNode('relays_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
