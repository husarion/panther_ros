#!/usr/bin/python3

import gpiod
import math
from threading import Lock

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import DriverState, IOState


class Watchdog:
    def __init__(self, line: gpiod.Line) -> None:
        self._last_state = False
        self._enabled = True

        self._watchdog_pin = line

    def __call__(self) -> None:
        if self._enabled:
            self._watchdog_pin.set_value(self._last_state)
            self._last_state = not self._last_state

    def turn_on(self) -> None:
        self._enabled = True

    def turn_off(self) -> None:
        self._enabled = False
        self._last_state = False
        self._watchdog_pin.set_value(self._last_state)


class PowerBoardNode:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        self._pins_lock = Lock()
        self._e_stop_lock = Lock()
        self._watchdog_lock = Lock()
        self._io_state_lock = Lock()

        self._clearing_e_stop = False

        out_line_names = {
            'AUX_PW_EN': False,  # Enable auxiliary power, eg. supply to robotic arms etc.
            'CHRG_DISABLE': True,  # Disable charger
            'DRIVER_EN': False,  # Enable motor drivers (1 - on)
            'FAN_SW': False,  # Turn on the fan (1 - on)
            'VDIG_OFF': False,  # Turn the digital power off eg. NUC, Router etc. (1 - off)
            'VMOT_ON': False,  # Enable mamin power supply to motors (1 - on)
            'WATCHDOG': False,  # Watchdog pin, if PWM is on this pin Panther will work
        }
        in_line_names = {
            'CHRG_SENSE': True,  # Charger sensor (0 - charger connected, 1 - not connected)
            'E_STOP_RESET': False,  # Works as IN/OUT,
            # IN - gives info if E-stop is on (1 - off), OUT - send 1 to reset estop
            'SHDN_INIT': False,  # Shutdown Init managed by systemd service
        }

        self._chip = gpiod.Chip('gpiochip0', gpiod.Chip.OPEN_BY_NAME)
        self._lines = {
            name: self._chip.find_line(name)
            for name in list(out_line_names.keys()) + list(in_line_names.keys())
        }
        for name, line in self._lines.items():
            line.request(
                self._node_name,
                gpiod.LINE_REQ_DIR_OUT if name in out_line_names.keys() else gpiod.LINE_REQ_DIR_IN,
                default_val=out_line_names[name]
                if name in out_line_names.keys()
                else in_line_names[name],
            )

        self._watchdog = Watchdog(self._lines['WATCHDOG'])
        self._motor_start_sequence()

        self._gpio_wait = 0.05  # seconds
        self._last_e_stop_state = not self._lines['E_STOP_RESET'].get_value()
        self._last_shdn_init_state = self._lines['SHDN_INIT'].get_value()
        self._e_stop_pressed_time = float('inf')
        self._shdn_init_detect_time = float('inf')

        self._cmd_vel_msg_time = rospy.get_time()
        self._can_net_err = True

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._e_stop_state_pub = rospy.Publisher('hardware/e_stop', Bool, queue_size=1, latch=True)
        self._io_state_pub = rospy.Publisher('hardware/io_state', IOState, queue_size=1, latch=True)

        msg = Bool(not self._lines['E_STOP_RESET'].get_value())
        self._e_stop_state_pub.publish(msg)

        io_state = IOState()
        io_state.aux_power = self._lines['AUX_PW_EN'].get_value()
        io_state.charger_connected = not self._lines['CHRG_SENSE'].get_value()
        io_state.fan = self._lines['FAN_SW'].get_value()
        io_state.power_button = False
        io_state.digital_power = not self._lines['VDIG_OFF'].get_value()
        io_state.charger_enabled = not self._lines['CHRG_DISABLE'].get_value()
        io_state.motor_on = self._lines['DRIVER_EN'].get_value()
        self._io_state_pub.publish(io_state)

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

        self._aux_power_enable_server = rospy.Service(
            'hardware/aux_power_enable', SetBool, self._aux_power_enable_cb
        )
        self._charger_enable_server = rospy.Service(
            'hardware/charger_enable', SetBool, self._charger_enable_cb
        )
        self._digital_power_enable_server = rospy.Service(
            'hardware/digital_power_enable', SetBool, self._digital_power_enable_cb
        )
        self._e_stop_reset_server = rospy.Service(
            'hardware/e_stop_reset', Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_server = rospy.Service(
            'hardware/e_stop_trigger', Trigger, self._e_stop_trigger_cb
        )
        self._fan_enable_server = rospy.Service('hardware/fan_enable', SetBool, self._fan_enable_cb)
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

        # 20 Hz publish non asynch pin state
        self._publish_pin_state_timer = rospy.Timer(
            rospy.Duration(0.05), self._publish_pin_state_cb
        )
        # 50 Hz of software PWM. Timer running at 100 Hz for raising and falling edges
        self._watchdog_timer = rospy.Timer(rospy.Duration(0.01), self._watchdog_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def __del__(self):
        with self._pins_lock:
            for line in self._lines.values():
                line.release()
            self._chip.close()

    def _cmd_vel_cb(self, *args) -> None:
        with self._e_stop_lock:
            self._cmd_vel_msg_time = rospy.get_time()

    def _motor_controllers_state_cb(self, msg: DriverState) -> None:
        with self._e_stop_lock:
            self._can_net_err = any(
                {msg.rear.fault_flag.can_net_err, msg.front.fault_flag.can_net_err}
            )

    def _publish_pin_state_cb(self, *args) -> None:
        pin_state_time = rospy.get_time()
        with self._pins_lock:
            charger_state = not self._lines['CHRG_SENSE'].get_value()
            shdn_init_val = self._lines['SHDN_INIT'].get_value()
            estop_state = not self._lines['E_STOP_RESET'].get_value()

        self._publish_io_state('charger_connected', charger_state)

        # filter short spikes of voltage on GPIO
        if math.isinf(self._shdn_init_detect_time) and shdn_init_val:
            self._shdn_init_detect_time = pin_state_time
        elif pin_state_time - self._shdn_init_detect_time > self._gpio_wait:
            if shdn_init_val and not self._last_shdn_init_state:
                self._last_shdn_init_state = shdn_init_val
                rospy.loginfo(f'[{rospy.get_name()}] Shutdown button pressed.')
                self._publish_io_state('power_button', True)
            self._shdn_init_detect_time = float('inf')

        with self._e_stop_lock:
            last_e_stop_state = self._last_e_stop_state

        if math.isinf(self._e_stop_pressed_time) and estop_state != last_e_stop_state:
            self._e_stop_pressed_time = pin_state_time
        elif pin_state_time - self._e_stop_pressed_time > self._gpio_wait:
            self._e_stop_event()
            self._e_stop_pressed_time = float('inf')

    def _watchdog_cb(self, *args) -> None:
        with self._watchdog_lock:
            self._watchdog()

    def _aux_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(req.data, 'AUX_PW_EN', 'Aux power enable')
        if res.success:
            self._publish_io_state('aux_power', req.data)
        return res

    def _charger_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(not req.data, 'CHRG_DISABLE', 'Charger disable')
        if res.success:
            self._publish_io_state('charger_enabled', req.data)
        return res

    def _digital_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(not req.data, 'VDIG_OFF', 'Digital power enable')
        if res.success:
            self._publish_io_state('digital_power', req.data)
        return res

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._pins_lock:
            estop_state = not self._lines['E_STOP_RESET'].get_value()

        with self._e_stop_lock:
            if not estop_state:
                return TriggerResponse(True, 'E-STOP is not active, reset is not needed')
            elif rospy.get_time() - self._cmd_vel_msg_time <= 2.0:
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

        self._reset_e_stop()

        with self._pins_lock:
            estop_state = not self._lines['E_STOP_RESET'].get_value()

        if estop_state:
            with self._watchdog_lock:
                self._watchdog.turn_off()
            return TriggerResponse(
                False,
                'E-STOP reset failed, check for pressed E-STOP buttons or other triggers',
            )

        return TriggerResponse(True, 'E-STOP reset successful')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._watchdog_lock:
            self._watchdog.turn_off()
        return TriggerResponse(True, f'E-STOP triggered, watchdog turned off')

    def _fan_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(req.data, 'FAN_SW', 'Fan enable')
        if res.success:
            self._publish_io_state('fan', req.data)
        return res

    def _motor_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        with self._pins_lock:
            if self._lines['DRIVER_EN'].get_value() == req.data:
                return SetBoolResponse(True, f'Motor state already set to: {req.data}')

        res = self._set_bool_srv_handle(req.data, 'DRIVER_EN', 'Motor drivers enable')
        if not res.success:
            return res

        self._publish_io_state('motor_on', req.data)

        if req.data:
            # wait for motor drivers to power on
            rospy.sleep(rospy.Duration(2.0))
            try:
                reset_script_res = self._reset_roboteq_script_client.call()
                if not reset_script_res.success:
                    res = self._set_bool_srv_handle(False, 'DRIVER_EN', 'Motor drivers enable')
                    if res.success:
                        self._publish_io_state('motor_on', False)
                    return SetBoolResponse(reset_script_res.success, reset_script_res.message)
            except rospy.ServiceException as e:
                res = self._set_bool_srv_handle(False, 'DRIVER_EN', 'Motor drivers enable')
                if res.success:
                    self._publish_io_state('motor_on', False)
                return SetBoolResponse(False, f'Failed to reset roboteq script: {e}')

        return res

    def _set_bool_srv_handle(self, value: bool, pin_name: str, name: str) -> SetBoolResponse:
        rospy.logdebug(f'[{rospy.get_name()}] Requested {name} = {value}')
        with self._pins_lock:
            self._lines[pin_name].set_value(value)
            success = self._lines[pin_name].get_value() == value
        msg = f'{name} write {value} failed'
        if success:
            msg = f'{name} write {value} successful'

        return SetBoolResponse(success, msg)

    def _reset_e_stop(self) -> None:
        with self._e_stop_lock:
            self._clearing_e_stop = True

        with self._pins_lock:
            req_type = gpiod.LINE_REQ_DIR_OUT
            self._lines['E_STOP_RESET'].release()
            self._lines['E_STOP_RESET'].request(self._node_name, type=req_type)

        with self._watchdog_lock:
            self._watchdog.turn_on()

        with self._pins_lock:
            self._lines['E_STOP_RESET'].set_value(True)
        rospy.sleep(0.1)

        with self._pins_lock:
            req_type = gpiod.LINE_REQ_DIR_IN
            self._lines['E_STOP_RESET'].release()
            self._lines['E_STOP_RESET'].request(self._node_name, type=req_type)
        rospy.sleep(0.1)

        with self._e_stop_lock:
            self._clearing_e_stop = False
        self._e_stop_event()

    def _e_stop_event(self) -> None:
        with self._pins_lock:
            e_stop_state = not self._lines['E_STOP_RESET'].get_value()

        with self._e_stop_lock:
            if e_stop_state != self._last_e_stop_state and not self._clearing_e_stop:
                self._last_e_stop_state = e_stop_state
                self._e_stop_state_pub.publish(e_stop_state)

    def _publish_io_state(self, attribute: str, val: bool) -> None:
        with self._io_state_lock:
            last_msg = self._io_state_pub.impl.latch
            if getattr(last_msg, attribute) != val:
                setattr(last_msg, attribute, val)
                self._io_state_pub.publish(last_msg)

    def _motor_start_sequence(self) -> None:
        self._lines['VMOT_ON'].set_value(True)
        rospy.sleep(0.5)
        self._lines['DRIVER_EN'].set_value(True)
        rospy.sleep(0.2)


def main():
    power_board_node = PowerBoardNode('power_board_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
