#!/usr/bin/python3

from dataclasses import dataclass
import RPi.GPIO as GPIO

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import DriverState, IOState


@dataclass
class PatherGPIO:
    AUX_PW_EN = 18      # Enable auxiliary power, eg. supply to robotic arms etc. 
    CHRG_EN = 19        # Enable charger
    CHRG_SENSE = 7      # Charger sensor (1 - charger plugged in)            
    DRIVER_EN = 23      # Enable motor drivers (1 - on)
    E_STOP_RESET = 27   # Works as IN/OUT, IN - gives info if E-stop in on (1 - off),
                        # OUT - send 1 to reset estop
    FAN_SW = 15         # Turn on the fan (1 - on)
    SHDN_INIT = 16      # Shutdown Init managed by systemd service
    VDIG_OFF = 21       # Turn the digital power off eg. NUC, Router etc. (1 - off)
    VMOT_ON = 6         # Enable mamin power supply to motors (1 - on)
    WATCHDOG = 14       # Watchdog pin, if PWM is on this pin Panther will work

    # define inverse logic pins here to be used by _read_pin() method
    inverse_logic_pins = [VDIG_OFF, E_STOP_RESET, CHRG_SENSE]

    def __setattr__(self, name: str, value: int) -> None:
        raise AttributeError(f'Can\'t reassign constant {name}')


class Watchdog:
    def __init__(self, pin: int) -> None:
        self._pin = pin
        self._last_state = False
        self._enabled = True

    def __call__(self) -> None:
        if self._enabled:
            GPIO.output(self._pin, self._last_state)
            self._last_state = not self._last_state

    def turn_on(self) -> None:
        self._enabled = True

    def turn_off(self) -> None:
        self._enabled = False
        self._last_state = False
        GPIO.output(self._pin, self._last_state)


class PowerBoardNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._clearing_e_stop = False
        self._pins = PatherGPIO()

        self._setup_gpio()
        self._motor_start_sequence()
        self._watchdog = Watchdog(self._pins.WATCHDOG)

        self._gpio_wait = 0.05  # seconds
        self._e_stop_interrupt_time = float('inf')
        self._chrg_sense_interrupt_time = float('inf')

        self._cmd_vel_msg_time = rospy.get_time()
        self._can_net_err = True

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._e_stop_state_pub = rospy.Publisher('hardware/e_stop', Bool, queue_size=1, latch=True)
        self._io_state_pub = rospy.Publisher('hardware/io_state', IOState, queue_size=1, latch=True)

        msg = Bool(self._read_pin(self._pins.E_STOP_RESET))
        self._e_stop_state_pub.publish(msg)

        io_state = IOState()
        io_state.aux_power = self._read_pin(self._pins.AUX_PW_EN)
        io_state.charger_connected = self._read_pin(self._pins.CHRG_SENSE)
        io_state.fan = self._read_pin(self._pins.FAN_SW)
        io_state.power_btn = False
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
        self._motors_enable_server = rospy.Service(
            'hardware/motors_enable', SetBool, self._motors_enable_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        # 5 Hz publish non asynch pin state
        self._publish_pin_state_timer = rospy.Timer(rospy.Duration(0.2), self._publish_pin_state_cb)
        # 50 Hz of software PWM. Timer running at 100 Hz for raising and falling edges
        self._watchdog_timer = rospy.Timer(rospy.Duration(0.01), self._watchdog_cb)

        # -------------------------------
        #   GPIO callbacks
        # -------------------------------

        # register e-stop and power button pin change imminently
        GPIO.add_event_detect(
            self._pins.E_STOP_RESET, GPIO.BOTH, callback=self._gpio_interrupt_cb, bouncetime=200
        )

        GPIO.add_event_detect(
            self._pins.SHDN_INIT, GPIO.RISING, callback=self._gpio_interrupt_cb, bouncetime=200
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _cmd_vel_cb(self, *args) -> None:
        self._cmd_vel_msg_time = rospy.get_time()

    def _motor_controllers_state_cb(self, msg: DriverState) -> None:
        self._can_net_err = any({msg.rear.fault_flag.can_net_err, msg.front.fault_flag.can_net_err})

    def _gpio_interrupt_cb(self, pin: int) -> None:
        if pin == self._pins.SHDN_INIT:
            self._chrg_sense_interrupt_time = rospy.get_time()

        if pin == self._pins.E_STOP_RESET:
            self._e_stop_interrupt_time = rospy.get_time()

    def _publish_pin_state_cb(self, *args) -> None:
        charger_state = self._read_pin(self._pins.CHRG_SENSE)
        self._publish_io_state('charger_connected', charger_state)

        # filter short spikes of voltage on GPIO
        if rospy.get_time() - self._chrg_sense_interrupt_time > self._gpio_wait:
            if self._read_pin(self._pins.SHDN_INIT):
                rospy.loginfo(f'[{rospy.get_name()}] Shutdown button pressed.')
                self._publish_io_state('power_btn', True)
            self._chrg_sense_interrupt_time = float('inf')

        if rospy.get_time() - self._e_stop_interrupt_time > self._gpio_wait:
            self._e_stop_event()
            self._e_stop_interrupt_time = float('inf')
        elif math.isinf(rospy.get_time()):
            self._e_stop_event()

    def _watchdog_cb(self, *args) -> None:
        self._watchdog()

    def _aux_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(req.data, self._pins.AUX_PW_EN, 'Aux power enable')
        if res.success:
            self._publish_io_state('aux_power', req.data)
        return res

    def _charger_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._set_bool_srv_handle(req.data, self._pins.CHRG_EN, 'Charger enable')

    def _digital_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._set_bool_srv_handle(req.data, self._pins.VDIG_OFF, 'Digital power enable')

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        if self._validate_gpio_pin(self._pins.E_STOP_RESET, False):
            return TriggerResponse(True, 'E-STOP is not active, reset is not needed')
        elif rospy.get_time() - self._cmd_vel_msg_time <= 2.0:
            return TriggerResponse(
                False,
                'E-STOP reset failed, messages are still published on /cmd_vel topic!',
            )
        elif self._can_net_err:
            return TriggerResponse(
                False,
                'E-STOP reset failed, unable to communicate with motor controllers! Please check connection with motor controllers.',
            )

        self._reset_e_stop()

        if self._validate_gpio_pin(self._pins.E_STOP_RESET, True):
            self._watchdog.turn_off()
            return TriggerResponse(
                False,
                'E-STOP reset failed, check for pressed E-STOP buttons or other triggers',
            )

        return TriggerResponse(True, 'E-STOP reset successful')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._watchdog.turn_off()
        return TriggerResponse(True, f'E-STOP triggered, watchdog turned off')

    def _fan_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        res = self._set_bool_srv_handle(req.data, self._pins.FAN_SW, 'Fan enable')
        if res.success:
            self._publish_io_state('fan', req.data)
        return res

    def _motors_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        resp_1 = self._set_bool_srv_handle(req.data, self._pins.VMOT_ON, 'Motors driver enable')
        resp_2 = self._set_bool_srv_handle(req.data, self._pins.DRIVER_EN, 'Motors driver enable')

        if resp_1.success and resp_2.success:
            return SetBoolResponse(True, resp_1.message)
        return SetBoolResponse(False, resp_1.message)

    def _set_bool_srv_handle(self, value: bool, pin: int, name: str) -> SetBoolResponse:
        rospy.logdebug(f'[{rospy.get_name()}] Requested {name} = {value}')
        self._write_to_pin(pin, value)
        success = self._validate_gpio_pin(pin, value)
        msg = f'{name} write {value} failed'
        if success:
            msg = f'{name} write {value} successful'

        return SetBoolResponse(success, msg)

    def _reset_e_stop(self) -> None:
        self._clearing_e_stop = True
        GPIO.setup(self._pins.E_STOP_RESET, GPIO.OUT)
        self._watchdog.turn_on()

        self._write_to_pin(self._pins.E_STOP_RESET, False)
        rospy.sleep(0.1)

        GPIO.setup(self._pins.E_STOP_RESET, GPIO.IN)
        rospy.sleep(0.1)
        self._clearing_e_stop = False
        self._e_stop_event()

    def _e_stop_event(self) -> None:
        e_stop_state = self._read_pin(self._pins.E_STOP_RESET)
        if e_stop_state != self._e_stop_state_pub.impl.latch.data and not self._clearing_e_stop:
            self._e_stop_state_pub.publish(e_stop_state)

    def _publish_io_state(self, attribute: str, val: bool) -> None:
        last_msg = self._io_state_pub.impl.latch
        if getattr(last_msg, attribute) != val:
            setattr(last_msg, attribute, val)
            self._io_state_pub.publish(last_msg)

    def _motor_start_sequence(self) -> None:
        self._write_to_pin(self._pins.VMOT_ON, 1)
        rospy.sleep(0.5)
        self._write_to_pin(self._pins.DRIVER_EN, 1)
        rospy.sleep(0.2)

    def _setup_gpio(self) -> None:
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pins.AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(self._pins.CHRG_SENSE, GPIO.IN)
        GPIO.setup(self._pins.DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.E_STOP_RESET, GPIO.IN)  # USED AS I/O
        GPIO.setup(self._pins.FAN_SW, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.SHDN_INIT, GPIO.IN)
        GPIO.setup(self._pins.VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.WATCHDOG, GPIO.OUT, initial=0)

    def _read_pin(self, pin: int) -> bool:
        if pin in self._pins.inverse_logic_pins:
            return not GPIO.input(pin)
        return GPIO.input(pin)

    def _write_to_pin(self, pin: int, value: bool) -> None:
        if pin in self._pins.inverse_logic_pins:
            GPIO.output(pin, not value)
            return
        GPIO.output(pin, value)

    def _validate_gpio_pin(self, pin: int, value: bool) -> bool:
        return self._read_pin(pin) == value


def main():
    power_board_node = PowerBoardNode('power_board_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
