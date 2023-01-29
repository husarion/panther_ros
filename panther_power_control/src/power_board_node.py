#!/usr/bin/python3

from dataclasses import dataclass
from gpiozero import PWMOutputDevice
import paramiko
import RPi.GPIO as GPIO
from threading import Thread, Lock
from time import sleep, time
import os

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


@dataclass
class PatherGPIO:              
    VMOT_ON = 6         # Enable mamin power supply to motors (1 - on)
    CHRG_SENSE = 7      # Charger sensor (1 - charger plugged in)            
    WATCHDOG = 14       # Watchdog pin, if PWM is on tish pin Panther will work
    FAN_SW = 15         # Turn on the fan (1 - on)
    SHDN_INIT = 16      # Shutdown Init managed by systemd service
    AUX_PW_EN = 18      # Enable auxiliary power, eg. supply to robotic arms etc. 
    CHRG_EN = 19        # Enable charger
    VDIG_OFF = 21       # Turn the digital power off eg. NUC, Router etc. (1 - off)
    DRIVER_EN = 23      # Enable motor drivers (1 - on)
    E_STOP_RESET = 27   # Works as IN/OUT, IN - gives info if E-stop in on (1 - off), OUT - send 1 to reset estop

    # define inverse logic pins here to be used by _read_pin() method
    inverse_logic_pins = [VDIG_OFF, E_STOP_RESET, CHRG_SENSE]

    def __setattr__(self, name, value):
        raise AttributeError(f'can\'t reassign constant {name}')


class Watchdog:
    def __init__(self) -> None:
        pins = PatherGPIO()
        self._watchdog_on = False
        self._watchdog_pwm = PWMOutputDevice(pins.WATCHDOG)

    def turn_on(self) -> None:
        if not self._watchdog_on:
            frequency = 50.0
            step = (1.0 / frequency) / 2.0
            self._watchdog_pwm.blink(on_time=step, off_time=step)
            self._watchdog_on = True

    def turn_off(self) -> None:
        if self._watchdog_on:
            self._watchdog_pwm.off()
            self._watchdog_on = False


class PowerBoardNode:
    def __init__(self, name):
        self._pins = PatherGPIO()

        self._setup_gpio()
        self._motor_start_sequence()

        self.soft_shutdown_thread = Thread(
            name='Soft shutdown thread', target=self._soft_shutdown
        )
        self.soft_shutdown_thread.start()

        self._watchdog = Watchdog()
        self._watchdog.turn_on()

        self._lock = Lock()

        rospy.init_node(name, anonymous=False)
        
        self._shutdown_timeout = rospy.get_param('~shutdown_timeout', 60.0)
        
        self._ip = rospy.get_param('~self_ip', '127.0.0.1')
        self._username = rospy.get_param('~self_username', 'husarion')
        self._default_identity_file = rospy.get_param('~default_identity_file', '~/.ssh/id_rsa')
        if rospy.has_para('~self_identity_file'):
            self._identity_file = rospy.get_param('~self_identity_file')
        else:
            self._identity_file = self._default_identity_file
            
        self._hosts = rospy.get_param('~hosts')
        for host in self._hosts:
            # check if all heys are provided
            if {'ip', 'username'} != set(host.keys()):
                rospy.logerr(f'[{rospy.get_name()}] Missing info for remote host!')
                raise Exception
            if 'identity_file' not in host.keys():
                host['identity_file'] = self._default_identity_file
                
            if not os.path.exists(host['identity_file']):
                rospy.logerr(f'[{rospy.get_name()}]'
                    'Can\'t find provided identity file for host {host["ip"]}!')
                raise Exception
        
        self._cmd_vel_msg_time = time()

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._e_stop_state_pub = rospy.Publisher('hardware/e_stop', Bool, queue_size=1)
        self._charger_state_pub = rospy.Publisher('hardware/charger_connected', Bool, queue_size=1)
        self._fan_state_pub = rospy.Publisher('hardware/fan_enabled', Bool, queue_size=1)
        
        # -------------------------------
        #   Subscribers
        # -------------------------------

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=1)
        
        # -------------------------------
        #   Services
        # -------------------------------

        self._aux_power_enable_srv = rospy.Service(
            'hardware/aux_power_enable', SetBool, self._aux_power_enable_cb
        )
        self._charger_enable_srv = rospy.Service(
            'hardware/charger_enable', SetBool, self._charger_enable_cb
        )
        self._digital_power_enable_srv = rospy.Service(
            'hardware/digital_power_enable', SetBool, self._digital_power_enable_cb,
        )
        self._motors_enable_srv = rospy.Service(
            'hardware/motors_enable', SetBool, self._motors_enable_cb
        )
        self._fan_enable_srv = rospy.Service(
            'hardware/fan_enable', SetBool, self._fan_enable_cb
        )
        self._e_stop_reset_srv = rospy.Service(
            'hardware/e_stop_reset', Trigger, self._e_stop_reset_cb
        )
        self._e_stop_trigger_srv = rospy.Service(
            'hardware/e_stop_trigger', Trigger, self._e_stop_trigger_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._timer_charger = rospy.Timer(rospy.Duration(0.5), self._publish_charger_state_cb)
        self._timer_e_stop = rospy.Timer(rospy.Duration(0.1), self._publish_e_stop_state_cb)
        self._timer_fan = rospy.Timer(rospy.Duration(1.0), self._publish_fan_state_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')
        
    def _check_ip(self, host):
      return os.system("ping -c 1 -w 1 " + host + " > /dev/null") == 0

    def _cmd_vel_cb(self, data) -> None:
        self._cmd_vel_msg_time = time()

    def _motor_start_sequence(self) -> None:
        self._write_to_pin(self._pins.VMOT_ON, 1)
        sleep(0.5)
        self._write_to_pin(self._pins.DRIVER_EN, 1)
        sleep(0.2)

    def _soft_shutdown(self) -> None:
        rospy.logwarn(f'[{rospy.get_name()}] Soft shutdown initialized.')
        for host in self._hosts:
            self._request_shutdown(host['ip'], host['key_path'], host['username'])
        
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < self._shutdown_timeout:
            any_host_available = False
            for host in self._hosts:
                if self._check_ip(host['ip']):
                    any_host_available = True
                
                if not any_host_available:
                    rospy.loginfo(f'[{rospy.get_name()}] All computes shat dowm gracefully.')
                    break
        
        else:
            rospy.loginfo(f'[{rospy.get_name()}] '
                'Shutdown timeout reached. Cutting out power from computers.')
            
        # ensure all computers did full shutdown
        rospy.loginfo(f'[{rospy.get_name()}] Shutting down itself.')
        self._request_shutdown(self._ip, self._identity_file, self._username)

    def _publish_e_stop_state_cb(self, event=None) -> None:
        with self._lock:
            msg = Bool()
            msg.data = self._read_pin(self._pins.E_STOP_RESET)
            self._e_stop_state_pub.publish(msg)

    def _publish_charger_state_cb(self, event=None) -> None:
        msg = Bool()
        msg.data = self._read_pin(self._pins.CHRG_SENSE)
        self._charger_state_pub.publish(msg)
        
    def _publish_fan_state_cb(self, event=None) -> None:
        msg = Bool()
        msg.data = self._read_pin(self._pins.FAN_SW)
        self._fan_state_pub.publish(msg)

    def _aux_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, self._pins.AUX_PW_EN, 'Aux power enable')

    def _charger_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, self._pins.CHRG_EN, 'Charger enable')

    def _digital_power_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, self._pins.VDIG_OFF, 'Digital power enable')

    def _motors_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        resp_1 = self._handle_set_bool_srv(req.data, self._pins.VMOT_ON, 'Motors driver enable')
        resp_2 = self._handle_set_bool_srv(req.data, self._pins.DRIVER_EN, 'Motors driver enable')

        if resp_1.success and resp_2.success:
            return SetBoolResponse(True, resp_1.message)
        return SetBoolResponse(False, resp_1.message)

    def _fan_enable_cb(self, req: SetBoolRequest) -> SetBoolResponse:
        return self._handle_set_bool_srv(req.data, self._pins.FAN_SW, 'Fan enable')

    def _e_stop_trigger_cb(self, req: TriggerRequest) -> TriggerResponse:
        self._watchdog.turn_off()
        return TriggerResponse(True, f'E-STOP triggered, watchdog turned off')

    def _e_stop_reset_cb(self, req: TriggerRequest) -> TriggerResponse:
        with self._lock:
            if self._validate_gpio_pin(self._pins.E_STOP_RESET, False):
                return TriggerResponse(True, 'E-STOP is not active, reset is not needed')
            elif time() - self._cmd_vel_msg_time <= 2.0:
                return TriggerResponse(
                    False,
                    'E-STOP reset failed, some messages are published on the /cmd_vel topic',
                )

            self._reset_e_stop()

            if self._validate_gpio_pin(self._pins.E_STOP_RESET, True):
                self._watchdog.turn_off()
                return TriggerResponse(
                    False,
                    'E-STOP reset failed, check for pressed E-STOP buttons or other triggers',
                )

            return TriggerResponse(True, 'E-STOP reset successful')

    def _reset_e_stop(self) -> None:
        GPIO.setup(self._pins.E_STOP_RESET, GPIO.OUT)
        self._watchdog.turn_on()

        # Sending False because of inverse logic
        self._write_to_pin(self._pins.E_STOP_RESET, False)
        sleep(0.1)

        GPIO.setup(self._pins.E_STOP_RESET, GPIO.IN)

    def _handle_set_bool_srv(self, value: bool, pin: int, name: str) -> SetBoolResponse:
        rospy.logdebug(f'[{rospy.get_name()}] Requested {name} = {value}')
        self._write_to_pin(pin, value)
        success = self._validate_gpio_pin(pin, value)
        msg = f'{name} write {value} failed'
        if success:
            msg = f'{name} write {value} successful'

        return SetBoolResponse(success, msg)

    def _validate_gpio_pin(self, pin: int, value: bool) -> bool:
        return self._read_pin(pin) == value

    def _request_shutdown(self, ip, identity_file, username) -> None:
        # shutdown only if host available
        if self._check_ip(ip):
            pkey = paramiko.RSAKey.from_private_key_file(identity_file)
            client = paramiko.SSHClient()
            policy = paramiko.AutoAddPolicy()
            client.set_missing_host_key_policy(policy)
            client.connect(ip, username=username, pkey=pkey)
            _, stdout, stderr = client.exec_command('sudo reboot now')
            if len(stdout.read().decode()) > 0:
                rospy.logerr(f'[{rospy.get_name()}] stdout: {stdout.read().decode()}')
            if len(stderr.read().decode()) > 0:
                rospy.logerr(f'[{rospy.get_name()}] stderr: {stderr.read().decode()}')
            client.close()
        else:
            rospy.loginfo(f'[{rospy.get_name()}] Device at IP: {ip} not available.')

    def _setup_gpio(self) -> None:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pins.VMOT_ON, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.CHRG_SENSE, GPIO.IN)
        GPIO.setup(self._pins.FAN_SW, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.AUX_PW_EN, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.CHRG_EN, GPIO.OUT, initial=1)
        GPIO.setup(self._pins.VDIG_OFF, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.DRIVER_EN, GPIO.OUT, initial=0)
        GPIO.setup(self._pins.E_STOP_RESET, GPIO.IN)  # USED AS I/O
        GPIO.setup(self._pins.SHDN_INIT, GPIO.IN)

    def _read_pin(self, pin: int) -> bool:
        if pin in self._pins.inverse_logic_pins:
            return not GPIO.input(pin)
        return GPIO.input(pin)

    def _write_to_pin(self, pin: int, value: bool) -> None:
        if pin in self._pins.inverse_logic_pins:
            GPIO.output(pin, not value)
            return
        GPIO.output(pin, value)


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
