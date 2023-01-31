#!/usr/bin/python3

import os
import paramiko
import socket

import rospy

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool, SetBoolRequest

from panther_msgs.msg import DriverState, SystemStatus


class ManagerNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._bat_temp = 0.0
        self._manager_timer_period = 1.0 / 10.0

        self._high_bat_temp = rospy.get_param('~high_bat_temp', 40.0)
        self._critical_bat_temp = rospy.get_param('~critical_bat_temp', 50.0)
        self._fatal_bat_temp = rospy.get_param('~fatal_bat_temp', 60.0)

        self._shutdown_timeout = rospy.get_param('~shutdown_timeout', 15.0)
        self._ip = rospy.get_param('~self_ip', '127.0.0.1')
        self._username = rospy.get_param('~self_username', 'husarion')

        dif = rospy.get_param('~default_identity_file', '~/.ssh/id_rsa')
        self._default_identity_file = os.path.expanduser(dif)
        if rospy.has_param('~self_identity_file'):
            sif = rospy.get_param('~self_identity_file')
            self._identity_file = os.path.expanduser(sif)
        else:
            self._identity_file = self._default_identity_file

        self._hosts = rospy.get_param('~shutdown_hosts_config_file', [])
        for host in self._hosts:
            # check if all keys are provided
            if {'ip', 'username'} != set(host.keys()):
                rospy.logerr(f'[{rospy.get_name()}] Missing info for remote host!')
                raise KeyError
            if 'identity_file' not in host.keys():
                host['identity_file'] = self._default_identity_file
            else:
                host['identity_file'] = os.path.expanduser(host['identity_file'])

            if not os.path.exists(host['identity_file']):
                rospy.logerr(
                    f'[{rospy.get_name()}]'
                    f' Can\'t find provided identity file for host {host["ip"]}!'
                    f' Path \'{host["identity_file"]}\' doesn\'t exist'
                )
                raise ValueError

            if 'cmd' not in host.keys():
                host['cmd'] = 'sudo shutdown now'

        self._cpu_fan_on_temp = rospy.get_param('~cpu_fan_on_temp', 70.0)
        self._cpu_fan_off_temp = rospy.get_param('~cpu_fan_off_temp', 60.0)
        self._driver_fan_on_temp = rospy.get_param('~driver_fan_on_temp', 45.0)
        self._driver_fan_off_temp = rospy.get_param('~driver_fan_off_temp', 35.0)
        self._hysteresis = rospy.get_param('~hysteresis', 60.0)
        self._cpu_window_len = rospy.get_param('~cpu_window_len', 6)
        self._driver_window_len = rospy.get_param('~driver_window_len', 6)
        self._overwrite_fan_control = rospy.get_param('~overwrite_fan_control', False)

        self._set_fan_state(self._overwrite_fan_control)

        if self._cpu_fan_on_temp < self._cpu_fan_off_temp:
            rospy.logerr(
                f'[{rospy.get_name()}] Error: '
                f'Turning off temperature for CPU is higher than turning on temperature!'
            )
            raise ValueError

        if self._driver_fan_on_temp < self._driver_fan_off_temp:
            rospy.logerr(
                f'[{rospy.get_name()}] Error: '
                f'Turning off temperature for driver is higher than turning on temperature!'
            )
            raise ValueError

        if self._hysteresis < 0.0:
            rospy.logerr(f'[{rospy.get_name()}] Error: ' f'Hysteresis can not be negative!')
            raise ValueError

        if self._cpu_window_len <= 0 or self._driver_window_len <= 0:
            rospy.logerr(f'[{rospy.get_name()}] Error: ' f'Smoothing window has to be positive!')
            raise ValueError

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._battery_sub = rospy.Subscriber('battery', BatteryState, self._battery_cb)
        self._driver_state_sub = rospy.Subscriber(
            'driver/motor_controllers_state', DriverState, self._driver_state_cb
        )
        self._e_stop_sub = rospy.Subscriber('hardware/e_stop', Bool, self._e_stop_cb)
        self._fan_state_sub = rospy.Subscriber('hardware/fan_enabled', Bool, self._fan_state_cb)
        self._system_status_sub = rospy.Subscriber(
            'system_status', SystemStatus, self._system_status_cb
        )

        # -------------------------------
        #   Services
        # -------------------------------

        self._aux_power_enable_client = rospy.ServiceProxy('hardware/aux_power_enable', SetBool)
        self._e_stop_trigger_client = rospy.ServiceProxy('hardware/e_stop_trigger', Trigger)
        self._fan_enable_client = rospy.ServiceProxy('hardware/fan_enable', SetBool)

        self._overwrite_fan_control_srv = rospy.ServiceProxy(
            'manager/overwrite_fan_control', SetBool, self._overwrite_fan_control_cb
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._manager_timer = rospy.Timer(
            rospy.Duration(self._manager_timer_period), self._manager_timer_cb
        )
        self._fan_control_timer = rospy.Timer(rospy.Duration(2.0), self._fan_control_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _battery_cb(self, battery_state: BatteryState) -> None:
        self._bat_temp = battery_state.temperature

    def _driver_state_cb(self, driver_state: DriverState) -> None:
        if self._front_driver_temp_window is not None and self._rear_driver_temp_window is not None:
            self._front_driver_temp_window = self._move_window(
                self._front_driver_temp_window, driver_state.front.temperature
            )
            self._rear_driver_temp_window = self._move_window(
                self._rear_driver_temp_window, driver_state.rear.temperature
            )
        else:
            self._front_driver_temp_window = [
                driver_state.front.temperature
            ] * self._driver_window_len
            self._rear_driver_temp_window = [
                driver_state.rear.temperature
            ] * self._driver_window_len

    def _e_stop_cb(self, e_stop_state: Bool) -> None:
        self._e_stop_state = e_stop_state.data

    def _fan_state_cb(self, fan_state: Bool) -> None:
        self._fan_state = fan_state.data

    def _system_status_cb(self, system_status: SystemStatus) -> None:
        if self._cpu_temp_window is not None:
            self._cpu_temp_window = self._move_window(self._cpu_temp_window, system_status.cpu_temp)
        else:
            self._cpu_temp_window = [system_status.cpu_temp] * self._cpu_window_len

    def _overwrite_fan_control_cb(self, req: SetBoolRequest):
        self._overwrite_fan_control = req.data
        self._set_fan_state(req.data)

    def _manager_timer_cb(self, *args) -> None:
        if self._bat_temp > self._fatal_bat_temp:
            rospy.logerr(f'[{rospy.get_name()}] Fatal battery temperature, shutting down robot')
            self._shutdown()
        elif self._bat_temp > self._critical_bat_temp:
            rospy.logerr(f'[{rospy.get_name()}] Critical battery temperature, disabling AUX')
            self._aux_power_enable_client.call(SetBoolRequest(False))
        elif self._bat_temp > self._high_bat_temp and not self._e_stop_state:
            rospy.logwarn(f'[{rospy.get_name()}] High battery temperature, triggering E-STOP')
            self._e_stop_trigger_client.call()

    def _shutdown(self) -> None:
        rospy.logwarn(f'[{rospy.get_name()}] Soft shutdown initialized.')
        # create new list of computers that confirmed shutdown procedure
        hosts_to_check = [
            h
            for h in self._hosts
            if self._request_shutdown(h['ip'], h['identity_file'], h['username'], h['cmd'])
        ]

        start_time = rospy.get_time()
        if len(hosts_to_check) > 0:
            while rospy.get_time() - start_time < self._shutdown_timeout:
                # reduce list to all computers that are still available
                hosts_to_check = [h for h in hosts_to_check if self._check_ip(h['ip'])]
                if len(hosts_to_check) == 0:
                    rospy.loginfo(f'[{rospy.get_name()}] All computes shut down gracefully.')
                    break
            else:
                rospy.loginfo(
                    f'[{rospy.get_name()}] '
                    'Shutdown timeout reached. Cutting out power from computers.'
                )

        # ensure all computers did full shutdown
        rospy.loginfo(f'[{rospy.get_name()}] Shutting down itself.')
        self._request_shutdown(self._ip, self._identity_file, self._username, 'sudo shutdown now')

    def _request_shutdown(self, ip: str, identity_file: str, username: str, cmd: str) -> bool:
        # shutdown only if host available
        if self._check_ip(ip):
            try:
                pkey = paramiko.RSAKey.from_private_key_file(identity_file)
                client = paramiko.SSHClient()
                policy = paramiko.AutoAddPolicy()
                client.set_missing_host_key_policy(policy)
                client.connect(ip, username=username, pkey=pkey, timeout=0.5)
                rospy.loginfo(f'[{rospy.get_name()}] Shutting down device at {ip}')
                try:
                    client.exec_command(cmd, timeout=0.5)
                except socket.timeout:
                    # some systems do not close SSH connection on shut down
                    # this will handle the timeout to allow shutting other devices
                    pass
                client.close()
                return True
            except Exception:
                rospy.logerr(f'[{rospy.get_name()}] Can\'t SSH to device at {ip}!')
                return False
        else:
            rospy.loginfo(f'[{rospy.get_name()}] Device at {ip} not available.')
            return False

    def _check_ip(self, host: str) -> bool:
        return os.system('ping -c 1 -w 1 ' + host + ' > /dev/null') == 0

    def _fan_control_timer_cb(self, *args) -> None:
        if self._fan_state is None:
            rospy.loginfo(f'[{rospy.get_name()}] Waiting for fan state message to arrive.')
            return

        if self._overwrite_fan_control:
            return

        if self._cpu_temp_window is None:
            rospy.loginfo(f'[{rospy.get_name()}] Waiting for system_status message to arrive.')
            return

        if self._front_driver_temp_window is None or self._rear_driver_temp_window is None:
            rospy.loginfo(
                f'[{rospy.get_name()}] Waiting for motor_controllers_state message to arrive.'
            )
            return

        self._cpu_avg_temp = self._get_mean(self._cpu_temp_window)
        self._front_driver_avg_temp = self._get_mean(self._front_driver_temp_window)
        self._rear_driver_avg_temp = self._get_mean(self._rear_driver_temp_window)

        if self._front_driver_avg_temp > self._critical_driver_temp:
            rospy.logerr_throttle(
                60,
                f'[{rospy.get_name()}] Front driver reached critical ',
                f'temperature of {int(round(self._front_driver_avg_temp) + 0.1)} deg C!',
            )
        if self._rear_driver_avg_temp > self._critical_driver_temp:
            rospy.logerr_throttle(
                60,
                f'[{rospy.get_name()}] Rear driver reached critical ',
                f'temperature of {int(round(self._rear_driver_avg_temp) + 0.1)} deg C!',
            )

        if not self._fan_state and (
            self._cpu_avg_temp > self._cpu_fan_on_temp
            or self._front_driver_avg_temp > self._driver_fan_on_temp
            or self._rear_driver_avg_temp > self._driver_fan_on_temp
        ):
            self._turn_on_time = rospy.Time.now()
            self._set_fan_state(True)
            rospy.loginfo(f'[{rospy.get_name()}] Turning on fan. Cooling the robot.')
            return

        if (
            self._fan_state
            and (rospy.Time.now() - self._turn_on_time).secs > self._hysteresis
            and (
                self._cpu_avg_temp < self._cpu_fan_off_temp
                and self._front_driver_avg_temp < self._driver_fan_on_temp
                and self._rear_driver_avg_temp < self._driver_fan_on_temp
            )
        ):
            self._set_fan_state(False)
            rospy.loginfo(f'[{rospy.get_name()}] Turning off fan.')
            return

    def _set_fan_state(self, state: bool) -> None:
        rospy.wait_for_service('hardware/fan_enable')
        self._fan_enable_client(SetBoolRequest(state))

    def _move_window(self, window: list, elem: float) -> list:
        window = window[1:]
        window.append(elem)
        return window

    def _get_mean(self, window: list) -> float:
        return sum(window) / len(window)


def main():
    manager_node = ManagerNode('manager_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
