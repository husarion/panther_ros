#!/usr/bin/python3

import rospy

from panther_msgs.msg import DriverState, SystemStatus
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest

class FanControllerNode:
    def __init__(self, name) -> None:

        rospy.init_node(name, anonymous=False)

        self._cpu_temp_window = None
        self._front_driver_temp_window = None
        self._rear_driver_temp_window = None
        self._fan_state = None
        self._turn_on_time = rospy.Time.now()
        
        self._critical_driver_temp = 60.0
        
        self._cpu_fan_on_temp = rospy.get_param('~cpu_fan_on_temp', 70.0)
        self._cpu_fan_off_temp = rospy.get_param('~cpu_fan_off_temp', 60.0)
        self._driver_fan_on_temp = rospy.get_param('~driver_fan_on_temp', 45.0)
        self._driver_fan_off_temp = rospy.get_param('~driver_fan_off_temp', 35.0)
        self._hysteresis = rospy.get_param('~hysteresis', 60.0)
        self._cpu_window_len = rospy.get_param('~cpu_window_len', 6)
        self._driver_window_len = rospy.get_param('~driver_window_len', 6)

        if self._cpu_fan_on_temp < self._cpu_fan_off_temp:
            rospy.logerr(f'[{rospy.get_name()}] Error: '
                f'Turning off temperature for CPU is higher than turning of temperature!')
            raise ValueError
        
        if self._driver_fan_on_temp < self._driver_fan_off_temp:
            rospy.logerr(f'[{rospy.get_name()}] Error: '
                f'Turning off temperature for driver is higher than turning of temperature!')
            raise ValueError

        if self._hysteresis < 0.0:
            rospy.logerr(f'[{rospy.get_name()}] Error: '
                f'Hysteresis can not be negative!')
            raise ValueError
            
        if self._cpu_window_len <= 0 or self._driver_window_len <= 0:
            rospy.logerr(f'[{rospy.get_name()}] Error: '
                f'Smoothing window has to be positive!')
            raise ValueError
        
        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._system_status_sub = rospy.Subscriber('system_status', SystemStatus, self._system_status_cb, queue_size=1)
        self._driver_state_sub = rospy.Subscriber('driver/motor_controllers_state', DriverState, self._driver_state_cb, queue_size=1)
        self._fan_state_sub = rospy.Subscriber('hardware/fan_enabled', Bool, self._fan_state_cb, queue_size=1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._fan_enable_service = rospy.ServiceProxy('hardware/fan_enable', SetBool)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._timer_set_motor = rospy.Timer(rospy.Duration(2.0), self._fan_control_loop_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')
        
    def _fan_control_loop_cb(self, event=None):
        if self._fan_state is None:
            rospy.loginfo(f'[{rospy.get_name()}] Waiting for fan state message to arrive.')
            return
        
        if self._cpu_temp_window is None:
            rospy.loginfo(f'[{rospy.get_name()}] Waiting for system_status message to arrive.')
            return
    
        if self._front_driver_temp_window is None or self._rear_driver_temp_window is None:
            rospy.loginfo(
                f'[{rospy.get_name()}] Waiting for motor_controllers_state message to arrive.')
            return
        
        self._cpu_avg_temp = self._get_mean(self._cpu_temp_window)
        self._front_driver_avg_temp = self._get_mean(self._front_driver_temp_window)
        self._rear_driver_avg_temp = self._get_mean(self._rear_driver_temp_window)
        
        if self._front_driver_avg_temp > self._critical_driver_temp:
            rospy.logerr_throttle(60,
                f'[{rospy.get_name()}] Rear driver reached critical ',
                f'temperature of {int(round(self._front_driver_avg_temp) + 0.1)} deg C!')
        if self._rear_driver_avg_temp > self._critical_driver_temp:
            rospy.logerr_throttle(60,
                f'[{rospy.get_name()}] Rear driver reached critical ',
                f'temperature of {int(round(self._rear_driver_avg_temp) + 0.1)} deg C!')
            
        if not self._fan_state and (self._cpu_avg_temp > self._cpu_fan_on_temp or \
                self._front_driver_avg_temp > self._driver_fan_on_temp or \
                self._rear_driver_avg_temp > self._driver_fan_on_temp):
            self._turn_on_time = rospy.Time.now()
            self._set_fan_state(True)
            rospy.loginfo(f'[{rospy.get_name()}] Turning on fan. Cooling the robot.')
            return

        if self._fan_state and (rospy.Time.now() - self._turn_on_time).secs > self._hysteresis and \
               (self._cpu_avg_temp < self._cpu_fan_off_temp and \
                self._front_driver_avg_temp < self._driver_fan_on_temp and \
                self._rear_driver_avg_temp < self._driver_fan_on_temp):
            self._set_fan_state(False)
            rospy.loginfo(f'[{rospy.get_name()}] Turning off fan.')
            return
        

    def _system_status_cb(self, data) -> None:
        if self._cpu_temp_window is not None:
            self._cpu_temp_window = self._move_window(self._cpu_temp_window, data.cpu_temp)
        else:
            self._cpu_temp_window = [data.cpu_temp] * self._cpu_window_len
        
    def _driver_state_cb(self, data) -> None:
        if self._front_driver_temp_window is not None and self._rear_driver_temp_window  is not None:
            self._front_driver_temp_window = self._move_window(self._front_driver_temp_window, data.front.temperature)
            self._rear_driver_temp_window = self._move_window(self._rear_driver_temp_window, data.rear.temperature)
        else:
            self._front_driver_temp_window = [data.front.temperature] * self._driver_window_len
            self._rear_driver_temp_window = [data.rear.temperature] * self._driver_window_len
            
    def _fan_state_cb(self, data) -> None:
        self._fan_state = data.data
        
    def _set_fan_state(self, state):
        rospy.wait_for_service('/panther_hardware/fan_enable')
        self._fan_enable_service(SetBoolRequest(state))
        
    def _move_window(self, window, elem):
        window = window[1:]
        window.append(elem)
        return window
    
    def _get_mean(self, window):
        return sum(window) / len(window)


def main():
    fan_controller_node = FanControllerNode('fan_controller_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
