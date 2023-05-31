#!/usr/bin/python3

from threading import Lock
from typing import Optional

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState


class RoboteqRepublisherNode:
    V_BAT_FATAL_MIN = 27.0
    V_BAT_FATAL_MAX = 43.0
    V_BAT_FULL = 41.4
    V_BAT_MIN = 32.0

    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._lock = Lock()

        self._battery_voltage: Optional[float] = None
        self._battery_current: Optional[float] = None
        self._battery_voltage_mean = 37.0

        self._volt_mean_length = 10
        self._battery_voltage_hist = [37.0] * self._volt_mean_length

        self._battery_timeout = 1.0
        self._last_battery_info_time = rospy.get_time()

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._motor_controllers_state_sub = rospy.Subscriber(
            'driver/motor_controllers_state', DriverState, self._motor_controllers_state_cb
        )

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        # Running at 10 HZ
        self._battery_pub_timer = rospy.Timer(rospy.Duration(0.1), self._battery_pub_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _motor_controllers_state_cb(self, msg: DriverState) -> None:
        with self._lock:
            new_voltage = (msg.front.voltage + msg.rear.voltage) / 2.0

            self._last_battery_info_time = rospy.get_time()
            self._battery_voltage = new_voltage
            self._battery_current = msg.front.current + msg.rear.current
            self._update_volt_mean(new_voltage)

    def _battery_pub_timer_cb(self, *args) -> None:
        with self._lock:
            battery_msg = BatteryState()
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.capacity = 20.0
            battery_msg.design_capacity = 20.0
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO

            if (
                self._battery_voltage == None
                or self._battery_current == None
                or rospy.get_time() - self._last_battery_info_time > self._battery_timeout
            ):
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            else:
                battery_msg.voltage = self._battery_voltage
                battery_msg.temperature = float('nan')
                battery_msg.current = self._battery_current
                battery_msg.percentage = self._clamp(
                    (battery_msg.voltage - self.V_BAT_MIN) / (self.V_BAT_FULL - self.V_BAT_MIN),
                    0.0,
                    1.0,
                )
                battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
                battery_msg.present = True

                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

                # check battery health
                error_msg = None
                if self._battery_voltage < self.V_BAT_FATAL_MIN:
                    battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
                    error_msg = 'Battery voltage is critically low!'
                elif self._battery_voltage > self.V_BAT_FATAL_MAX:
                    battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
                    error_msg = 'Battery overvoltage!'
                else:
                    battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

                if error_msg is not None:
                    rospy.logerr_throttle_identical(10.0, f'[{rospy.get_name()}] {error_msg}')

            self._battery_pub.publish(battery_msg)

    def _update_volt_mean(self, new_val: float) -> float:
        # Updates the average by adding the newest and removing the oldest component of mean value,
        # in order to avoid recalculating the entire sum every time.
        self._battery_voltage_mean += (
            new_val - self._battery_voltage_hist[0]
        ) / self._volt_mean_length

        self._battery_voltage_hist.pop(0)
        self._battery_voltage_hist.append(new_val)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main():
    roboteq_republisher_node = RoboteqRepublisherNode('roboteq_republisher_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
