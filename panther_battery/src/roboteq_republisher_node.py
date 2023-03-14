#!/usr/bin/python3

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState


class RoboteqRepublisherNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._battery_voltage = None
        self._battery_current = None
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
        self._last_battery_info_time = rospy.get_time()
        self._battery_voltage = (msg.front.voltage + msg.rear.voltage) / 2.0
        self._battery_current = (msg.front.current + msg.rear.current) / 2.0

    def _battery_pub_timer_cb(self, *args) -> None:
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.capacity = 20.0
        battery_msg.design_capacity = 20.0
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        if (
            not self._battery_voltage
            or not self._battery_current
            or rospy.get_time() - self._last_battery_info_time > self._battery_timeout
        ):
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            battery_msg.voltage = self._battery_voltage
            battery_msg.temperature = float('nan')
            battery_msg.current = self._battery_current
            battery_msg.percentage = (battery_msg.voltage - 32.0) / 10.0
            battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
            battery_msg.present = True
            # TODO:
            # battery_msg.power_supply_health

        self._battery_pub.publish(battery_msg)


def main():
    roboteq_republisher_node = RoboteqRepublisherNode('roboteq_republisher_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
