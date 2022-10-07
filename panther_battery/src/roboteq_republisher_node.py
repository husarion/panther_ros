#!/usr/bin/python3

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState


class RoboteqRepublisherNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        self._battery_driv_sub = rospy.Subscriber('motor_controllers_state', DriverState, self._battery_driv_cb)

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _battery_driv_cb(self, msg) -> None:
        V_bat = (msg.front.voltage + msg.rear.voltage) / 2.0
        I_bat = (msg.front.current + msg.rear.current) / 2.0
        self._publish_battery_msg(V_bat, I_bat)

    def _publish_battery_msg(self, V_bat=float('nan'), I_bat=float('nan')) -> None:
        battery_msg = BatteryState()

        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = V_bat
        battery_msg.temperature = float('nan')
        battery_msg.current = I_bat
        battery_msg.percentage = (battery_msg.voltage - 32.0) / 10.0
        battery_msg.capacity = 20.0
        battery_msg.design_capacity = 20.0
        battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
        battery_msg.power_supply_status
        battery_msg.power_supply_health
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_msg.present = True

        self._battery_publisher.publish(battery_msg)


def main():
    roboteq_republisher_node = RoboteqRepublisherNode('roboteq_republisher_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
