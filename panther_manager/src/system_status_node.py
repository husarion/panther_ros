#!/usr/bin/python3

import psutil

import rospy

from panther_msgs.msg import SystemStatus


class SystemStatusNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._critical_cpu_temp = 80.0

        if self._disc_usage_percent > 0.95:
            rospy.logwarn(
                f'[{rospy.get_name()}] High disc usage. '
                f'{round(self._disc_usage_percent * 100.0, 2)}% used.'
            )

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._system_status_pub = rospy.Publisher('system_status', SystemStatus, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._stats_timer = rospy.Timer(rospy.Duration(1.0), self._stats_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _stats_timer_cb(self, *args) -> None:
        status_msg = SystemStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.cpu_percent = self._cpu_percent
        status_msg.cpu_temp = self._cpu_temp
        status_msg.avg_load_percent = self._avg_load_percent
        status_msg.ram_usage_percent = self._ram_usage_percent
        status_msg.disc_usage_percent = self._disc_usage_percent
        self._system_status_pub.publish(status_msg)

        if self._cpu_temp > self._critical_cpu_temp:
            rospy.logerr_throttle(
                60.0,
                f'[{rospy.get_name()}] CPU reached critical ',
                f'temperature of {int(round(self._cpu_temp) + 0.1)} deg C!'
            )

    @property
    def _cpu_percent(self) -> float:
        return [p / 100.0 for p in psutil.cpu_percent(interval=1, percpu=True)]

    @property
    def _cpu_temp(self) -> float:
        return psutil.sensors_temperatures()['cpu_thermal'][0].current

    @property
    def _avg_load_percent(self) -> float:
        return psutil.getloadavg()[2] / 100.0

    @property
    def _ram_usage_percent(self) -> float:
        return psutil.virtual_memory().percent / 100.0

    @property
    def _disc_usage_percent(self) -> float:
        return psutil.disk_usage('/').percent / 100.0


def main():
    system_status_node = SystemStatusNode('system_status_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
