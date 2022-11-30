#!/usr/bin/python3

import psutil

import rospy

from panther_msgs.msg import SystemStatus


class SystemStatusNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        self._critical_cpu_temp = 80.0
        
        if self._disc_usage_percent > 95.0:
            rospy.logwarn(f'[{rospy.get_name()}] High disc usage. {self._disc_usage_percent}% used.')

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._system_status_publisher = rospy.Publisher('system_status', SystemStatus, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._stats_timer = rospy.Timer(rospy.Duration(1.0), self._stats_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _stats_timer_cb(self, event=None) -> None:
        status_msg = SystemStatus()
        status_msg.header.stamp = rospy.Time.now()
        status_msg.cpu_percent = [round(p) for p in self._cpu_percent]
        status_msg.cpu_temp = round(self._cpu_temp)
        status_msg.avg_load_percent = round(self._avg_load_percent)
        status_msg.ram_usage_percent = round(self._ram_usage_percent)
        status_msg.disc_usage_percent = round(self._disc_usage_percent)
        self._system_status_publisher.publish(status_msg)

        if self._cpu_temp > self._critical_cpu_temp:
            rospy.logerr_throttle(60,
                f'[{rospy.get_name()}] CPU reached critical '
                f'temperature of {int(round(self._cpu_temp) + 0.1)} deg C!')

    @property
    def _cpu_percent(self):
        return psutil.cpu_percent(interval=1, percpu=True)

    @property
    def _cpu_temp(self):
        return psutil.sensors_temperatures()['cpu_thermal'][0].current

    @property
    def _avg_load_percent(self):
        return psutil.getloadavg()[2]

    @property
    def _ram_usage_percent(self):
        return psutil.virtual_memory().percent

    @property
    def _disc_usage_percent(self):
        return psutil.disk_usage('/').percent


def main():
    system_status_node = SystemStatusNode('system_status_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass