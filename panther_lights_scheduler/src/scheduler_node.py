#!/usr/bin/env python3

import rospy


class LightsSchedulerNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)


def main():
    lights_scheduler_node = LightsSchedulerNode('lights_scheduler_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
