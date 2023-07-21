#!/usr/bin/python3

import click
import os
import textwrap

import rospy
import rospkg


class WelcomMsgNode:
    PANTHER_TEXT = '''
     ____             _   _               
    |  _ \ __ _ _ __ | |_| |__   ___ _ __ 
    | |_) / _` | '_ \| __| '_ \ / _ \ '__|
    |  __/ (_| | | | | |_| | | |  __/ |   
    |_|   \__,_|_| |_|\__|_| |_|\___|_|   
                                                                    
    '''
    ERROR_MESSAGE = '''
    OS detected incorrect hardware. ROS nodes are prevented from starting!
    Refer to infstructions in manual or those shown on terminal login.
    '''

    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        correct_hw_env = os.environ.get('PANTHER_HW_CONFIG_CORRECT')

        exit_on_wrong_hw = rospy.get_param('~exit_on_wrong_hw', True)

        rospack = rospkg.RosPack()
        stats_to_show = {
            'Serial number': rospy.get_param('/panther/serial_no', '----'),
            'Robot version': rospy.get_param('/panther/robot_version', '1.0'),
            'ROS driver version': rospack.get_manifest('panther').version,
            'Website': 'https://husarion.com',
            'Support': 'https://community.husarion.com/',
            'Bugtracker': 'https://github.com/husarion/panther_ros/issues',
        }

        pth_txt = textwrap.dedent(WelcomMsgNode.PANTHER_TEXT)
        stats_msg = click.style(pth_txt, bold=True) + ''.join(
            [f'{click.style(name, bold=True)}: {value}\n' for name, value in stats_to_show.items()]
        )
        rospy.loginfo(f'[{rospy.get_name()}] Panther statistics: {stats_msg}')

        if not correct_hw_env or correct_hw_env.lower() == 'false':
            for msg in textwrap.dedent(WelcomMsgNode.ERROR_MESSAGE).strip('\n').split('\n'):
                rospy.logerr(f'[{rospy.get_name()}] {msg}')

            if exit_on_wrong_hw:
                rospy.signal_shutdown('Panther configuration is incorrect!')
                return

        else:
            rospy.loginfo(f'[{rospy.get_name()}] Panther configuration is correct')


def main():
    welcome_msg_node = WelcomMsgNode('welcome_msg_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
