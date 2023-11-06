from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lights_driver_node = Node(
        package='panther_lights',
        executable='driver_node',
        name='lights_driver_node',
    )

    lights_controller_node = Node(
        package='panther_lights',
        executable='controller_node',
        name='lights_controller_node',
    )

    actions = [
        lights_driver_node,
        lights_controller_node,
    ]

    return LaunchDescription(actions)
