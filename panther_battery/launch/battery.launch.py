from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    panther_version = LaunchConfiguration("panther_version")
    declare_panther_version_arg = DeclareLaunchArgument("panther_version")

    battery_node = Node(
        package="panther_battery",
        executable="battery_node",
        name="battery_node",
        parameters=[{"panther_version": panther_version}],
    )

    actions = [
        declare_panther_version_arg,
        battery_node,
    ]

    return LaunchDescription(actions)
