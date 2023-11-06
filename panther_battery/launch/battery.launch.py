from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    panther_version = LaunchConfiguration('panther_version')
    declare_panther_version_arg = DeclareLaunchArgument('panther_version')

    roboteq_republisher_node = Node(
        condition=IfCondition(PythonExpression([panther_version, '< 1.2'])),
        package='panther_battery',
        executable='roboteq_republisher_node',
        name='battery_node',
    )

    adc_node = Node(
        condition=IfCondition(PythonExpression([panther_version, '>= 1.2'])),
        package='panther_battery',
        executable='adc_node',
        name='battery_node',
    )

    actions = [
        declare_panther_version_arg,
        roboteq_republisher_node,
        adc_node,
    ]

    return LaunchDescription(actions)
