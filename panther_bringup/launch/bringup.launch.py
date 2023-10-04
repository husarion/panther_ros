#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    wheel_type = LaunchConfiguration("wheel_type")
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description="Type of wheel. If you choose a value from the preset options "
        "('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and "
        "'controller_config_path' parameters. For custom wheels, please define these parameters "
        "to point to files that accurately describe the custom wheels.",
        choices=['WH01', 'WH02', 'WH04', 'CUSTOM'],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description="Path to wheel configuration file. By default, it is located in "
        "'panther_description/config/<wheel_type arg>.yaml'. You can also specify the path "
        "to your custom wheel configuration file here. ",
    )

    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_controller"),
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description="Path to wheel configuration file. By default, it is be located in "
        "'panther_controller/config/<wheel_type arg>_controller.yaml'. You can also specify the path "
        "to your custom controller configuration file here. ",
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description="Path to the Ignition LinearBatteryPlugin configuration file. "
        "This configuration is intended for use in simulations only.",
        condition=IfCondition(use_sim),
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "wheel_type": wheel_type,
            "wheel_config_path": wheel_config_path,
            "controller_config_path": controller_config_path,
            "battery_config_path": battery_config_path,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
        }.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [get_package_share_directory("panther_bringup"), "config", "ekf.yaml"]
            )
        ],
    )

    actions = [
        declare_use_sim_arg,
        declare_wheel_type_arg,
        declare_wheel_config_path_arg,
        declare_controller_config_path_arg,
        declare_battery_config_path_arg,
        declare_simulation_engine_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        controller_launch,
        robot_localization_node,
    ]

    return LaunchDescription(actions)
