#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    wheel_type = LaunchConfiguration("wheel_type")
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description="Type of wheel. Possible: 'WH01', 'WH02', 'WH04' or a user-defined custom name.",
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
        description="Path to controller configuration file. "
        "By default, it should be located in "
        "panther_controller/config/<wheel_type arg>_controller.yaml",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
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
            "controller_config_path": controller_config_path,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
        }.items(),
    )

    # robot_localization_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[
    #         PathJoinSubstitution(
    #             [get_package_share_directory("panther_bringup"), "config", "ekf.yaml"]
    #         )
    #     ],
    # )

    # laser_filter_node = Node(
    #     package="laser_filters",
    #     executable="scan_to_scan_filter_chain",
    #     parameters=[
    #         PathJoinSubstitution(
    #             [
    #                 get_package_share_directory("panther_bringup"),
    #                 "config",
    #                 "laser_filter.yaml",
    #             ]
    #         )
    #     ],
    # )

    actions = [
        declare_wheel_type_arg,
        declare_use_sim_arg,
        declare_simulation_engine_arg,
        declare_controller_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        controller_launch,
        # robot_localization_node,
        # laser_filter_node,
    ]

    return LaunchDescription(actions)