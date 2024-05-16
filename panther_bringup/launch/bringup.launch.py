#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import textwrap

import click
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")
    panther_serial_no = EnvironmentVariable(name="PANTHER_SERIAL_NO", default_value="----")
    panther_pkg_version = Command(command="ros2 pkg xml -t version panther")

    PANTHER_TEXT = """
     ____             _   _
    |  _ \ __ _ _ __ | |_| |__   ___ _ __
    | |_) / _` | '_ \| __| '_ \ / _ \ '__|
    |  __/ (_| | | | | |_| | | |  __/ |
    |_|   \__,_|_| |_|\__|_| |_|\___|_|

    """  # noqa: W605

    stats_to_show = {
        "Serial number": panther_serial_no,
        "Robot version": panther_version,
        "ROS driver version": panther_pkg_version,
        "Website": "https://husarion.com",
        "Support": "https://community.husarion.com/",
        "Bugtracker": "https://github.com/husarion/panther_ros/issues",
    }

    stats_msg = [
        item
        for name, value in stats_to_show.items()
        for item in (f"{click.style(name, bold=True)}: ", value, "\n")
    ]
    pth_txt = textwrap.dedent(PANTHER_TEXT)
    stats_msg.insert(0, click.style(pth_txt, bold=True))

    welcome_msg = LogInfo(msg=stats_msg)

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    led_config_file = LaunchConfiguration("led_config_file")
    declare_led_config_file_arg = DeclareLaunchArgument(
        "led_config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_lights"), "config", "led_config.yaml"]
        ),
        description="Path to a YAML file with a description of led configuration",
    )

    user_led_animations_file = LaunchConfiguration("user_led_animations_file")
    declare_user_led_animations_file_arg = DeclareLaunchArgument(
        "user_led_animations_file",
        default_value="",
        description="Path to a YAML file with a description of the user defined animations",
    )

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF",
    )

    ekf_config_path = LaunchConfiguration("ekf_config_path")
    declare_ekf_config_path_arg = DeclareLaunchArgument(
        "ekf_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_bringup"), "config", "ekf.yaml"]
        ),
        description="Path to the EKF config file",
        condition=IfCondition(use_ekf),
    )

    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node",
        choices=["True", "False"],
    )

    shutdown_hosts_config_path = LaunchConfiguration("shutdown_hosts_config_path")
    declare_shutdown_hosts_config_path_arg = DeclareLaunchArgument(
        "shutdown_hosts_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_bringup"),
                "config",
                "shutdown_hosts.yaml",
            ]
        ),
        description="Path to file with list of hosts to request shutdown.",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim": use_sim,
            "namespace": namespace,
        }.items(),
    )

    system_status_node = Node(
        package="panther_diagnostics",
        executable="system_status",
        name="system_status",
        output="screen",
        condition=UnlessCondition(use_sim),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_lights"),
                    "launch",
                    "lights.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "led_config_file": led_config_file,
            "namespace": namespace,
            "user_led_animations_file": user_led_animations_file,
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_battery"),
                    "launch",
                    "battery.launch.py",
                ]
            ),
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "namespace": namespace,
            "panther_version": panther_version,
        }.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "ekf_node/enable"),
            ("set_pose", "ekf_node/set_pose"),
            ("toggle", "ekf_node/toggle"),
        ],
        condition=IfCondition(use_ekf),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_manager"),
                    "launch",
                    "manager_bt.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(PythonExpression([use_sim, " or ", disable_manager])),
        launch_arguments={
            "namespace": namespace,
            "panther_version": panther_version,
            "shutdown_hosts_config_path": shutdown_hosts_config_path,
        }.items(),
    )

    other_action_timer = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            lights_launch,
            robot_localization_node,
            manager_launch,
        ],
    )

    actions = [
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_led_config_file_arg,
        declare_user_led_animations_file_arg,
        declare_use_ekf_arg,
        declare_ekf_config_path_arg,
        declare_disable_manager_arg,
        declare_shutdown_hosts_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        welcome_msg,
        controller_launch,
        system_status_node,
        other_action_timer,
    ]

    return LaunchDescription(actions)
