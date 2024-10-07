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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from panther_utils.messages import welcome_msg


def generate_launch_description():
    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "true", "False", "false"],
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF.",
        choices=["True", "true", "False", "false"],
    )

    robot_model_dict = {"LNX": "lynx", "PTH": "panther"}
    robot_model_env = os.environ.get("ROBOT_MODEL", default="PTH")
    robot_model = robot_model_dict[robot_model_env]
    robot_serial_no = EnvironmentVariable(name="ROBOT_SERIAL_NO", default_value="----")
    robot_version = EnvironmentVariable(name="ROBOT_VERSION", default_value="1.0")
    welcome_info = welcome_msg(robot_model, robot_serial_no, robot_version)

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    system_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_diagnostics"),
                    "launch",
                    "system_monitor.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_battery"), "launch", "battery.launch.py"]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_localization"), "launch", "localization.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace}.items(),
        condition=IfCondition(use_ekf),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_manager"), "launch", "manager.launch.py"]
            )
        ),
        condition=UnlessCondition(disable_manager),
        launch_arguments={"namespace": namespace}.items(),
    )

    delayed_action = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            lights_launch,
            manager_launch,
            ekf_launch,
        ],
    )

    actions = [
        declare_disable_manager_arg,
        declare_namespace_arg,
        declare_use_ekf_arg,
        welcome_info,
        controller_launch,
        system_monitor_launch,
        delayed_action,
    ]

    return LaunchDescription(actions)
