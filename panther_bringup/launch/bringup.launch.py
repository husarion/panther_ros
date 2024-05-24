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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from panther_utils.welcomeMsg import welcomeMsg


def generate_launch_description():
    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "False"],
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    serial_no = EnvironmentVariable(name="PANTHER_SERIAL_NO", default_value="----")
    robot_hw_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.2")
    welcome_msg = welcomeMsg(serial_no, robot_hw_version)

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    system_status_node = Node(
        package="panther_diagnostics",
        executable="system_status",
        name="system_status",
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_battery"), "launch", "battery.launch.py"]
            ),
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("panther_bringup"), "launch", "ekf.launch.py"])
        ),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_manager"), "launch", "manager_bt.launch.py"]
            )
        ),
        condition=UnlessCondition(disable_manager),
        launch_arguments={
            "namespace": namespace,
        }.items(),
    )

    other_action_timer = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            lights_launch,
            ekf_launch,
            manager_launch,
        ],
    )

    actions = [
        declare_disable_manager_arg,
        declare_namespace_arg,
        welcome_msg,
        controller_launch,
        system_status_node,
        other_action_timer,
    ]

    return LaunchDescription(actions)
