#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    led_config_file = LaunchConfiguration("led_config_file")
    declare_led_config_file_arg = DeclareLaunchArgument(
        "led_config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_lights"), "config", "led_config.yaml"]
        ),
        description="Path to a YAML file with a description of led configuration.",
    )

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

    user_led_animations_file = LaunchConfiguration("user_led_animations_file")
    declare_user_led_animations_file_arg = DeclareLaunchArgument(
        "user_led_animations_file",
        default_value="",
        description="Path to a YAML file with a description of the user defined animations.",
    )

    lights_container = ComposableNodeContainer(
        package="rclcpp_components",
        name="lights_container",
        namespace=namespace,
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="panther_lights",
                plugin="panther_lights::LightsDriverNode",
                name="lights_driver",
                namespace=namespace,
                remappings=[("/diagnostics", "diagnostics")],
                extra_arguments=[
                    {"use_intra_process_comms": True},
                ],
                condition=UnlessCondition(use_sim),
            ),
            ComposableNode(
                package="panther_lights",
                plugin="panther_lights::LightsControllerNode",
                name="lights_controller",
                namespace=namespace,
                parameters=[
                    {"led_config_file": led_config_file},
                    {"user_led_animations_file": user_led_animations_file},
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True},
                ],
            ),
        ],
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    actions = [
        declare_led_config_file_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_user_led_animations_file_arg,
        lights_container,
    ]

    return LaunchDescription(actions)
