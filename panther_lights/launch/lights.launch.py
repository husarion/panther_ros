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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    lights_pkg = FindPackageShare("panther_lights")
    animations_file = PythonExpression(["'", robot_model, "_animation.yaml'"])
    default_animations_path = PathJoinSubstitution([lights_pkg, "config", animations_file])

    animations_config_path = LaunchConfiguration("animations_config_path")
    declare_animations_config_path_arg = DeclareLaunchArgument(
        "animations_config_path",
        default_value=default_animations_path,
        description="Path to a YAML file with a description of led configuration.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    robot_model_dict = {"LNX": "lynx", "PTH": "panther"}
    robot_model_env = os.environ.get("ROBOT_MODEL", default="PTH")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=robot_model_dict[robot_model_env],
        description="Specify robot model",
        choices=["lynx", "panther"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    user_led_animations_path = LaunchConfiguration("user_led_animations_path")
    declare_user_led_animations_path_arg = DeclareLaunchArgument(
        "user_led_animations_path",
        default_value="",
        description="Path to a YAML file with a description of the user defined animations.",
    )

    driver_file = PythonExpression(["'", robot_model, "_driver.yaml'"])
    driver_path = PathJoinSubstitution([lights_pkg, "config", driver_file])
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
                parameters=[driver_path],
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
                    {"animations_config_path": animations_config_path},
                    {"user_led_animations_path": user_led_animations_path},
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
        declare_robot_model_arg,  # robot_model is used by animations_config_path
        declare_animations_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_user_led_animations_path_arg,
        lights_container,
    ]

    return LaunchDescription(actions)
