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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config_path = LaunchConfiguration("ekf_config_path")
    declare_ekf_config_path_arg = DeclareLaunchArgument(
        "ekf_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_localization"), "config", "ekf.yaml"]
        ),
        description="Path to the EKF config file.",
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
        description="Whether simulation is used.",
        choices=["True", "False"],
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        parameters=[ekf_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "ekf_node/enable"),
            ("set_pose", "ekf_node/set_pose"),
            ("toggle", "ekf_node/toggle"),
        ],
        emulate_tty=True,
    )

    actions = [
        declare_ekf_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        robot_localization_node,
    ]

    return LaunchDescription(actions)
