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
from launch.conditions import LaunchConfigurationEquals
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

    declare_ekf_configuration_arg = DeclareLaunchArgument(
        "ekf_configuration",
        default_value="local",
        description=(
            "Set the EKF mode: "
            "'local' combines wheel odometer and IMU data. "
            "'global' adds GPS data to this fusion."
        ),
        choices=["local", "global"],
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

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[ekf_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "~/enable"),
            ("set_pose", "~/set_pose"),
            ("toggle", "~/toggle"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        parameters=[ekf_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "~/enable"),
            ("set_pose", "~/set_pose"),
            ("toggle", "~/toggle"),
            ("odometry/filtered", "odometry/filtered/global"),
        ],
        condition=LaunchConfigurationEquals("ekf_configuration", "global"),
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[ekf_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("gps/fix", "gps/fix"),
            ("odometry/filtered", "odometry/filtered/global"),
            ("odometry/gps", "_odometry/gps"),
        ],
        condition=LaunchConfigurationEquals("ekf_configuration", "global"),
    )

    actions = [
        declare_ekf_config_path_arg,
        declare_ekf_configuration_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        ekf_local,
        ekf_global,
        navsat_transform,
    ]

    return LaunchDescription(actions)
