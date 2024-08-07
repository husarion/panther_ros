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
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fuse_gps = LaunchConfiguration("fuse_gps")
    declare_fuse_gps_arg = DeclareLaunchArgument(
        "fuse_gps",
        default_value="False",
        description="Include GPS for data fusion",
        choices=["True", "true", "False", "false"],
    )

    localization_mode = LaunchConfiguration("localization_mode")
    declare_localization_mode_arg = DeclareLaunchArgument(
        "localization_mode",
        default_value="relative",
        description=(
            "Specifies the localization mode:\n"
            "\t- 'relative' odometry/filtered data is relative to the initial position and orientation.\n"
            "\t- 'enu' odometry/filtered data is relative to initial position and ENU (East North Up) orientation."
        ),
        choices=["relative", "enu"],
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
        choices=["True", "true", "False", "false"],
    )

    mode_prefix = PythonExpression(["'", localization_mode, "_'"])
    gps_postfix = PythonExpression(["'_with_gps' if ", fuse_gps, " else ''"])
    localization_config_filename = PythonExpression(
        ["'", mode_prefix, "localization", gps_postfix, ".yaml'"]
    )

    localization_config_path = LaunchConfiguration("localization_config_path")
    declare_localization_config_path_arg = DeclareLaunchArgument(
        "localization_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_localization"), "config", localization_config_filename]
        ),
        description="Specify the path to the localization configuration file.",
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter",
        parameters=[localization_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "localization/enable"),
            ("set_pose", "localization/set_pose"),
            ("toggle", "localization/toggle"),
        ],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[localization_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("imu", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("odometry/gps", "_odometry/gps"),
        ],
        condition=IfCondition(fuse_gps),
    )

    actions = [
        declare_fuse_gps_arg,
        declare_localization_mode_arg,
        declare_localization_config_path_arg,  # localization_config_path use fuse_gps and localization_mode
        declare_namespace_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        ekf_filter_node,
        navsat_transform_node,
    ]

    return LaunchDescription(actions)
