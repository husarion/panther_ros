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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from panther_utils.arguments import load_yaml_file, normalize_robot_configuration


def generate_launch_description():

    gz_gui = LaunchConfiguration("gz_gui")
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "teleop_with_estop.config"]
        ),
        description="Run simulation with specific GUI layout.",
    )

    path = PathJoinSubstitution(
        [FindPackageShare("panther_gazebo"), "config", "configuration.yaml"]
    )
    yaml_data = load_yaml_file(path)
    yaml_data = normalize_robot_configuration(yaml_data)
    ns_from_config = list(yaml_data.keys())[0]

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=ns_from_config),
        description="Add namespace to all launched nodes.",
    )

    namespaced_gz_gui = ReplaceString(
        source_file=gz_gui,
        replacements={"{namespace}": namespace},
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_gui": namespaced_gz_gui, "gz_log_level": "1"}.items(),
    )

    simulate_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_gazebo"),
                    "launch",
                    "simulate_multiple_robots.launch.py",
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            declare_gz_gui,
            declare_namespace_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetUseSimTime(True),
            gz_sim,
            simulate_robots,
        ]
    )
