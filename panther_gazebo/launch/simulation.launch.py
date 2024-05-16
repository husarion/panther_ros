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
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    robots = LaunchConfiguration("robots")
    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value=[],
        description=(
            "The list of the robots spawned in the simulation e. g. robots:='robot1={x: 0.0, y:"
            " -1.0}; robot2={x: 1.0, y: -1.0}'"
        ),
    )

    add_world_transform = LaunchConfiguration("add_world_transform")
    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_gz_worlds"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_gazebo"),
                    "launch",
                    "spawn.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robots": robots,
            "add_world_transform": add_world_transform,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_namespace_arg,
            declare_robots_arg,
            declare_add_world_transform_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            spawn_robots_launch,
        ]
    )
