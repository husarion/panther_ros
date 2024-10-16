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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")

    battery_driver_node = Node(
        package="panther_battery",
        executable="battery_driver_node",
        name="battery_driver",
        parameters=[{"panther_version": panther_version}],
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
        emulate_tty=True,
    )

    actions = [
        declare_namespace_arg,
        battery_driver_node,
    ]

    return LaunchDescription(actions)
