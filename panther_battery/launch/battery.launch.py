#!/usr/bin/env python3

# Copyright 2023 Husarion sp. z o.o.
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    panther_version = LaunchConfiguration("panther_version")
    declare_panther_version_arg = DeclareLaunchArgument("panther_version")

    roboteq_republisher_node = Node(
        condition=IfCondition(PythonExpression([panther_version, "< 1.2"])),
        package="panther_battery",
        executable="roboteq_republisher_node",
        name="battery_node",
    )

    adc_node = Node(
        condition=IfCondition(PythonExpression([panther_version, ">= 1.2"])),
        package="panther_battery",
        executable="adc_node",
        name="battery_node",
    )

    actions = [
        declare_panther_version_arg,
        roboteq_republisher_node,
        adc_node,
    ]

    return LaunchDescription(actions)
