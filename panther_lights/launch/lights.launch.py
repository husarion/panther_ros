#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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
from launch_ros.actions import Node


def generate_launch_description():
    lights_driver_node = Node(
        package="panther_lights",
        executable="driver_node",
        name="lights_driver_node",
    )

    lights_controller_node = Node(
        package="panther_lights",
        executable="controller_node",
        name="lights_controller_node",
    )

    actions = [
        lights_driver_node,
        lights_controller_node,
    ]

    return LaunchDescription(actions)
