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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    imu_config_file = LaunchConfiguration("imu_config_file")
    declare_imu_config_file_arg = DeclareLaunchArgument(
        "imu_config_file",
        description="Path to IMU configuration file",
    )

    imu_container = ComposableNodeContainer(
        name="imu_container",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="phidgets_spatial",
                plugin="phidgets::SpatialRosI",
                name="phidgets_spatial_node",
                parameters=[imu_config_file],
            ),
            ComposableNode(
                package="imu_filter_madgwick",
                plugin="ImuFilterMadgwickRos",
                name="imu_filter_node",
                parameters=[imu_config_file],
            ),
        ],
    )

    actions = [
        declare_imu_config_file_arg,
        imu_container,
    ]

    return LaunchDescription(actions)
