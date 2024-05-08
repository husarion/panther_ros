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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    panther_version = LaunchConfiguration("panther_version")
    declare_panther_version_arg = DeclareLaunchArgument(
        "panther_version",
        description="Panther robot version",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    panther_version = float(LaunchConfiguration("panther_version").perform(context))
    panther_manager_shared_dir = FindPackageShare("panther_manager")
    if panther_version >= 1.2:
        manager_bt_config_path = PathJoinSubstitution(
            [panther_manager_shared_dir, "config", "manager_bt_config.yaml"]
        )
        default_bt_project_path = PathJoinSubstitution(
            [panther_manager_shared_dir, "behavior_trees", "Panther12BT.btproj"]
        )
    else:
        manager_bt_config_path = PathJoinSubstitution(
            [panther_manager_shared_dir, "config", "manager_bt_config_106.yaml"]
        )
        default_bt_project_path = PathJoinSubstitution(
            [panther_manager_shared_dir, "behavior_trees", "Panther106BT.btproj"]
        )

    bt_project_path = LaunchConfiguration("bt_project_path")
    declare_bt_project_path_arg = DeclareLaunchArgument(
        "bt_project_path",
        default_value=default_bt_project_path,
        description="Path to BehaviorTree project file.",
    )

    shutdown_hosts_config_path = LaunchConfiguration("shutdown_hosts_config_path")
    declare_shutdown_hosts_config_path_arg = DeclareLaunchArgument(
        "shutdown_hosts_config_path",
        default_value="",
        description="Path to file with list of hosts to request shutdown.",
    )

    manager_bt_node = Node(
        package="panther_manager",
        executable="manager_bt_node",
        name="manager_bt_node",
        parameters=[
            manager_bt_config_path,
            {
                "bt_project_path": bt_project_path,
                "shutdown_hosts_path": shutdown_hosts_config_path,
            },
        ],
        namespace=namespace,
    )

    return [
        declare_panther_version_arg,
        declare_bt_project_path_arg,
        declare_namespace_arg,
        declare_shutdown_hosts_config_path_arg,
        manager_bt_node,
    ]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    return LaunchDescription([opfunc])
