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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
        choices=[True, False, "True", "False", "true", "false", "1", "0"],
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    docking_server_config_path = LaunchConfiguration("docking_server_config_path")
    declare_docking_server_config_path_arg = DeclareLaunchArgument(
        "docking_server_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_docking"), "config", "panther_docking_server.yaml"]
        ),
        description=("Path to docking server configuration file."),
    )

    namespaced_docking_server_config = ReplaceString(
        source_file=docking_server_config_path,
        replacements={"<robot_namespace>": namespace, "//": "/"},
    )

    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        parameters=[
            {"panther_version": panther_version},
            namespaced_docking_server_config,
            {"use_sim_time": True},
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    docking_server_activate_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav2_docking_lifecycle_manager",
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "docking_server",
                ],
                "use_sim_time": use_sim,
            },
        ],
        namespace=namespace,
    )

    return LaunchDescription(
        [
            declare_use_sim_arg,
            declare_namespace_arg,
            declare_docking_server_config_path_arg,
            docking_server_node,
            docking_server_activate_node,
        ]
    )
