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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from panther_utils.welcomeMsg import welcomeMsg


def generate_launch_description():

    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "battery_plugin_config.yaml"]
        ),
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_description"), "config", "components.yaml"]
        ),
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in Panther's urdf."
            "Panther options are described here "
            "https://husarion.com/manuals/panther/panther-options/"
        ),
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "gz_bridge.yaml"]
        ),
        description="Path to the parameter_bridge configuration file.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF.",
        choices=["True", "False"],
    )

    x = LaunchConfiguration("x")
    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial robot position in the global 'x' axis."
    )

    y = LaunchConfiguration("y")
    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="-2.0", description="Initial robot position in the global 'y' axis."
    )

    z = LaunchConfiguration("z")
    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.2", description="Initial robot position in the global 'z' axis."
    )

    roll = LaunchConfiguration("roll")
    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial robot 'roll' orientation."
    )

    pitch = LaunchConfiguration("pitch")
    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial robot 'pitch' orientation."
    )

    yaw = LaunchConfiguration("yaw")
    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial robot 'yaw' orientation."
    )

    log_stats = {
        "Robot namespace": namespace,
        "Initial pose": ["(", x, ", ", y, ", ", z, ", ", roll, ", ", pitch, ", ", yaw, ")"],
    }
    welcome_msg = welcomeMsg("---", "simulation", log_stats)

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            namespace,
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        namespace=namespace,
    )

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={"<namespace>": namespace, "//": "/"},
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="panther_base_gz_bridge",
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=namespace,
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_localization"), "launch", "ekf.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
        condition=IfCondition(use_ekf),
    )

    simulate_components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_components_description"),
                    "launch",
                    "gz_components.launch.py",
                ]
            )
        ),
        launch_arguments={
            "components_config_path": components_config_path,
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_battery_config_path_arg,
            declare_components_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_namespace_arg,
            declare_use_ekf_arg,
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            SetUseSimTime(True),
            welcome_msg,
            spawn_robot,
            controller_launch,
            ekf_launch,
            gz_bridge,
            simulate_components,
        ]
    )
