#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    add_wheel_joints = LaunchConfiguration("add_wheel_joints")
    declared_add_wheel_joints_arg = DeclareLaunchArgument(
        "add_wheel_joints",
        default_value="True",
        description="Flag enabling joint_state_publisher to publish information about the wheel position. Should be false when there is a controller that sends this information.",
        choices=["True", "False"],
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value="",
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

    wheel_type = LaunchConfiguration("wheel_type")  # wheel_type is used by controller_config_path
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_controller"),
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file. By default, it is located in"
            " 'panther_controller/config/<wheel_type arg>_controller.yaml'. You can also specify"
            " the path to your custom controller configuration file here. "
        ),
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    panther_version = LaunchConfiguration("panther_version")
    declare_panther_version_arg = DeclareLaunchArgument(
        "panther_version",
        default_value=EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0"),
        description="Hardware version of Panther robot.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used.",
        choices=["True", "False"],
    )

    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description=(
            "Path to wheel configuration file. By default, it is located in "
            "'panther_description/config/<wheel_type arg>.yaml'. You can also specify the path "
            "to your custom wheel configuration file here. "
        ),
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description=(
            "Type of wheel. If you choose a value from the preset options ('WH01', 'WH02',"
            " 'WH04'), you can ignore the 'wheel_config_path' and 'controller_config_path'"
            " parameters. For custom wheels, please define these parameters to point to files that"
            " accurately describe the custom wheels."
        ),
        choices=["WH01", "WH02", "WH04", "custom"],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("panther_description"), "urdf", "panther.urdf.xacro"]
            ),
            " panther_version:=",
            panther_version,
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            controller_config_path,
            " battery_config_file:=",
            battery_config_path,
            " namespace:=",
            namespace,
            " components_config_path:=",
            components_config_path,
        ]
    )

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content},
            {"frame_prefix": namespace_ext},
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(add_wheel_joints),
    )

    actions = [
        declared_add_wheel_joints_arg,
        declare_battery_config_path_arg,
        declare_components_config_path_arg,
        declare_wheel_type_arg,  # wheel_type is used by controller_config_path
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_panther_version_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_pub_node,
        joint_state_publisher_node,  # do not publish, when use_sim is true
    ]

    return LaunchDescription(actions)
