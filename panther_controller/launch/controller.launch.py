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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
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
    wheel_type = LaunchConfiguration("wheel_type")
    battery_config_path = LaunchConfiguration("battery_config_path")
    components_config_path = LaunchConfiguration("components_config_path")
    controller_config_path = LaunchConfiguration("controller_config_path")
    namespace = LaunchConfiguration("namespace")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    wheel_config_path = LaunchConfiguration("wheel_config_path")
    use_sim = LaunchConfiguration("use_sim")

    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value="",
    )

    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value="None",
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in Panther's urdf."
            "Panther options are described here "
            "https://husarion.com/manuals/panther/panther-options/"
        ),
    )

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

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="True",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
        choices=["True", "False"],
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
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
                [
                    FindPackageShare("panther_description"),
                    "urdf",
                    "panther.urdf.xacro",
                ]
            ),
            " panther_version:=",
            os.environ.get("PANTHER_ROBOT_VERSION", "1.2"),
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            controller_config_path,
            " battery_config_file:=",
            battery_config_path,
            " imu_pos_x:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_X", "0.168"),
            " imu_pos_y:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_Y", "0.028"),
            " imu_pos_z:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_Z", "0.083"),
            " imu_rot_r:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_R", "3.14"),
            " imu_rot_p:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_P", "-1.57"),
            " imu_rot_y:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_Y", "0.0"),
            " namespace:=",
            namespace,
            " components_config_path:=",
            components_config_path,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            (
                "panther_system_node/driver/motor_controllers_state",
                "driver/motor_controllers_state",
            ),
            ("panther_base_controller/cmd_vel_unstamped", "cmd_vel"),
            ("panther_base_controller/odom", "odom/wheels"),
            ("panther_system_node/io_state", "hardware/io_state"),
            ("panther_system_node/e_stop", "hardware/e_stop"),
            ("panther_system_node/e_stop_trigger", "hardware/e_stop_trigger"),
            ("panther_system_node/e_stop_reset", "hardware/e_stop_reset"),
            ("panther_system_node/fan_enable", "hardware/fan_enable"),
            ("panther_system_node/aux_power_enable", "hardware/aux_power_enable"),
            ("panther_system_node/charger_enable", "hardware/charger_enable"),
            ("panther_system_node/digital_power_enable", "hardware/digital_power_enable"),
            ("panther_system_node/motor_power_enable", "hardware/motor_power_enable"),
        ],
        condition=UnlessCondition(use_sim),
    )

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"frame_prefix": namespace_ext}],
        namespace=namespace,
        condition=IfCondition(publish_robot_state),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panther_base_controller",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
            "--namespace",
            namespace,
        ],
        namespace=namespace,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
            "--namespace",
            namespace,
        ],
        namespace=namespace,
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            "controller_manager",
            "--controller-manager-timeout",
            "10",
            "--namespace",
            namespace,
        ],
        namespace=namespace,
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        ),
    )

    actions = [
        declare_battery_config_path_arg,
        declare_wheel_type_arg,  # wheel_type must be before controller_config_path
        declare_components_config_path_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_publish_robot_state_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
    ]

    return LaunchDescription(actions)
