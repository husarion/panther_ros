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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    namespace = LaunchConfiguration("namespace")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    use_sim = LaunchConfiguration("use_sim")

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
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
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

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_description"),
                    "launch",
                    "urdf.launch.py",
                ]
            )
        ),
        condition=IfCondition(publish_robot_state),
        launch_arguments={
            "wheel_type": wheel_type,
            "controller_config_path": controller_config_path,
            "namespace": namespace,
            "use_sim": use_sim,
        }.items(),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        namespace=namespace,
        remappings=[
            (
                "controller_manager/robot_description",
                "robot_description",
            ),  # Bug in ros2_control on humble
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
        declare_wheel_type_arg,  # wheel_type must be before controller_config_path
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_publish_robot_state_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        load_urdf,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
    ]

    return LaunchDescription(actions)
