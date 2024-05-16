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
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        description="Path to controller configuration file.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    publish_robot_state = LaunchConfiguration("publish_robot_state")
    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="True",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    panther_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_description"),
                    "launch",
                    "panther_description.launch.py",
                ]
            )
        ),
        condition=publish_robot_state,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        namespace=namespace,
        remappings=[
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
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_publish_robot_state_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        panther_description,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
    ]

    return LaunchDescription(actions)
