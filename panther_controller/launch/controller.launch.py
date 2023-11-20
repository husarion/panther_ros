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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        description="Path to wheel configuration file.",
    )

    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        description="Path to controller configuration file.",
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        condition=IfCondition(use_sim),
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
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
            " use_sim:=",
            use_sim,
            " simulation_engine:=",
            simulation_engine,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            controller_config_path,
            " battery_config_file:=",
            battery_config_path,
            " imu_pos_x:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_X", 0.168),
            " imu_pos_y:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_Y", 0.028),
            " imu_pos_z:=",
            os.environ.get("PANTHER_IMU_LOCALIZATION_Z", 0.083),
            " imu_rot_r:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_R", 3.14),
            " imu_rot_p:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_P", -1.57),
            " imu_rot_y:=",
            os.environ.get("PANTHER_IMU_ORIENTATION_Y", 0.0),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_path],
        remappings=[
            ("panther_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(publish_robot_state),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panther_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
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
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )

    actions = [
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        declare_controller_config_path_arg,
        declare_battery_config_path_arg,
        declare_simulation_engine_arg,
        declare_publish_robot_state_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
    ]

    return LaunchDescription(actions)
