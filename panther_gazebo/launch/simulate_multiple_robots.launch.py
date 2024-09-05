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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from panther_utils.arguments import load_yaml_file, normalize_robot_configuration


def generate_launch_description():
    add_world_transform = LaunchConfiguration("add_world_transform")
    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
        choices=["True", "true", "False", "false"],
    )

    path = PathJoinSubstitution(
        [FindPackageShare("panther_gazebo"), "config", "configuration.yaml"]
    )
    yaml_data = load_yaml_file(path)
    yaml_data = normalize_robot_configuration(yaml_data)

    simulate_robot = []
    idx = 0
    for namespace, robot_config in yaml_data.items():
        robot_model = robot_config.get("robot_model", "panther")
        x, y, z = robot_config.get("init_pose", [0.0, 0.0, 0.0])
        roll, pitch, yaw = robot_config.get("init_rotation", [0.0, 0.0, 0.0])
        x, y, z, roll, pitch, yaw = map(str, [x, y, z, roll, pitch, yaw])
        configuration_data = robot_config.get("configuration", {})
        wheel_type = configuration_data.get("wheel_type", "")

        # Prefer declaration over configuration:
        namespace = LaunchConfiguration("namespace", default=namespace)
        robot_model = LaunchConfiguration("robot_model", default=robot_model)
        wheel_type = LaunchConfiguration("wheel_type", default=wheel_type)
        x = LaunchConfiguration("x", default=x)
        y = LaunchConfiguration("y", default=y)
        z = LaunchConfiguration("z", default=z)
        roll = LaunchConfiguration("roll", default=roll)
        pitch = LaunchConfiguration("pitch", default=pitch)
        yaw = LaunchConfiguration("yaw", default=yaw)

        spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("panther_gazebo"), "launch", "simulate_robot.launch.py"]
                )
            ),
            launch_arguments={
                "namespace": namespace,
                "robot_model": robot_model,
                "wheel_type": wheel_type,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }.items(),
        )

        child_tf = PythonExpression(
            ["'", namespace, "' + '/odom' if '", namespace, "' else 'odom'"]
        )

        world_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_publisher",
            arguments=[x, y, z, roll, pitch, yaw, "world", child_tf],
            namespace=namespace,
            emulate_tty=True,
            condition=IfCondition(add_world_transform),
        )

        # Add small delay to prevent namespace overwriting
        group = TimerAction(
            period=5.0 * idx,
            actions=[
                spawn_robot_launch,
                world_transform,
            ],
        )
        idx += 1
        simulate_robot.append(group)

    return LaunchDescription(
        [
            declare_add_world_transform_arg,
            SetUseSimTime(True),
            *simulate_robot,
        ]
    )
