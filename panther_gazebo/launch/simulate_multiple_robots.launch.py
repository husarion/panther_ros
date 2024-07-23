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
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    add_world_transform = LaunchConfiguration("add_world_transform")
    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
        choices=["True", "False"],
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value=[],
        description=(
            "The list of the robots spawned in the simulation e. g. "
            "robots:='robot1={x: 1.0, y: -2.0}; robot2={x: 1.0, y: -4.0}'."
        ),
    )

    robots_list = ParseMultiRobotPose("robots").value()
    # If robots arg is empty, use default arguments from simulate_robot.launch.py
    if len(robots_list) == 0:
        robots_list = {
            LaunchConfiguration("namespace", default=""): {
                "x": LaunchConfiguration("x", default="0.0"),
                "y": LaunchConfiguration("y", default="-2.0"),
                "z": LaunchConfiguration("z", default="0.2"),
                "roll": LaunchConfiguration("roll", default="0.0"),
                "pitch": LaunchConfiguration("pitch", default="0.0"),
                "yaw": LaunchConfiguration("yaw", default="0.0"),
            }
        }
    else:
        for robot_name, init_pose in robots_list.items():
            robots_list[robot_name] = {k: str(v) for k, v in init_pose.items()}

    simulate_robot = []
    for idx, robot_name in enumerate(robots_list):
        init_pose = robots_list[robot_name]
        x, y, z, roll, pitch, yaw = [value for value in init_pose.values()]

        spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("panther_gazebo"), "launch", "simulate_robot.launch.py"]
                )
            ),
            launch_arguments={
                "namespace": robot_name,
                "x": x,
                "y": y,
                "z": z,
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
            }.items(),
        )

        child_tf = PythonExpression(
            ["'", robot_name, "' + '/odom' if '", robot_name, "' else 'odom'"]
        )

        world_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_publisher",
            arguments=[x, y, z, roll, pitch, yaw, "world", child_tf],
            namespace=robot_name,
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
        simulate_robot.append(group)

    return LaunchDescription(
        [
            declare_add_world_transform_arg,
            declare_robots_arg,
            SetUseSimTime(True),
            *simulate_robot,
        ]
    )
