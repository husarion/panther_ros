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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from panther_utils.arguments import DeclareRobotArgs
from panther_utils.messages import welcome_msg


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    robot_configuration = LaunchConfiguration("robot_configuration")
    robot_model = LaunchConfiguration("robot_model")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    wheel_type = LaunchConfiguration("wheel_type")

    declare_robot_configuration_arg = DeclareLaunchArgument(
        "robot_configuration",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "configuration.yaml"]
        ),
        description="Path to robot configuration YAML file.",
    )

    log_stats = {
        "Robot namespace": namespace,
        "Initial pose": ["(", x, ", ", y, ", ", z, ", ", roll, ", ", pitch, ", ", yaw, ")"],
    }
    welcome_info = welcome_msg(robot_model, "----", "simulation", log_stats)

    urdf_packages = PythonExpression(["'", robot_model, "_description'"])
    add_wheel_joints = LaunchConfiguration("add_wheel_joints", default="True")

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(urdf_packages), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "add_wheel_joints": add_wheel_joints,
            "namespace": namespace,
            "robot_model": robot_model,
            "wheel_type": wheel_type,
            "use_sim": "True",
        }.items(),
    )

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
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            declare_robot_configuration_arg,
            DeclareRobotArgs(robot_configuration),
            SetUseSimTime(True),
            welcome_info,
            load_urdf,
            spawn_robot,
        ]
    )
