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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    add_world_transform = LaunchConfiguration("add_world_transform")
    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
    )

    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_gazebo"),
                "config",
                "gz_bridge.yaml",
            ]
        ),
        description="Path to the parameter_bridge configuration file.",
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value=[],
        description=(
            "The list of the robots spawned in the simulation e. g. robots:='robot1={x: 1.0, y:"
            " -2.0}; robot2={x: 1.0, y: -4.0}'."
        ),
    )

    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="5.0", description="Initial robot position in the global 'x' axis."
    )

    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="-5.0", description="Initial robot position in the global 'y' axis."
    )

    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.2", description="Initial robot position in the global 'z' axis."
    )

    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial robot 'roll' orientation."
    )

    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial robot 'pitch' orientation."
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial robot 'yaw' orientation."
    )

    robots_list = ParseMultiRobotPose("robots").value()
    if len(robots_list) == 0:
        robots_list = {
            namespace: {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
        }
    else:
        for robot_name, init_pose in robots_list.items():
            robots_list[robot_name] = {k: str(v) for k, v in init_pose.items()}

    spawn_group = []
    for idx, robot_name in enumerate(robots_list):
        init_pose = robots_list[robot_name]
        x, y, z, roll, pitch, yaw = [value for value in init_pose.values()]

        spawn_log = LogInfo(msg=[f"Launching namespace={robot_name} with init_pose={init_pose}"])

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                robot_name,
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
            namespace=robot_name,
            output="screen",
        )

        gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="panther_base_gz_bridge",
            parameters=[{"config_file": gz_bridge_config_path}],
            namespace=robot_name,
            output="screen",
        )

        bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("panther_bringup"),
                        "launch",
                        "bringup.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "use_sim": "True",
                "namespace": robot_name,
            }.items(),
        )

        gz_components = IncludeLaunchDescription(
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
                "namespace": namespace,
            }.items(),
        )

        child_tf = PythonExpression(
            ["'", robot_name, "' + '/odom' if '", robot_name, "' else 'odom'"]
        )

        world_transform = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_publisher",
            namespace=robot_name,
            output="screen",
            arguments=[x, y, z, roll, pitch, yaw, "world", child_tf],
            condition=IfCondition(add_world_transform),
        )

        # bringup.launch.py has a timerAction in it. If the timerAction in simulation.launch.py
        # ​​is smaller than bringup.launch.py, the namespace will be overwritten,
        # resulting creating nodes with the same namespace.
        group = TimerAction(
            period=12.0 * idx,
            actions=[
                spawn_log,
                spawn_robot,
                gz_bridge,
                bringup_launch,
                world_transform,
                gz_components,
            ],
        )
        spawn_group.append(group)

    return LaunchDescription(
        [
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            declare_gz_bridge_config_path_arg,
            declare_namespace_arg,
            declare_robots_arg,
            declare_add_world_transform_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            *spawn_group,
        ]
    )
