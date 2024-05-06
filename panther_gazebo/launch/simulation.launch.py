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
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wheel_type = LaunchConfiguration("wheel_type")
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description=(
            "Specify the type of wheel. If you select a value from the provided options ('WH01',"
            " 'WH02', 'WH04'), you can disregard the 'wheel_config_path' and"
            " 'controller_config_path' parameters. If you have custom wheels, set this parameter"
            " to 'CUSTOM' and provide the necessary configurations."
        ),
        choices=["WH01", "WH02", "WH04", "CUSTOM"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
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

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_gazebo"),
                "config",
                "battery_plugin_config.yaml",
            ]
        ),
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_gazebo"),
                "config",
                "gz_bridge.yaml",
            ]
        ),
        description="Path to the parameter_bridge configuration file",
    )

    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=[
            "-r ",
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_gz_worlds"),
                    "worlds",
                    "husarion_world.sdf",
                ],
            ),
        ],
        description="SDF world file",
    )

    pose_x = LaunchConfiguration("pose_x")
    declare_pose_x_arg = DeclareLaunchArgument(
        "pose_x",
        default_value=["5.0"],
        description="Initial robot position in the global 'x' axis.",
    )

    pose_y = LaunchConfiguration("pose_y")
    declare_pose_y_arg = DeclareLaunchArgument(
        "pose_y",
        default_value=["-5.0"],
        description="Initial robot position in the global 'y' axis.",
    )

    pose_z = LaunchConfiguration("pose_z")
    declare_pose_z_arg = DeclareLaunchArgument(
        "pose_z",
        default_value=["0.2"],
        description="Initial robot position in the global 'z' axis.",
    )

    rot_yaw = LaunchConfiguration("rot_yaw")
    declare_rot_yaw_arg = DeclareLaunchArgument(
        "rot_yaw", default_value=["0.0"], description="Initial robot orientation."
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

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "panther",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            pose_x,
            "-y",
            pose_y,
            "-z",
            pose_z,
            "-Y",
            rot_yaw,
        ],
        output="screen",
        namespace=namespace,
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config_path}],
        namespace=namespace,
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
            "wheel_type": wheel_type,
            "wheel_config_path": wheel_config_path,
            "controller_config_path": controller_config_path,
            "battery_config_path": battery_config_path,
            "publish_robot_state": publish_robot_state,
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo",
            "namespace": namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_pose_x_arg,
            declare_pose_y_arg,
            declare_pose_z_arg,
            declare_rot_yaw_arg,
            declare_wheel_type_arg,
            declare_wheel_config_path_arg,
            declare_controller_config_path_arg,
            declare_battery_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_publish_robot_state_arg,
            declare_namespace_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            gz_spawn_entity,
            bringup_launch,
        ]
    )
