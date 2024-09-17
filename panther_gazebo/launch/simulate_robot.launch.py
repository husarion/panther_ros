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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString
from panther_utils.arguments import DeclareRobotArgs


def generate_launch_description():

    add_world_transform = LaunchConfiguration("add_world_transform")
    components_config_path = LaunchConfiguration("components_config_path")
    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    namespace = LaunchConfiguration("namespace")
    robot_configuration = LaunchConfiguration("robot_configuration")
    robot_model = LaunchConfiguration("robot_model")
    use_ekf = LaunchConfiguration("use_ekf")
    wheel_type = LaunchConfiguration("wheel_type")

    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
        choices=["True", "true", "False", "false"],
    )

    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "battery_plugin_config.yaml"]
        ),
    )

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

    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "gz_bridge.yaml"]
        ),
        description="Path to the parameter_bridge configuration file.",
    )

    declare_robot_configuration_arg = DeclareLaunchArgument(
        "robot_configuration",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "configuration.yaml"]
        ),
        description="Path to robot configuration YAML file.",
    )

    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF.",
        choices=["True", "true", "False", "false"],
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_gazebo"), "launch", "spawn_robot.launch.py"]
            )
        ),
        launch_arguments={
            "add_wheel_joints": "False",
            "namespace": namespace,
            "robot_model": robot_model,
            "use_sim": "True",
            "wheel_type": wheel_type,
        }.items(),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_manager"), "launch", "manager.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    gz_led_strip_config = PathJoinSubstitution(
        [FindPackageShare("panther_gazebo"), "config", "led_strips.yaml"]
    )

    gz_led_strip_config = ReplaceString(
        source_file=gz_led_strip_config,
        replacements={"parent_link: panther": ["parent_link: ", namespace]},
        condition=UnlessCondition(PythonExpression(["'", namespace, "' == ''"])),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "publish_robot_state": "False",
            "use_sim": "True",
            "wheel_type": wheel_type,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_localization"),
                    "launch",
                    "localization.launch.py",
                ]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
        condition=IfCondition(use_ekf),
    )

    simulate_components = IncludeLaunchDescription(
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
            "components_config_path": components_config_path,
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
    )

    model_name = PythonExpression(["'", namespace, "' if '", namespace, "' else 'panther'"])

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={"<model_name>": model_name, "<namespace>": namespace, "//": "/"},
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="panther_base_gz_bridge",
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=namespace,
        emulate_tty=True,
    )

    child_tf = PythonExpression(["'", namespace, "' + '/odom' if '", namespace, "' else 'odom'"])

    world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        arguments=[
            LaunchConfiguration("x"),
            LaunchConfiguration("y"),
            LaunchConfiguration("z"),
            LaunchConfiguration("roll"),
            LaunchConfiguration("pitch"),
            LaunchConfiguration("yaw"),
            "world",
            child_tf,
        ],
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(add_world_transform),
    )

    return LaunchDescription(
        [
            declare_robot_configuration_arg,
            DeclareRobotArgs(robot_configuration),
            declare_add_world_transform_arg,
            declare_battery_config_path_arg,
            declare_components_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_use_ekf_arg,
            SetUseSimTime(True),
            spawn_robot_launch,
            lights_launch,
            manager_launch,
            controller_launch,
            ekf_launch,
            simulate_components,
            gz_bridge,
            world_transform,
        ]
    )
