#!/usr/bin/env python3

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

import textwrap

import click
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace, SetParameter


def generate_launch_description():
    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")
    panther_serial_no = EnvironmentVariable(name="PANTHER_SERIAL_NO", default_value="----")
    panther_pkg_version = Command(command="ros2 pkg xml -t version panther")

    PANTHER_TEXT = """
     ____             _   _
    |  _ \ __ _ _ __ | |_| |__   ___ _ __
    | |_) / _` | '_ \| __| '_ \ / _ \ '__|
    |  __/ (_| | | | | |_| | | |  __/ |
    |_|   \__,_|_| |_|\__|_| |_|\___|_|

    """  # noqa: W605

    stats_to_show = {
        "Serial number": panther_serial_no,
        "Robot version": panther_version,
        "ROS driver version": panther_pkg_version,
        "Website": "https://husarion.com",
        "Support": "https://community.husarion.com/",
        "Bugtracker": "https://github.com/husarion/panther_ros/issues",
    }

    stats_msg = [
        item
        for name, value in stats_to_show.items()
        for item in (f"{click.style(name, bold=True)}: ", value, "\n")
    ]
    pth_txt = textwrap.dedent(PANTHER_TEXT)
    stats_msg.insert(0, click.style(pth_txt, bold=True))

    welcome_msg = LogInfo(msg=stats_msg)

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Panther robot namespace",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    wheel_type = LaunchConfiguration("wheel_type")
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

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_description"),
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
                get_package_share_directory("panther_controller"),
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
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value="",
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

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF",
    )

    ekf_config_path = LaunchConfiguration("ekf_config_path")
    declare_ekf_config_path_arg = DeclareLaunchArgument(
        "ekf_config_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory("panther_bringup"), "config", "ekf.yaml"]
        ),
        description="Path to the EKF config file",
        condition=IfCondition(use_ekf),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "panther_version": panther_version,
            "wheel_type": wheel_type,
            "wheel_config_path": wheel_config_path,
            "controller_config_path": controller_config_path,
            "battery_config_path": battery_config_path,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
            "publish_robot_state": publish_robot_state,
        }.items(),
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_bringup"),
                    "launch",
                    "imu.launch.py",
                ]
            )
        ),
        launch_arguments={
            "imu_config_path": PathJoinSubstitution(
                [get_package_share_directory("panther_bringup"), "config", "imu.yaml"]
            ),
        }.items(),
        condition=UnlessCondition(use_sim),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_lights"),
                    "launch",
                    "lights.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_sim),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_battery"),
                    "launch",
                    "battery.launch.py",
                ]
            )
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "panther_version": panther_version,
        }.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[ekf_config_path],
        condition=IfCondition(use_ekf),
    )

    other_action_timer = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            imu_launch,
            lights_launch,
            robot_localization_node,
        ],
    )

    waiting_msg = TimerAction(
        period=7.0,
        actions=[
            LogInfo(
                msg=(
                    "We're working on ensuring everything functions properly... Please wait a few"
                    " seconds more!"
                )
            )
        ],
    )

    actions = [
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_wheel_type_arg,
        declare_wheel_config_path_arg,
        declare_controller_config_path_arg,
        declare_battery_config_path_arg,
        declare_simulation_engine_arg,
        declare_publish_robot_state_arg,
        declare_use_ekf_arg,
        declare_ekf_config_path_arg,
        PushRosNamespace(namespace),
        SetParameter(name="use_sim_time", value=use_sim),
        welcome_msg,
        controller_launch,
        waiting_msg,
        other_action_timer,
    ]

    return LaunchDescription(actions)
