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
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ParseMultiRobotPose


def launch_setup(context):
    wheel_type = LaunchConfiguration("wheel_type").perform(context)
    wheel_config_path = LaunchConfiguration("wheel_config_path").perform(context)
    controller_config_path = LaunchConfiguration("controller_config_path").perform(context)
    battery_config_path = LaunchConfiguration("battery_config_path").perform(context)
    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path").perform(context)
    world_cfg = LaunchConfiguration("world").perform(context)
    x = LaunchConfiguration("x").perform(context)
    y = LaunchConfiguration("y").perform(context)
    z = LaunchConfiguration("z").perform(context)
    roll = LaunchConfiguration("roll").perform(context)
    pitch = LaunchConfiguration("pitch").perform(context)
    yaw = LaunchConfiguration("yaw").perform(context)
    publish_robot_state = LaunchConfiguration("publish_robot_state").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)

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

    robots_list = ParseMultiRobotPose("robots").value()
    if len(robots_list) == 0:
        robots_list = {
            namespace: {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}
        }

    spawn_group = []
    for idx, robot_name in enumerate(robots_list):
        init_pose = robots_list[robot_name]

        spawn_log = LogInfo(
            msg=[f"Launching namespace={robot_name} with init_pose= {str(init_pose)}"]
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                robot_name,
                "-topic",
                "robot_description",
                "-x",
                TextSubstitution(text=str(init_pose["x"])),
                "-y",
                TextSubstitution(text=str(init_pose["y"])),
                "-z",
                TextSubstitution(text=str(init_pose["z"])),
                "-Y",
                TextSubstitution(text=str(init_pose["yaw"])),
            ],
            namespace=robot_name,
            output="screen",
        )

        gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
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
                "wheel_type": wheel_type,
                "wheel_config_path": wheel_config_path,
                "controller_config_path": controller_config_path,
                "battery_config_path": battery_config_path,
                "publish_robot_state": publish_robot_state,
                "use_sim": "True",
                "simulation_engine": "ignition-gazebo",
                "namespace": robot_name,
            }.items(),
        )

        group = TimerAction(
            period=10.0 * idx,
            actions=[
                spawn_log,
                spawn_robot,
                gz_bridge,
                bringup_launch,
            ],
        )
        spawn_group.append(group)

    return [gz_sim, *spawn_group]


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

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=[
            "-r ",
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_office_gz"),
                    "worlds",
                    "husarion_world.sdf",
                ],
            ),
        ],
        description="SDF world file",
    )

    declare_x_arg = DeclareLaunchArgument(
        "x",
        default_value=["5.0"],
        description="Initial robot position in the global 'x' axis.",
    )

    declare_y_arg = DeclareLaunchArgument(
        "y",
        default_value=["-5.0"],
        description="Initial robot position in the global 'y' axis.",
    )

    declare_z_arg = DeclareLaunchArgument(
        "z",
        default_value=["0.2"],
        description="Initial robot position in the global 'z' axis.",
    )

    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value=["0.0"], description="Initial robot 'roll' orientation."
    )

    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value=["0.0"], description="Initial robot orientation."
    )

    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value=["0.0"], description="Initial robot orientation."
    )

    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="True",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace for all Panther topics",
    )

    declare_robots_arg = DeclareLaunchArgument(
        "robots",
        default_value=[],
        description=(
            "The list of the robots spawned in the simulation e. g. robots:='robot1={x: 0.0, y:"
            " -1.0}; robot2={x: 1.0, y: -1.0}'"
        ),
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_x_arg,
            declare_y_arg,
            declare_z_arg,
            declare_roll_arg,
            declare_pitch_arg,
            declare_yaw_arg,
            declare_wheel_type_arg,
            declare_wheel_config_path_arg,
            declare_controller_config_path_arg,
            declare_battery_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_publish_robot_state_arg,
            declare_namespace_arg,
            declare_robots_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            OpaqueFunction(function=launch_setup),
        ]
    )
