#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.substitutions import (
    PythonExpression,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    wheel_type = LaunchConfiguration("wheel_type")
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description="Type of wheel. If you choose a value from the preset options "
        "('WH01', 'WH02', 'WH04'), you can ignore the 'wheel_config_path' and "
        "'controller_config_path' parameters. For custom wheels, please define these parameters "
        "to point to files that accurately describe the custom wheels.",
        choices=['WH01', 'WH02', 'WH04', 'CUSTOM'],
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
        description="Path to wheel configuration file. "
        "By default, it should be located in 'panther_description/config/<wheel_type arg>.yaml'. "
        "You can also specify the path to your custom wheel configuration file here. "
        "It can be located in any directory.",
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
        description="Path to wheel configuration file. By default, it should be located in "
        "'panther_controller/config/<wheel_type arg>_controller.yaml'. You can also specify the path "
        "to your custom controller configuration file here. It can be located in any directory.",
    )

    map_package = get_package_share_directory("husarion_office_gz")
    world_file = PathJoinSubstitution([map_package, "worlds", "husarion_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
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
            "5.0",
            "-y",
            "-5.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/model/panther/battery/panther_battery/state" + 
            "@sensor_msgs/msg/BatteryState" + 
            "[ignition.msgs.BatteryState",
        ],
        remappings=[
            ("model/panther/battery/panther_battery/state", "/battery"),
        ],
        output="screen",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "wheel_type": wheel_type,
            "wheel_config_path": wheel_config_path,
            "controller_config_path": controller_config_path,
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_wheel_type_arg,
            declare_wheel_config_path_arg,
            declare_controller_config_path_arg,
            LogInfo(msg=["Controller configuration file path set to: ", controller_config_path]),
            # # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            ign_bridge,
            gz_spawn_entity,
            bringup_launch,
        ]
    )
