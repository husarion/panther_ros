#!/usr/bin/env python3

# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import sys
from typing import Iterable, Tuple

import yaml
from launch import LaunchContext, Substitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Define valid configurations for each robot model
VALID_CONFIGURATIONS = {
    "panther": {"wheel_type": ["WH01", "WH02", "WH03"]},
    "lynx": {"wheel_type": ["WH04"]},
}


def resolve_path(path: str | Substitution) -> str:
    """Resolve a path that might be substitutions."""
    if isinstance(path, Substitution):
        return path.perform(LaunchContext())
    return path


def load_yaml_file(path: str) -> dict:
    """Load YAML file and return its contents."""
    try:
        with open(path, "r") as file:
            return yaml.safe_load(file)
    except yaml.YAMLError as exc:
        raise ValueError(f"Error reading YAML file: {exc}") from exc


def load_robot_configuration(yaml_data: dict) -> Tuple[str, dict]:
    """Retrieve first element data if exists; otherwise, return the base structure."""
    if "configuration" not in yaml_data:
        namespace = next(iter(yaml_data.keys()))
        return namespace, yaml_data[namespace]
    return "", yaml_data


def validate_configuration(yaml_data: dict) -> None:
    """Validate the robot model and wheel type configuration."""
    robot_model = yaml_data.get("robot_model", "")
    configuration_data = yaml_data.get("configuration", {})
    wheel_type = configuration_data.get("wheel_type", "")

    if robot_model not in VALID_CONFIGURATIONS:
        raise ValueError(
            f"Invalid robot model '{robot_model}'. "
            f"Valid models are: {', '.join(VALID_CONFIGURATIONS.keys())}"
        )

    valid_wheel_types = VALID_CONFIGURATIONS[robot_model]["wheel_type"]
    if wheel_type not in valid_wheel_types:
        raise ValueError(
            f"Invalid wheel type '{wheel_type}' for {robot_model}. "
            f"Valid wheel types are: {', '.join(valid_wheel_types)}"
        )


def create_launch_arguments(namespace: str, yaml_data: dict) -> Iterable[DeclareLaunchArgument]:
    """Generate ROS 2 launch description based on the YAML configuration."""
    x, y, z = yaml_data.get("initial_pose", [0.0, 0.0, 0.0])
    roll, pitch, yaw = yaml_data.get("initial_rotation", [0.0, 0.0, 0.0])
    configuration_data = yaml_data.get("configuration", {})

    return [
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=namespace),
            description="Add namespace to all launched nodes.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value=EnvironmentVariable(
                "ROBOT_MODEL", default_value=yaml_data.get("robot_model", "panther")
            ),
            description="Specify robot model type.",
        ),
        DeclareLaunchArgument(
            "wheel_type",
            default_value=configuration_data["wheel_type"],
            description=(
                "Specify the wheel type. If the selected wheel type is not 'custom', "
                "the 'wheel_config_path' and 'controller_config_path' arguments will be "
                "automatically adjusted and can be omitted."
            ),
        ),
        DeclareLaunchArgument(
            "x", default_value=x, description="Initial robot position in the global 'x' axis."
        ),
        DeclareLaunchArgument(
            "y", default_value=y, description="Initial robot position in the global 'y' axis."
        ),
        DeclareLaunchArgument(
            "z", default_value=z, description="Initial robot position in the global 'z' axis."
        ),
        DeclareLaunchArgument(
            "roll", default_value=roll, description="Initial robot 'roll' orientation."
        ),
        DeclareLaunchArgument(
            "pitch", default_value=pitch, description="Initial robot 'pitch' orientation."
        ),
        DeclareLaunchArgument(
            "yaw", default_value=yaw, description="Initial robot 'yaw' orientation."
        ),
    ]


def declare_robot_args(path: str | Substitution) -> Iterable[DeclareLaunchArgument]:
    """Declare launch arguments based on the YAML configuration files."""
    path = resolve_path(path)
    yaml_data = load_yaml_file(path)
    namespace, robot_config = load_robot_configuration(yaml_data)
    try:
        validate_configuration(robot_config)
    except ValueError as error:
        print(f"Validation Error: {error}")
        sys.exit(1)
    list_of_args = create_launch_arguments(namespace, robot_config)
    return list_of_args


path = PathJoinSubstitution([FindPackageShare("panther_bringup"), "config", "configuration.yaml"])
print(declare_robot_args(path))
