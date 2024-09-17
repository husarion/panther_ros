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

from typing import List, Text, Tuple

import yaml
from launch.action import Action
from launch.actions import DeclareLaunchArgument
from launch.frontend import Entity, Parser
from launch.launch_context import LaunchContext
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

# Define valid configurations for each robot model
VALID_CONFIGURATIONS = {
    "panther": {"wheel_type": ["WH01", "WH02", "WH04"]},
    "lynx": {"wheel_type": ["WH05"]},
}


class DeclareRobotArgs(Action):
    """Retrieves and validate the robot configuration from the YAML data."""

    def __init__(self, path: LaunchConfiguration, **kwargs) -> None:
        """Create a DeclareRobotArgs action."""
        super().__init__(**kwargs)
        self.__path = normalize_to_list_of_substitutions(path)

    @classmethod
    def parse(cls, entity: Entity, parser: "Parser"):
        """Parse `arg` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs["name"] = parser.escape_characters(entity.get_attr("name"))
        default_value = entity.get_attr("default", optional=True)
        if default_value is not None:
            kwargs["default_value"] = parser.parse_substitution(default_value)
        description = entity.get_attr("description", optional=True)
        if description is not None:
            kwargs["description"] = parser.escape_characters(description)
        choices = entity.get_attr("choice", data_type=List[Entity], optional=True)
        if choices is not None:
            kwargs["choices"] = [
                parser.escape_characters(choice.get_attr("value")) for choice in choices
            ]
        return cls, kwargs

    @property
    def path(self) -> Text:
        """Getter for self.__path."""
        return self.__path

    def execute(self, context: LaunchContext):
        """Execute the action."""
        path = perform_substitutions(context, self.path)
        yaml_data = self.load_yaml_file(path)
        yaml_data = self.normalize_robot_configuration(yaml_data)
        namespace, robot_config = self.extract_single_robot_configuration(yaml_data)
        list_of_args = self.create_launch_arguments(namespace, robot_config)
        for arg in list_of_args:
            arg.execute(context)

    def load_yaml_file(self, path: str) -> dict:
        """Load YAML file and return its contents."""
        try:
            with open(path, "r") as file:
                return yaml.safe_load(file)
        except yaml.YAMLError as exc:
            raise ValueError(f"Error reading YAML file: {exc}") from exc

    def validate_robot_configuration(self, yaml_data: dict) -> None:
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

    def normalize_robot_configuration(self, yaml_data: dict) -> dict:
        """Normalizes the YAML dictionary structure to a flat to nested format and validates the structure."""

        obligatory_keys = {"configuration"}

        # Checking and normalize flat structure
        if obligatory_keys.issubset(yaml_data.keys()):
            self.validate_robot_configuration(yaml_data)
            return {"": yaml_data}

        # Checking nested structure
        if all(isinstance(value, dict) for value in yaml_data.values()):
            nested_data = {}
            for namespace, robot_config in yaml_data.items():
                if not obligatory_keys.issubset(robot_config.keys()):
                    raise ValueError(
                        f"Invalid nested YAML structure at '{namespace}': Missing required keys."
                    )
                self.validate_robot_configuration(robot_config)
                nested_data[namespace] = robot_config
            return nested_data

        raise ValueError("Invalid YAML structure: The data does not match expected formats.")

    def extract_single_robot_configuration(
        self, yaml_data: dict, idx: int = 0
    ) -> Tuple[str, dict]:
        """Extracts the namespace and configuration based on the provided index from a YAML dictionary."""
        keys = list(yaml_data.keys())
        namespace = keys[idx]
        configuration = yaml_data[namespace]
        return namespace, configuration

    def create_launch_arguments(
        self, namespace: str, robot_config: dict
    ) -> list[DeclareLaunchArgument]:
        """Declare launch arguments based on the YAML configuration files."""
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

        return [
            DeclareLaunchArgument(
                "namespace",
                default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=namespace),
                description="Add namespace to all launched nodes.",
            ),
            DeclareLaunchArgument(
                "robot_model",
                default_value=EnvironmentVariable("ROBOT_MODEL", default_value=robot_model),
                description="Specify robot model type.",
            ),
            DeclareLaunchArgument(
                "wheel_type",
                default_value=wheel_type,
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
