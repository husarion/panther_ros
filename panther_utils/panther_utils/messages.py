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

import textwrap
from typing import Dict

import click
from launch.actions import LogInfo
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import Command, PythonExpression

# Correct ASCII art strings
LYNX_ASCII = r"""
     _
    | |   _   _ _ __ __  __
    | |  | | | | '_ \\ \/ /
    | |__| |_| | | | |>  <
    |_____\__, |_| |_/_/\_\
          |___/

    """  # noqa: W605

PANTHER_ASCII = r"""
     ____             _   _
    |  _ \ __ _ _ __ | |_| |__   ___ _ __
    | |_) / _` | '_ \| __| '_ \ / _ \ '__|
    |  __/ (_| | | | | |_| | | |  __/ |
    |_|   \__,_|_| |_|\__|_| |_|\___|_|

    """  # noqa: W605

# Ensure the text substitution correctly wraps the ASCII art
LYNX_TEXT = click.style(textwrap.dedent(LYNX_ASCII), bold=True)
PANTHER_TEXT = click.style(textwrap.dedent(PANTHER_ASCII), bold=True)


def flatten(lst):
    """Flatten a nested list into a single list."""
    if isinstance(lst, list):
        flat_list = []
        for element in lst:
            flat_list.extend(flatten(element))
        return flat_list
    else:
        return [lst]


def welcome_msg(
    robot_model: SomeSubstitutionsType,
    serial_number: SomeSubstitutionsType,
    robot_hw_version: SomeSubstitutionsType,
    additional_stats: Dict = {},
):
    """Generate a welcome message with robot information and stats."""
    pkg_version = Command(command="ros2 pkg xml -t version panther")

    # Properly format the PythonExpression to avoid syntax errors
    robot_model_expr = PythonExpression(
        [f"r'''{LYNX_TEXT}''' if '", robot_model, f"' == 'lynx' else r'''{PANTHER_TEXT}'''"]
    )

    # Prepare stats to display in the message
    stats_to_show = {
        "Serial Number": serial_number,
        "Robot Version": robot_hw_version,
        "ROS Driver Version": pkg_version,
        **additional_stats,
        "Website": "https://husarion.com",
        "Support": "https://community.husarion.com/",
        "Bug Tracker": "https://github.com/husarion/panther_ros/issues",
    }

    # Flatten stats into a message format
    nested_list_of_stats = [
        item
        for name, value in stats_to_show.items()
        for item in (f"{click.style(name, bold=True)}: ", value, "\n")
    ]
    stats_msg = flatten(nested_list_of_stats)

    # Insert robot model ASCII art at the beginning of the message
    stats_msg.insert(0, robot_model_expr)

    return LogInfo(msg=stats_msg)
