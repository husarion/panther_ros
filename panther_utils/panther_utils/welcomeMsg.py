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
from launch.substitutions import Command


def flatten(lst):
    if isinstance(lst, list):
        flat_list = []
        for element in lst:
            flat_list.extend(flatten(element))
        return flat_list
    else:
        return [lst]


def welcomeMsg(
    serial_number: SomeSubstitutionsType,
    robot_hw_version: SomeSubstitutionsType,
    additional_stats: Dict = {},
):
    pkg_version = Command(command="ros2 pkg xml -t version panther")

    PANTHER_TEXT = """
     _                     
    | |   _   _ _ __ __  __
    | |  | | | | '_ \\\ \/ /
    | |__| |_| | | | |>  < 
    |_____\__, |_| |_/_/\_\\
          |___/              

    """  # noqa: W605
    pth_txt = textwrap.dedent(PANTHER_TEXT)
    pth_txt = click.style(pth_txt, bold=True)

    stats_to_show = {
        "Serial Number": serial_number,
        "Robot Version": robot_hw_version,
        "ROS Driver Version": pkg_version,
        **additional_stats,
        "Website": "https://husarion.com",
        "Support": "https://community.husarion.com/",
        "Bug Tracker": "https://github.com/husarion/panther_ros/issues",
    }

    nested_list_of_stats = [
        item
        for name, value in stats_to_show.items()
        for item in (f"{click.style(name, bold=True)}: ", value, "\n")
    ]
    stats_msg = flatten(nested_list_of_stats)

    stats_msg.insert(0, pth_txt)

    return LogInfo(msg=stats_msg)
