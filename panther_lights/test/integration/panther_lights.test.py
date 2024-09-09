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


import unittest

import launch
import launch_testing
import rclpy
import rclpy.qos
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_testing_ros import WaitForTopics
from panther_msgs.msg import LEDAnimation
from panther_msgs.srv import SetLEDAnimation
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


def generate_test_description():

    led_config_file = (
        PathJoinSubstitution([FindPackageShare("panther_lights"), "config", "led_config.yaml"]),
    )

    lights_controller_node = Node(
        package="panther_lights",
        executable="lights_controller_node",
        parameters=[{"led_config_file": led_config_file}],
    )

    lights_driver_node = Node(
        package="panther_lights",
        executable="lights_driver_node",
    )

    # Start test after 1s
    delay_timer = launch.actions.TimerAction(
        period=1.0, actions=[launch_testing.actions.ReadyToTest()]
    )

    actions = [lights_controller_node, lights_driver_node, delay_timer]

    context = {}

    return (
        LaunchDescription(actions),
        context,
    )


class TestNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")
        self.led_control_requested = None

        self.led_control_enable_srv = self.test_node.create_service(
            srv_type=SetBool,
            srv_name="hardware/led_control_enable",
            callback=self.led_control_enable_cb,
            qos_profile=rclpy.qos.qos_profile_services_default,
        )

        self.set_led_animation_client = self.test_node.create_client(
            srv_type=SetLEDAnimation,
            srv_name="lights/set_animation",
            qos_profile=rclpy.qos.qos_profile_services_default,
        )

    def tearDown(self):
        self.test_node.destroy_node()

    def led_control_enable_cb(self, request, response):
        self.led_control_requested = request.data
        response.success = True
        response.message = "LED control enabled"
        return response

    def test_initialization(self, proc_output):
        rclpy.spin_until_future_complete(self.test_node, rclpy.task.Future(), timeout_sec=2.0)

        self.assertTrue(self.led_control_requested)

        # Controller initialization
        proc_output.assertWaitFor("[lights_controller]: Loaded default animations.")
        proc_output.assertWaitFor("[lights_controller]: Initialized successfully.")
        # Driver initialization
        proc_output.assertWaitFor("[lights_driver]: Node constructed successfully.")
        proc_output.assertWaitFor("[lights_driver]: LED control granted.")

    def request_error_animation(self):
        request = SetLEDAnimation.Request()
        request.animation = LEDAnimation(id=LEDAnimation.ERROR)
        request.repeating = False

        self.set_led_animation_client.wait_for_service(timeout_sec=1.0)
        self.set_led_animation_client.call_async(request)

    def test_msg_publishers(self):
        self.request_error_animation()

        topic_list = [
            ("lights/channel_1_frame", Image),
            ("lights/channel_2_frame", Image),
            ("diagnostics", DiagnosticArray),
        ]

        with WaitForTopics(topic_list, timeout=5.0) as wait_for_topics:
            received_topics_str = ", ".join(wait_for_topics.topics_received())
            print("Received messages from the following topics: [" + received_topics_str + "]")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
