#!/usr/bin/env python3

"""launch_testing entry point for the SCAN MID-360 world-input scenario."""

import os
import sys
import unittest
from pathlib import Path

import launch
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from scan_mid360_world_input_test_node import WorldInputVerifier  # noqa: E402


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    config = (
        Path(get_package_share_directory("scan_planner"))
        / "config"
        / "scan_mid360.yaml"
    )
    planner = Node(
        package="scan_planner",
        executable="motion_plan",
        name="scan_planner_interactive_node",
        parameters=[str(config)],
        output="screen",
    )
    return launch.LaunchDescription(
        [planner, launch_testing.actions.ReadyToTest()]
    ), {"planner": planner}


class TestScanMid360WorldInput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_world_input_pipeline(self):
        verifier = WorldInputVerifier()
        try:
            verifier.run(timeout_sec=45.0)
        finally:
            verifier.destroy_node()
