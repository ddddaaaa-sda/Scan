#!/usr/bin/env python3

"""Run SCAN-Planner with the synthetic MID-360 world-input verifier."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("scan_planner"))
    config = package_share / "config" / "scan_mid360.yaml"

    planner = Node(
        package="scan_planner",
        executable="motion_plan",
        name="scan_planner_interactive_node",
        parameters=[str(config)],
        output="screen",
    )
    verifier = Node(
        package="scan_planner",
        executable="scan_mid360_world_input_test",
        name="scan_mid360_world_input_test",
        output="screen",
    )

    stop_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=verifier,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="SCAN MID-360 world-input verification finished"
                    )
                )
            ],
        )
    )

    return LaunchDescription([planner, verifier, stop_when_done])
