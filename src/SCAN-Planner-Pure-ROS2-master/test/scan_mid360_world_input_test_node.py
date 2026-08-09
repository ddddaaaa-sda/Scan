#!/usr/bin/env python3

"""Synthetic ROS 2 integration verifier for SCAN MID-360 world inputs."""

import math
import struct
import sys
import time
from typing import Callable, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray


Point3 = Tuple[float, float, float]


class VerificationError(RuntimeError):
    """Raised when an expected ROS interface behavior is not observed."""


def make_odometry(node: Node, frame_id: str) -> Odometry:
    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.child_frame_id = "robot_0"
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 1.0
    msg.pose.pose.orientation.w = 1.0
    return msg


def make_path(node: Node, frame_id: str) -> Path:
    msg = Path()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    for x, y, z in ((0.0, 0.0, 1.0), (1.5, 0.0, 1.1), (3.0, 0.0, 1.2)):
        stamped = PoseStamped()
        stamped.header = msg.header
        stamped.pose.position.x = x
        stamped.pose.position.y = y
        stamped.pose.position.z = z
        stamped.pose.orientation.w = 1.0
        msg.poses.append(stamped)
    return msg


def make_cloud(node: Node, frame_id: str) -> PointCloud2:
    points: Sequence[Point3] = (
        (1.0, 2.0, 1.0),       # kept
        (1.01, 2.01, 1.01),    # same 0.08 m voxel as the first point
        (-2.0, 1.0, 0.6),      # kept
        (0.1, 0.1, 1.0),       # robot self-filter
        (5.0, 0.0, 1.0),       # outside local x range
        (0.0, 0.0, 2.6),       # above local z range
        (0.0, 0.0, -0.1),      # below local z range
        (math.nan, 0.0, 1.0),  # non-finite
    )

    msg = PointCloud2()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = b"".join(struct.pack("<fff", *point) for point in points)
    return msg


def read_xyz(msg: PointCloud2) -> List[Point3]:
    offsets = {field.name: field.offset for field in msg.fields}
    missing = {"x", "y", "z"}.difference(offsets)
    if missing:
        raise VerificationError(f"PointCloud2 is missing fields: {sorted(missing)}")

    endian = ">" if msg.is_bigendian else "<"
    data = bytes(msg.data)
    points: List[Point3] = []
    for row in range(msg.height):
        row_start = row * msg.row_step
        for column in range(msg.width):
            base = row_start + column * msg.point_step
            points.append(
                tuple(
                    struct.unpack_from(endian + "f", data, base + offsets[name])[0]
                    for name in ("x", "y", "z")
                )
            )
    return points


class WorldInputVerifier(Node):
    """Publishes a fixed scenario and verifies planner visualization outputs."""

    def __init__(self) -> None:
        super().__init__("scan_mid360_world_input_test")

        self.odom_pub = self.create_publisher(
            Odometry, "/robot_0/odometry", qos_profile_sensor_data
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/robot_0/cloud_registered_world", qos_profile_sensor_data
        )
        self.path_pub = self.create_publisher(Path, "/scan_global_path", 10)
        self.trigger_pub = self.create_publisher(Bool, "/trigger_plan", 10)

        self.visual_obstacles: Optional[PointCloud2] = None
        self.visual_global_path: Optional[Path] = None
        self.visual_local_trajectory: Optional[Path] = None
        self.inflated_cloud: Optional[PointCloud2] = None
        self.trajectories: Optional[MarkerArray] = None

        self.create_subscription(
            PointCloud2,
            "/visual_obstacles",
            lambda msg: setattr(self, "visual_obstacles", msg),
            10,
        )
        self.create_subscription(
            Path,
            "/visual_global_path",
            lambda msg: setattr(self, "visual_global_path", msg),
            10,
        )
        self.create_subscription(
            Path,
            "/visual_local_trajectory",
            lambda msg: setattr(self, "visual_local_trajectory", msg),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/inflated_cloud",
            lambda msg: setattr(self, "inflated_cloud", msg),
            10,
        )
        self.create_subscription(
            MarkerArray,
            "/trajectories",
            lambda msg: setattr(self, "trajectories", msg),
            10,
        )

    def _spin_until(
        self,
        predicate: Callable[[], bool],
        timeout_sec: float,
        description: str,
        periodic: Optional[Callable[[], None]] = None,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        next_periodic = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if periodic is not None and now >= next_periodic:
                periodic()
                next_periodic = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise VerificationError(f"Timed out waiting for {description}")

    def _publish_inputs(self, frame_id: str) -> None:
        self.odom_pub.publish(make_odometry(self, frame_id))
        self.cloud_pub.publish(make_cloud(self, frame_id))
        self.path_pub.publish(make_path(self, frame_id))

    def _spin_for(
        self,
        duration_sec: float,
        periodic: Optional[Callable[[], None]] = None,
    ) -> None:
        deadline = time.monotonic() + duration_sec
        next_periodic = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if periodic is not None and now >= next_periodic:
                periodic()
                next_periodic = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.05)

    def _interfaces_ready(self) -> bool:
        publishers = (
            "/visual_obstacles",
            "/visual_global_path",
            "/visual_local_trajectory",
            "/inflated_cloud",
        )
        return (
            self.odom_pub.get_subscription_count() > 0
            and self.cloud_pub.get_subscription_count() > 0
            and self.path_pub.get_subscription_count() > 0
            and self.trigger_pub.get_subscription_count() > 0
            and all(self.count_publishers(topic) > 0 for topic in publishers)
        )

    @staticmethod
    def _assert_frame(frame_id: str, label: str) -> None:
        if frame_id != "world":
            raise VerificationError(
                f"{label} frame_id is {frame_id!r}, expected 'world'"
            )

    def run(self, timeout_sec: float = 45.0) -> None:
        """Publish the synthetic scenario and validate all expected outputs."""
        started = time.monotonic()
        self.get_logger().info("Waiting for SCAN-Planner interfaces")
        self._spin_until(self._interfaces_ready, 10.0, "planner topic discovery")

        # Wrong-frame messages must not populate the planner's input state.
        self.get_logger().info("Checking rejection of non-world input frames")
        self._spin_until(
            lambda: self.visual_obstacles is not None
            and self.visual_global_path is not None,
            3.0,
            "initial empty visualization messages",
            periodic=lambda: self._publish_inputs("map"),
        )
        if read_xyz(self.visual_obstacles):
            raise VerificationError("Non-world cloud was accepted unexpectedly")
        if self.visual_global_path.poses:
            raise VerificationError("Non-world global path was accepted unexpectedly")

        # Odometry is sent first because cloud filtering is centered on the current pose.
        self.get_logger().info("Publishing synthetic world odometry")
        self._spin_for(
            0.5,
            periodic=lambda: self.odom_pub.publish(make_odometry(self, "world")),
        )

        remaining = max(5.0, timeout_sec - (time.monotonic() - started))
        self.get_logger().info("Publishing filtered cloud and 3D global path")
        self._spin_until(
            self._valid_visual_inputs_received,
            min(10.0, remaining),
            "filtered obstacle and global path visualization",
            periodic=lambda: self._publish_inputs("world"),
        )
        self._validate_visual_inputs()

        self.get_logger().info("Triggering planning")
        self.trigger_pub.publish(Bool(data=True))

        remaining = max(5.0, timeout_sec - (time.monotonic() - started))
        self._spin_until(
            self._planning_outputs_received,
            remaining,
            "non-empty local trajectory and inflated cloud",
        )
        self._validate_planning_outputs()

        marker_count = (
            len(self.trajectories.markers) if self.trajectories is not None else 0
        )
        self.get_logger().info(
            "PASS: world input, XYZ filtering, path Z, local planning and inflation; "
            f"A* markers observed={marker_count}"
        )

    def _valid_visual_inputs_received(self) -> bool:
        if self.visual_obstacles is None or self.visual_global_path is None:
            return False
        return (
            self.visual_obstacles.header.frame_id == "world"
            and len(read_xyz(self.visual_obstacles)) == 2
            and self.visual_global_path.header.frame_id == "world"
            and len(self.visual_global_path.poses) == 3
        )

    def _validate_visual_inputs(self) -> None:
        assert self.visual_obstacles is not None
        assert self.visual_global_path is not None
        self._assert_frame(self.visual_obstacles.header.frame_id, "visual_obstacles")
        self._assert_frame(self.visual_global_path.header.frame_id, "visual_global_path")

        actual_points = read_xyz(self.visual_obstacles)
        expected_points: Iterable[Point3] = ((1.0, 2.0, 1.0), (-2.0, 1.0, 0.6))
        if len(actual_points) != 2:
            raise VerificationError(
                f"Expected 2 filtered obstacle points, got {len(actual_points)}"
            )
        for expected in expected_points:
            if not any(
                all(math.isclose(a, e, abs_tol=1e-4) for a, e in zip(actual, expected))
                for actual in actual_points
            ):
                raise VerificationError(
                    f"Expected obstacle {expected}, got {actual_points}"
                )

        actual_z = [pose.pose.position.z for pose in self.visual_global_path.poses]
        expected_z = [1.0, 1.1, 1.2]
        if len(actual_z) != len(expected_z) or not all(
            math.isclose(actual, expected, abs_tol=1e-5)
            for actual, expected in zip(actual_z, expected_z)
        ):
            raise VerificationError(
                f"Global path Z values changed: expected {expected_z}, got {actual_z}"
            )

    def _planning_outputs_received(self) -> bool:
        return (
            self.visual_local_trajectory is not None
            and len(self.visual_local_trajectory.poses) > 0
            and self.inflated_cloud is not None
            and self.inflated_cloud.width * self.inflated_cloud.height > 0
        )

    def _validate_planning_outputs(self) -> None:
        assert self.visual_local_trajectory is not None
        assert self.inflated_cloud is not None
        self._assert_frame(
            self.visual_local_trajectory.header.frame_id,
            "visual_local_trajectory",
        )
        self._assert_frame(self.inflated_cloud.header.frame_id, "inflated_cloud")

        trajectory_z = [
            pose.pose.position.z for pose in self.visual_local_trajectory.poses
        ]
        if not trajectory_z or not all(math.isfinite(z) for z in trajectory_z):
            raise VerificationError("Local trajectory contains no finite Z values")
        if not any(abs(z) > 0.5 for z in trajectory_z):
            raise VerificationError(
                f"Local trajectory lost its world height: Z sample={trajectory_z[:5]}"
            )
        if not read_xyz(self.inflated_cloud):
            raise VerificationError("Inflated cloud message contains no points")


def main() -> int:
    rclpy.init()
    verifier = WorldInputVerifier()
    try:
        verifier.run()
    except Exception as error:  # The executable must return a useful failure status.
        verifier.get_logger().error(f"FAIL: {error}")
        return_code = 1
    else:
        return_code = 0
    finally:
        verifier.destroy_node()
        rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
