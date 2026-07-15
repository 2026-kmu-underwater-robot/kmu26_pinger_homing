#!/usr/bin/env python3
"""Runtime contract test for the imported zetex1001 visual controller."""

from __future__ import annotations

import argparse
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn, State
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("vision_mission_controller_test_probe")
        self.bbox_pub = self.create_publisher(Float32MultiArray, "/test/vision/bbox", 10)
        self.depth_pub = self.create_publisher(PoseWithCovarianceStamped, "/test/vision/depth", 10)
        self.enable_pub = self.create_publisher(Bool, "/test/vision/enable", 10)
        self.vehicle_pub = self.create_publisher(State, "/test/vision/mavros_state", 10)
        self.state = ""
        self.rc: OverrideRCIn | None = None
        self.state_sub = self.create_subscription(
            String, "/test/vision/state", self._on_state, 10
        )
        self.rc_sub = self.create_subscription(
            OverrideRCIn, "/test/vision/rc", self._on_rc, 10
        )

    def _on_state(self, message: String) -> None:
        self.state = message.data

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.rc = message

    def publish_inputs(self, *, armed: bool, depth_m: float, enabled: bool = True) -> None:
        vehicle = State()
        vehicle.connected = True
        vehicle.armed = armed
        vehicle.mode = "MANUAL"
        self.vehicle_pub.publish(vehicle)
        depth = PoseWithCovarianceStamped()
        depth.pose.pose.position.z = depth_m
        self.depth_pub.publish(depth)
        enable = Bool()
        enable.data = enabled
        self.enable_pub.publish(enable)

    def publish_bbox(self, class_id: int, *, width: float, height: float) -> None:
        message = Float32MultiArray()
        message.data = [
            time.time(), 1.0, float(class_id), 0.95,
            320.0, 240.0, width, height, 640.0, 480.0,
        ]
        self.bbox_pub.publish(message)


def spin_for(probe: Probe, duration: float, callback) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        callback()
        rclpy.spin_once(probe, timeout_sec=0.02)


def wait_state(probe: Probe, expected: str, timeout: float, callback) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        callback()
        rclpy.spin_once(probe, timeout_sec=0.02)
        if probe.state == expected:
            return
    raise RuntimeError(f"timed out waiting for {expected}; latest={probe.state!r}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    os.environ.setdefault("ROS_DOMAIN_ID", "180" if args.dry_run else "179")
    command = [
        args.controller,
        "--ros-args",
        "-p", "bbox_topic:=/test/vision/bbox",
        "-p", "depth_topic:=",
        "-p", "depth_pose_topic:=/test/vision/depth",
        "-p", "depth_odom_topic:=",
        "-p", "enable_topic:=/test/vision/enable",
        "-p", "state_topic:=/test/vision/state",
        "-p", "status_topic:=/test/vision/status",
        "-p", "vehicle_state_topic:=/test/vision/mavros_state",
        "-p", "rc_override_topic:=/test/vision/rc",
        "-p", f"dry_run:={'true' if args.dry_run else 'false'}",
        "-p", "work_depth_m:=1.0",
        "-p", "surface_depth_m:=0.2",
        "-p", "max_depth_m:=2.0",
        "-p", "depth_stable_sec:=0.08",
        "-p", "detection_timeout_sec:=0.25",
        "-p", "min_detection_hits:=2",
        "-p", "approach_area_ratio:=0.05",
        "-p", "align_stable_sec:=0.08",
        "-p", "insert_duration_sec:=0.08",
        "-p", "detach_duration_sec:=0.08",
        "-p", "backoff_duration_sec:=0.08",
        "-p", "verify_clear_sec:=0.08",
        "-p", "verify_timeout_sec:=1.0",
        "-p", "expected_target_count:=1",
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    probe = Probe()
    try:
        spin_for(probe, 0.35, lambda: probe.publish_inputs(armed=False, depth_m=1.0))
        if probe.state != "IDLE":
            raise RuntimeError(f"unarmed controller left IDLE: {probe.state}")
        if probe.rc is None or any(
            probe.rc.channels[index] != OverrideRCIn.CHAN_RELEASE for index in (2, 3, 4)
        ):
            raise RuntimeError("unarmed controller did not release its controlled RC channels")

        base = lambda: probe.publish_inputs(armed=True, depth_m=1.0)
        wait_state(probe, "SEARCH", 1.5, base)
        if probe.rc is None:
            raise RuntimeError("controller did not publish RC in SEARCH")
        controlled = [probe.rc.channels[index] for index in (2, 3, 4)]
        if args.dry_run:
            if any(value != OverrideRCIn.CHAN_RELEASE for value in probe.rc.channels):
                raise RuntimeError(f"dry-run controller claimed RC: {probe.rc.channels}")
        elif all(
            value in (OverrideRCIn.CHAN_RELEASE, OverrideRCIn.CHAN_NOCHANGE)
            for value in controlled
        ):
            raise RuntimeError("active SEARCH controller did not claim controlled RC channels")

        def buoy_only() -> None:
            base()
            probe.publish_bbox(0, width=180.0, height=150.0)

        wait_state(probe, "APPROACH_BUOY", 1.0, buoy_only)

        def buoy_and_stick() -> None:
            base()
            probe.publish_bbox(0, width=180.0, height=150.0)
            probe.publish_bbox(1, width=70.0, height=190.0)

        wait_state(probe, "ALIGN_STICK", 1.0, buoy_and_stick)
        wait_state(probe, "INSERT_FORK", 1.0, buoy_and_stick)
        wait_state(probe, "VERIFY_RELEASE", 1.5, base)
        wait_state(probe, "ASCEND", 1.0, base)
        wait_state(
            probe,
            "COMPLETE",
            1.0,
            lambda: probe.publish_inputs(armed=True, depth_m=0.2),
        )
        if probe.rc is None:
            raise RuntimeError("controller never published RC")
        print("vision_mission_controller_runtime=PASS")
        return 0
    finally:
        stop = Bool()
        stop.data = False
        probe.enable_pub.publish(stop)
        rclpy.spin_once(probe, timeout_sec=0.05)
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        if process.returncode not in (0, -15):
            raise RuntimeError(f"controller exited {process.returncode}:\n{output}")
