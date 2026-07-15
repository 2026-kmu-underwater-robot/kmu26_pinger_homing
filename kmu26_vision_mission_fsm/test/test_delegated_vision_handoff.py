#!/usr/bin/env python3
"""Contract test for full-mission handoff to the dedicated vision controller."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time

os.environ.setdefault("ROS_DOMAIN_ID", "178")

import rclpy
from hit25_auv_ros2_msg.msg import BuoyObservation
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("delegated_vision_handoff_test_probe")
        self.vehicle_pub = self.create_publisher(State, "/test/handoff/mavros_state", 10)
        self.odom_pub = self.create_publisher(Odometry, "/test/handoff/odometry", 10)
        self.vision_state_pub = self.create_publisher(
            String, "/test/handoff/vision_state", 10
        )
        self.observation_pub = self.create_publisher(
            BuoyObservation, "/test/handoff/observation", 10
        )
        self.status: dict[str, object] = {}
        self.rc: OverrideRCIn | None = None
        self.vision_enabled: bool | None = None
        self.status_sub = self.create_subscription(
            String, "/test/handoff/status", self._on_status, 10
        )
        self.rc_sub = self.create_subscription(
            OverrideRCIn, "/test/handoff/mission_rc", self._on_rc, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, "/test/handoff/vision_enable", self._on_enable, 10
        )

    def _on_status(self, message: String) -> None:
        self.status = json.loads(message.data)

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.rc = message

    def _on_enable(self, message: Bool) -> None:
        self.vision_enabled = message.data

    def publish_inputs(self, vision_state: str, *, detected: bool = False) -> None:
        vehicle = State()
        vehicle.connected = True
        vehicle.armed = True
        vehicle.mode = "MANUAL"
        self.vehicle_pub.publish(vehicle)
        odometry = Odometry()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.z = -0.31
        odometry.pose.pose.orientation.w = math.cos(0.0)
        self.odom_pub.publish(odometry)
        state = String()
        state.data = vision_state
        self.vision_state_pub.publish(state)
        if detected:
            observation = BuoyObservation()
            observation.detected = True
            observation.class_id = 0
            observation.class_label = "buoy"
            observation.confidence = 0.95
            observation.bbox_center_x = 640.0
            observation.bbox_center_y = 360.0
            observation.bbox_width = 420.0
            observation.bbox_height = 420.0
            observation.image_width = 1280
            observation.image_height = 720
            observation.normalized_error_x = 0.0
            observation.normalized_error_y = 0.0
            self.observation_pub.publish(observation)


def wait_for(
    probe: Probe, predicate, timeout: float, vision_state: str, *, detected: bool = False
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.publish_inputs(vision_state, detected=detected)
        rclpy.spin_once(probe, timeout_sec=0.02)
        if predicate():
            return
    raise RuntimeError(
        f"delegated handoff timeout state={vision_state}: "
        f"status={probe.status}, enabled={probe.vision_enabled}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    args = parser.parse_args()
    command = [
        args.controller,
        "--ros-args",
        "-p", "odom_topic:=/test/handoff/odometry",
        "-p", "state_topic:=/test/handoff/mavros_state",
        "-p", "observation_topic:=/test/handoff/observation",
        "-p", "output_topic:=/test/handoff/mission_rc",
        "-p", "status_topic:=/test/handoff/status",
        "-p", "vision_enable_topic:=/test/handoff/vision_enable",
        "-p", "vision_state_topic:=/test/handoff/vision_state",
        "-p", "enabled:=true",
        "-p", "dry_run:=true",
        "-p", "require_armed:=true",
        "-p", "delegate_vision_control:=true",
        "-p", "vision_state_timeout_s:=0.5",
        "-p", "vision_search_handoff_s:=0.15",
        "-p", "observation_timeout_s:=0.10",
        "-p", "use_pinger_first:=false",
        "-p", "start_surface_phase:=false",
        "-p", "vision_complete_requires_detach_count:=true",
        "-p", "expected_detach_count:=1",
        "-p", "own_course:=all",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    rclpy.init()
    probe = Probe()
    try:
        wait_for(
            probe,
            lambda: probe.status.get("internal_state") == "BUOY_SEARCHING"
            and probe.vision_enabled is False,
            2.0,
            "SEARCH",
        )
        wait_for(
            probe,
            lambda: probe.status.get("internal_state") == "VISION_CONTROL"
            and probe.status.get("vision_state") == "SEARCH"
            and probe.vision_enabled is True,
            2.0,
            "SEARCH",
            detected=True,
        )
        if probe.rc is None or any(
            value != OverrideRCIn.CHAN_RELEASE for value in probe.rc.channels
        ):
            raise RuntimeError("full mission did not release RC during delegated control")
        if probe.status.get("state") != "BUOY_SEARCH":
            raise RuntimeError(f"unexpected public mission state: {probe.status}")

        wait_for(
            probe,
            lambda: probe.status.get("internal_state") == "BUOY_SEARCHING"
            and probe.vision_enabled is False,
            2.0,
            "SEARCH",
        )

        wait_for(
            probe,
            lambda: probe.status.get("internal_state") == "VISION_CONTROL"
            and probe.vision_enabled is True,
            2.0,
            "SEARCH",
            detected=True,
        )
        wait_for(
            probe,
            lambda: probe.status.get("internal_state") == "FAILED"
            and "before required detach count" in str(probe.status.get("failure", ""))
            and probe.vision_enabled is False,
            2.0,
            "ASCEND",
            detected=True,
        )
        print("delegated_vision_handoff=PASS")
        return 0
    finally:
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


if __name__ == "__main__":
    raise SystemExit(main())
