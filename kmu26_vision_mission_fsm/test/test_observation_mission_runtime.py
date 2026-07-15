#!/usr/bin/env python3
"""End-to-end ROS graph test for the sensor-only mission FSM."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from hit25_auv_ros2_msg.msg import BuoyObservation, CollectorState
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64, String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("observation_mission_runtime_test")
        self.state_pub = self.create_publisher(State, "/test/mavros/state", 10)
        self.odom_pub = self.create_publisher(Odometry, "/test/odometry", 10)
        self.vision_pub = self.create_publisher(BuoyObservation, "/test/vision", 10)
        self.collector_pub = self.create_publisher(CollectorState, "/test/collector", 10)
        self.hydrophone_pub = self.create_publisher(Vector3Stamped, "/test/homing/direction", 10)
        self.iq_pub = self.create_publisher(Float64, "/test/homing/iq", 10)
        self.delta_range_pub = self.create_publisher(Float64, "/test/homing/delta_range", 10)
        self.latest_status: dict[str, object] = {}
        self.latest_mission_state = ""
        self.latest_rc: OverrideRCIn | None = None
        self.iq_magnitude = 0.30
        self.delta_range_m = -0.002
        self.hydrophone_direction = (0.96, 0.20, -0.18)
        self.robot_yaw_rad = 0.0
        self.create_subscription(String, "/test/mission/status", self._on_status, 10)
        self.create_subscription(OverrideRCIn, "/test/mission/rc", self._on_rc, 10)

    def _on_status(self, message: String) -> None:
        self.latest_status = json.loads(message.data)
        self.latest_mission_state = str(self.latest_status.get("state", ""))
        # Runtime-control assertions below intentionally keep exercising the
        # stable internal phases while the externally published state follows
        # the competition mission contract.
        self.latest_status["state"] = self.latest_status.get(
            "internal_state", self.latest_mission_state
        )

    def _on_rc(self, message: OverrideRCIn) -> None:
        self.latest_rc = message

    def publish_vehicle(self) -> None:
        state = State()
        state.connected = True
        state.armed = True
        state.mode = "MANUAL"
        self.state_pub.publish(state)
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.z = -0.31
        odom.pose.pose.orientation.z = math.sin(0.5 * self.robot_yaw_rad)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.robot_yaw_rad)
        self.odom_pub.publish(odom)

    def publish_observation(
        self,
        normalized_error_x: float = 0.0,
        normalized_error_y: float = 0.0,
        class_label: str = "buoy",
    ) -> None:
        observation = BuoyObservation()
        observation.detected = True
        observation.class_id = 1 if class_label.lower() == "stick" else 0
        observation.class_label = class_label
        observation.confidence = 0.95
        observation.bbox_center_x = 640.0
        observation.bbox_center_y = 360.0
        observation.bbox_width = 300.0
        observation.bbox_height = 350.0
        observation.image_width = 1280
        observation.image_height = 720
        observation.normalized_error_x = normalized_error_x
        observation.normalized_error_y = normalized_error_y
        self.vision_pub.publish(observation)

    def publish_hydrophone(self) -> None:
        direction = Vector3Stamped()
        direction.header.frame_id = "odom"
        direction.vector.x = self.hydrophone_direction[0]
        direction.vector.y = self.hydrophone_direction[1]
        direction.vector.z = self.hydrophone_direction[2]
        self.hydrophone_pub.publish(direction)
        iq = Float64()
        iq.data = self.iq_magnitude
        self.iq_pub.publish(iq)
        delta_range = Float64()
        delta_range.data = self.delta_range_m
        self.delta_range_pub.publish(delta_range)

    def publish_collector(self, state: str, target_id: str = "test_target") -> None:
        event = CollectorState()
        event.target_id = target_id
        event.state = state
        event.detached = state == "DETACHED"
        event.captured = state == "NETTED"
        event.netted = state == "NETTED"
        event.released = state == "RELEASED"
        event.collector_eq_active = state == "NETTED"
        event.capture_state = "NETTED" if state == "NETTED" else state
        self.collector_pub.publish(event)


def wait_state(probe: Probe, expected: str, timeout: float, *, vision: bool = False,
               vision_label: str = "buoy", hydrophone: bool = False,
               collector: str = "", collector_target: str = "test_target") -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.publish_vehicle()
        if vision:
            probe.publish_observation(class_label=vision_label)
        if hydrophone:
            probe.publish_hydrophone()
        if collector:
            probe.publish_collector(collector, collector_target)
        rclpy.spin_once(probe, timeout_sec=0.02)
        if probe.latest_status.get("state") == expected:
            return
        time.sleep(0.01)
    raise RuntimeError(f"state {expected} not reached; latest={probe.latest_status}")


def wait_head_on_command(
    probe: Probe,
    expected_state: str,
    forward_sign: int,
    timeout: float,
    *,
    vision_label: str = "stick",
    hydrophone: bool = False,
    collector: str = "",
    collector_target: str = "test_target",
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.publish_vehicle()
        probe.publish_observation(class_label=vision_label)
        if hydrophone:
            probe.publish_hydrophone()
        if collector:
            probe.publish_collector(collector, collector_target)
        rclpy.spin_once(probe, timeout_sec=0.02)
        command = probe.latest_status.get("command", {})
        if probe.latest_status.get("state") != expected_state or not isinstance(command, dict):
            time.sleep(0.01)
            continue
        forward = float(command.get("forward", 0.0))
        if forward_sign * forward <= 1.0e-4:
            time.sleep(0.01)
            continue
        if abs(float(command.get("sway", 0.0))) > 1.0e-6:
            raise RuntimeError(f"head-on rake motion used sway: {probe.latest_status}")
        if abs(float(command.get("pitch", 0.0))) > 1.0e-6:
            raise RuntimeError(f"head-on rake motion used pitch lift: {probe.latest_status}")
        if probe.latest_status.get("capture_heading_locked") is not True:
            raise RuntimeError(f"head-on rake motion did not lock heading/depth: {probe.latest_status}")
        if probe.latest_status.get("target_label") != vision_label:
            raise RuntimeError(f"FSM did not retain the selected stick target: {probe.latest_status}")
        return
    raise RuntimeError(
        f"head-on command state={expected_state} sign={forward_sign} not reached; "
        f"latest={probe.latest_status}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", required=True)
    parser.add_argument("--pinger-first", action="store_true")
    parser.add_argument("--pinger-probe-timeout", action="store_true")
    parser.add_argument("--pinger-range-regression", action="store_true")
    parser.add_argument("--pinger-visual-handoff", action="store_true")
    parser.add_argument("--pinger-drive-pause", action="store_true")
    parser.add_argument("--pinger-vertical-align", action="store_true")
    parser.add_argument("--pinger-spin-rehome", action="store_true")
    parser.add_argument("--pinger-heading-first", action="store_true")
    parser.add_argument("--boundary-rejection", action="store_true")
    parser.add_argument("--surface-boundary-rejection", action="store_true")
    parser.add_argument("--early-detach", action="store_true")
    parser.add_argument("--detached-rejection", action="store_true")
    parser.add_argument("--capture-retry", action="store_true")
    parser.add_argument("--stick-rake-flow", action="store_true")
    args = parser.parse_args()
    command = [
        args.controller,
        "--ros-args",
        "-p", "observation_topic:=/test/vision",
        "-p", "collector_topic:=/test/collector",
        "-p", "odom_topic:=/test/odometry",
        "-p", "state_topic:=/test/mavros/state",
        "-p", "output_topic:=/test/mission/rc",
        "-p", "status_topic:=/test/mission/status",
        "-p", "enabled:=true",
        "-p", "dry_run:=true",
        "-p", "delegate_vision_control:=false",
        "-p", "require_armed:=true",
        "-p", "score_zone_radius:=100.0",
        "-p", "capture_drive_s:=0.15",
        "-p", f"capture_alignment_hold_s:={'0.05' if args.stick_rake_flow else '0.35'}",
        "-p", "capture_backoff_s:=0.55",
        "-p", f"collector_timeout_s:={'0.25' if args.capture_retry else '2.0'}",
        "-p", f"use_pinger_first:={'true' if args.pinger_first else 'false'}",
        "-p", f"start_surface_phase:={'true' if args.surface_boundary_rejection else 'false'}",
        "-p", "hydrophone_direction_topic:=/test/homing/direction",
        "-p", "iq_magnitude_topic:=/test/homing/iq",
        "-p", "delta_range_topic:=/test/homing/delta_range",
        "-p", f"pinger_min_probe_s:={'0.05' if args.pinger_probe_timeout else '0.0'}",
        "-p", f"pinger_max_probe_s:={'0.6' if args.pinger_probe_timeout else '45.0'}",
        "-p", f"pinger_homing_leg_s:={'0.12' if args.pinger_probe_timeout else '10.0'}",
        "-p", f"pinger_homing_drive_s:={'0.20' if args.pinger_drive_pause else '0.0'}",
        "-p", f"pinger_homing_pause_s:={'0.25' if args.pinger_drive_pause else '0.0'}",
        "-p", f"pinger_forward_turn:={'0.0' if args.pinger_heading_first else '0.10'}",
        "-p", "pinger_heading_drive_tolerance_rad:=0.18",
        "-p", f"pinger_spin_rehome_yaw_rad:={'1.0' if args.pinger_spin_rehome else '5.75'}",
        "-p", "pinger_spin_rehome_max_translation_m:=0.50",
        "-p", f"pinger_spin_rehome_stop_s:={'0.25' if args.pinger_spin_rehome else '1.0'}",
        "-p", f"pinger_range_regression_margin_m:={'0.15' if args.pinger_range_regression else '0.40'}",
        "-p", f"pinger_range_regression_hold_s:={'0.10' if args.pinger_range_regression else '0.65'}",
        "-p", f"pinger_range_progress_grace_s:={'0.05' if args.pinger_range_regression else '0.80'}",
        "-p", "pinger_range_progress_check_s:=2.8",
        "-p", f"observation_timeout_s:={'0.10' if args.pinger_visual_handoff else '1.5'}",
        "-p", f"pinger_visual_reacquire_timeout_s:={'0.25' if args.pinger_visual_handoff else '1.0'}",
        "-p", f"require_pinger_position_fit:={'true' if args.pinger_probe_timeout else 'false'}",
        "-p", f"expected_detach_count:={'2' if args.detached_rejection else '1'}",
        "-p", "expected_net_count:=1",
        "-p", f"own_course:={'a' if (args.boundary_rejection or args.surface_boundary_rejection) else 'all'}",
    ]
    if args.pinger_vertical_align:
        command.extend([
            "-p", "pinger_depth_z:=-0.31",
            "-p", "pinger_transit_depth_z:=-0.31",
        ])
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    probe = Probe()
    try:
        if args.surface_boundary_rejection:
            wait_state(probe, "SURFACE_BUOY_SEARCHING", 4.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                probe.publish_vehicle()
                probe.publish_observation()
                rclpy.spin_once(probe, timeout_sec=0.03)
            if probe.latest_status.get("state") != "SURFACE_BUOY_SEARCHING":
                raise RuntimeError(
                    f"opponent-side surface target was accepted: {probe.latest_status}")
            if probe.latest_status.get("target_in_own_course") is not False:
                raise RuntimeError(
                    f"surface boundary decision was not reported: {probe.latest_status}")
            print("observation_surface_boundary_rejection=PASS")
            return 0
        if args.boundary_rejection:
            wait_state(probe, "BUOY_SEARCHING", 4.0)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                probe.publish_vehicle()
                probe.publish_observation()
                rclpy.spin_once(probe, timeout_sec=0.03)
            if probe.latest_status.get("state") != "BUOY_SEARCHING":
                raise RuntimeError(f"opponent-side target was accepted: {probe.latest_status}")
            if probe.latest_status.get("target_in_own_course") is not False:
                raise RuntimeError(f"boundary decision was not reported: {probe.latest_status}")
            print("observation_boundary_rejection=PASS")
            return 0
        if args.pinger_first:
            if args.pinger_vertical_align:
                probe.iq_magnitude = 0.10
                probe.hydrophone_direction = (0.45, 0.0, -0.89)
            wait_state(probe, "PINGER_SEARCH", 4.0)
            if args.stick_rake_flow:
                command_status = probe.latest_status.get("command", {})
                if probe.latest_status.get("pinger_motion_stage") != "CURVE_PROBE":
                    raise RuntimeError(
                        f"pinger did not start with the acoustic curve probe: {probe.latest_status}")
                if not isinstance(command_status, dict) or \
                        float(command_status.get("forward", 0.0)) <= 0.0 or \
                        abs(float(command_status.get("yaw", 0.0))) <= 1.0e-4:
                    raise RuntimeError(
                        f"pinger curve probe did not command a moving arc: {probe.latest_status}")
            if (args.pinger_probe_timeout or args.pinger_range_regression or
                    args.pinger_visual_handoff or args.pinger_drive_pause or
                    args.pinger_heading_first):
                probe.iq_magnitude = 0.10
            wait_state(probe, "PINGER_HOMING", 2.0, hydrophone=True)
            if probe.latest_status.get("hydrophone_source") != "upstream_ekf":
                raise RuntimeError(
                    f"FSM did not preserve the upstream hydrophone estimate: {probe.latest_status}")
            if probe.latest_status.get("hydrophone_internal_fallback_enabled") is not False:
                raise RuntimeError(
                    f"internal hydrophone fallback was unexpectedly enabled: {probe.latest_status}")
            if args.pinger_heading_first:
                probe.hydrophone_direction = (0.20, 0.98, 0.0)
                turn_deadline = time.monotonic() + 0.7
                while time.monotonic() < turn_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    command_status = probe.latest_status.get("command", {})
                    if (isinstance(command_status, dict) and
                            probe.latest_status.get("pinger_heading_drive_blocked") is True and
                            abs(float(command_status.get("forward", 1.0))) <= 1.0e-6 and
                            abs(float(command_status.get("yaw", 0.0))) > 0.10):
                        break
                else:
                    raise RuntimeError(
                        f"pinger turn did not neutralize forward thrust: {probe.latest_status}")
                probe.hydrophone_direction = (1.0, 0.0, 0.0)
                drive_deadline = time.monotonic() + 0.7
                while time.monotonic() < drive_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    command_status = probe.latest_status.get("command", {})
                    if (isinstance(command_status, dict) and
                            probe.latest_status.get("pinger_heading_drive_blocked") is False and
                            float(command_status.get("forward", 0.0)) > 0.05):
                        print("pinger_yaw_first_then_forward=PASS")
                        return 0
                raise RuntimeError(
                    f"pinger did not resume forward after yaw alignment: {probe.latest_status}")
            if args.pinger_spin_rehome:
                spin_deadline = time.monotonic() + 1.5
                while time.monotonic() < spin_deadline:
                    probe.robot_yaw_rad += 0.16
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    if (probe.latest_status.get("state") == "PINGER_SEARCH" and
                            probe.latest_status.get("pinger_spin_rehome_active") is True):
                        break
                else:
                    raise RuntimeError(
                        f"full-turn watchdog did not restart homing: {probe.latest_status}")
                command_status = probe.latest_status.get("command", {})
                if (not isinstance(command_status, dict) or
                        abs(float(command_status.get("forward", 1.0))) > 1.0e-6 or
                        abs(float(command_status.get("yaw", 1.0))) > 1.0e-6):
                    raise RuntimeError(
                        f"full-turn watchdog did not command an immediate stop: {probe.latest_status}")
                if int(probe.latest_status.get("pinger_spin_rehome_count", 0)) != 1:
                    raise RuntimeError(
                        f"full-turn watchdog count is wrong: {probe.latest_status}")
                print("pinger_full_turn_stop_and_rehome=PASS")
                return 0
            if args.pinger_vertical_align:
                diagonal_deadline = time.monotonic() + 0.8
                while time.monotonic() < diagonal_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    command_status = probe.latest_status.get("command", {})
                    if not isinstance(command_status, dict):
                        continue
                    if (float(command_status.get("heave", 0.0)) > 0.25 and
                            float(command_status.get("forward", 0.0)) > 0.05):
                        break
                else:
                    raise RuntimeError(
                        f"far pinger homing did not descend diagonally: {probe.latest_status}")

                probe.iq_magnitude = 0.30
                wait_state(probe, "PINGER_YOLO_FINE_ALIGN", 2.0, hydrophone=True)
                vertical_deadline = time.monotonic() + 0.8
                while time.monotonic() < vertical_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    command_status = probe.latest_status.get("command", {})
                    if not isinstance(command_status, dict):
                        continue
                    if (probe.latest_status.get("pinger_motion_stage") == "VERTICAL_ALIGN" and
                            probe.latest_status.get("pinger_vertical_alignment_required") is True and
                            float(command_status.get("heave", 0.0)) > 0.08 and
                            abs(float(command_status.get("forward", 1.0))) <= 1.0e-6):
                        break
                else:
                    raise RuntimeError(
                        f"near pinger did not prioritize vertical alignment: {probe.latest_status}")

                probe.hydrophone_direction = (0.995, 0.0, -0.05)
                horizontal_deadline = time.monotonic() + 0.8
                while time.monotonic() < horizontal_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    command_status = probe.latest_status.get("command", {})
                    if not isinstance(command_status, dict):
                        continue
                    if (probe.latest_status.get("pinger_motion_stage") == "HEAD_ON_ALIGN" and
                            probe.latest_status.get("pinger_vertical_alignment_required") is False and
                            abs(float(command_status.get("forward", 1.0))) <= 1.0e-6):
                        print("pinger_acoustic_vertical_alignment=PASS")
                        return 0
                raise RuntimeError(
                    f"near pinger did not hold position for YOLO after vertical alignment: "
                    f"{probe.latest_status}")
            if args.pinger_drive_pause:
                saw_drive = False
                saw_pause = False
                schedule_deadline = time.monotonic() + 1.2
                while time.monotonic() < schedule_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                    if probe.latest_status.get("state") != "PINGER_HOMING":
                        raise RuntimeError(
                            f"drive/pause schedule interrupted homing: {probe.latest_status}")
                    active = probe.latest_status.get("pinger_homing_drive_active")
                    command_status = probe.latest_status.get("command", {})
                    if active is True:
                        saw_drive = True
                    elif active is False:
                        saw_pause = True
                        if isinstance(command_status, dict) and \
                                abs(float(command_status.get("forward", 1.0))) > 1.0e-6:
                            raise RuntimeError(
                                f"pause did not neutralize forward RC: {probe.latest_status}")
                if not saw_drive or not saw_pause:
                    raise RuntimeError(
                        f"drive/pause phases were not both observed: {probe.latest_status}")
                print("pinger_drive_pause_schedule=PASS")
                return 0
            if args.pinger_visual_handoff:
                far_deadline = time.monotonic() + 0.6
                while time.monotonic() < far_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    probe.publish_observation()
                    rclpy.spin_once(probe, timeout_sec=0.03)
                if probe.latest_status.get("state") != "PINGER_HOMING":
                    raise RuntimeError(
                        f"far visual target bypassed the acoustic gate: {probe.latest_status}")
                if float(probe.latest_status.get("acoustic_range_m", 0.0)) <= 2.5:
                    raise RuntimeError(
                        f"visual handoff was not tested at far acoustic range: {probe.latest_status}")
                probe.iq_magnitude = 0.30
                wait_state(
                    probe, "PINGER_YOLO_FINE_ALIGN", 2.0,
                    vision=True, hydrophone=True,
                )
                wait_state(probe, "PINGER_SEARCH", 1.0, hydrophone=True)
                if probe.latest_status.get("pinger_visual_acquired") is not False:
                    raise RuntimeError(
                        f"stale pinger visual lock was retained: {probe.latest_status}")
                print("pinger_acoustic_gated_visual_handoff_and_loss_recovery=PASS")
                return 0
            if args.pinger_range_regression:
                # Immediate Doppler reversal is intentionally a near-source
                # guard; far-field sign flicker must not interrupt homing.
                probe.iq_magnitude = 0.08
                probe.delta_range_m = -0.01
                far_approach_deadline = time.monotonic() + 0.25
                while time.monotonic() < far_approach_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                probe.delta_range_m = 0.01
                far_reversal_deadline = time.monotonic() + 0.20
                while time.monotonic() < far_reversal_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                if probe.latest_status.get("state") != "PINGER_HOMING" or \
                        probe.latest_status.get("pinger_recovery_probe") is True:
                    raise RuntimeError(
                        f"far-field sign flicker interrupted homing: {probe.latest_status}")

                probe.iq_magnitude = 0.30
                probe.delta_range_m = -0.01
                approach_deadline = time.monotonic() + 0.55
                while time.monotonic() < approach_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.02)
                if probe.latest_status.get("pinger_doppler_approach_confirmed") is not True:
                    raise RuntimeError(
                        f"Doppler approach trend was not confirmed: {probe.latest_status}")
                reversal_started = time.monotonic()
                probe.delta_range_m = 0.01
                wait_state(probe, "PINGER_SEARCH", 0.5, hydrophone=True)
                if time.monotonic() - reversal_started > 0.55:
                    raise RuntimeError(
                        f"Doppler reversal brake was too late: {probe.latest_status}")
                if probe.latest_status.get("pinger_range_worsening") is not True:
                    raise RuntimeError(
                        f"range-regression rejection was not reported: {probe.latest_status}")
                if probe.latest_status.get("pinger_recovery_probe") is not True:
                    raise RuntimeError(
                        f"range regression did not select lateral recovery: {probe.latest_status}")
                command_status = probe.latest_status.get("command", {})
                if isinstance(command_status, dict) and command_status.get("forward", 1.0) > 0.0:
                    raise RuntimeError(
                        f"reversal brake continued forward: {probe.latest_status}")
                print("pinger_immediate_doppler_reversal_guard=PASS")
                return 0
            if args.pinger_probe_timeout:
                continuous_deadline = time.monotonic() + 0.6
                while time.monotonic() < continuous_deadline:
                    probe.publish_vehicle()
                    probe.publish_hydrophone()
                    rclpy.spin_once(probe, timeout_sec=0.03)
                    if probe.latest_status.get("state") != "PINGER_HOMING":
                        raise RuntimeError(
                            f"committed pinger homing was interrupted: {probe.latest_status}")
                probe.iq_magnitude = 0.30
            wait_state(probe, "PINGER_YOLO_FINE_ALIGN", 2.0, hydrophone=True)
            stop_deadline = time.monotonic() + 0.6
            while time.monotonic() < stop_deadline:
                probe.publish_vehicle()
                probe.publish_hydrophone()
                rclpy.spin_once(probe, timeout_sec=0.03)
                command_status = probe.latest_status.get("command", {})
                if (isinstance(command_status, dict) and
                        probe.latest_status.get("pinger_heading_drive_blocked") is True and
                        abs(float(command_status.get("forward", 1.0))) <= 1.0e-6):
                    break
            else:
                raise RuntimeError(
                    f"pinger acoustic fine-align did not stop at the YOLO handoff radius: "
                    f"{probe.latest_status}")
            mismatch_deadline = time.monotonic() + 1.0
            while time.monotonic() < mismatch_deadline:
                probe.publish_vehicle()
                probe.publish_hydrophone()
                probe.publish_observation(
                    normalized_error_x=0.95,
                    class_label="stick" if args.stick_rake_flow else "buoy",
                )
                rclpy.spin_once(probe, timeout_sec=0.03)
            if probe.latest_status.get("state") != "PINGER_YOLO_FINE_ALIGN":
                raise RuntimeError(
                    f"acoustically inconsistent pinger candidate was accepted: {probe.latest_status}")
            visual_label = "stick" if args.stick_rake_flow else "buoy"
            wait_state(
                probe, "PINGER_YOLO_FINE_ALIGN", 2.0,
                vision=True, vision_label=visual_label, hydrophone=True,
            )
            if args.stick_rake_flow:
                wait_state(
                    probe, "PINGER_CAPTURE", 2.0,
                    vision=True, vision_label="stick", hydrophone=True,
                )
                wait_head_on_command(
                    probe, "PINGER_CAPTURE", 1, 0.5,
                    vision_label="stick", hydrophone=True,
                )
            wait_state(
                probe, "PINGER_VERIFY", 3.0,
                vision=True, vision_label=visual_label, hydrophone=True,
            )
            if args.stick_rake_flow:
                wait_state(
                    probe, "PINGER_BACKOFF", 1.0,
                    vision=True, vision_label="stick", hydrophone=True,
                    collector="DETACHED",
                    collector_target="course_buoy_pinger_white_1",
                )
                wait_head_on_command(
                    probe, "PINGER_BACKOFF", -1, 0.5,
                    vision_label="stick", hydrophone=True,
                    collector="DETACHED",
                    collector_target="course_buoy_pinger_white_1",
                )
            wait_state(
                probe, "SURFACE_BUOY_SEARCHING", 2.0, hydrophone=True,
                collector="DETACHED", collector_target="course_buoy_pinger_white_1")
            if args.stick_rake_flow:
                print("pinger_stick_head_on_rake_flow=PASS")
                return 0
        else:
            wait_state(probe, "BUOY_SEARCHING", 4.0)
            depth_deadline = time.monotonic() + 1.0
            while time.monotonic() < depth_deadline:
                probe.publish_vehicle()
                rclpy.spin_once(probe, timeout_sec=0.03)
                command_status = probe.latest_status.get("command", {})
                if isinstance(command_status, dict) and command_status.get("heave", 0.0) > 0.0:
                    break
            else:
                raise RuntimeError(
                    f"underwater depth hold did not command descent: {probe.latest_status}")
            if args.detached_rejection:
                wait_state(probe, "BUOY_FINE_ALIGN", 2.0, vision=True)
                wait_state(probe, "BUOY_SEARCHING", 2.0, collector="DETACHED")
                rejection_deadline = time.monotonic() + 0.8
                while time.monotonic() < rejection_deadline:
                    probe.publish_vehicle()
                    probe.publish_observation()
                    rclpy.spin_once(probe, timeout_sec=0.03)
                if probe.latest_status.get("state") != "BUOY_SEARCHING":
                    raise RuntimeError(
                        f"detached contact was reacquired: {probe.latest_status}")
                if probe.latest_status.get("target_in_own_course") is not False:
                    raise RuntimeError(
                        f"detached contact rejection was not reported: {probe.latest_status}")
                command_status = probe.latest_status.get("command", {})
                if not isinstance(command_status, dict) or command_status.get("forward", 0.0) <= 0.0:
                    raise RuntimeError(
                        f"detached contact escape was not commanded: {probe.latest_status}")
                print("observation_detached_rejection=PASS")
                return 0
            if args.capture_retry:
                wait_state(probe, "VERIFY_CAPTURE", 3.0, vision=True)
                wait_state(probe, "BUOY_FINE_ALIGN", 1.5, vision=True)
                if probe.latest_status.get("failure"):
                    raise RuntimeError(f"capture retry failed: {probe.latest_status}")
                print("observation_capture_retry=PASS")
                return 0
            if args.stick_rake_flow:
                wait_state(
                    probe, "CAPTURE", 3.0,
                    vision=True, vision_label="stick",
                )
                wait_head_on_command(
                    probe, "CAPTURE", 1, 0.5, vision_label="stick",
                )
                wait_state(
                    probe, "VERIFY_CAPTURE", 1.0,
                    vision=True, vision_label="stick",
                )
                wait_state(
                    probe, "BUOY_BACKOFF", 1.0,
                    vision=True, vision_label="stick", collector="DETACHED",
                )
                wait_head_on_command(
                    probe, "BUOY_BACKOFF", -1, 0.5,
                    vision_label="stick", collector="DETACHED",
                )
                wait_state(probe, "SURFACE_BUOY_SEARCHING", 1.0)
                print("observation_stick_head_on_rake_flow=PASS")
                return 0
            if args.early_detach:
                wait_state(
                    probe, "SURFACE_BUOY_SEARCHING", 3.0,
                    vision=True, collector="DETACHED"
                )
            else:
                wait_state(probe, "VERIFY_CAPTURE", 3.0, vision=True)
                wait_state(probe, "SURFACE_BUOY_SEARCHING", 2.0, collector="DETACHED")
        wait_state(probe, "SURFACE_COLLECT", 2.0, vision=True)
        wait_state(probe, "RETURN_ZONE", 2.0, vision=True, collector="NETTED")
        wait_state(probe, "RELEASE", 2.0)
        wait_state(probe, "COMPLETE", 2.0, collector="RELEASED")

        score_zone = probe.latest_status.get("score_zone")
        if (
            not isinstance(score_zone, dict)
            or not isinstance(score_zone.get("xyz"), list)
            or len(score_zone["xyz"]) != 3
            or float(score_zone.get("radius", 0.0)) <= 0.0
        ):
            raise RuntimeError(
                f"observation FSM did not publish the physical score-zone contract: "
                f"{probe.latest_status}"
            )

        if probe.latest_rc is None or any(
            value != OverrideRCIn.CHAN_RELEASE for value in probe.latest_rc.channels
        ):
            raise RuntimeError("dry-run controller emitted a non-release RC command")
        for topic in ("/mujoco/course_buoys/status", "/mujoco/ground_truth/pose"):
            offenders = [
                info.node_name for info in probe.get_subscriptions_info_by_topic(topic)
                if info.node_name == "observation_mission_fsm"
            ]
            if offenders:
                raise RuntimeError(f"sensor FSM subscribes to forbidden ground truth: {topic}")
        print("observation_mission_runtime=PASS")
        return 0
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        if process.returncode not in (0, -15):
            print(output)


if __name__ == "__main__":
    raise SystemExit(main())
