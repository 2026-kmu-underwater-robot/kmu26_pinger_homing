#!/usr/bin/python3
"""Dedicated web GUI for kmu26_mission_fsm.

This is intentionally separate from the simulator control GUI and the real
vehicle localization GUI. It only manages mission-FSM bringup, RViz helpers,
camera preview, state telemetry, and guarded RC override commands.
"""

from __future__ import annotations

import argparse
import contextlib
import errno
import io
import json
import math
import mimetypes
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

from ament_index_python.packages import get_package_share_directory

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import ManualControl
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import String

cv2 = None
np = None
RAW_IMAGE_CONVERSION_ERROR = ""
try:  # Optional raw image conversion.
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        import cv2 as _cv2  # type: ignore
        import numpy as _np  # type: ignore
    cv2 = _cv2
    np = _np
except Exception as exc:  # pragma: no cover - depends on host image stack.
    RAW_IMAGE_CONVERSION_ERROR = str(exc) or exc.__class__.__name__


RC_CHANNEL_COUNT = 18
RC_NOCHANGE = 65535
RC_RELEASE = 0
RC_NEUTRAL = 1500
RUNTIME_TOPIC_KEYS = {
    "pose_topic",
    "pose_type",
    "state_topic",
    "camera_compressed_topic",
    "camera_annotated_topic",
    "camera_raw_topic",
    "vision_status_topic",
    "fsm_status_topic",
    "pinger_status_topic",
}


def now_s() -> float:
    return time.time()


def read_json_file(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if hasattr(value, "item"):
        try:
            return json_safe(value.item())
        except Exception:
            pass
    return str(value)


@dataclass
class TopicHealth:
    name: str
    last_s: float | None = None
    stamps: deque[float] = field(default_factory=lambda: deque(maxlen=80))

    def tick(self) -> None:
        stamp = now_s()
        self.last_s = stamp
        self.stamps.append(stamp)

    def snapshot(self) -> dict[str, Any]:
        now = now_s()
        recent = [stamp for stamp in self.stamps if now - stamp <= 2.0]
        hz = 0.0
        if len(recent) >= 2 and recent[-1] > recent[0]:
            hz = (len(recent) - 1) / (recent[-1] - recent[0])
        age = None if self.last_s is None else max(0.0, now - self.last_s)
        return {
            "name": self.name,
            "alive": age is not None and age <= 2.0,
            "age": age,
            "hz": hz,
        }


class ManagedProcess:
    def __init__(self, label: str, command: list[str], log: deque[str]):
        self.label = label
        self.command = command
        self.log = log
        self.process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def start(self) -> None:
        if self.running:
            return
        self._append(f"start: {' '.join(self.command)}")
        self.process = subprocess.Popen(
            self.command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            preexec_fn=os.setsid,
        )
        self._thread = threading.Thread(target=self._read_output, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self.running or self.process is None:
            return
        self._append("stop requested")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=4.0)
        except Exception:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception:
                pass

    def snapshot(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "running": self.running,
            "pid": None if self.process is None else self.process.pid,
            "command": self.command,
        }

    def _append(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.log.append(f"[{stamp}] [{self.label}] {message}")

    def _read_output(self) -> None:
        assert self.process is not None
        if self.process.stdout is not None:
            for line in self.process.stdout:
                self._append(line.rstrip())
        code = self.process.wait()
        self._append(f"exited code={code}")


class FsmGuiNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("mission_fsm_web_gui_bridge")
        self.args = args
        self.lock = threading.Lock()
        self.camera_cv = threading.Condition(self.lock)
        self.logs: deque[str] = deque(maxlen=300)
        self.config_path = Path(args.config_json)
        self.launch_config = self._load_launch_config()
        self.camera_enabled = bool(args.camera_on)
        self.last_camera_jpeg: bytes | None = None
        self.last_camera_stamp_s: float | None = None
        self.last_annotated_stamp_s: float | None = None
        self.last_camera_source = "none"
        self.last_compressed_error = ""
        self.last_rc_override: list[int] = []
        self.last_manual_control: dict[str, Any] = {}
        self.last_yolo_text = ""
        self.last_vision_status: dict[str, Any] = {}
        self.last_fsm_status: dict[str, Any] = {}
        self.last_pinger_status: dict[str, Any] = {}
        self.pose = {"x": None, "y": None, "z": None, "yaw": None}
        self.velocity = {"x": None, "y": None, "z": None}
        self.depth = {"z": None}
        self.mavros_state = {"connected": False, "armed": False, "mode": ""}
        self.battery = {"voltage": None, "current": None, "percentage": None}
        self.joy = {"axes": [], "buttons": []}
        self.imu = {"orientation": {}, "angular_velocity": {}, "linear_acceleration": {}}
        self.processes: dict[str, ManagedProcess] = {}

        self.health = {
            "odom": TopicHealth(args.pose_topic),
            "mavros_state": TopicHealth(args.state_topic),
            "rc_override": TopicHealth(args.rc_topic),
            "manual_control": TopicHealth(args.manual_topic),
            "dvl": TopicHealth(args.dvl_twist_topic),
            "depth": TopicHealth(args.depth_topic),
            "imu": TopicHealth(args.imu_topic),
            "joy": TopicHealth(args.joy_topic),
            "battery": TopicHealth(args.battery_topic),
            "camera_compressed": TopicHealth(args.camera_compressed_topic),
            "camera_annotated": TopicHealth(args.camera_annotated_topic),
            "camera_raw": TopicHealth(args.camera_raw_topic),
            "vision_status": TopicHealth(args.vision_status_topic),
            "fsm_status": TopicHealth(args.fsm_status_topic),
            "pinger_status": TopicHealth(args.pinger_status_topic),
        }

        telemetry_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.rc_pub = self.create_publisher(OverrideRCIn, args.rc_topic, 10)
        self.create_subscription(Odometry, args.pose_topic, self._on_odom, telemetry_qos)
        self.create_subscription(State, args.state_topic, self._on_state, telemetry_qos)
        self.create_subscription(OverrideRCIn, args.rc_topic, self._on_rc_override, telemetry_qos)
        self.create_subscription(ManualControl, args.manual_topic, self._on_manual_control, telemetry_qos)
        self.create_subscription(TwistWithCovarianceStamped, args.dvl_twist_topic, self._on_dvl, telemetry_qos)
        self.create_subscription(PoseWithCovarianceStamped, args.depth_topic, self._on_depth, telemetry_qos)
        self.create_subscription(Imu, args.imu_topic, self._on_imu, telemetry_qos)
        self.create_subscription(Joy, args.joy_topic, self._on_joy, telemetry_qos)
        self.create_subscription(BatteryState, args.battery_topic, self._on_battery, telemetry_qos)
        self.create_subscription(CompressedImage, args.camera_compressed_topic, self._on_compressed_image, telemetry_qos)
        self.create_subscription(CompressedImage, args.camera_annotated_topic, self._on_annotated_image, telemetry_qos)
        self.create_subscription(Image, args.camera_raw_topic, self._on_raw_image, telemetry_qos)
        self.create_subscription(String, args.vision_status_topic, self._on_vision_status, telemetry_qos)
        self.create_subscription(String, args.fsm_status_topic, self._on_fsm_status, telemetry_qos)
        self.create_subscription(String, args.pinger_status_topic, self._on_pinger_status, telemetry_qos)

    def _load_launch_config(self) -> dict[str, Any]:
        defaults = {
            "course": "all",
            "own_course": "a",
            "dry_run": True,
            "transport": "rc_override",
            "rate_hz": 30.0,
            "course_boundary_x": 0.0,
            "course_boundary_margin": 0.8,
            "course_boundary_standoff": 0.7,
            "tank_x_min": -12.0,
            "tank_x_max": 12.0,
            "tank_y_min": -8.0,
            "tank_y_max": 8.0,
            "robot_start_x": 0.0,
            "robot_start_y": 0.0,
            "robot_start_yaw_deg": 0.0,
            "pinger_depth_z": -8.5,
            "tank_max_depth_m": 11.0,
            "score_zone_x": -6.8,
            "score_zone_y": 0.0,
            "score_zone_radius": 1.5,
            "pose_topic": self.args.pose_topic,
            "pose_type": self.args.pose_type,
            "state_topic": self.args.state_topic,
            "camera_compressed_topic": self.args.camera_compressed_topic,
            "camera_annotated_topic": self.args.camera_annotated_topic,
            "camera_raw_topic": self.args.camera_raw_topic,
            "vision_status_topic": self.args.vision_status_topic,
            "fsm_status_topic": self.args.fsm_status_topic,
            "pinger_status_topic": self.args.pinger_status_topic,
        }
        saved = read_json_file(Path(self.args.config_json))
        if isinstance(saved, dict):
            defaults.update({k: v for k, v in saved.items() if k in defaults and k not in RUNTIME_TOPIC_KEYS})
        defaults.update({
            "pose_topic": self.args.pose_topic,
            "pose_type": self.args.pose_type,
            "state_topic": self.args.state_topic,
            "camera_compressed_topic": self.args.camera_compressed_topic,
            "camera_annotated_topic": self.args.camera_annotated_topic,
            "camera_raw_topic": self.args.camera_raw_topic,
            "vision_status_topic": self.args.vision_status_topic,
            "fsm_status_topic": self.args.fsm_status_topic,
            "pinger_status_topic": self.args.pinger_status_topic,
        })
        return defaults

    def save_launch_config(self, patch: dict[str, Any]) -> dict[str, Any]:
        allowed = set(self.launch_config)
        with self.lock:
            for key, value in patch.items():
                if key not in allowed or key in RUNTIME_TOPIC_KEYS:
                    continue
                if key in {"dry_run"}:
                    self.launch_config[key] = bool(value)
                elif key in {
                    "rate_hz",
                    "course_boundary_x",
                    "course_boundary_margin",
                    "course_boundary_standoff",
                    "tank_x_min",
                    "tank_x_max",
                    "tank_y_min",
                    "tank_y_max",
                    "robot_start_x",
                    "robot_start_y",
                    "robot_start_yaw_deg",
                    "pinger_depth_z",
                    "tank_max_depth_m",
                    "score_zone_x",
                    "score_zone_y",
                    "score_zone_radius",
                }:
                    self.launch_config[key] = float(value)
                else:
                    self.launch_config[key] = str(value)
            self.camera_enabled = bool(patch.get("camera_enabled", self.camera_enabled))
            atomic_write_json(self.config_path, self.launch_config)
            return dict(self.launch_config)

    def start_process(self, kind: str, extra: dict[str, Any] | None = None) -> dict[str, Any]:
        extra = extra or {}
        with self.lock:
            cfg = dict(self.launch_config)
        if kind == "mission":
            command = [
                "ros2", "launch", "kmu26_vision_mission_fsm", "mission_fsm_real.launch.py",
                "use_mission_fsm:=false",
                "use_observation_mission_fsm:=true",
                "use_vision_detector:=true",
                "use_pinger_homing:=true",
                "pinger_controller:=active_range",
                "vehicle_disconnect_grace:=12.0",
                "state_timeout:=30.0",
                "mission_enabled:=true",
                "use_mission_rviz_visualizer:=true",
                "use_rviz:=false",
                f"dry_run:={str(bool(extra.get('dry_run', cfg['dry_run']))).lower()}",
                f"course:={cfg['course']}",
                f"own_course:={cfg['own_course']}",
                f"transport:={cfg['transport']}",
                f"rate_hz:={cfg['rate_hz']}",
                f"pose_topic:={cfg['pose_topic']}",
                f"pose_type:={cfg['pose_type']}",
                f"state_topic:={cfg['state_topic']}",
                f"course_boundary_x:={cfg['course_boundary_x']}",
                f"course_boundary_margin:={cfg['course_boundary_margin']}",
                f"course_boundary_standoff:={cfg['course_boundary_standoff']}",
                f"pinger_depth_z:={cfg['pinger_depth_z']}",
                f"tank_max_depth_m:={cfg['tank_max_depth_m']}",
                f"score_zone_x:={cfg['score_zone_x']}",
                f"score_zone_y:={cfg['score_zone_y']}",
                f"score_zone_radius:={cfg['score_zone_radius']}",
                f"camera_compressed_topic:={cfg['camera_compressed_topic']}",
            ]
        elif kind == "pinger":
            command = [
                "ros2", "launch", "kmu26_vision_mission_fsm", "mission_fsm_real.launch.py",
                "use_mission_fsm:=false",
                "use_pinger_homing:=true",
                "pinger_controller:=active_range",
                "vehicle_disconnect_grace:=12.0",
                "use_mission_rviz_visualizer:=true",
                "use_rviz:=false",
                f"transport:={cfg['transport']}",
                f"rate_hz:={cfg['rate_hz']}",
                f"pose_topic:={cfg['pose_topic']}",
                f"tank_max_depth_m:={cfg['tank_max_depth_m']}",
            ]
        elif kind == "rviz":
            command = [
                "ros2", "launch", "kmu26_vision_mission_fsm", "mission_fsm_real.launch.py",
                "use_mission_fsm:=false",
                "use_mission_rviz_visualizer:=true",
                "use_rviz:=true",
                f"pose_topic:={cfg['pose_topic']}",
                f"pose_type:={cfg['pose_type']}",
                f"course_boundary_x:={cfg['course_boundary_x']}",
                f"course_boundary_margin:={cfg['course_boundary_margin']}",
                f"course_boundary_standoff:={cfg['course_boundary_standoff']}",
            ]
        elif kind == "markers":
            command = [
                "ros2", "launch", "kmu26_vision_mission_fsm", "mission_fsm_real.launch.py",
                "use_mission_fsm:=false",
                "use_mission_rviz_visualizer:=true",
                "use_rviz:=false",
                f"pose_topic:={cfg['pose_topic']}",
                f"pose_type:={cfg['pose_type']}",
            ]
        else:
            raise ValueError(f"unknown process kind: {kind}")

        with self.lock:
            proc = self.processes.get(kind)
            if proc is None:
                proc = ManagedProcess(kind, command, self.logs)
                self.processes[kind] = proc
            elif not proc.running:
                proc.command = command
            proc.start()
            return proc.snapshot()

    def stop_process(self, kind: str) -> dict[str, Any]:
        with self.lock:
            proc = self.processes.get(kind)
            if proc is None:
                return {"label": kind, "running": False, "pid": None, "command": []}
            proc.stop()
            return proc.snapshot()

    def publish_rc(self, mode: str, axes: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self.args.allow_rc_send:
            raise PermissionError("RC send is disabled. Relaunch with allow_rc_send:=true.")
        msg = OverrideRCIn()
        channels = [RC_NOCHANGE] * RC_CHANNEL_COUNT
        if mode == "release":
            channels = [RC_RELEASE] * RC_CHANNEL_COUNT
        elif mode == "center":
            for i in range(6):
                channels[i] = RC_NEUTRAL
        elif mode == "axes":
            axes = axes or {}
            for i in range(6):
                channels[i] = RC_NEUTRAL
            span = 350.0
            mapping = [
                ("pitch", 0),
                ("heave", 1),
                ("yaw", 2),
                ("forward", 3),
                ("lateral", 4),
            ]
            for key, idx in mapping:
                raw = float(axes.get(key, 0.0))
                channels[idx] = int(round(RC_NEUTRAL + max(-1.0, min(1.0, raw)) * span))
        else:
            raise ValueError(f"unknown rc mode: {mode}")
        msg.channels = channels
        self.rc_pub.publish(msg)
        with self.lock:
            self.last_rc_override = list(channels)
            self.logs.append(f"[{time.strftime('%H:%M:%S')}] [rc] publish {mode}: {channels[:6]}")
        return {"mode": mode, "channels": channels, "allowed": self.args.allow_rc_send}

    def snapshot(self) -> dict[str, Any]:
        status_path = Path(self.args.mission_status_json)
        file_mission_status = read_json_file(status_path)
        with self.lock:
            pinger_process = self.processes.get("pinger")
            pinger_running = pinger_process is not None and pinger_process.running
            if pinger_running and self.last_pinger_status:
                mission_status = dict(self.last_pinger_status)
            else:
                mission_status = dict(self.last_fsm_status) if self.last_fsm_status else file_mission_status
            return {
                "time": now_s(),
                "config": dict(self.launch_config),
                "camera": {
                    "enabled": self.camera_enabled,
                    "compressed_topic": self.args.camera_compressed_topic,
                    "annotated_topic": self.args.camera_annotated_topic,
                    "raw_topic": self.args.camera_raw_topic,
                    "source": self.last_camera_source,
                    "age": None if self.last_camera_stamp_s is None else max(0.0, now_s() - self.last_camera_stamp_s),
                    "has_frame": self.last_camera_jpeg is not None,
                    "raw_conversion": "opencv" if cv2 is not None and np is not None else "unavailable",
                    "raw_conversion_error": RAW_IMAGE_CONVERSION_ERROR,
                    "error": self.last_compressed_error,
                },
                "mission_status_path": str(status_path),
                "mission_status": mission_status,
                "telemetry": {
                    "pose": dict(self.pose),
                    "velocity": dict(self.velocity),
                    "depth": dict(self.depth),
                    "mavros_state": dict(self.mavros_state),
                    "battery": dict(self.battery),
                    "joy": {"axes": list(self.joy["axes"]), "buttons": list(self.joy["buttons"])},
                    "imu": json.loads(json.dumps(self.imu)),
                    "last_rc_override": list(self.last_rc_override),
                    "last_manual_control": dict(self.last_manual_control),
                    "last_yolo_text": self.last_yolo_text,
                    "vision_status": dict(self.last_vision_status),
                },
                "topics": {key: value.snapshot() for key, value in self.health.items()},
                "processes": {key: value.snapshot() for key, value in self.processes.items()},
                "rc_send_allowed": self.args.allow_rc_send,
                "logs": list(self.logs),
            }

    def wait_camera_frame(self, last_stamp: float | None, timeout_s: float) -> tuple[bytes | None, float | None]:
        end = now_s() + timeout_s
        with self.camera_cv:
            while now_s() < end:
                if self.last_camera_jpeg is not None and self.last_camera_stamp_s != last_stamp:
                    return self.last_camera_jpeg, self.last_camera_stamp_s
                self.camera_cv.wait(timeout=0.2)
            return self.last_camera_jpeg, self.last_camera_stamp_s

    def _on_odom(self, msg: Odometry) -> None:
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            self.health["odom"].tick()
            self.pose = {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "yaw": yaw,
            }

    def _on_state(self, msg: State) -> None:
        with self.lock:
            self.health["mavros_state"].tick()
            self.mavros_state = {"connected": msg.connected, "armed": msg.armed, "mode": msg.mode}

    def _on_rc_override(self, msg: OverrideRCIn) -> None:
        with self.lock:
            self.health["rc_override"].tick()
            self.last_rc_override = list(msg.channels)

    def _on_manual_control(self, msg: ManualControl) -> None:
        with self.lock:
            self.health["manual_control"].tick()
            self.last_manual_control = {"x": msg.x, "y": msg.y, "z": msg.z, "r": msg.r, "buttons": msg.buttons}

    def _on_dvl(self, msg: TwistWithCovarianceStamped) -> None:
        with self.lock:
            self.health["dvl"].tick()
            linear = msg.twist.twist.linear
            self.velocity = {"x": linear.x, "y": linear.y, "z": linear.z}

    def _on_depth(self, msg: PoseWithCovarianceStamped) -> None:
        with self.lock:
            self.health["depth"].tick()
            self.depth = {"z": msg.pose.pose.position.z}

    def _on_imu(self, msg: Imu) -> None:
        with self.lock:
            self.health["imu"].tick()
            self.imu = {
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w,
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
            }

    def _on_joy(self, msg: Joy) -> None:
        with self.lock:
            self.health["joy"].tick()
            self.joy = {"axes": list(msg.axes), "buttons": list(msg.buttons)}

    def _on_battery(self, msg: BatteryState) -> None:
        with self.lock:
            self.health["battery"].tick()
            self.battery = {
                "voltage": msg.voltage,
                "current": msg.current,
                "percentage": msg.percentage,
            }

    def _on_vision_status(self, msg: String) -> None:
        with self.lock:
            self.health["vision_status"].tick()
            self.last_yolo_text = msg.data[:2000]
            try:
                payload = json.loads(msg.data)
                self.last_vision_status = payload if isinstance(payload, dict) else {}
            except json.JSONDecodeError:
                self.last_vision_status = {}

    def _on_fsm_status(self, msg: String) -> None:
        with self.lock:
            self.health["fsm_status"].tick()
            try:
                payload = json.loads(msg.data)
                self.last_fsm_status = payload if isinstance(payload, dict) else {}
            except json.JSONDecodeError:
                self.last_fsm_status = {}

    def _on_pinger_status(self, msg: String) -> None:
        with self.lock:
            self.health["pinger_status"].tick()
            try:
                payload = json.loads(msg.data)
                self.last_pinger_status = payload if isinstance(payload, dict) else {}
            except json.JSONDecodeError:
                self.last_pinger_status = {}

    def _on_annotated_image(self, msg: CompressedImage) -> None:
        with self.camera_cv:
            self.health["camera_annotated"].tick()
            self.last_annotated_stamp_s = now_s()
            if not self.camera_enabled:
                return
            self.last_camera_jpeg = bytes(msg.data)
            self.last_camera_stamp_s = self.last_annotated_stamp_s
            self.last_camera_source = "yolo_annotated"
            self.last_compressed_error = ""
            self.camera_cv.notify_all()

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        with self.camera_cv:
            self.health["camera_compressed"].tick()
            if not self.camera_enabled:
                return
            if self.last_annotated_stamp_s is not None and now_s() - self.last_annotated_stamp_s <= 1.0:
                return
            self.last_camera_jpeg = bytes(msg.data)
            self.last_camera_stamp_s = now_s()
            self.last_camera_source = "camera_compressed"
            self.last_compressed_error = ""
            self.camera_cv.notify_all()

    def _on_raw_image(self, msg: Image) -> None:
        with self.camera_cv:
            self.health["camera_raw"].tick()
            if not self.camera_enabled:
                return
            if self.last_camera_stamp_s is not None and now_s() - self.last_camera_stamp_s <= 1.0:
                return
        data = raw_image_to_jpeg(msg)
        with self.camera_cv:
            if data is None:
                self.last_compressed_error = f"raw conversion unavailable for {msg.encoding}"
                return
            self.last_camera_jpeg = data
            self.last_camera_stamp_s = now_s()
            self.last_camera_source = "camera_raw"
            self.last_compressed_error = ""
            self.camera_cv.notify_all()


def yaw_from_quat(q: Any) -> float:
    import math
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def raw_image_to_jpeg(msg: Image) -> bytes | None:
    if cv2 is None or np is None:
        return None
    enc = msg.encoding.lower()
    channels = {
        "mono8": 1,
        "8uc1": 1,
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }.get(enc)
    if channels is None or msg.height <= 0 or msg.width <= 0:
        return None
    try:
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.step))[:, : msg.width * channels]
        if channels > 1:
            arr = arr.reshape((msg.height, msg.width, channels))
        else:
            arr = arr.reshape((msg.height, msg.width))
        if enc == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif enc == "rgba8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        elif enc == "bgra8":
            arr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
        ok, out = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None
        return bytes(out)
    except Exception:
        return None


class FsmWebHandler(BaseHTTPRequestHandler):
    server_version = "Kmu26MissionFsmGui/1.0"
    controller: FsmGuiNode
    static_dir: Path

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802 - stdlib API
        parsed = urlparse(self.path)
        if parsed.path == "/api/status":
            self._send_json(self.controller.snapshot())
            return
        if parsed.path == "/api/camera.mjpg":
            self._serve_mjpeg()
            return
        if parsed.path == "/api/camera.jpg":
            frame, _stamp = self.controller.wait_camera_frame(None, 0.1)
            if frame is None:
                self._send_error_json(HTTPStatus.NOT_FOUND, "no camera frame")
            else:
                self._send_binary(frame, "image/jpeg")
            return
        self._serve_static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802 - stdlib API
        try:
            payload = self._read_json()
            parsed = urlparse(self.path)
            if parsed.path == "/api/config":
                self._send_json({"ok": True, "config": self.controller.save_launch_config(payload)})
                return
            if parsed.path == "/api/process/start":
                kind = str(payload.get("kind", ""))
                self._send_json({"ok": True, "process": self.controller.start_process(kind, payload)})
                return
            if parsed.path == "/api/process/stop":
                kind = str(payload.get("kind", ""))
                self._send_json({"ok": True, "process": self.controller.stop_process(kind)})
                return
            if parsed.path == "/api/rc":
                mode = str(payload.get("mode", ""))
                axes = payload.get("axes") if isinstance(payload.get("axes"), dict) else {}
                self._send_json({"ok": True, "rc": self.controller.publish_rc(mode, axes)})
                return
            self._send_error_json(HTTPStatus.NOT_FOUND, "not found")
        except PermissionError as exc:
            self._send_error_json(HTTPStatus.FORBIDDEN, str(exc))
        except Exception as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))

    def _serve_static(self, request_path: str) -> None:
        rel = unquote(request_path.lstrip("/")) or "index.html"
        relative_path = Path(rel)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            self._send_error_json(HTTPStatus.FORBIDDEN, "forbidden path")
            return
        # colcon --symlink-install intentionally points package assets back to
        # the source tree, so validating the resolved target would reject every
        # legitimate static file. Validate the request path before following it.
        path = self.static_dir / relative_path
        if not path.is_file():
            self._send_error_json(HTTPStatus.NOT_FOUND, "not found")
            return
        data = path.read_bytes()
        content_type, _encoding = mimetypes.guess_type(str(path))
        self._send_binary(data, content_type or "application/octet-stream")

    def _serve_mjpeg(self) -> None:
        boundary = "missionfsmframe"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_stamp = None
        while True:
            frame, stamp = self.controller.wait_camera_frame(last_stamp, 2.0)
            if frame is None:
                continue
            last_stamp = stamp
            try:
                self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
            except BrokenPipeError:
                break
            except ConnectionResetError:
                break

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        data = self.rfile.read(min(length, 1024 * 1024))
        payload = json.loads(data.decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("payload must be an object")
        return payload

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(json_safe(payload), ensure_ascii=False, allow_nan=False).encode("utf-8")
        self._send_binary(data, "application/json; charset=utf-8", status)

    def _send_binary(self, data: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _send_error_json(self, status: HTTPStatus, message: str) -> None:
        self._send_json({"ok": False, "error": message}, status)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def build_handler(node: FsmGuiNode, static_dir: Path) -> type[FsmWebHandler]:
    class Handler(FsmWebHandler):
        pass
    Handler.controller = node
    Handler.static_dir = static_dir
    return Handler


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Dedicated web GUI for kmu26_mission_fsm")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8890)
    parser.add_argument("--static-dir", default="")
    parser.add_argument("--mission-status-json", default="/tmp/kmu26_mission_fsm_status.json")
    parser.add_argument("--config-json", default="/tmp/kmu26_mission_fsm_gui_config.json")
    parser.add_argument("--pose-topic", default="/odometry/filtered")
    parser.add_argument("--pose-type", default="odometry")
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--rc-topic", default="/mavros/rc/override")
    parser.add_argument("--manual-topic", default="/mavros/manual_control/send")
    parser.add_argument("--dvl-twist-topic", default="/dvl/twist")
    parser.add_argument("--depth-topic", default="/depth/pose")
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--joy-topic", default="/joy")
    parser.add_argument("--battery-topic", default="/battery")
    parser.add_argument("--camera-compressed-topic", default="/camera/camera/color/image_raw/compressed")
    parser.add_argument("--camera-annotated-topic", default="/vision/buoy/image_annotated/compressed")
    parser.add_argument("--camera-raw-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--vision-status-topic", default="/vision/buoy/status")
    parser.add_argument("--fsm-status-topic", default="/mission/fsm/status")
    parser.add_argument("--pinger-status-topic", default="/pinger_homing/status")
    parser.add_argument("--camera-on", action="store_true")
    parser.add_argument("--allow-rc-send", action="store_true")
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args if ros_args else None)
    node = FsmGuiNode(args)

    if args.static_dir:
        static_dir = Path(args.static_dir)
    else:
        static_dir = Path(get_package_share_directory("kmu26_vision_mission_fsm")) / "web" / "fsm_gui"
    handler_cls = build_handler(node, static_dir)
    try:
        httpd = ReusableThreadingHTTPServer((args.host, args.port), handler_cls)
    except OSError as exc:
        node.destroy_node()
        rclpy.shutdown()
        if exc.errno == errno.EADDRINUSE:
            print(
                f"[fsm-gui] {args.host}:{args.port} is already in use; "
                "stop the previous GUI launch or set port:=8891",
                file=sys.stderr,
                flush=True,
            )
            return 1
        raise

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="fsm-web-rclpy", daemon=True)
    spin_thread.start()

    stop_event = threading.Event()

    def stop(_signum: int, _frame: Any) -> None:
        stop_event.set()
        threading.Thread(target=httpd.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    print(f"[fsm-gui] serving http://{args.host}:{args.port}/", flush=True)
    try:
        httpd.serve_forever(poll_interval=0.2)
    finally:
        for proc in list(node.processes.values()):
            proc.stop()
        httpd.server_close()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
