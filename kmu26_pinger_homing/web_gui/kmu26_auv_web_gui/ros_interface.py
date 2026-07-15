import math
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from dvl_msgs.msg import CommandResponse
from dvl_msgs.msg import ConfigCommand
from dvl_msgs.msg import ConfigStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import String


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


@dataclass
class TopicHealth:
    name: str
    stale_after: float = 1.0
    last_seen: float | None = None
    stamps: deque[float] = field(default_factory=lambda: deque(maxlen=120))

    def tick(self) -> None:
        now = time.monotonic()
        self.last_seen = now
        self.stamps.append(now)

    def snapshot(self) -> dict:
        now = time.monotonic()
        age = None if self.last_seen is None else now - self.last_seen
        return {
            "name": self.name,
            "alive": age is not None and age <= self.stale_after,
            "age": age,
            "hz": self._hz(now),
        }

    def _hz(self, now: float) -> float:
        recent = [stamp for stamp in self.stamps if now - stamp <= 2.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])


@dataclass(frozen=True)
class TopicConfig:
    odom: str = "/odometry/filtered"
    dvl_twist: str = "/dvl/twist"
    dvl_command_response: str = "/dvl/command/response"
    dvl_config_status: str = "/dvl/config/status"
    dvl_config_command: str = "/dvl/config/command"
    depth: str = "/depth/pose"
    battery: str = "/battery"
    imu: str = "/mavros/imu/data"
    joy: str = "/joy"
    mavros_state: str = "/mavros/state"
    yolo: str = "/vision/buoy/status"
    pinger_homing_status: str = "/pinger_homing/status"
    hydrophone_direction: str = "/homing/direction"
    mission_status_json: str = "/tmp/kmu26_mission_fsm_status.json"


class LocalizationRosNode(Node):
    def __init__(self, topic_config: TopicConfig | None = None):
        super().__init__("kmu26_auv_web_gui_bridge")
        self._topic_config = topic_config or TopicConfig()
        self._lock = threading.Lock()
        self._health = {
            "odom": TopicHealth(self._topic_config.odom),
            "dvl": TopicHealth(self._topic_config.dvl_twist),
            "depth": TopicHealth(self._topic_config.depth),
            "imu": TopicHealth(self._topic_config.imu),
            "joy": TopicHealth(self._topic_config.joy, stale_after=0.5),
            "battery": TopicHealth(self._topic_config.battery, stale_after=3.0),
            "mavros_state": TopicHealth(self._topic_config.mavros_state, stale_after=1.5),
            "yolo": TopicHealth(self._topic_config.yolo, stale_after=1.5),
            "pinger_homing": TopicHealth(self._topic_config.pinger_homing_status, stale_after=1.5),
            "hydrophone_direction": TopicHealth(self._topic_config.hydrophone_direction, stale_after=1.5),
        }
        self._pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self._velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._depth = {"z": 0.0}
        self._mavros_state = {
            "connected": False,
            "armed": False,
            "guided": False,
            "manual_input": False,
            "mode": "",
            "system_status": 0,
        }
        self._battery = {
            "voltage": None,
            "current": None,
            "temperature": None,
            "percentage": None,
            "present": False,
        }
        self._joy = {"axes": [], "buttons": []}
        self._yolo_status = {
            "raw": "",
            "active": False,
            "count": 0,
            "target": "",
            "range_m": None,
        }
        self._pinger_homing_status = {
            "raw": "",
            "state": "",
            "dry_run": True,
            "control_output_active": False,
            "estimated_distance_m": None,
            "amplitude_distance_m": None,
            "bearing_error_deg": None,
        }
        self._hydrophone_direction = {"x": None, "y": None, "z": None, "bearing_rad": None}
        self._dvl_config: dict = {}
        self._dvl_events: deque[dict] = deque(maxlen=40)
        self._path: deque[dict[str, float]] = deque(maxlen=1200)

        self._dvl_config_pub = self.create_publisher(
            ConfigCommand,
            self._topic_config.dvl_config_command,
            10,
        )
        self.create_subscription(Odometry, self._topic_config.odom, self._on_odom, 20)
        self.create_subscription(TwistWithCovarianceStamped, self._topic_config.dvl_twist, self._on_dvl, 20)
        self.create_subscription(
            CommandResponse,
            self._topic_config.dvl_command_response,
            self._on_dvl_response,
            20,
        )
        self.create_subscription(ConfigStatus, self._topic_config.dvl_config_status, self._on_dvl_config, 20)
        self.create_subscription(PoseWithCovarianceStamped, self._topic_config.depth, self._on_depth, 20)
        self.create_subscription(BatteryState, self._topic_config.battery, self._on_battery, 20)
        self.create_subscription(Imu, self._topic_config.imu, self._on_imu, 20)
        self.create_subscription(Joy, self._topic_config.joy, self._on_joy, 20)
        self.create_subscription(State, self._topic_config.mavros_state, self._on_mavros_state, 20)
        self.create_subscription(String, self._topic_config.yolo, self._on_yolo, 20)
        self.create_subscription(String, self._topic_config.pinger_homing_status, self._on_pinger_homing, 20)
        self.create_subscription(
            Vector3Stamped,
            self._topic_config.hydrophone_direction,
            self._on_hydrophone_direction,
            20,
        )

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        msg = ConfigCommand()
        msg.command = command
        msg.parameter_name = parameter_name
        msg.parameter_value = parameter_value
        self._dvl_config_pub.publish(msg)
        self._append_dvl_event(
            "sent",
            command,
            parameter_name,
            parameter_value,
            True,
            "",
        )

    def snapshot(self) -> dict:
        mission_status = self._read_mission_status()
        with self._lock:
            return {
                "config": {
                    "topics": {
                        "odom": self._topic_config.odom,
                        "mavros_state": self._topic_config.mavros_state,
                        "yolo": self._topic_config.yolo,
                        "pinger_homing_status": self._topic_config.pinger_homing_status,
                        "hydrophone_direction": self._topic_config.hydrophone_direction,
                    },
                    "mission_status_json": self._topic_config.mission_status_json,
                },
                "topics": {name: item.snapshot() for name, item in self._health.items()},
                "pose": dict(self._pose),
                "velocity": dict(self._velocity),
                "depth": dict(self._depth),
                "mavros_state": dict(self._mavros_state),
                "battery": dict(self._battery),
                "joy": {
                    "axes": list(self._joy["axes"]),
                    "buttons": list(self._joy["buttons"]),
                },
                "mission_status": mission_status,
                "yolo_status": dict(self._yolo_status),
                "pinger_homing_status": dict(self._pinger_homing_status),
                "hydrophone_direction": dict(self._hydrophone_direction),
                "dvl_config": dict(self._dvl_config),
                "dvl_events": list(self._dvl_events),
                "path": list(self._path),
            }

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._health["odom"].tick()
            self._pose = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": yaw,
            }
            self._path.append({"x": pose.position.x, "y": pose.position.y})

    def _on_dvl(self, msg: TwistWithCovarianceStamped) -> None:
        linear = msg.twist.twist.linear
        with self._lock:
            self._health["dvl"].tick()
            self._velocity = {"x": linear.x, "y": linear.y, "z": linear.z}

    def _on_dvl_response(self, msg: CommandResponse) -> None:
        self._append_dvl_event(
            "response",
            msg.response_to,
            "",
            str(msg.result),
            msg.success,
            msg.error_message,
        )

    def _on_dvl_config(self, msg: ConfigStatus) -> None:
        with self._lock:
            self._dvl_config = {
                "updated_at": time.strftime("%H:%M:%S"),
                "response_to": msg.response_to,
                "success": msg.success,
                "error_message": msg.error_message,
                "speed_of_sound": msg.speed_of_sound,
                "acoustic_enabled": msg.acoustic_enabled,
                "dark_mode_enabled": msg.dark_mode_enabled,
                "mounting_rotation_offset": msg.mounting_rotation_offset,
                "range_mode": msg.range_mode,
                "format": msg.format,
                "type": msg.type,
            }
        self._append_dvl_event(
            "config",
            msg.response_to,
            "",
            msg.range_mode,
            msg.success,
            msg.error_message,
        )

    def _on_depth(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._health["depth"].tick()
            self._depth = {"z": msg.pose.pose.position.z}

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._health["battery"].tick()
            self._battery = {
                "voltage": _finite_or_none(msg.voltage),
                "current": _finite_or_none(msg.current),
                "temperature": _finite_or_none(msg.temperature),
                "percentage": _finite_or_none(msg.percentage),
                "present": bool(msg.present),
            }

    def _on_imu(self, msg: Imu) -> None:
        del msg
        with self._lock:
            self._health["imu"].tick()

    def _on_mavros_state(self, msg: State) -> None:
        with self._lock:
            self._health["mavros_state"].tick()
            self._mavros_state = {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "guided": bool(msg.guided),
                "manual_input": bool(msg.manual_input),
                "mode": msg.mode,
                "system_status": int(msg.system_status),
            }

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._health["joy"].tick()
            self._joy = {
                "axes": [round(value, 3) for value in msg.axes],
                "buttons": list(msg.buttons),
            }

    def _on_yolo(self, msg: String) -> None:
        parsed = _json_object(msg.data)
        with self._lock:
            self._health["yolo"].tick()
            self._yolo_status = {
                "raw": msg.data[-600:],
                "active": bool(parsed.get("active", False)) if parsed else False,
                "count": int(parsed.get("count", 0)) if isinstance(parsed.get("count", 0), (int, float)) else 0,
                "target": str(parsed.get("target", parsed.get("target_id", ""))) if parsed else "",
                "range_m": _number_or_none(parsed.get("range_m")) if parsed else None,
            }

    def _on_pinger_homing(self, msg: String) -> None:
        parsed = _json_object(msg.data)
        with self._lock:
            self._health["pinger_homing"].tick()
            self._pinger_homing_status = {
                "raw": msg.data[-1600:],
                "state": str(parsed.get("state", "")) if parsed else "",
                "dry_run": bool(parsed.get("dry_run", True)) if parsed else True,
                "control_output_active": (
                    bool(parsed.get("control_output_active", False)) if parsed else False
                ),
                "connected": bool(parsed.get("connected", False)) if parsed else False,
                "armed": bool(parsed.get("armed", False)) if parsed else False,
                "audio_fresh": bool(parsed.get("audio_fresh", False)) if parsed else False,
                "sample_count": _integer_or_zero(parsed.get("sample_count")) if parsed else 0,
                "probe_attempt": _integer_or_zero(parsed.get("probe_attempt")) if parsed else 0,
                "minimum_probe_legs": (
                    _integer_or_zero(parsed.get("minimum_probe_legs")) if parsed else 0
                ),
                "estimated_source_world": (
                    parsed.get("estimated_source_world") if parsed else None
                ),
                "source_locked": bool(parsed.get("source_locked", False)) if parsed else False,
                "estimated_distance_m": (
                    _number_or_none(parsed.get("estimated_distance_m")) if parsed else None
                ),
                "amplitude_distance_m": (
                    _number_or_none(parsed.get("amplitude_distance_m")) if parsed else None
                ),
                "rms_residual_m": (
                    _number_or_none(parsed.get("rms_residual_m")) if parsed else None
                ),
                "condition_number": (
                    _number_or_none(parsed.get("condition_number")) if parsed else None
                ),
                "bias_range_rate_mps": (
                    _number_or_none(parsed.get("bias_range_rate_mps")) if parsed else None
                ),
                "control_direction_source": (
                    str(parsed.get("control_direction_source", "")) if parsed else ""
                ),
                "command": parsed.get("command", {}) if parsed else {},
                "requested_command": parsed.get("requested_command", {}) if parsed else {},
                "depth_safety": parsed.get("depth_safety", {}) if parsed else {},
                "bearing_error_deg": (
                    _number_or_none(parsed.get("bearing_error_deg")) if parsed else None
                ),
                "capture_confirmed": (
                    bool(parsed.get("capture_confirmed", False)) if parsed else False
                ),
                "range_complete": bool(parsed.get("range_complete", False)) if parsed else False,
                "amplitude_range_constant": (
                    _number_or_none(parsed.get("amplitude_range_constant")) if parsed else None
                ),
            }

    def _on_hydrophone_direction(self, msg: Vector3Stamped) -> None:
        x = msg.vector.x
        y = msg.vector.y
        z = msg.vector.z
        bearing = math.atan2(y, x) if math.isfinite(x) and math.isfinite(y) else None
        with self._lock:
            self._health["hydrophone_direction"].tick()
            self._hydrophone_direction = {
                "x": _finite_or_none(x),
                "y": _finite_or_none(y),
                "z": _finite_or_none(z),
                "bearing_rad": bearing,
            }

    def _read_mission_status(self) -> dict:
        path = Path(self._topic_config.mission_status_json)
        if not path.exists():
            return {"available": False, "path": str(path), "age": None}
        try:
            stat = path.stat()
            text = path.read_text(encoding="utf-8")
            data = json.loads(text)
        except Exception as exc:
            return {
                "available": False,
                "path": str(path),
                "age": None,
                "error": str(exc),
            }
        if not isinstance(data, dict):
            return {"available": False, "path": str(path), "age": None, "error": "status JSON is not an object"}
        data = dict(data)
        data["available"] = True
        data["path"] = str(path)
        data["age"] = max(0.0, time.time() - stat.st_mtime)
        return data

    def _append_dvl_event(
        self,
        event_type: str,
        command: str,
        parameter_name: str,
        parameter_value: str,
        success: bool,
        error_message: str,
    ) -> None:
        with self._lock:
            self._dvl_events.append(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "type": event_type,
                    "command": command,
                    "parameter_name": parameter_name,
                    "parameter_value": parameter_value,
                    "success": success,
                    "error_message": error_message,
                }
            )


class RosInterface:
    def __init__(self, topic_config: TopicConfig | None = None):
        self.topic_config = topic_config or TopicConfig()
        self.node: LocalizationRosNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None

    def start(self) -> None:
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = LocalizationRosNode(self.topic_config)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        if self._executor is None:
            return
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if rclpy.ok():
            rclpy.shutdown()

    def status(self) -> dict:
        if self.node is None:
            return {}
        return self.node.snapshot()

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.publish_dvl_command(command, parameter_name, parameter_value)

    def reset_dvl_dead_reckoning(self) -> None:
        self.publish_dvl_command("reset_dead_reckoning")


def _json_object(text: str) -> dict:
    try:
        data = json.loads(text)
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _number_or_none(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return _finite_or_none(number)


def _integer_or_zero(value) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0
