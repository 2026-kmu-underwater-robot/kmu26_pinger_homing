#!/usr/bin/env python3
"""ROS 2 YOLO buoy detector for the vehicle compressed camera stream."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3Stamped
from hit25_auv_ros2_msg.msg import BuoyObservation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, String

from buoy_vision_core import (
    PerClassTemporalTracker,
    TemporalTargetTracker,
    VisionDetection,
    bearing_error_rad,
    detections_from_ultralytics,
    draw_overlay,
    filter_by_max_center_y_ratio,
    filter_target_geometry,
    preprocess_underwater_bgr,
    select_detection_by_bearing,
    select_target,
    synchronize_detection_to_frame,
    vision_state,
)


@dataclass
class PendingInference:
    detections: list[VisionDetection]
    candidates: list[VisionDetection]
    elapsed_ms: float
    error_text: str
    source_gray: np.ndarray
    captured_monotonic: float
    source_sequence: int


class YoloBuoyDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_buoy_detector")
        self._declare_parameters()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.bbox_topic = str(self.get_parameter("bbox_topic").value)
        self.observation_topic = str(self.get_parameter("observation_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.model_path = self._resolve_model_path(str(self.get_parameter("model_path").value))
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip()
        self.pinger_target_class_names = {
            str(name).strip().lower()
            for name in self.get_parameter("pinger_target_class_names").value
            if str(name).strip()
        }
        self.underwater_target_class_names = {
            str(name).strip().lower()
            for name in self.get_parameter("underwater_target_class_names").value
            if str(name).strip()
        }
        self.alignment_preferred_class_name = str(
            self.get_parameter("alignment_preferred_class_name").value
        ).strip().lower()
        self.publish_per_class = bool(self.get_parameter("publish_per_class").value)
        self.confidence = float(self.get_parameter("confidence_threshold").value)
        self.iou = float(self.get_parameter("iou_threshold").value)
        self.device = self._resolve_device(str(self.get_parameter("device").value))
        self.cpu_threads = max(1, int(self.get_parameter("cpu_threads").value))
        self.cpu_affinity_cores = max(
            self.cpu_threads, int(self.get_parameter("cpu_affinity_cores").value)
        )
        self.limit_cpu_affinity = bool(self.get_parameter("limit_cpu_affinity").value)
        self.cpu_affinity: list[int] = []
        self._configure_cpu_threads()
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.inference_hz = max(0.2, float(self.get_parameter("inference_hz").value))
        self.selection_policy = str(self.get_parameter("selection_policy").value)
        self.min_vertical_aspect = float(self.get_parameter("min_vertical_aspect").value)
        self.min_mask_area_ratio = float(self.get_parameter("min_mask_area_ratio").value)
        self.max_mask_area_ratio = float(self.get_parameter("max_mask_area_ratio").value)
        self.preprocess_enabled = bool(self.get_parameter("preprocess_enabled").value)
        self.white_balance = bool(self.get_parameter("white_balance").value)
        self.clahe_clip = float(self.get_parameter("clahe_clip").value)
        self.clahe_grid = int(self.get_parameter("clahe_grid").value)
        self.center_tolerance = float(self.get_parameter("center_tolerance").value)
        self.fine_height_ratio = float(self.get_parameter("fine_height_ratio").value)
        self.capture_height_ratio = float(self.get_parameter("capture_height_ratio").value)
        self.surface_max_center_y_ratio = float(
            self.get_parameter("surface_max_center_y_ratio").value
        )
        self.underwater_max_center_y_ratio = float(
            self.get_parameter("underwater_max_center_y_ratio").value
        )
        self.hfov_rad = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.jpeg_quality = int(np.clip(self.get_parameter("jpeg_quality").value, 30, 100))
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.preview_window_name = str(self.get_parameter("preview_window_name").value)
        self.mission_gated = bool(self.get_parameter("mission_gated").value)
        self.mission_inference_enabled = not self.mission_gated
        self.mission_state = ""
        self.previous_mission_state = ""
        self.locked_target_center: Optional[tuple[float, float]] = None
        self.pinger_max_abs_error_y = float(
            self.get_parameter("pinger_max_abs_error_y").value
        )
        self.pinger_bearing_tolerance_rad = float(
            self.get_parameter("pinger_bearing_tolerance_rad").value
        )
        self.pinger_lock_max_diagonal_ratio = float(
            self.get_parameter("pinger_lock_max_diagonal_ratio").value
        )
        self.hydrophone_timeout_s = float(self.get_parameter("hydrophone_timeout_s").value)
        self.hydrophone_bearing_rad: Optional[float] = None
        self.hydrophone_received_monotonic = -1.0
        cv2.setNumThreads(1)

        self.model = self._load_model(self.model_path)
        self.class_names = getattr(self.model, "names", {}) or {}
        self.track_hold_seconds = float(self.get_parameter("track_hold_seconds").value)
        self.max_inference_result_age_s = float(
            self.get_parameter("max_inference_result_age_s").value
        )
        self.tracker = TemporalTargetTracker(hold_seconds=self.track_hold_seconds)
        self.class_tracker = PerClassTemporalTracker(
            hold_seconds=self.track_hold_seconds,
            association_max_diagonal_ratio=float(
                self.get_parameter("class_association_max_diagonal_ratio").value
            ),
        )
        self.last_inference_monotonic = -1.0
        self.last_inference_ms = 0.0
        self.last_detections: list[VisionDetection] = []
        self.current_class_detections: list[VisionDetection] = []
        self.last_candidate_count = 0
        self.last_raw_detection_count = 0
        self.last_result_age_ms = 0.0
        self.last_result_frame_lag = 0
        self.dropped_stale_results = 0
        self.last_pinger_bearing_error_rad: Optional[float] = None
        self.pinger_missing_inferences = 0
        self.inference_lock = threading.Lock()
        self.inference_active = False
        self.inference_thread: Optional[threading.Thread] = None
        self.pending_inference: Optional[PendingInference] = None
        self.last_state = "SEARCH"
        self.frame_sequence = 0

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_sub = self.create_subscription(
            CompressedImage, self.image_topic, self.on_image, image_qos
        )
        self.mission_status_sub = self.create_subscription(
            String,
            str(self.get_parameter("mission_status_topic").value),
            self._on_mission_status,
            10,
        )
        self.hydrophone_sub = self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("hydrophone_body_topic").value),
            self._on_hydrophone_direction,
            10,
        )
        self.bbox_pub = self.create_publisher(Float32MultiArray, self.bbox_topic, 10)
        self.observation_pub = self.create_publisher(BuoyObservation, self.observation_topic, 10)
        self.annotated_pub = self.create_publisher(CompressedImage, self.annotated_topic, image_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.get_logger().info(
            f"YOLO ready model={self.model_path} task={getattr(self.model, 'task', '')} "
            f"classes={self.class_names} "
            f"device={self.device} imgsz={self.imgsz} inference={self.inference_hz:.1f}Hz"
        )
        if self.cpu_affinity:
            self.get_logger().info(
                f"CPU affinity={self.cpu_affinity} torch_threads={self.cpu_threads}"
            )
        self.get_logger().info(
            f"camera={self.image_topic} observation={self.observation_topic} "
            f"annotated={self.annotated_topic} status={self.status_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw/compressed")
        self.declare_parameter("bbox_topic", "/vision/buoy_bbox")
        self.declare_parameter("observation_topic", "/vision/buoy_observation")
        self.declare_parameter("annotated_topic", "/vision/buoy/image_annotated/compressed")
        self.declare_parameter("status_topic", "/vision/buoy/status")
        self.declare_parameter("model_path", "")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "buoy")
        self.declare_parameter("pinger_target_class_names", ["buoy", "stick"])
        self.declare_parameter("underwater_target_class_names", ["stick", "buoy"])
        self.declare_parameter("alignment_preferred_class_name", "stick")
        self.declare_parameter("publish_per_class", True)
        self.declare_parameter("confidence_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "auto")
        self.declare_parameter("cpu_threads", 2)
        self.declare_parameter("cpu_affinity_cores", 3)
        self.declare_parameter("limit_cpu_affinity", False)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("inference_hz", 3.0)
        self.declare_parameter("selection_policy", "largest_mask")
        self.declare_parameter("min_vertical_aspect", 0.50)
        self.declare_parameter("min_mask_area_ratio", 0.0001)
        self.declare_parameter("max_mask_area_ratio", 0.20)
        self.declare_parameter("track_hold_seconds", 1.60)
        self.declare_parameter("max_inference_result_age_s", 2.00)
        self.declare_parameter("class_association_max_diagonal_ratio", 0.025)
        self.declare_parameter("preprocess_enabled", True)
        self.declare_parameter("white_balance", True)
        self.declare_parameter("clahe_clip", 2.0)
        self.declare_parameter("clahe_grid", 8)
        self.declare_parameter("center_tolerance", 0.10)
        self.declare_parameter("fine_height_ratio", 0.18)
        self.declare_parameter("capture_height_ratio", 0.40)
        self.declare_parameter("surface_max_center_y_ratio", 0.65)
        self.declare_parameter("underwater_max_center_y_ratio", 0.72)
        self.declare_parameter("horizontal_fov_deg", 69.4)
        self.declare_parameter("jpeg_quality", 88)
        self.declare_parameter("show_preview", False)
        self.declare_parameter("preview_window_name", "KMU26 Buoy Vision")
        self.declare_parameter("mission_gated", False)
        self.declare_parameter("mission_status_topic", "/mission/fsm/status")
        self.declare_parameter("pinger_max_abs_error_y", 0.45)
        self.declare_parameter("pinger_bearing_tolerance_rad", 0.45)
        self.declare_parameter("pinger_lock_max_diagonal_ratio", 0.22)
        self.declare_parameter("hydrophone_body_topic", "/mission/hydrophone/direction_body")
        self.declare_parameter("hydrophone_timeout_s", 5.0)

    def _on_mission_status(self, message: String) -> None:
        try:
            state = str(json.loads(message.data).get("state", ""))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.previous_mission_state = self.mission_state
        self.mission_state = state
        if state in {"PINGER_SEARCH", "BUOY_SEARCHING", "SURFACE_BUOY_SEARCHING"} and state != self.previous_mission_state:
            self.locked_target_center = None
            self.tracker.reset()
            self.class_tracker.reset()
            self.pinger_missing_inferences = 0
        if self.mission_gated:
            self.mission_inference_enabled = state not in {
                "",
                "IDLE",
                "WAIT_VEHICLE",
                "WAIT_ARM",
                "PINGER_SEARCH",
                "PINGER_HOMING",
                "COMPLETE",
                "FAILED",
            }

    def _on_hydrophone_direction(self, message: Vector3Stamped) -> None:
        x = float(message.vector.x)
        y = float(message.vector.y)
        if not math.isfinite(x) or not math.isfinite(y) or math.hypot(x, y) < 1.0e-6:
            return
        self.hydrophone_bearing_rad = -math.atan2(y, x)
        self.hydrophone_received_monotonic = time.monotonic()

    def _load_model(self, model_path: str) -> Any:
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"YOLO model not found: {model_path}")
        ultralytics = importlib.import_module("ultralytics")
        model = ultralytics.YOLO(model_path)
        task = str(getattr(model, "task", ""))
        if task != "segment":
            self.get_logger().warning(
                f"model task is '{task}'; segmentation masks will be unavailable"
            )
        return model

    def _configure_cpu_threads(self) -> None:
        if not self.device.lower().startswith("cpu"):
            return
        if self.limit_cpu_affinity and hasattr(os, "sched_getaffinity"):
            try:
                available = sorted(os.sched_getaffinity(0))
                if len(available) > self.cpu_affinity_cores:
                    os.sched_setaffinity(0, set(available[: self.cpu_affinity_cores]))
                self.cpu_affinity = sorted(os.sched_getaffinity(0))
            except OSError as error:
                self.get_logger().warning(f"CPU affinity limit failed: {error}")
        try:
            cv2.setNumThreads(1)
        except Exception:
            pass
        torch = importlib.import_module("torch")
        torch.set_num_threads(self.cpu_threads)
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            # PyTorch only permits setting the inter-op pool before parallel work starts.
            pass

    @staticmethod
    def _resolve_device(device: str) -> str:
        requested = device.strip()
        if requested and requested.lower() != "auto":
            return requested
        try:
            torch = importlib.import_module("torch")
            return "cuda:0" if torch.cuda.is_available() else "cpu"
        except ImportError:
            return "cpu"

    @staticmethod
    def _resolve_model_path(configured: str) -> str:
        candidates: list[Path] = []
        explicit = configured.strip() or os.getenv("KMU26_BUOY_MODEL", "").strip()
        if explicit:
            candidates.append(Path(explicit).expanduser())
        try:
            candidates.append(
                Path(get_package_share_directory("kmu26_vision_mission_fsm")) / "models" / "best.pt"
            )
        except Exception:
            pass
        here = Path(__file__).resolve()
        candidates.extend(parent / "YOLO" / "best.pt" for parent in here.parents)
        candidates.append(Path.cwd() / "YOLO" / "best.pt")
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate.resolve())
        attempted = ", ".join(str(candidate) for candidate in candidates)
        raise FileNotFoundError(f"best.pt not found; checked: {attempted}")

    def on_image(self, message: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("compressed camera frame decode failed", throttle_duration_sec=2.0)
            return
        self.frame_sequence += 1
        now = time.monotonic()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        selected, inference_applied = self._consume_inference(image, gray, now)
        self._start_inference_if_due(image, gray, now)
        source = "yolo_synced" if inference_applied else "optical_flow"
        state = vision_state(
            selected,
            image.shape[:2],
            center_tolerance=self.center_tolerance,
            fine_height_ratio=self.fine_height_ratio,
            capture_height_ratio=self.capture_height_ratio,
        )
        self.last_state = state
        self._publish_observation(message, selected, image.shape[:2])
        if self.publish_per_class:
            self._publish_bbox_candidates(
                message, self.current_class_detections, image.shape[:2]
            )
        else:
            self._publish_bbox(message, selected, image.shape[:2])
        self._publish_status(message, selected, image.shape[:2], state, source)
        annotated = draw_overlay(
            image,
            self.current_class_detections,
            selected,
            state=state,
            inference_ms=self.last_inference_ms,
            pinger_mode=self.mission_state.startswith("PINGER_"),
            pinger_locked=self.locked_target_center is not None,
        )
        self._publish_annotated(message, annotated)
        if self.show_preview:
            cv2.imshow(self.preview_window_name, annotated)
            cv2.waitKey(1)

    def _start_inference_if_due(
        self, image: np.ndarray, gray: np.ndarray, now: float
    ) -> None:
        if not self.mission_inference_enabled:
            return
        with self.inference_lock:
            if self.inference_active:
                return
            if (
                self.last_inference_monotonic >= 0.0
                and now - self.last_inference_monotonic < 1.0 / self.inference_hz
            ):
                return
            self.inference_active = True
            self.last_inference_monotonic = now
        self.inference_thread = threading.Thread(
            target=self._inference_worker,
            args=(image.copy(), gray.copy(), now, self.frame_sequence),
            name="kmu26-yolo-inference",
            daemon=True,
        )
        self.inference_thread.start()

    def _inference_worker(
        self,
        image: np.ndarray,
        source_gray: np.ndarray,
        captured_monotonic: float,
        source_sequence: int,
    ) -> None:
        model_input = (
            preprocess_underwater_bgr(
                image,
                white_balance=self.white_balance,
                clahe_clip=self.clahe_clip,
                clahe_grid=self.clahe_grid,
            )
            if self.preprocess_enabled
            else image
        )
        started = time.perf_counter()
        detections: list[VisionDetection] = []
        candidates: list[VisionDetection] = []
        error_text = ""
        try:
            results = self.model.predict(
                source=model_input,
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
                retina_masks=True,
            )
            detections = (
                detections_from_ultralytics(results[0], image.shape[:2], self.class_names)
                if results
                else []
            )
            candidates = filter_target_geometry(
                detections,
                image.shape[:2],
                min_vertical_aspect=self.min_vertical_aspect,
                min_area_ratio=self.min_mask_area_ratio,
                max_area_ratio=self.max_mask_area_ratio,
            )
        except Exception as error:
            error_text = str(error)
        finally:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            with self.inference_lock:
                self.pending_inference = PendingInference(
                    detections=detections,
                    candidates=candidates,
                    elapsed_ms=elapsed_ms,
                    error_text=error_text,
                    source_gray=source_gray,
                    captured_monotonic=captured_monotonic,
                    source_sequence=source_sequence,
                )
                self.inference_active = False

    def _consume_inference(
        self, image: np.ndarray, gray: np.ndarray, now: float
    ) -> tuple[Optional[VisionDetection], bool]:
        with self.inference_lock:
            pending = self.pending_inference
            self.pending_inference = None
        if pending is None:
            self.current_class_detections = self.class_tracker.propagate(gray, now)
            self.last_detections = self.current_class_detections
            if self.mission_state.startswith("PINGER_"):
                propagated = self.tracker.propagate(gray, now)
                if not self._pinger_bearing_matches(propagated, image.shape[1], now):
                    self.locked_target_center = None
                    self.tracker.reset()
                    propagated = None
                return propagated, False
            selected = self._select_current_class_target(
                self.current_class_detections, image.shape[:2]
            )
            self.locked_target_center = selected.center if selected is not None else None
            return selected, False

        result_age_s = max(0.0, now - pending.captured_monotonic)
        self.last_result_age_ms = result_age_s * 1000.0
        self.last_result_frame_lag = max(0, self.frame_sequence - pending.source_sequence)
        self.last_inference_ms = pending.elapsed_ms
        self.last_raw_detection_count = len(pending.detections)
        self.last_candidate_count = len(pending.candidates)
        if pending.error_text:
            self.get_logger().error(
                f"YOLO inference failed: {pending.error_text}", throttle_duration_sec=2.0
            )
        if result_age_s > self.max_inference_result_age_s:
            self.dropped_stale_results += 1
            self.get_logger().warning(
                f"dropping stale YOLO result age={result_age_s:.3f}s "
                f"frame_lag={self.last_result_frame_lag}",
                throttle_duration_sec=2.0,
            )
            self.current_class_detections = self.class_tracker.propagate(gray, now)
            self.last_detections = self.current_class_detections
            if self.mission_state.startswith("PINGER_"):
                return self.tracker.propagate(gray, now), False
            selected = self._select_current_class_target(
                self.current_class_detections, image.shape[:2]
            )
            self.locked_target_center = selected.center if selected is not None else None
            return selected, False

        candidates = [
            synchronize_detection_to_frame(candidate, pending.source_gray, gray)
            for candidate in pending.candidates
        ]
        self.current_class_detections = self.class_tracker.correct(candidates, gray, now)
        self.last_detections = self.current_class_detections
        if not self.mission_state.startswith("PINGER_"):
            selected = self._select_current_class_target(
                self.current_class_detections, image.shape[:2]
            )
            self.locked_target_center = selected.center if selected is not None else None
            return selected, True

        raw_selected = self._select_acoustic_pinger(candidates, image.shape[:2])
        if raw_selected is None and self.locked_target_center is not None:
            self.pinger_missing_inferences += 1
        else:
            self.pinger_missing_inferences = 0
        if self.pinger_missing_inferences >= 3:
            self.locked_target_center = None
            self.tracker.reset()
            self.pinger_missing_inferences = 0
        selected = (
            self.tracker.correct(raw_selected, gray, now)
            if raw_selected is not None
            else self.tracker.propagate(gray, now)
        )
        if not self._pinger_bearing_matches(selected, image.shape[1], now):
            selected = None
            self.locked_target_center = None
            self.tracker.reset()
        if selected is not None:
            self.locked_target_center = selected.center
        return selected, True

    def _select_current_class_target(
        self,
        candidates: list[VisionDetection],
        image_shape: tuple[int, int],
    ) -> Optional[VisionDetection]:
        """Select control output from the already synchronized class tracks."""

        if self.mission_state.startswith("SURFACE_"):
            return self._select_surface_candidate(candidates, image_shape)
        if self.mission_state in {
            "BUOY_SEARCHING",
            "BUOY_DETECTED_APPROACH",
            "BUOY_FINE_ALIGN",
            "CAPTURE",
            "BUOY_HEAD_ON_ALIGN",
            "BUOY_HEAD_ON_INSERT",
            "VERIFY_CAPTURE",
            "BUOY_BACKOFF",
        }:
            return self._select_underwater_candidate(candidates, image_shape)
        eligible = [
            candidate
            for candidate in candidates
            if (self.target_class_id < 0 or candidate.class_id == self.target_class_id)
            and (
                not self.target_class_name.strip()
                or candidate.label.strip().lower()
                == self.target_class_name.strip().lower()
            )
        ]
        if self.locked_target_center is not None:
            return self._select_locked_candidate(
                eligible,
                image_shape,
                max_diagonal_ratio=self.class_tracker.association_max_diagonal_ratio,
            )
        return select_target(
            eligible,
            target_class_id=-1,
            target_class_name="",
            policy=self.selection_policy,
        )

    def _select_surface_candidate(
        self, candidates: list[VisionDetection], image_shape: tuple[int, int]
    ) -> Optional[VisionDetection]:
        surface_candidates = filter_by_max_center_y_ratio(
            candidates, image_shape, self.surface_max_center_y_ratio
        )
        if not surface_candidates:
            return None
        if self.mission_state == "SURFACE_COLLECT" and self.locked_target_center is not None:
            return self._select_locked_candidate(surface_candidates, image_shape)
        return select_target(
            surface_candidates,
            target_class_id=self.target_class_id,
            target_class_name=self.target_class_name,
            policy=self.selection_policy,
        )

    def _select_underwater_candidate(
        self, candidates: list[VisionDetection], image_shape: tuple[int, int]
    ) -> Optional[VisionDetection]:
        water_column_candidates = filter_by_max_center_y_ratio(
            candidates, image_shape, self.underwater_max_center_y_ratio
        )
        if self.underwater_target_class_names:
            water_column_candidates = [
                candidate
                for candidate in water_column_candidates
                if candidate.label.strip().lower() in self.underwater_target_class_names
            ]
        if not water_column_candidates:
            return None
        preferred = [
            candidate
            for candidate in water_column_candidates
            if candidate.label.strip().lower() == self.alignment_preferred_class_name
        ]
        if self.locked_target_center is not None:
            locked = self._select_locked_candidate(preferred, image_shape)
            if locked is not None:
                return locked
            locked = self._select_locked_candidate(water_column_candidates, image_shape)
            if locked is not None:
                return locked
            return None
        return select_target(
            preferred or water_column_candidates,
            target_class_id=-1,
            target_class_name="",
            policy=self.selection_policy,
        )

    def _select_locked_candidate(
        self,
        candidates: list[VisionDetection],
        image_shape: tuple[int, int],
        max_diagonal_ratio: float = 0.28,
    ) -> Optional[VisionDetection]:
        if not candidates or self.locked_target_center is None:
            return None
        height, width = image_shape
        diagonal = max(1.0, math.hypot(width, height))
        selected = min(
            candidates,
            key=lambda item: math.hypot(
                item.center[0] - self.locked_target_center[0],
                item.center[1] - self.locked_target_center[1],
            ),
        )
        distance = math.hypot(
            selected.center[0] - self.locked_target_center[0],
            selected.center[1] - self.locked_target_center[1],
        ) / diagonal
        return selected if distance <= max_diagonal_ratio else None

    def _select_acoustic_pinger(
        self, candidates: list[VisionDetection], image_shape: tuple[int, int]
    ) -> Optional[VisionDetection]:
        now = time.monotonic()
        self.last_pinger_bearing_error_rad = None
        if (
            self.hydrophone_bearing_rad is None
            or self.hydrophone_received_monotonic < 0.0
            or now - self.hydrophone_received_monotonic > self.hydrophone_timeout_s
        ):
            return None
        eligible: list[VisionDetection] = []
        height, width = image_shape
        for candidate in candidates:
            if self.target_class_id >= 0 and candidate.class_id != self.target_class_id:
                continue
            if (
                self.target_class_id < 0
                and self.pinger_target_class_names
                and candidate.label.strip().lower() not in self.pinger_target_class_names
            ):
                continue
            error_y = (2.0 * candidate.center[1] / max(height, 1)) - 1.0
            if abs(error_y) > self.pinger_max_abs_error_y:
                continue
            eligible.append(candidate)
        preferred = [
            candidate
            for candidate in eligible
            if candidate.label.strip().lower() == self.alignment_preferred_class_name
        ]
        selected = select_detection_by_bearing(
            preferred or eligible,
            image_shape,
            horizontal_fov_rad=self.hfov_rad,
            expected_bearing_rad=self.hydrophone_bearing_rad,
            tolerance_rad=self.pinger_bearing_tolerance_rad,
            locked_center=self.locked_target_center,
            lock_max_diagonal_ratio=self.pinger_lock_max_diagonal_ratio,
        )
        if selected is None and preferred:
            selected = select_detection_by_bearing(
                eligible,
                image_shape,
                horizontal_fov_rad=self.hfov_rad,
                expected_bearing_rad=self.hydrophone_bearing_rad,
                tolerance_rad=self.pinger_bearing_tolerance_rad,
                locked_center=self.locked_target_center,
                lock_max_diagonal_ratio=self.pinger_lock_max_diagonal_ratio,
            )
        if selected is not None:
            self.last_pinger_bearing_error_rad = bearing_error_rad(
                selected,
                width,
                self.hfov_rad,
                self.hydrophone_bearing_rad,
            )
        return selected

    def _pinger_bearing_error(
        self, candidate: VisionDetection, image_width: int, now: float
    ) -> float:
        if (
            self.hydrophone_bearing_rad is None
            or self.hydrophone_received_monotonic < 0.0
            or now - self.hydrophone_received_monotonic > self.hydrophone_timeout_s
        ):
            return math.inf
        error_x = (2.0 * candidate.center[0] / max(image_width, 1)) - 1.0
        candidate_bearing = error_x * self.hfov_rad * 0.5
        delta = candidate_bearing - self.hydrophone_bearing_rad
        return abs(math.atan2(math.sin(delta), math.cos(delta)))

    def _pinger_bearing_matches(
        self, candidate: Optional[VisionDetection], image_width: int, now: float
    ) -> bool:
        return candidate is not None and self._pinger_bearing_error(
            candidate, image_width, now
        ) <= self.pinger_bearing_tolerance_rad

    def _publish_observation(
        self,
        source: CompressedImage,
        target: Optional[VisionDetection],
        image_shape: tuple[int, int],
    ) -> None:
        height, width = image_shape
        message = BuoyObservation()
        message.header = source.header
        message.image_width = int(width)
        message.image_height = int(height)
        message.range_m = float("nan")
        message.range_valid = False
        if target is None:
            message.detected = False
            message.class_id = -1
        else:
            error_x = (2.0 * target.center[0] / max(width, 1)) - 1.0
            error_y = (2.0 * target.center[1] / max(height, 1)) - 1.0
            message.detected = True
            message.class_id = int(target.class_id)
            message.class_label = target.label
            message.confidence = float(target.confidence)
            message.bbox_center_x = float(target.center[0])
            message.bbox_center_y = float(target.center[1])
            message.bbox_width = float(target.width)
            message.bbox_height = float(target.height)
            message.normalized_error_x = float(error_x)
            message.normalized_error_y = float(error_y)
            message.bearing_rad = float(error_x * self.hfov_rad * 0.5)
        self.observation_pub.publish(message)

    def _publish_bbox(
        self,
        source: CompressedImage,
        target: Optional[VisionDetection],
        image_shape: tuple[int, int],
    ) -> None:
        height, width = image_shape
        stamp = float(source.header.stamp.sec) + float(source.header.stamp.nanosec) * 1.0e-9
        message = Float32MultiArray()
        values = [stamp, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(width), float(height)]
        if target is not None:
            values[1:8] = [
                1.0,
                float(target.class_id),
                float(target.confidence),
                float(target.center[0]),
                float(target.center[1]),
                float(target.width),
                float(target.height),
            ]
        message.data = values
        self.bbox_pub.publish(message)

    def _publish_bbox_candidates(
        self,
        source: CompressedImage,
        detections: list[VisionDetection],
        image_shape: tuple[int, int],
    ) -> None:
        if not detections:
            self._publish_bbox(source, None, image_shape)
            return
        for candidate in sorted(detections, key=lambda item: item.class_id):
            self._publish_bbox(source, candidate, image_shape)

    def _publish_status(
        self,
        source: CompressedImage,
        target: Optional[VisionDetection],
        image_shape: tuple[int, int],
        state: str,
        source_kind: str,
    ) -> None:
        height, width = image_shape
        payload: dict[str, Any] = {
            "stamp": {
                "sec": int(source.header.stamp.sec),
                "nanosec": int(source.header.stamp.nanosec),
            },
            "state": state,
            "detected": target is not None,
            "source": target.source if target is not None else source_kind,
            "frame_sequence": self.frame_sequence,
            "image": {"width": width, "height": height},
            "model": {
                "path": self.model_path,
                "task": str(getattr(self.model, "task", "")),
                "classes": self.class_names,
                "device": self.device,
                "cpu_threads": self.cpu_threads,
                "cpu_affinity": self.cpu_affinity,
                "imgsz": self.imgsz,
            },
            "inference_ms": round(self.last_inference_ms, 3),
            "raw_detection_count": self.last_raw_detection_count,
            "control_candidate_count": self.last_candidate_count,
            "stable_class_count": len(self.current_class_detections),
            "synchronization": {
                "result_age_ms": round(self.last_result_age_ms, 3),
                "frame_lag": self.last_result_frame_lag,
                "track_hold_seconds": self.track_hold_seconds,
                "max_result_age_s": self.max_inference_result_age_s,
                "dropped_stale_results": self.dropped_stale_results,
                "class_association_max_diagonal_ratio": self.class_tracker.association_max_diagonal_ratio,
            },
            "shape_filter": {
                "min_vertical_aspect": self.min_vertical_aspect,
                "min_area_ratio": self.min_mask_area_ratio,
                "max_area_ratio": self.max_mask_area_ratio,
            },
            "pinger_acoustic_bearing_rad": self.hydrophone_bearing_rad,
            "pinger_acoustic_age_s": (
                round(time.monotonic() - self.hydrophone_received_monotonic, 3)
                if self.hydrophone_received_monotonic >= 0.0
                else None
            ),
            "pinger_selector": {
                "mode": "hydrophone_bearing",
                "classes": sorted(self.pinger_target_class_names),
                "bearing_error_rad": self.last_pinger_bearing_error_rad,
                "tolerance_rad": self.pinger_bearing_tolerance_rad,
                "lock_max_diagonal_ratio": self.pinger_lock_max_diagonal_ratio,
            },
            "underwater_selector": {
                "classes": sorted(self.underwater_target_class_names),
                "preferred_class": self.alignment_preferred_class_name,
            },
            "mission_state": self.mission_state,
            "pinger_visual_locked": self.locked_target_center is not None,
            "pinger_missing_inferences": self.pinger_missing_inferences,
        }
        if target is not None:
            payload["target"] = {
                "class_id": target.class_id,
                "label": target.label,
                "confidence": round(target.confidence, 5),
                "bbox_xyxy": list(target.xyxy),
                "mask_center_px": [round(value, 2) for value in target.center],
                "mask_bottom_px": [round(value, 2) for value in target.bottom],
                "mask_area_px": round(target.mask_area, 1),
                "height_ratio": round(target.height / max(height, 1), 5),
                "area_ratio": round(target.mask_area / max(width * height, 1), 5),
                "error_norm": [
                    round((2.0 * target.center[0] / max(width, 1)) - 1.0, 5),
                    round((2.0 * target.center[1] / max(height, 1)) - 1.0, 5),
                ],
            }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        self.status_pub.publish(message)

    def _publish_annotated(self, source: CompressedImage, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return
        message = CompressedImage()
        message.header = source.header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.annotated_pub.publish(message)

    def close(self) -> None:
        thread = self.inference_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        if self.show_preview:
            cv2.destroyWindow(self.preview_window_name)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = YoloBuoyDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
