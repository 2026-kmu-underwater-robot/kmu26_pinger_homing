#!/usr/bin/env python3
"""OpenCV helpers for underwater buoy segmentation and temporal tracking."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Any, Iterable, Mapping, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionDetection:
    class_id: int
    label: str
    confidence: float
    xyxy: tuple[int, int, int, int]
    center: tuple[float, float]
    bottom: tuple[float, float]
    mask_area: float
    mask: Optional[np.ndarray] = None
    source: str = "yolo"

    @property
    def width(self) -> float:
        return float(max(0, self.xyxy[2] - self.xyxy[0]))

    @property
    def height(self) -> float:
        return float(max(0, self.xyxy[3] - self.xyxy[1]))


def preprocess_underwater_bgr(
    image: np.ndarray,
    *,
    white_balance: bool = True,
    clahe_clip: float = 2.0,
    clahe_grid: int = 8,
) -> np.ndarray:
    """Apply conservative gray-world white balance and luminance CLAHE."""

    if image is None or image.size == 0:
        raise ValueError("empty image")
    output = np.ascontiguousarray(image.copy())
    if white_balance:
        channel_means = output.reshape(-1, 3).mean(axis=0).astype(np.float32)
        target = float(np.mean(channel_means))
        gains = np.clip(target / np.maximum(channel_means, 1.0), 0.55, 1.8)
        output = np.clip(output.astype(np.float32) * gains.reshape(1, 1, 3), 0, 255).astype(np.uint8)
    if clahe_clip > 0.0:
        lab = cv2.cvtColor(output, cv2.COLOR_BGR2LAB)
        lightness, color_a, color_b = cv2.split(lab)
        grid = max(2, int(clahe_grid))
        clahe = cv2.createCLAHE(clipLimit=float(clahe_clip), tileGridSize=(grid, grid))
        output = cv2.cvtColor(cv2.merge((clahe.apply(lightness), color_a, color_b)), cv2.COLOR_LAB2BGR)
    return output


def detections_from_ultralytics(
    result: Any,
    image_shape: tuple[int, int],
    class_names: Mapping[int, str] | Mapping[str, str],
) -> list[VisionDetection]:
    """Convert Ultralytics boxes and optional segmentation masks to OpenCV data."""

    image_height, image_width = image_shape
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []
    xyxy = _to_numpy(getattr(boxes, "xyxy", []))
    confidences = _to_numpy(getattr(boxes, "conf", []))
    class_ids = _to_numpy(getattr(boxes, "cls", [])).astype(int)
    masks = _source_masks(result, len(xyxy), image_width, image_height)
    detections: list[VisionDetection] = []
    for index, coords in enumerate(xyxy):
        x1, y1, x2, y2 = _clip_box(coords, image_width, image_height)
        if x2 <= x1 or y2 <= y1:
            continue
        class_id = int(class_ids[index]) if index < len(class_ids) else -1
        confidence = float(confidences[index]) if index < len(confidences) else 0.0
        mask = masks[index] if index < len(masks) else None
        if mask is not None and int(np.count_nonzero(mask)) > 0:
            center, bottom, mask_area, mask_box = _mask_geometry(mask)
            x1, y1, x2, y2 = mask_box
        else:
            center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5)
            bottom = ((x1 + x2) * 0.5, float(y2))
            mask_area = float((x2 - x1) * (y2 - y1))
        label = str(class_names.get(class_id, class_names.get(str(class_id), class_id)))
        detections.append(
            VisionDetection(
                class_id=class_id,
                label=label,
                confidence=confidence,
                xyxy=(x1, y1, x2, y2),
                center=center,
                bottom=bottom,
                mask_area=mask_area,
                mask=mask,
            )
        )
    return detections


def select_target(
    detections: Iterable[VisionDetection],
    *,
    target_class_id: int = -1,
    target_class_name: str = "",
    policy: str = "largest_mask",
) -> Optional[VisionDetection]:
    class_name = target_class_name.strip().lower()
    candidates = [
        detection
        for detection in detections
        if (target_class_id < 0 or detection.class_id == target_class_id)
        and (not class_name or detection.label.strip().lower() == class_name)
    ]
    if not candidates:
        return None
    if policy == "confidence":
        return max(candidates, key=lambda item: (item.confidence, item.mask_area))
    return max(candidates, key=lambda item: (item.mask_area, item.confidence))


def best_detection_per_class(
    detections: Iterable[VisionDetection],
) -> list[VisionDetection]:
    """Return one highest-confidence detection for every visible class.

    The upstream visual mission controller keeps independent recent `buoy`
    and `stick` observations. Publishing one target per class preserves that
    contract while the typed observation can continue exposing the single
    stabilized target used by the GUI and mission telemetry.
    """

    best: dict[int, VisionDetection] = {}
    for detection in detections:
        previous = best.get(detection.class_id)
        if previous is None or (detection.confidence, detection.mask_area) > (
            previous.confidence,
            previous.mask_area,
        ):
            best[detection.class_id] = detection
    return [best[class_id] for class_id in sorted(best)]


def filter_target_geometry(
    detections: Iterable[VisionDetection],
    image_shape: tuple[int, int],
    *,
    min_vertical_aspect: float = 1.10,
    min_area_ratio: float = 0.0001,
    max_area_ratio: float = 0.20,
) -> list[VisionDetection]:
    """Reject horizontal structures and frame-filling masks before control."""

    image_height, image_width = image_shape
    image_area = max(1.0, float(image_height * image_width))
    accepted: list[VisionDetection] = []
    for detection in detections:
        vertical_aspect = detection.height / max(detection.width, 1.0)
        area_ratio = detection.mask_area / image_area
        if vertical_aspect < max(0.0, min_vertical_aspect):
            continue
        if area_ratio < max(0.0, min_area_ratio) or area_ratio > max_area_ratio:
            continue
        accepted.append(detection)
    return accepted


def filter_by_max_center_y_ratio(
    detections: Iterable[VisionDetection],
    image_shape: tuple[int, int],
    max_center_y_ratio: float,
) -> list[VisionDetection]:
    """Keep detections above a configurable image-row boundary."""

    image_height, _image_width = image_shape
    denominator = max(1, image_height)
    limit = float(np.clip(max_center_y_ratio, 0.0, 1.0))
    return [
        detection
        for detection in detections
        if detection.center[1] / denominator <= limit
    ]


def _detection_pixels(image: np.ndarray, detection: VisionDetection) -> np.ndarray:
    if image is None or image.size == 0:
        return np.empty((0, 3), dtype=np.uint8)
    if detection.mask is not None and detection.mask.shape == image.shape[:2]:
        return image[detection.mask.astype(bool)]
    x1, y1, x2, y2 = detection.xyxy
    return image[y1:y2, x1:x2].reshape(-1, image.shape[2])


def white_pixel_ratio(
    image: np.ndarray,
    detection: VisionDetection,
    *,
    max_chroma: int = 35,
    min_brightness: int = 165,
) -> float:
    """Return the fraction of low-chroma, bright pixels inside a detection."""

    pixels = _detection_pixels(image, detection)
    if pixels.size == 0:
        return 0.0
    values = pixels.astype(np.int16)
    chroma = values.max(axis=1) - values.min(axis=1)
    brightness = values.mean(axis=1)
    return float(np.mean((chroma <= max_chroma) & (brightness >= min_brightness)))


def warm_pixel_ratio(
    image: np.ndarray,
    detection: VisionDetection,
    *,
    min_saturation: int = 75,
    min_brightness: int = 100,
    min_hue: int = 3,
    max_hue: int = 45,
) -> float:
    """Return the yellow/orange pixel fraction inside a detection."""

    pixels = _detection_pixels(image, detection)
    if pixels.size == 0:
        return 0.0
    hsv = cv2.cvtColor(pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    hue = hsv[:, 0]
    saturation = hsv[:, 1]
    brightness = hsv[:, 2]
    warm = (
        (hue >= min_hue)
        & (hue <= max_hue)
        & (saturation >= min_saturation)
        & (brightness >= min_brightness)
    )
    return float(np.mean(warm))


def luma_standard_deviation(
    image: np.ndarray,
    detection: VisionDetection,
) -> float:
    """Return luminance contrast inside a detection bbox or mask."""

    pixels = _detection_pixels(image, detection)
    if pixels.size == 0:
        return 0.0
    values = pixels.astype(np.float32)
    luma = 0.114 * values[:, 0] + 0.587 * values[:, 1] + 0.299 * values[:, 2]
    return float(np.std(luma))


def classify_buoy_color(
    image: np.ndarray,
    detection: VisionDetection,
    *,
    white_min_ratio: float = 0.18,
    warm_max_ratio: float = 0.12,
    min_luma_std: float = 8.0,
) -> tuple[str, float, float]:
    """Classify a shape detection using white and yellow/orange pixel evidence."""

    white_ratio = white_pixel_ratio(image, detection)
    warm_ratio = warm_pixel_ratio(image, detection)
    if warm_ratio > warm_max_ratio:
        return "YELLOW/ORANGE", white_ratio, warm_ratio
    if white_ratio >= white_min_ratio and luma_standard_deviation(image, detection) >= min_luma_std:
        return "WHITE/GRAY", white_ratio, warm_ratio
    return "UNKNOWN", white_ratio, warm_ratio


class TemporalTargetTracker:
    """Kalman smoothing with optical-flow propagation between YOLO frames."""

    def __init__(self, *, hold_seconds: float = 0.35) -> None:
        self.hold_seconds = max(0.0, float(hold_seconds))
        self._kalman = cv2.KalmanFilter(8, 4)
        self._kalman.transitionMatrix = np.array(
            [
                [1, 0, 0, 0, 1, 0, 0, 0],
                [0, 1, 0, 0, 0, 1, 0, 0],
                [0, 0, 1, 0, 0, 0, 1, 0],
                [0, 0, 0, 1, 0, 0, 0, 1],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )
        self._kalman.measurementMatrix = np.hstack(
            (np.eye(4, dtype=np.float32), np.zeros((4, 4), dtype=np.float32))
        )
        self._kalman.processNoiseCov = np.eye(8, dtype=np.float32) * 0.025
        self._kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.12
        self._kalman.errorCovPost = np.eye(8, dtype=np.float32)
        self._initialized = False
        self._last_detection: Optional[VisionDetection] = None
        self._last_detection_time = -1.0
        self._prev_gray: Optional[np.ndarray] = None
        self._points: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._initialized = False
        self._last_detection = None
        self._last_detection_time = -1.0
        self._prev_gray = None
        self._points = None

    def correct(self, detection: VisionDetection, gray: np.ndarray, now: float) -> VisionDetection:
        measurement = np.array(
            [[detection.center[0]], [detection.center[1]], [detection.width], [detection.height]],
            dtype=np.float32,
        )
        if not self._initialized:
            self._kalman.statePost = np.vstack((measurement, np.zeros((4, 1), dtype=np.float32)))
            self._initialized = True
            estimate = measurement
        else:
            self._kalman.predict()
            estimate = self._kalman.correct(measurement)
        stabilized = self._from_estimate(detection, estimate, gray.shape)
        self._last_detection = stabilized
        self._last_detection_time = float(now)
        self._prev_gray = gray.copy()
        self._points = _feature_points(gray, detection.mask, detection.xyxy)
        return stabilized

    def propagate(self, gray: np.ndarray, now: float) -> Optional[VisionDetection]:
        if (
            not self._initialized
            or self._last_detection is None
            or self._last_detection_time < 0.0
            or now - self._last_detection_time > self.hold_seconds
        ):
            self.reset()
            return None
        estimate = self._kalman.predict()
        source = "kalman"
        if self._prev_gray is not None and self._points is not None and len(self._points) >= 3:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self._prev_gray,
                gray,
                self._points,
                None,
                winSize=(21, 21),
                maxLevel=3,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01),
            )
            if next_points is not None and status is not None:
                valid = status.reshape(-1) == 1
                old = self._points.reshape(-1, 2)[valid]
                new = next_points.reshape(-1, 2)[valid]
                if len(new) >= 3:
                    delta = np.median(new - old, axis=0)
                    # `predict()` already includes Kalman velocity. Optical
                    # flow is an observed frame-to-frame displacement, so
                    # adding it to that prediction counts motion twice. Anchor
                    # the observation to the previous published center.
                    estimate[0, 0] = self._last_detection.center[0] + float(delta[0])
                    estimate[1, 0] = self._last_detection.center[1] + float(delta[1])
                    self._kalman.statePost[0, 0] = estimate[0, 0]
                    self._kalman.statePost[1, 0] = estimate[1, 0]
                    self._kalman.statePost[4, 0] = float(delta[0])
                    self._kalman.statePost[5, 0] = float(delta[1])
                    self._points = new.reshape(-1, 1, 2).astype(np.float32)
                    source = "optical_flow"
        propagated = self._from_estimate(
            replace(self._last_detection, source=source, mask=None), estimate, gray.shape
        )
        self._last_detection = propagated
        self._prev_gray = gray.copy()
        return propagated

    @staticmethod
    def _from_estimate(
        template: VisionDetection, estimate: np.ndarray, image_shape: tuple[int, int]
    ) -> VisionDetection:
        height, width = image_shape
        center_x = float(np.clip(estimate[0, 0], 0, max(0, width - 1)))
        center_y = float(np.clip(estimate[1, 0], 0, max(0, height - 1)))
        box_width = float(np.clip(abs(estimate[2, 0]), 2, max(2, width)))
        box_height = float(np.clip(abs(estimate[3, 0]), 2, max(2, height)))
        x1 = int(np.clip(round(center_x - box_width * 0.5), 0, max(0, width - 1)))
        y1 = int(np.clip(round(center_y - box_height * 0.5), 0, max(0, height - 1)))
        x2 = int(np.clip(round(center_x + box_width * 0.5), x1 + 1, max(1, width)))
        y2 = int(np.clip(round(center_y + box_height * 0.5), y1 + 1, max(1, height)))
        return replace(
            template,
            xyxy=(x1, y1, x2, y2),
            center=(center_x, center_y),
            bottom=(center_x, float(y2)),
        )


def synchronize_detection_to_frame(
    detection: VisionDetection,
    source_gray: np.ndarray,
    current_gray: np.ndarray,
) -> VisionDetection:
    """Move a delayed YOLO detection into the current camera frame.

    Inference runs asynchronously, so its bbox belongs to the frame captured
    before inference started. Feature flow on the detected object transports
    that bbox to the frame being published. A conservative unchanged-box
    fallback is used when the object has too little texture for optical flow.
    """

    if (
        source_gray is None
        or current_gray is None
        or source_gray.size == 0
        or current_gray.size == 0
        or source_gray.shape != current_gray.shape
    ):
        return replace(detection, mask=None, source="yolo_sync_fallback")
    points = _feature_points(source_gray, detection.mask, detection.xyxy)
    if points is None or len(points) < 3:
        return replace(detection, mask=None, source="yolo_sync_fallback")
    next_points, forward_status, _ = cv2.calcOpticalFlowPyrLK(
        source_gray,
        current_gray,
        points,
        None,
        winSize=(25, 25),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, 0.01),
    )
    if next_points is None or forward_status is None:
        return replace(detection, mask=None, source="yolo_sync_fallback")
    back_points, back_status, _ = cv2.calcOpticalFlowPyrLK(
        current_gray,
        source_gray,
        next_points,
        None,
        winSize=(25, 25),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, 0.01),
    )
    if back_points is None or back_status is None:
        return replace(detection, mask=None, source="yolo_sync_fallback")
    source_points = points.reshape(-1, 2)
    target_points = next_points.reshape(-1, 2)
    forward_valid = forward_status.reshape(-1) == 1
    backward_valid = back_status.reshape(-1) == 1
    roundtrip_error = np.linalg.norm(back_points.reshape(-1, 2) - source_points, axis=1)
    valid = forward_valid & backward_valid & (roundtrip_error <= 2.0)
    source_points = source_points[valid]
    target_points = target_points[valid]
    if len(target_points) < 3:
        return replace(detection, mask=None, source="yolo_sync_fallback")

    transform = None
    if len(target_points) >= 4:
        transform, inliers = cv2.estimateAffinePartial2D(
            source_points,
            target_points,
            method=cv2.RANSAC,
            ransacReprojThreshold=2.0,
            maxIters=100,
            confidence=0.98,
            refineIters=5,
        )
        if transform is not None and inliers is not None and int(np.count_nonzero(inliers)) < 3:
            transform = None
    if transform is None:
        delta = np.median(target_points - source_points, axis=0)
        transform = np.array(
            [[1.0, 0.0, float(delta[0])], [0.0, 1.0, float(delta[1])]],
            dtype=np.float32,
        )

    x1, y1, x2, y2 = detection.xyxy
    geometry = np.array(
        [
            [x1, y1, 1.0],
            [x2, y1, 1.0],
            [x2, y2, 1.0],
            [x1, y2, 1.0],
            [detection.center[0], detection.center[1], 1.0],
            [detection.bottom[0], detection.bottom[1], 1.0],
        ],
        dtype=np.float32,
    )
    warped = geometry @ transform.T
    height, width = current_gray.shape
    warped_corners = warped[:4]
    synced_box = _clip_box(
        (
            np.min(warped_corners[:, 0]),
            np.min(warped_corners[:, 1]),
            np.max(warped_corners[:, 0]),
            np.max(warped_corners[:, 1]),
        ),
        width,
        height,
    )
    center = (
        float(np.clip(warped[4, 0], 0, max(0, width - 1))),
        float(np.clip(warped[4, 1], 0, max(0, height - 1))),
    )
    bottom = (
        float(np.clip(warped[5, 0], 0, max(0, width - 1))),
        float(np.clip(warped[5, 1], 0, max(0, height - 1))),
    )
    area_scale = abs(float(np.linalg.det(transform[:, :2])))
    return replace(
        detection,
        xyxy=synced_box,
        center=center,
        bottom=bottom,
        mask_area=max(1.0, detection.mask_area * area_scale),
        mask=None,
        source="yolo_synced",
    )


class PerClassTemporalTracker:
    """Keep independent, current-frame buoy and stick tracks."""

    def __init__(
        self,
        *,
        hold_seconds: float = 1.6,
        association_max_diagonal_ratio: float = 0.025,
    ) -> None:
        self.hold_seconds = max(0.0, float(hold_seconds))
        self.association_max_diagonal_ratio = max(
            0.01, float(association_max_diagonal_ratio)
        )
        self._trackers: dict[int, TemporalTargetTracker] = {}
        self._current: dict[int, VisionDetection] = {}

    def reset(self) -> None:
        for tracker in self._trackers.values():
            tracker.reset()
        self._trackers.clear()
        self._current.clear()

    def correct(
        self,
        detections: Iterable[VisionDetection],
        gray: np.ndarray,
        now: float,
    ) -> list[VisionDetection]:
        grouped: dict[int, list[VisionDetection]] = {}
        for detection in detections:
            grouped.setdefault(detection.class_id, []).append(detection)
        observed: set[int] = set()
        image_diagonal = max(1.0, math.hypot(gray.shape[1], gray.shape[0]))
        max_distance = self.association_max_diagonal_ratio * image_diagonal
        for class_id, candidates in grouped.items():
            current = self._current.get(class_id)
            if current is None:
                detection = max(
                    candidates,
                    key=lambda item: (item.confidence, item.mask_area),
                )
            else:
                detection = min(
                    candidates,
                    key=lambda item: math.hypot(
                        item.center[0] - current.center[0],
                        item.center[1] - current.center[1],
                    ),
                )
                distance = math.hypot(
                    detection.center[0] - current.center[0],
                    detection.center[1] - current.center[1],
                )
                if distance > max_distance:
                    continue
            observed.add(class_id)
            tracker = self._trackers.setdefault(
                class_id,
                TemporalTargetTracker(hold_seconds=self.hold_seconds),
            )
            self._current[class_id] = tracker.correct(detection, gray, now)
        self._propagate_missing(gray, now, observed)
        return self.current()

    def propagate(self, gray: np.ndarray, now: float) -> list[VisionDetection]:
        self._propagate_missing(gray, now, set())
        return self.current()

    def _propagate_missing(
        self, gray: np.ndarray, now: float, observed: set[int]
    ) -> None:
        for class_id, tracker in list(self._trackers.items()):
            if class_id in observed:
                continue
            propagated = tracker.propagate(gray, now)
            if propagated is None:
                self._trackers.pop(class_id, None)
                self._current.pop(class_id, None)
            else:
                self._current[class_id] = propagated

    def current(self) -> list[VisionDetection]:
        return [self._current[class_id] for class_id in sorted(self._current)]


def vision_state(
    target: Optional[VisionDetection],
    image_shape: tuple[int, int],
    *,
    center_tolerance: float = 0.10,
    fine_height_ratio: float = 0.18,
    capture_height_ratio: float = 0.40,
) -> str:
    if target is None:
        return "SEARCH"
    height, width = image_shape
    error_x = (2.0 * target.center[0] / max(width, 1)) - 1.0
    error_y = (2.0 * target.center[1] / max(height, 1)) - 1.0
    height_ratio = target.height / max(height, 1)
    centered = abs(error_x) <= center_tolerance and abs(error_y) <= center_tolerance
    if centered and height_ratio >= capture_height_ratio:
        return "CAPTURE_READY"
    if height_ratio >= fine_height_ratio:
        return "FINE_ALIGN"
    return "APPROACH"


def detection_bearing_rad(
    detection: VisionDetection,
    image_width: int,
    horizontal_fov_rad: float,
) -> float:
    """Return a detection's horizontal camera bearing, right-positive."""

    error_x = (2.0 * detection.center[0] / max(image_width, 1)) - 1.0
    return error_x * horizontal_fov_rad * 0.5


def bearing_error_rad(
    detection: VisionDetection,
    image_width: int,
    horizontal_fov_rad: float,
    expected_bearing_rad: float,
) -> float:
    """Return the wrapped absolute error to an expected camera bearing."""

    delta = (
        detection_bearing_rad(detection, image_width, horizontal_fov_rad)
        - expected_bearing_rad
    )
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def select_detection_by_bearing(
    detections: Iterable[VisionDetection],
    image_shape: tuple[int, int],
    *,
    horizontal_fov_rad: float,
    expected_bearing_rad: float,
    tolerance_rad: float,
    locked_center: Optional[tuple[float, float]] = None,
    lock_max_diagonal_ratio: float = 0.08,
) -> Optional[VisionDetection]:
    """Select a detection using hydrophone bearing, with lock continuity."""

    height, width = image_shape
    diagonal = max(1.0, math.hypot(width, height))
    scored: list[tuple[float, float, float, float, VisionDetection]] = []
    for detection in detections:
        error = bearing_error_rad(
            detection,
            width,
            horizontal_fov_rad,
            expected_bearing_rad,
        )
        if error > tolerance_rad:
            continue
        lock_distance = (
            math.hypot(
                detection.center[0] - locked_center[0],
                detection.center[1] - locked_center[1],
            )
            / diagonal
            if locked_center is not None
            else math.inf
        )
        scored.append(
            (lock_distance, error, -detection.mask_area, -detection.confidence, detection)
        )
    if not scored:
        return None
    if locked_center is not None:
        continuous = [item for item in scored if item[0] <= lock_max_diagonal_ratio]
        if not continuous:
            return None
        return min(continuous, key=lambda item: (item[0], item[1], item[2], item[3]))[4]
    return min(scored, key=lambda item: (item[1], item[2], item[3]))[4]


def draw_overlay(
    image: np.ndarray,
    detections: Iterable[VisionDetection],
    selected: Optional[VisionDetection],
    *,
    state: str,
    inference_ms: float,
    pinger_mode: bool = False,
    pinger_locked: bool = False,
) -> np.ndarray:
    output = np.ascontiguousarray(image.copy())
    height, width = output.shape[:2]
    display_detections = list(detections)
    if selected is not None and not any(
        _same_detection(detection, selected) for detection in display_detections
    ):
        display_detections.append(selected)
    for detection in display_detections:
        selected_detection = selected is not None and _same_detection(detection, selected)
        color = (40, 225, 70) if selected_detection else (0, 165, 255)
        if detection.mask is not None:
            tint = np.zeros_like(output)
            tint[:, :] = color
            active = detection.mask.astype(bool)
            output[active] = cv2.addWeighted(output, 0.55, tint, 0.45, 0)[active]
            contours, _ = cv2.findContours(
                detection.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(output, contours, -1, color, 2)
        x1, y1, x2, y2 = detection.xyxy
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2 if selected_detection else 1)
        label = f"{detection.label} {detection.confidence:.2f}"
        if pinger_mode and selected_detection and pinger_locked:
            label = "PINGER HYDRO LOCK"
        (label_width, _), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 2
        )
        label_x = int(np.clip(x1, 0, max(0, width - label_width - 4)))
        cv2.putText(
            output,
            label,
            (label_x, max(20, y1 - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.drawMarker(output, (width // 2, height // 2), (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
    if selected is not None:
        center = tuple(int(round(value)) for value in selected.center)
        bottom = tuple(int(round(value)) for value in selected.bottom)
        cv2.arrowedLine(output, (width // 2, height // 2), center, (40, 225, 70), 2, tipLength=0.08)
        cv2.drawMarker(output, center, (255, 255, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.drawMarker(output, bottom, (255, 80, 40), cv2.MARKER_TRIANGLE_DOWN, 16, 2)
    cv2.rectangle(output, (0, 0), (width, 42), (8, 18, 28), -1)
    cv2.putText(
        output,
        f"VISION {state} | YOLO {inference_ms:.1f} ms",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (235, 245, 250),
        2,
        cv2.LINE_AA,
    )
    return output


def _source_masks(result: Any, count: int, width: int, height: int) -> list[Optional[np.ndarray]]:
    masks_object = getattr(result, "masks", None)
    polygons = getattr(masks_object, "xy", None) if masks_object is not None else None
    masks: list[Optional[np.ndarray]] = []
    if polygons is not None:
        for polygon in list(polygons)[:count]:
            mask = np.zeros((height, width), dtype=np.uint8)
            points = np.asarray(polygon, dtype=np.float32)
            if points.ndim == 2 and len(points) >= 3:
                points[:, 0] = np.clip(points[:, 0], 0, max(0, width - 1))
                points[:, 1] = np.clip(points[:, 1], 0, max(0, height - 1))
                cv2.fillPoly(mask, [np.round(points).astype(np.int32)], 1)
            masks.append(mask)
    while len(masks) < count:
        masks.append(None)
    return masks


def _mask_geometry(mask: np.ndarray) -> tuple[tuple[float, float], tuple[float, float], float, tuple[int, int, int, int]]:
    binary = mask.astype(np.uint8)
    moments = cv2.moments(binary, binaryImage=True)
    ys, xs = np.nonzero(binary)
    if len(xs) == 0:
        return (0.0, 0.0), (0.0, 0.0), 0.0, (0, 0, 0, 0)
    if moments["m00"] > 0.0:
        center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
    else:
        center = (float(np.mean(xs)), float(np.mean(ys)))
    lower_cutoff = np.percentile(ys, 92)
    lower_x = xs[ys >= lower_cutoff]
    bottom = (float(np.median(lower_x)) if len(lower_x) else center[0], float(np.max(ys)))
    x, y, width, height = cv2.boundingRect(binary)
    return center, bottom, float(len(xs)), (x, y, x + width, y + height)


def _feature_points(gray: np.ndarray, mask: Optional[np.ndarray], xyxy: tuple[int, int, int, int]) -> Optional[np.ndarray]:
    feature_mask = np.zeros(gray.shape, dtype=np.uint8)
    if mask is not None and mask.shape == gray.shape:
        feature_mask[mask.astype(bool)] = 255
    else:
        x1, y1, x2, y2 = xyxy
        feature_mask[y1:y2, x1:x2] = 255
    return cv2.goodFeaturesToTrack(
        gray,
        maxCorners=60,
        qualityLevel=0.01,
        minDistance=5,
        mask=feature_mask,
        blockSize=5,
    )


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def _clip_box(coords: Any, width: int, height: int) -> tuple[int, int, int, int]:
    values = np.asarray(coords, dtype=np.float32).reshape(-1)
    if len(values) < 4:
        return 0, 0, 0, 0
    x1 = int(np.clip(np.floor(values[0]), 0, max(0, width - 1)))
    y1 = int(np.clip(np.floor(values[1]), 0, max(0, height - 1)))
    x2 = int(np.clip(np.ceil(values[2]), x1 + 1, max(1, width)))
    y2 = int(np.clip(np.ceil(values[3]), y1 + 1, max(1, height)))
    return x1, y1, x2, y2


def _same_detection(left: VisionDetection, right: VisionDetection) -> bool:
    return left.class_id == right.class_id and abs(left.center[0] - right.center[0]) < 2.0 and abs(left.center[1] - right.center[1]) < 2.0


__all__ = [
    "bearing_error_rad",
    "best_detection_per_class",
    "classify_buoy_color",
    "detection_bearing_rad",
    "TemporalTargetTracker",
    "PerClassTemporalTracker",
    "VisionDetection",
    "detections_from_ultralytics",
    "draw_overlay",
    "filter_by_max_center_y_ratio",
    "filter_target_geometry",
    "luma_standard_deviation",
    "preprocess_underwater_bgr",
    "select_detection_by_bearing",
    "select_target",
    "synchronize_detection_to_frame",
    "vision_state",
    "warm_pixel_ratio",
    "white_pixel_ratio",
]
