#!/usr/bin/env python3
"""Deterministic tests for segmentation geometry and temporal tracking."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from buoy_vision_core import (  # noqa: E402
    PerClassTemporalTracker,
    TemporalTargetTracker,
    VisionDetection,
    bearing_error_rad,
    best_detection_per_class,
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


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def check_segmentation_geometry() -> None:
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            xyxy=np.array([[8.0, 10.0, 32.0, 50.0], [45.0, 12.0, 78.0, 65.0]]),
            conf=np.array([0.94, 0.70]),
            cls=np.array([0.0, 0.0]),
            __len__=lambda self: 2,
        ),
        masks=SimpleNamespace(
            xy=[
                np.array([[10, 12], [30, 12], [28, 48], [12, 48]], dtype=np.float32),
                np.array([[47, 14], [76, 14], [75, 63], [48, 63]], dtype=np.float32),
            ]
        ),
    )
    # SimpleNamespace does not expose a special-method lambda through len().
    result.boxes = type(
        "Boxes",
        (),
        {
            "xyxy": result.boxes.xyxy,
            "conf": result.boxes.conf,
            "cls": result.boxes.cls,
            "__len__": lambda self: 2,
        },
    )()
    detections = detections_from_ultralytics(result, (80, 100), {0: "buoy_stick"})
    require(len(detections) == 2, f"expected two masks, got {len(detections)}")
    selected = select_target(detections, target_class_name="buoy_stick")
    require(selected is not None, "largest mask was not selected")
    require(selected.center[0] > 55.0, f"wrong mask selected: center={selected.center}")
    require(selected.bottom[1] >= 62.0, f"mask lower point is wrong: {selected.bottom}")
    require(selected.mask_area > detections[0].mask_area, "selection ignored segmentation area")


def check_preprocessing_and_overlay() -> None:
    image = np.zeros((90, 120, 3), dtype=np.uint8)
    image[:, :, 0] = 130
    image[:, :, 1] = np.linspace(15, 100, 120, dtype=np.uint8)
    image[:, :, 2] = 20
    processed = preprocess_underwater_bgr(image)
    require(processed.shape == image.shape and processed.dtype == np.uint8, "preprocess contract changed")
    require(float(processed.std()) > 1.0, "CLAHE output lost image contrast")
    detection = VisionDetection(
        class_id=0,
        label="buoy_stick",
        confidence=0.9,
        xyxy=(35, 12, 75, 78),
        center=(55.0, 45.0),
        bottom=(55.0, 77.0),
        mask_area=1800.0,
    )
    overlay = draw_overlay(image, [detection], detection, state="FINE_ALIGN", inference_ms=12.3)
    require(not np.array_equal(overlay, image), "OpenCV overlay was not drawn")
    require(
        vision_state(
            detection,
            image.shape[:2],
            fine_height_ratio=0.4,
            capture_height_ratio=0.9,
        ) == "FINE_ALIGN",
        "FSM state mismatch",
    )

    horizontal_wall = VisionDetection(
        class_id=0,
        label="buoy_stick",
        confidence=0.99,
        xyxy=(5, 20, 115, 50),
        center=(60.0, 35.0),
        bottom=(60.0, 50.0),
        mask_area=2500.0,
    )
    filtered = filter_target_geometry([horizontal_wall, detection], image.shape[:2])
    require(filtered == [detection], "horizontal wall was accepted as a buoy control target")

    floor_anchor = VisionDetection(
        class_id=0,
        label="buoy_stick",
        confidence=0.98,
        xyxy=(48, 68, 72, 89),
        center=(60.0, 80.0),
        bottom=(60.0, 88.0),
        mask_area=420.0,
    )
    water_column = filter_by_max_center_y_ratio(
        [detection, floor_anchor], image.shape[:2], 0.72
    )
    require(water_column == [detection], "floor anchor passed the water-column filter")


def check_optical_flow_propagation() -> None:
    first = np.zeros((100, 140), dtype=np.uint8)
    rng = np.random.default_rng(7)
    first[30:70, 45:85] = rng.integers(60, 255, size=(40, 40), dtype=np.uint8)
    second = np.zeros_like(first)
    second[33:73, 51:91] = first[30:70, 45:85]
    mask = np.zeros_like(first)
    mask[30:70, 45:85] = 1
    detection = VisionDetection(
        class_id=0,
        label="buoy_stick",
        confidence=0.9,
        xyxy=(45, 30, 85, 70),
        center=(65.0, 50.0),
        bottom=(65.0, 69.0),
        mask_area=1600.0,
        mask=mask,
    )
    tracker = TemporalTargetTracker(hold_seconds=1.0)
    tracker.correct(detection, first, 1.0)
    propagated = tracker.propagate(second, 1.05)
    require(propagated is not None, "tracker dropped a fresh target")
    require(propagated.center[0] > 68.0, f"optical flow did not follow horizontal motion: {propagated.center}")
    require(propagated.center[1] > 51.0, f"optical flow did not follow vertical motion: {propagated.center}")
    require(propagated.source == "optical_flow", f"unexpected tracker source: {propagated.source}")

    third = np.zeros_like(first)
    third[36:76, 57:97] = first[30:70, 45:85]
    second_mask = np.zeros_like(first)
    second_mask[33:73, 51:91] = 1
    second_detection = VisionDetection(
        class_id=0,
        label="buoy_stick",
        confidence=0.9,
        xyxy=(51, 33, 91, 73),
        center=(71.0, 53.0),
        bottom=(71.0, 72.0),
        mask_area=1600.0,
        mask=second_mask,
    )
    corrected = tracker.correct(second_detection, second, 1.10)
    propagated_again = tracker.propagate(third, 1.15)
    require(propagated_again is not None, "tracker dropped the second flow frame")
    require(
        abs(propagated_again.center[0] - (corrected.center[0] + 6.0)) < 1.5,
        f"Kalman and optical-flow motion were double counted: {propagated_again.center}",
    )
    require(
        abs(propagated_again.center[1] - (corrected.center[1] + 3.0)) < 1.5,
        f"vertical optical-flow motion was double counted: {propagated_again.center}",
    )


def check_delayed_inference_frame_synchronization() -> None:
    source = np.zeros((120, 180), dtype=np.uint8)
    rng = np.random.default_rng(27)
    texture = rng.integers(40, 255, size=(46, 42), dtype=np.uint8)
    source[34:80, 58:100] = texture
    current = np.zeros_like(source)
    current[39:85, 66:108] = texture
    mask = np.zeros_like(source)
    mask[34:80, 58:100] = 1
    delayed = VisionDetection(
        class_id=0,
        label="buoy",
        confidence=0.91,
        xyxy=(58, 34, 100, 80),
        center=(79.0, 57.0),
        bottom=(79.0, 79.0),
        mask_area=1932.0,
        mask=mask,
    )
    synced = synchronize_detection_to_frame(delayed, source, current)
    require(synced.source == "yolo_synced", f"delayed bbox was not synchronized: {synced.source}")
    require(abs(synced.center[0] - 87.0) < 1.0, f"wrong synchronized x: {synced.center}")
    require(abs(synced.center[1] - 62.0) < 1.0, f"wrong synchronized y: {synced.center}")
    require(synced.mask is None, "source-frame mask leaked into the current frame")


def check_per_class_temporal_hold() -> None:
    first = np.zeros((120, 180), dtype=np.uint8)
    rng = np.random.default_rng(41)
    first[20:62, 20:56] = rng.integers(50, 255, size=(42, 36), dtype=np.uint8)
    first[55:105, 105:135] = rng.integers(50, 255, size=(50, 30), dtype=np.uint8)
    second = np.zeros_like(first)
    second[23:65, 26:62] = first[20:62, 20:56]
    second[58:108, 111:141] = first[55:105, 105:135]
    buoy_mask = np.zeros_like(first)
    buoy_mask[20:62, 20:56] = 1
    stick_mask = np.zeros_like(first)
    stick_mask[55:105, 105:135] = 1
    buoy = VisionDetection(0, "buoy", 0.9, (20, 20, 56, 62), (38, 41), (38, 62), 1512, buoy_mask)
    stick = VisionDetection(1, "stick", 0.85, (105, 55, 135, 105), (120, 80), (120, 105), 1500, stick_mask)
    tracker = PerClassTemporalTracker(hold_seconds=1.6)
    initial = tracker.correct([buoy, stick], first, 10.0)
    require([item.class_id for item in initial] == [0, 1], "initial per-class tracks are incomplete")
    propagated = tracker.propagate(second, 10.2)
    require([item.class_id for item in propagated] == [0, 1], "one inference gap dropped a class")
    require(all(item.source == "optical_flow" for item in propagated), "per-class tracks bypassed optical flow")
    require(propagated[0].center[0] > 41.0, f"buoy track did not follow motion: {propagated[0].center}")
    require(propagated[1].center[0] > 123.0, f"stick track did not follow motion: {propagated[1].center}")
    require(tracker.propagate(second, 11.5), "track expired before the configured hold interval")
    require(not tracker.propagate(second, 11.7), "stale tracks survived beyond the hold interval")


def check_per_class_identity_association() -> None:
    gray = np.zeros((100, 200), dtype=np.uint8)
    tracked = VisionDetection(0, "buoy", 0.80, (25, 20, 45, 60), (35, 40), (35, 60), 800)
    same_buoy = VisionDetection(0, "buoy", 0.55, (31, 22, 51, 62), (41, 42), (41, 62), 800)
    other_buoy = VisionDetection(0, "buoy", 0.99, (150, 20, 175, 65), (162, 42), (162, 65), 1125)
    tracker = PerClassTemporalTracker(
        hold_seconds=1.6,
        association_max_diagonal_ratio=0.18,
    )
    tracker.correct([tracked], gray, 20.0)
    updated = tracker.correct([other_buoy, same_buoy], gray, 20.2)
    require(len(updated) == 1, "same-class association created duplicate tracks")
    require(updated[0].center[0] < 60.0, f"track jumped to another buoy: {updated[0].center}")
    held = tracker.correct([other_buoy], gray, 20.4)
    require(len(held) == 1, "far same-class target dropped the active track immediately")
    require(held[0].center[0] < 70.0, f"far target replaced the active identity: {held[0].center}")


def check_hydrophone_bearing_selection() -> None:
    left = VisionDetection(0, "buoy", 0.8, (10, 10, 30, 60), (20, 35), (20, 60), 1000)
    center = VisionDetection(0, "buoy", 0.9, (50, 10, 70, 60), (60, 35), (60, 60), 1000)
    right = VisionDetection(0, "buoy", 0.7, (90, 10, 110, 60), (100, 35), (100, 60), 1000)
    hfov = np.deg2rad(60.0)
    selected = select_detection_by_bearing(
        [left, center, right],
        (80, 120),
        horizontal_fov_rad=hfov,
        expected_bearing_rad=np.deg2rad(18.0),
        tolerance_rad=np.deg2rad(12.0),
    )
    require(selected == right, "hydrophone bearing did not select the right-hand buoy")
    require(
        bearing_error_rad(right, 120, hfov, np.deg2rad(18.0)) < np.deg2rad(4.0),
        "camera/hydrophone bearing conversion is wrong",
    )
    require(
        select_detection_by_bearing(
            [left, center, right],
            (80, 120),
            horizontal_fov_rad=hfov,
            expected_bearing_rad=np.deg2rad(18.0),
            tolerance_rad=np.deg2rad(12.0),
            locked_center=left.center,
        )
        is None,
        "tracker switched targets despite the existing bbox lock",
    )


def check_per_class_publication_selection() -> None:
    buoy_low = VisionDetection(0, "buoy", 0.55, (0, 0, 10, 20), (5, 10), (5, 20), 200)
    buoy_high = VisionDetection(0, "buoy", 0.92, (0, 0, 8, 18), (4, 9), (4, 18), 144)
    stick = VisionDetection(1, "stick", 0.80, (20, 0, 28, 30), (24, 15), (24, 30), 240)
    selected = best_detection_per_class([buoy_low, stick, buoy_high])
    require(selected == [buoy_high, stick], "per-class bbox contract did not keep the best buoy and stick")


def main() -> int:
    check_segmentation_geometry()
    check_preprocessing_and_overlay()
    check_optical_flow_propagation()
    check_delayed_inference_frame_synchronization()
    check_per_class_temporal_hold()
    check_per_class_identity_association()
    check_hydrophone_bearing_selection()
    check_per_class_publication_selection()
    print("buoy_vision_core=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
