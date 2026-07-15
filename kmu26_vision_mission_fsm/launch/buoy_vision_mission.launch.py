#!/usr/bin/env python3
"""Bring up YOLO perception and the sensor-only mission FSM."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw/compressed"),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("device", default_value="auto"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("inference_hz", default_value="3.0"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.35"),
        DeclareLaunchArgument("target_class_name", default_value="buoy"),
        DeclareLaunchArgument("min_vertical_aspect", default_value="0.50"),
        DeclareLaunchArgument("max_mask_area_ratio", default_value="0.20"),
        DeclareLaunchArgument("track_hold_seconds", default_value="1.60"),
        DeclareLaunchArgument("max_inference_result_age_s", default_value="2.00"),
        DeclareLaunchArgument("preprocess_enabled", default_value="true"),
        DeclareLaunchArgument("show_preview", default_value="false"),
        DeclareLaunchArgument("start_fsm", default_value="true"),
        DeclareLaunchArgument("use_vision_mission_controller", default_value="true"),
        DeclareLaunchArgument("mission_enabled", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("wait_armed", default_value="true"),
        DeclareLaunchArgument("pose_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("collector_topic", default_value="/collector/state"),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_markers", default_value="true"),
        DeclareLaunchArgument("expected_detach_count", default_value="1"),
        DeclareLaunchArgument("score_zone_x", default_value="0.0"),
        DeclareLaunchArgument("score_zone_y", default_value="0.0"),
        DeclareLaunchArgument("score_zone_radius", default_value="0.8"),
    ]

    detector = Node(
        package="kmu26_vision_mission_fsm",
        executable="run_yolo_buoy_detector",
        name="yolo_buoy_detector",
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "model_path": LaunchConfiguration("model_path"),
            "device": LaunchConfiguration("device"),
            "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
            "inference_hz": ParameterValue(LaunchConfiguration("inference_hz"), value_type=float),
            "confidence_threshold": ParameterValue(
                LaunchConfiguration("confidence_threshold"), value_type=float
            ),
            "target_class_name": LaunchConfiguration("target_class_name"),
            "min_vertical_aspect": ParameterValue(
                LaunchConfiguration("min_vertical_aspect"), value_type=float
            ),
            "max_mask_area_ratio": ParameterValue(
                LaunchConfiguration("max_mask_area_ratio"), value_type=float
            ),
            "track_hold_seconds": ParameterValue(
                LaunchConfiguration("track_hold_seconds"), value_type=float
            ),
            "max_inference_result_age_s": ParameterValue(
                LaunchConfiguration("max_inference_result_age_s"), value_type=float
            ),
            "preprocess_enabled": ParameterValue(
                LaunchConfiguration("preprocess_enabled"), value_type=bool
            ),
            "show_preview": ParameterValue(LaunchConfiguration("show_preview"), value_type=bool),
            "publish_per_class": True,
        }],
        additional_env={
            "OMP_NUM_THREADS": "2",
            "MKL_NUM_THREADS": "2",
            "OPENBLAS_NUM_THREADS": "2",
            "NUMEXPR_NUM_THREADS": "2",
        },
    )

    mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("kmu26_vision_mission_fsm") + "/launch/mission_fsm_real.launch.py"
        ),
        launch_arguments={
            "use_mission_fsm": "false",
            "use_observation_mission_fsm": LaunchConfiguration("start_fsm"),
            "use_vision_mission_controller": LaunchConfiguration(
                "use_vision_mission_controller"),
            "mission_enabled": LaunchConfiguration("mission_enabled"),
            "dry_run": LaunchConfiguration("dry_run"),
            "wait_armed": LaunchConfiguration("wait_armed"),
            "pose_topic": LaunchConfiguration("pose_topic"),
            "state_topic": LaunchConfiguration("state_topic"),
            "collector_state_topic": LaunchConfiguration("collector_topic"),
            "rc_topic": LaunchConfiguration("rc_topic"),
            "yolo_detection_topic": "/vision/buoy_observation",
            "vision_target_class": LaunchConfiguration("target_class_name"),
            "vision_min_confidence": LaunchConfiguration("confidence_threshold"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_mission_rviz_visualizer": LaunchConfiguration("use_markers"),
            "expected_detach_count": LaunchConfiguration("expected_detach_count"),
            "score_zone_x": LaunchConfiguration("score_zone_x"),
            "score_zone_y": LaunchConfiguration("score_zone_y"),
            "score_zone_radius": LaunchConfiguration("score_zone_radius"),
        }.items(),
    )

    return LaunchDescription(arguments + [detector, mission_launch])
