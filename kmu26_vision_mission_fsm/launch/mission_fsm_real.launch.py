#!/usr/bin/env python3
"""Focused launch file for KMU26 mission FSM bringup on the real vehicle."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_bool(context, value: LaunchConfiguration) -> bool:
    return value.perform(context).strip().lower() in {"1", "true", "yes", "on"}


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("kmu26_vision_mission_fsm")
    default_scene = os.path.join(share_dir, "config", "tank_current_scene.xml")
    default_rviz = os.path.join(share_dir, "rviz", "mission_fsm.rviz")

    args = [
        DeclareLaunchArgument("use_mission_fsm", default_value="false"),
        DeclareLaunchArgument("allow_ground_truth_controller", default_value="false"),
        DeclareLaunchArgument("use_observation_mission_fsm", default_value="false"),
        DeclareLaunchArgument("use_hydrophone_estimator", default_value="true"),
        DeclareLaunchArgument("use_vision_detector", default_value="false"),
        DeclareLaunchArgument("use_vision_mission_controller", default_value="true"),
        DeclareLaunchArgument("use_pinger_homing", default_value="false"),
        DeclareLaunchArgument("use_rc_mux", default_value="true"),
        DeclareLaunchArgument("use_mission_rviz_visualizer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("mission_status_json", default_value="/tmp/kmu26_mission_fsm_status.json"),
        DeclareLaunchArgument("scene", default_value=default_scene),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("pose_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("pose_type", default_value="odometry"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("mission_rc_topic", default_value="/control/mission/rc_override"),
        DeclareLaunchArgument("pinger_rc_topic", default_value="/control/pinger/rc_override"),
        DeclareLaunchArgument("joystick_rc_topic", default_value="/control/joystick/rc_override"),
        DeclareLaunchArgument("vision_rc_topic", default_value="/control/vision/rc_override"),
        DeclareLaunchArgument("vision_enable_topic", default_value="/control/vision/enable"),
        DeclareLaunchArgument("vision_controller_state_topic", default_value="/control/vision/state"),
        DeclareLaunchArgument("vision_controller_status_topic", default_value="/control/vision/status"),
        DeclareLaunchArgument("vision_bbox_topic", default_value="/vision/buoy_bbox"),
        DeclareLaunchArgument("vision_depth_pose_scale", default_value="1.0"),
        DeclareLaunchArgument("vision_vehicle_state_timeout_s", default_value="8.0"),
        DeclareLaunchArgument("vision_work_depth_m", default_value="8.5"),
        DeclareLaunchArgument("vision_surface_depth_m", default_value="0.4"),
        DeclareLaunchArgument("vision_max_depth_m", default_value="10.8"),
        DeclareLaunchArgument("vision_depth_kp_pwm_per_m", default_value="120.0"),
        DeclareLaunchArgument(
            "vision_expected_target_count",
            default_value="0",
            description="Zero keeps one-target handoffs in SEARCH; collector feedback ends the phase.",
        ),
        DeclareLaunchArgument("vision_search_handoff_s", default_value="0.75"),
        DeclareLaunchArgument(
            "vision_complete_requires_detach_count",
            default_value="true",
            description="Reject visual completion until physical collector detach feedback agrees.",
        ),
        DeclareLaunchArgument("rc_mux_stale_timeout", default_value="0.35"),
        DeclareLaunchArgument("manual_topic", default_value="/mavros/manual_control/send"),
        DeclareLaunchArgument("command_override_topic", default_value="/uuv_mujoco/sitl/command_override"),
        DeclareLaunchArgument("buoy_status_topic", default_value="/mujoco/course_buoys/status"),
        DeclareLaunchArgument("yolo_detection_topic", default_value="/vision/buoy_observation"),
        DeclareLaunchArgument("camera_compressed_topic", default_value="/camera/camera/color/image_raw/compressed"),
        DeclareLaunchArgument("vision_model_path", default_value=""),
        DeclareLaunchArgument("vision_status_topic", default_value="/vision/buoy/status"),
        DeclareLaunchArgument("vision_detector_confidence", default_value="0.25"),
        DeclareLaunchArgument("vision_detector_inference_hz", default_value="3.0"),
        DeclareLaunchArgument("vision_detector_imgsz", default_value="640"),
        DeclareLaunchArgument("vision_detector_preprocess", default_value="true"),
        DeclareLaunchArgument("vision_detector_track_hold", default_value="1.6"),
        DeclareLaunchArgument("vision_detector_max_result_age", default_value="2.0"),
        DeclareLaunchArgument("pinger_white_min_ratio", default_value="0.18"),
        DeclareLaunchArgument("pinger_bearing_tolerance", default_value="0.45"),
        DeclareLaunchArgument("fsm_status_topic", default_value="/mission/fsm/status"),
        DeclareLaunchArgument("hydrophone_direction_topic", default_value="/homing/direction"),
        DeclareLaunchArgument("hydrophone_audio_topic", default_value="/audio"),
        DeclareLaunchArgument("hydrophone_depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("hydrophone_reference_frequency_hz", default_value="21164.0"),
        DeclareLaunchArgument("collector_state_topic", default_value="/collector/state"),
        DeclareLaunchArgument("marker_topic", default_value="/mission/rviz_markers"),
        DeclareLaunchArgument("marker_frame", default_value="odom"),
        DeclareLaunchArgument("course", default_value="all"),
        DeclareLaunchArgument("own_course", default_value="a"),
        DeclareLaunchArgument("transport", default_value="rc_override"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("wait_armed", default_value="true"),
        DeclareLaunchArgument("no_pinger", default_value="false"),
        DeclareLaunchArgument("nearest_first", default_value="true"),
        DeclareLaunchArgument("require_live_status", default_value="false"),
        DeclareLaunchArgument("surface_collect_yolo", default_value="true"),
        DeclareLaunchArgument("pinger_hydrophone", default_value="false"),
        DeclareLaunchArgument("mission_log", default_value="auto"),
        DeclareLaunchArgument("course_boundary_x", default_value="0.0"),
        DeclareLaunchArgument("course_boundary_margin", default_value="0.8"),
        DeclareLaunchArgument("course_boundary_standoff", default_value="0.7"),
        DeclareLaunchArgument("detached_exclusion_radius", default_value="1.8"),
        DeclareLaunchArgument("search_area_center_x", default_value="-8.5"),
        DeclareLaunchArgument("search_area_center_y", default_value="0.0"),
        DeclareLaunchArgument("search_area_width", default_value="9.0"),
        DeclareLaunchArgument("search_area_height", default_value="16.0"),
        DeclareLaunchArgument("search_lane_spacing", default_value="3.0"),
        DeclareLaunchArgument("search_turn_yaw_limit", default_value="0.65"),
        DeclareLaunchArgument("search_heading_tolerance", default_value="0.70"),
        DeclareLaunchArgument("search_escape_forward", default_value="0.55"),
        DeclareLaunchArgument("search_accept_bbox_ratio", default_value="0.15"),
        DeclareLaunchArgument("pinger_mode", default_value="MANUAL"),
        DeclareLaunchArgument(
            "pinger_controller",
            default_value="active_range",
            description="active_range (validated moving-sensor localizer) or direction (legacy direct follower)",
        ),
        DeclareLaunchArgument("tank_max_depth_m", default_value="0.0"),
        DeclareLaunchArgument("pinger_success_range_m", default_value="1.05"),
        DeclareLaunchArgument("pinger_success_hold_s", default_value="0.8"),
        DeclareLaunchArgument("pinger_max_runtime_s", default_value="0.0"),
        # This command remains below the phase-unwrapping radial-speed limit of
        # the unmodified 4096-sample, 96 kHz estimator at 21.164 kHz.
        DeclareLaunchArgument("pinger_forward_fast", default_value="0.55"),
        DeclareLaunchArgument("pinger_forward_turn", default_value="0.14"),
        DeclareLaunchArgument("pinger_heading_drive_tolerance", default_value="0.22"),
        DeclareLaunchArgument("pinger_near_slow_range", default_value="8.0"),
        DeclareLaunchArgument("pinger_near_forward", default_value="0.30"),
        DeclareLaunchArgument("pinger_final_slow_range", default_value="2.5"),
        DeclareLaunchArgument("pinger_final_forward", default_value="0.18"),
        DeclareLaunchArgument("pinger_probe_forward", default_value="0.24"),
        DeclareLaunchArgument("pinger_probe_yaw", default_value="0.30"),
        DeclareLaunchArgument("pinger_min_probe_s", default_value="6.0"),
        DeclareLaunchArgument("pinger_homing_leg_s", default_value="10.0"),
        DeclareLaunchArgument("pinger_homing_sway_amplitude", default_value="0.0"),
        DeclareLaunchArgument("pinger_homing_sway_period_s", default_value="6.0"),
        DeclareLaunchArgument("pinger_homing_yaw_dither_amplitude", default_value="0.06"),
        DeclareLaunchArgument("pinger_homing_yaw_dither_period_s", default_value="5.0"),
        DeclareLaunchArgument("pinger_homing_drive_s", default_value="0.0"),
        DeclareLaunchArgument("pinger_homing_pause_s", default_value="0.0"),
        DeclareLaunchArgument("pinger_spin_rehome_yaw_rad", default_value="5.75"),
        DeclareLaunchArgument("pinger_spin_rehome_max_translation", default_value="0.75"),
        DeclareLaunchArgument("pinger_spin_rehome_stop_s", default_value="1.0"),
        DeclareLaunchArgument("pinger_range_regression_margin", default_value="0.40"),
        DeclareLaunchArgument("pinger_range_regression_hold", default_value="0.65"),
        DeclareLaunchArgument("pinger_range_progress_grace", default_value="0.80"),
        DeclareLaunchArgument("pinger_range_progress_check", default_value="2.8"),
        DeclareLaunchArgument("pinger_range_min_progress", default_value="0.12"),
        DeclareLaunchArgument("pinger_doppler_approach_delta", default_value="0.003"),
        DeclareLaunchArgument("pinger_doppler_recede_delta", default_value="0.003"),
        DeclareLaunchArgument("pinger_doppler_reversal_max_range", default_value="4.0"),
        DeclareLaunchArgument("pinger_doppler_approach_samples", default_value="3"),
        DeclareLaunchArgument("pinger_doppler_recede_samples", default_value="2"),
        DeclareLaunchArgument("pinger_doppler_brake_s", default_value="0.70"),
        DeclareLaunchArgument("pinger_doppler_brake_reverse", default_value="-0.22"),
        DeclareLaunchArgument(
            "allow_internal_hydrophone_direction_fallback", default_value="false"),
        DeclareLaunchArgument("prefer_upstream_hydrophone_direction", default_value="true"),
        DeclareLaunchArgument("prefer_internal_hydrophone_direction", default_value="true"),
        DeclareLaunchArgument(
            "pinger_position_fit_bearing_tolerance", default_value="0.45"),
        DeclareLaunchArgument("use_acoustic_position_fusion", default_value="false"),
        DeclareLaunchArgument("pinger_position_fit_timeout_s", default_value="8.0"),
        DeclareLaunchArgument("pinger_acoustic_position_lock_range", default_value="1.4"),
        DeclareLaunchArgument("pinger_acoustic_position_min_range", default_value="0.0"),
        DeclareLaunchArgument("use_phase_range_position_fusion", default_value="false"),
        DeclareLaunchArgument("phase_range_position_timeout_s", default_value="120.0"),
        DeclareLaunchArgument("phase_range_min_fit_duration_s", default_value="11.5"),
        DeclareLaunchArgument("phase_range_measurement_delay_s", default_value="0.128"),
        DeclareLaunchArgument("pinger_max_probe_s", default_value="18.0"),
        DeclareLaunchArgument("pinger_yolo_acoustic_range", default_value="1.3"),
        DeclareLaunchArgument("pinger_yolo_min_bbox_ratio", default_value="0.02"),
        DeclareLaunchArgument("require_pinger_yolo_for_capture", default_value="true"),
        DeclareLaunchArgument("pinger_acoustic_crawl_forward", default_value="0.20"),
        DeclareLaunchArgument("pinger_acoustic_crawl_bearing", default_value="0.45"),
        DeclareLaunchArgument("pinger_acoustic_capture_range", default_value="0.78"),
        DeclareLaunchArgument("pinger_acoustic_capture_bearing", default_value="0.16"),
        DeclareLaunchArgument("pinger_yaw_gain", default_value="1.15"),
        DeclareLaunchArgument("pinger_depth_z", default_value="-8.5"),
        DeclareLaunchArgument("pinger_acoustic_source_depth_z", default_value="-8.865"),
        DeclareLaunchArgument("pinger_transit_depth_z", default_value="-7.7"),
        DeclareLaunchArgument("pinger_depth_transition_range", default_value="3.0"),
        DeclareLaunchArgument("pinger_heave_gain", default_value="0.42"),
        DeclareLaunchArgument("pinger_heave_limit", default_value="0.34"),
        DeclareLaunchArgument("pinger_vertical_direction_deadband", default_value="0.08"),
        DeclareLaunchArgument("pinger_vertical_alignment_tolerance", default_value="0.22"),
        DeclareLaunchArgument("pinger_vertical_forward_limit", default_value="0.0"),
        DeclareLaunchArgument("pinger_vertical_yaw_limit", default_value="0.08"),
        DeclareLaunchArgument("pinger_acoustic_vertical_zero_range", default_value="0.80"),
        DeclareLaunchArgument("pinger_acoustic_vertical_full_range", default_value="2.0"),
        DeclareLaunchArgument("pinger_camera_hfov_rad", default_value="1.211"),
        DeclareLaunchArgument("pinger_capture_commit_range", default_value="0.55"),
        DeclareLaunchArgument("pinger_rake_lane_blend_start", default_value="0.90"),
        DeclareLaunchArgument("pinger_rake_lane_full_range", default_value="0.45"),
        DeclareLaunchArgument("underwater_target_depth_z", default_value="-8.5"),
        DeclareLaunchArgument("pinger_yolo_min_confidence", default_value="0.45"),
        DeclareLaunchArgument("pinger_yolo_min_bbox_height_ratio", default_value="0.08"),
        DeclareLaunchArgument("pinger_visual_reacquire_timeout", default_value="1.0"),
        DeclareLaunchArgument("pinger_auto_arm", default_value="false"),
        DeclareLaunchArgument("pinger_auto_mode", default_value="true"),
        DeclareLaunchArgument("pinger_use_yolo_final", default_value="true"),
        DeclareLaunchArgument("mission_enabled", default_value="false"),
        DeclareLaunchArgument("observation_use_pinger_first", default_value="true"),
        DeclareLaunchArgument("observation_start_surface", default_value="false"),
        DeclareLaunchArgument("expected_detach_count", default_value="8"),
        DeclareLaunchArgument("expected_net_count", default_value="13"),
        DeclareLaunchArgument("score_zone_x", default_value="-6.8"),
        DeclareLaunchArgument("score_zone_y", default_value="0.0"),
        DeclareLaunchArgument("score_zone_radius", default_value="0.8"),
        DeclareLaunchArgument("surface_collect_depth_z", default_value="-0.31"),
        DeclareLaunchArgument("surface_collect_depth_tolerance", default_value="0.12"),
        DeclareLaunchArgument("surface_yaw_kp", default_value="0.55"),
        DeclareLaunchArgument("surface_yaw_limit", default_value="0.35"),
        DeclareLaunchArgument("surface_forward", default_value="0.45"),
        DeclareLaunchArgument("surface_turn_forward", default_value="0.0"),
        DeclareLaunchArgument("surface_center_tolerance", default_value="0.10"),
        DeclareLaunchArgument("surface_steer_timeout", default_value="0.65"),
        DeclareLaunchArgument("odom_timeout", default_value="0.8"),
        DeclareLaunchArgument("state_timeout", default_value="8.0"),
        DeclareLaunchArgument("vehicle_disconnect_grace", default_value="0.0"),
        DeclareLaunchArgument("vision_target_class", default_value="buoy"),
        DeclareLaunchArgument("vision_min_confidence", default_value="0.015"),
        DeclareLaunchArgument("vision_observation_timeout", default_value="2.5"),
        DeclareLaunchArgument("vision_alignment_hold", default_value="0.15"),
        DeclareLaunchArgument("vision_fine_height_ratio", default_value="0.18"),
        DeclareLaunchArgument("vision_capture_height_ratio", default_value="0.32"),
        DeclareLaunchArgument("vision_capture_drive_s", default_value="1.4"),
        DeclareLaunchArgument("vision_capture_insert_forward", default_value="0.28"),
        DeclareLaunchArgument("vision_capture_backoff_s", default_value="0.55"),
        DeclareLaunchArgument("vision_capture_backoff_forward", default_value="-0.16"),
        DeclareLaunchArgument("vision_capture_heading_kp", default_value="0.90"),
        DeclareLaunchArgument("vision_capture_heading_kd", default_value="0.15"),
        DeclareLaunchArgument("vision_capture_heading_yaw_limit", default_value="0.18"),
        DeclareLaunchArgument("vision_capture_center_tolerance_x", default_value="0.08"),
        DeclareLaunchArgument("vision_capture_center_tolerance_y", default_value="0.12"),
        DeclareLaunchArgument("vision_capture_alignment_hold_s", default_value="0.35"),
        DeclareLaunchArgument("vision_capture_aim_offset_x", default_value="0.0"),
        DeclareLaunchArgument("vision_capture_aim_offset_y", default_value="0.0"),
        DeclareLaunchArgument("vision_center_tolerance", default_value="0.16"),
        DeclareLaunchArgument("vision_yaw_kp", default_value="0.30"),
        DeclareLaunchArgument("vision_yaw_kd", default_value="0.02"),
        DeclareLaunchArgument("vision_command_slew", default_value="0.8"),
        DeclareLaunchArgument("vision_horizontal_fov_deg", default_value="69.4"),
    ]

    use_mission_fsm = LaunchConfiguration("use_mission_fsm")
    allow_ground_truth_controller = LaunchConfiguration("allow_ground_truth_controller")
    use_observation_mission_fsm = LaunchConfiguration("use_observation_mission_fsm")
    use_hydrophone_estimator = LaunchConfiguration("use_hydrophone_estimator")
    use_vision_detector = LaunchConfiguration("use_vision_detector")
    use_vision_mission_controller = LaunchConfiguration("use_vision_mission_controller")
    use_pinger_homing = LaunchConfiguration("use_pinger_homing")
    use_rc_mux = LaunchConfiguration("use_rc_mux")
    use_mission_rviz_visualizer = LaunchConfiguration("use_mission_rviz_visualizer")
    use_rviz = LaunchConfiguration("use_rviz")
    mission_status_json = LaunchConfiguration("mission_status_json")
    scene = LaunchConfiguration("scene")
    rviz_config = LaunchConfiguration("rviz_config")
    pose_topic = LaunchConfiguration("pose_topic")
    pose_type = LaunchConfiguration("pose_type")
    state_topic = LaunchConfiguration("state_topic")
    rc_topic = LaunchConfiguration("rc_topic")
    mission_rc_topic = LaunchConfiguration("mission_rc_topic")
    pinger_rc_topic = LaunchConfiguration("pinger_rc_topic")
    joystick_rc_topic = LaunchConfiguration("joystick_rc_topic")
    vision_rc_topic = LaunchConfiguration("vision_rc_topic")
    vision_enable_topic = LaunchConfiguration("vision_enable_topic")
    vision_controller_state_topic = LaunchConfiguration("vision_controller_state_topic")
    vision_controller_status_topic = LaunchConfiguration("vision_controller_status_topic")
    vision_bbox_topic = LaunchConfiguration("vision_bbox_topic")
    vision_depth_pose_scale = LaunchConfiguration("vision_depth_pose_scale")
    vision_vehicle_state_timeout_s = LaunchConfiguration("vision_vehicle_state_timeout_s")
    vision_work_depth_m = LaunchConfiguration("vision_work_depth_m")
    vision_surface_depth_m = LaunchConfiguration("vision_surface_depth_m")
    vision_max_depth_m = LaunchConfiguration("vision_max_depth_m")
    vision_depth_kp_pwm_per_m = LaunchConfiguration("vision_depth_kp_pwm_per_m")
    vision_expected_target_count = LaunchConfiguration("vision_expected_target_count")
    vision_search_handoff_s = LaunchConfiguration("vision_search_handoff_s")
    vision_complete_requires_detach_count = LaunchConfiguration(
        "vision_complete_requires_detach_count")
    rc_mux_stale_timeout = LaunchConfiguration("rc_mux_stale_timeout")
    manual_topic = LaunchConfiguration("manual_topic")
    command_override_topic = LaunchConfiguration("command_override_topic")
    buoy_status_topic = LaunchConfiguration("buoy_status_topic")
    yolo_detection_topic = LaunchConfiguration("yolo_detection_topic")
    camera_compressed_topic = LaunchConfiguration("camera_compressed_topic")
    vision_model_path = LaunchConfiguration("vision_model_path")
    vision_status_topic = LaunchConfiguration("vision_status_topic")
    vision_detector_confidence = LaunchConfiguration("vision_detector_confidence")
    vision_detector_inference_hz = LaunchConfiguration("vision_detector_inference_hz")
    vision_detector_imgsz = LaunchConfiguration("vision_detector_imgsz")
    vision_detector_preprocess = LaunchConfiguration("vision_detector_preprocess")
    vision_detector_track_hold = LaunchConfiguration("vision_detector_track_hold")
    vision_detector_max_result_age = LaunchConfiguration("vision_detector_max_result_age")
    pinger_white_min_ratio = LaunchConfiguration("pinger_white_min_ratio")
    pinger_bearing_tolerance = LaunchConfiguration("pinger_bearing_tolerance")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    hydrophone_direction_topic = LaunchConfiguration("hydrophone_direction_topic")
    hydrophone_audio_topic = LaunchConfiguration("hydrophone_audio_topic")
    hydrophone_depth_topic = LaunchConfiguration("hydrophone_depth_topic")
    hydrophone_reference_frequency_hz = LaunchConfiguration("hydrophone_reference_frequency_hz")
    collector_state_topic = LaunchConfiguration("collector_state_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    marker_frame = LaunchConfiguration("marker_frame")
    course = LaunchConfiguration("course")
    own_course = LaunchConfiguration("own_course")
    transport = LaunchConfiguration("transport")
    rate_hz = LaunchConfiguration("rate_hz")
    dry_run = LaunchConfiguration("dry_run")
    wait_armed = LaunchConfiguration("wait_armed")
    no_pinger = LaunchConfiguration("no_pinger")
    nearest_first = LaunchConfiguration("nearest_first")
    require_live_status = LaunchConfiguration("require_live_status")
    surface_collect_yolo = LaunchConfiguration("surface_collect_yolo")
    pinger_hydrophone = LaunchConfiguration("pinger_hydrophone")
    mission_log = LaunchConfiguration("mission_log")
    course_boundary_x = LaunchConfiguration("course_boundary_x")
    course_boundary_margin = LaunchConfiguration("course_boundary_margin")
    course_boundary_standoff = LaunchConfiguration("course_boundary_standoff")
    detached_exclusion_radius = LaunchConfiguration("detached_exclusion_radius")
    search_area_center_x = LaunchConfiguration("search_area_center_x")
    search_area_center_y = LaunchConfiguration("search_area_center_y")
    search_area_width = LaunchConfiguration("search_area_width")
    search_area_height = LaunchConfiguration("search_area_height")
    search_lane_spacing = LaunchConfiguration("search_lane_spacing")
    search_turn_yaw_limit = LaunchConfiguration("search_turn_yaw_limit")
    search_heading_tolerance = LaunchConfiguration("search_heading_tolerance")
    search_escape_forward = LaunchConfiguration("search_escape_forward")
    search_accept_bbox_ratio = LaunchConfiguration("search_accept_bbox_ratio")
    pinger_mode = LaunchConfiguration("pinger_mode")
    pinger_controller = LaunchConfiguration("pinger_controller")
    tank_max_depth_m = LaunchConfiguration("tank_max_depth_m")
    pinger_success_range_m = LaunchConfiguration("pinger_success_range_m")
    pinger_success_hold_s = LaunchConfiguration("pinger_success_hold_s")
    pinger_max_runtime_s = LaunchConfiguration("pinger_max_runtime_s")
    pinger_forward_fast = LaunchConfiguration("pinger_forward_fast")
    pinger_forward_turn = LaunchConfiguration("pinger_forward_turn")
    pinger_heading_drive_tolerance = LaunchConfiguration(
        "pinger_heading_drive_tolerance")
    pinger_near_slow_range = LaunchConfiguration("pinger_near_slow_range")
    pinger_near_forward = LaunchConfiguration("pinger_near_forward")
    pinger_final_slow_range = LaunchConfiguration("pinger_final_slow_range")
    pinger_final_forward = LaunchConfiguration("pinger_final_forward")
    pinger_probe_forward = LaunchConfiguration("pinger_probe_forward")
    pinger_probe_yaw = LaunchConfiguration("pinger_probe_yaw")
    pinger_min_probe_s = LaunchConfiguration("pinger_min_probe_s")
    pinger_homing_leg_s = LaunchConfiguration("pinger_homing_leg_s")
    pinger_homing_sway_amplitude = LaunchConfiguration("pinger_homing_sway_amplitude")
    pinger_homing_sway_period_s = LaunchConfiguration("pinger_homing_sway_period_s")
    pinger_homing_yaw_dither_amplitude = LaunchConfiguration(
        "pinger_homing_yaw_dither_amplitude")
    pinger_homing_yaw_dither_period_s = LaunchConfiguration(
        "pinger_homing_yaw_dither_period_s")
    pinger_homing_drive_s = LaunchConfiguration("pinger_homing_drive_s")
    pinger_homing_pause_s = LaunchConfiguration("pinger_homing_pause_s")
    pinger_spin_rehome_yaw_rad = LaunchConfiguration("pinger_spin_rehome_yaw_rad")
    pinger_spin_rehome_max_translation = LaunchConfiguration(
        "pinger_spin_rehome_max_translation")
    pinger_spin_rehome_stop_s = LaunchConfiguration("pinger_spin_rehome_stop_s")
    pinger_range_regression_margin = LaunchConfiguration("pinger_range_regression_margin")
    pinger_range_regression_hold = LaunchConfiguration("pinger_range_regression_hold")
    pinger_range_progress_grace = LaunchConfiguration("pinger_range_progress_grace")
    pinger_range_progress_check = LaunchConfiguration("pinger_range_progress_check")
    pinger_range_min_progress = LaunchConfiguration("pinger_range_min_progress")
    pinger_doppler_approach_delta = LaunchConfiguration("pinger_doppler_approach_delta")
    pinger_doppler_recede_delta = LaunchConfiguration("pinger_doppler_recede_delta")
    pinger_doppler_reversal_max_range = LaunchConfiguration(
        "pinger_doppler_reversal_max_range")
    pinger_doppler_approach_samples = LaunchConfiguration("pinger_doppler_approach_samples")
    pinger_doppler_recede_samples = LaunchConfiguration("pinger_doppler_recede_samples")
    pinger_doppler_brake_s = LaunchConfiguration("pinger_doppler_brake_s")
    pinger_doppler_brake_reverse = LaunchConfiguration("pinger_doppler_brake_reverse")
    allow_internal_hydrophone_direction_fallback = LaunchConfiguration(
        "allow_internal_hydrophone_direction_fallback")
    prefer_upstream_hydrophone_direction = LaunchConfiguration(
        "prefer_upstream_hydrophone_direction")
    prefer_internal_hydrophone_direction = LaunchConfiguration(
        "prefer_internal_hydrophone_direction")
    pinger_position_fit_bearing_tolerance = LaunchConfiguration(
        "pinger_position_fit_bearing_tolerance")
    use_acoustic_position_fusion = LaunchConfiguration("use_acoustic_position_fusion")
    pinger_position_fit_timeout_s = LaunchConfiguration("pinger_position_fit_timeout_s")
    pinger_acoustic_position_lock_range = LaunchConfiguration(
        "pinger_acoustic_position_lock_range")
    pinger_acoustic_position_min_range = LaunchConfiguration(
        "pinger_acoustic_position_min_range")
    use_phase_range_position_fusion = LaunchConfiguration(
        "use_phase_range_position_fusion")
    phase_range_position_timeout_s = LaunchConfiguration(
        "phase_range_position_timeout_s")
    phase_range_min_fit_duration_s = LaunchConfiguration(
        "phase_range_min_fit_duration_s")
    phase_range_measurement_delay_s = LaunchConfiguration(
        "phase_range_measurement_delay_s")
    pinger_max_probe_s = LaunchConfiguration("pinger_max_probe_s")
    pinger_yolo_acoustic_range = LaunchConfiguration("pinger_yolo_acoustic_range")
    pinger_yolo_min_bbox_ratio = LaunchConfiguration("pinger_yolo_min_bbox_ratio")
    require_pinger_yolo_for_capture = LaunchConfiguration(
        "require_pinger_yolo_for_capture")
    pinger_acoustic_crawl_forward = LaunchConfiguration("pinger_acoustic_crawl_forward")
    pinger_acoustic_crawl_bearing = LaunchConfiguration("pinger_acoustic_crawl_bearing")
    pinger_acoustic_capture_range = LaunchConfiguration("pinger_acoustic_capture_range")
    pinger_acoustic_capture_bearing = LaunchConfiguration("pinger_acoustic_capture_bearing")
    pinger_yaw_gain = LaunchConfiguration("pinger_yaw_gain")
    pinger_depth_z = LaunchConfiguration("pinger_depth_z")
    pinger_acoustic_source_depth_z = LaunchConfiguration("pinger_acoustic_source_depth_z")
    pinger_transit_depth_z = LaunchConfiguration("pinger_transit_depth_z")
    pinger_depth_transition_range = LaunchConfiguration("pinger_depth_transition_range")
    pinger_heave_gain = LaunchConfiguration("pinger_heave_gain")
    pinger_heave_limit = LaunchConfiguration("pinger_heave_limit")
    pinger_vertical_direction_deadband = LaunchConfiguration(
        "pinger_vertical_direction_deadband")
    pinger_vertical_alignment_tolerance = LaunchConfiguration(
        "pinger_vertical_alignment_tolerance")
    pinger_vertical_forward_limit = LaunchConfiguration("pinger_vertical_forward_limit")
    pinger_vertical_yaw_limit = LaunchConfiguration("pinger_vertical_yaw_limit")
    pinger_acoustic_vertical_zero_range = LaunchConfiguration(
        "pinger_acoustic_vertical_zero_range")
    pinger_acoustic_vertical_full_range = LaunchConfiguration(
        "pinger_acoustic_vertical_full_range")
    pinger_camera_hfov_rad = LaunchConfiguration("pinger_camera_hfov_rad")
    pinger_capture_commit_range = LaunchConfiguration("pinger_capture_commit_range")
    pinger_rake_lane_blend_start = LaunchConfiguration("pinger_rake_lane_blend_start")
    pinger_rake_lane_full_range = LaunchConfiguration("pinger_rake_lane_full_range")
    underwater_target_depth_z = LaunchConfiguration("underwater_target_depth_z")
    pinger_yolo_min_confidence = LaunchConfiguration("pinger_yolo_min_confidence")
    pinger_yolo_min_bbox_height_ratio = LaunchConfiguration("pinger_yolo_min_bbox_height_ratio")
    pinger_auto_arm = LaunchConfiguration("pinger_auto_arm")
    pinger_auto_mode = LaunchConfiguration("pinger_auto_mode")
    pinger_use_yolo_final = LaunchConfiguration("pinger_use_yolo_final")
    mission_enabled = LaunchConfiguration("mission_enabled")
    observation_use_pinger_first = LaunchConfiguration("observation_use_pinger_first")
    observation_start_surface = LaunchConfiguration("observation_start_surface")
    expected_detach_count = LaunchConfiguration("expected_detach_count")
    expected_net_count = LaunchConfiguration("expected_net_count")
    score_zone_x = LaunchConfiguration("score_zone_x")
    score_zone_y = LaunchConfiguration("score_zone_y")
    score_zone_radius = LaunchConfiguration("score_zone_radius")
    surface_collect_depth_z = LaunchConfiguration("surface_collect_depth_z")
    surface_collect_depth_tolerance = LaunchConfiguration("surface_collect_depth_tolerance")
    surface_yaw_kp = LaunchConfiguration("surface_yaw_kp")
    surface_yaw_limit = LaunchConfiguration("surface_yaw_limit")
    surface_forward = LaunchConfiguration("surface_forward")
    surface_turn_forward = LaunchConfiguration("surface_turn_forward")
    surface_center_tolerance = LaunchConfiguration("surface_center_tolerance")
    surface_steer_timeout = LaunchConfiguration("surface_steer_timeout")
    odom_timeout = LaunchConfiguration("odom_timeout")
    state_timeout = LaunchConfiguration("state_timeout")
    vehicle_disconnect_grace = LaunchConfiguration("vehicle_disconnect_grace")
    vision_target_class = LaunchConfiguration("vision_target_class")
    vision_min_confidence = LaunchConfiguration("vision_min_confidence")
    vision_observation_timeout = LaunchConfiguration("vision_observation_timeout")
    pinger_visual_reacquire_timeout = LaunchConfiguration(
        "pinger_visual_reacquire_timeout")
    vision_alignment_hold = LaunchConfiguration("vision_alignment_hold")
    vision_fine_height_ratio = LaunchConfiguration("vision_fine_height_ratio")
    vision_capture_height_ratio = LaunchConfiguration("vision_capture_height_ratio")
    vision_capture_drive_s = LaunchConfiguration("vision_capture_drive_s")
    vision_capture_insert_forward = LaunchConfiguration("vision_capture_insert_forward")
    vision_capture_backoff_s = LaunchConfiguration("vision_capture_backoff_s")
    vision_capture_backoff_forward = LaunchConfiguration("vision_capture_backoff_forward")
    vision_capture_heading_kp = LaunchConfiguration("vision_capture_heading_kp")
    vision_capture_heading_kd = LaunchConfiguration("vision_capture_heading_kd")
    vision_capture_heading_yaw_limit = LaunchConfiguration(
        "vision_capture_heading_yaw_limit")
    vision_capture_center_tolerance_x = LaunchConfiguration(
        "vision_capture_center_tolerance_x")
    vision_capture_center_tolerance_y = LaunchConfiguration(
        "vision_capture_center_tolerance_y")
    vision_capture_alignment_hold_s = LaunchConfiguration(
        "vision_capture_alignment_hold_s")
    vision_capture_aim_offset_x = LaunchConfiguration("vision_capture_aim_offset_x")
    vision_capture_aim_offset_y = LaunchConfiguration("vision_capture_aim_offset_y")
    vision_center_tolerance = LaunchConfiguration("vision_center_tolerance")
    vision_yaw_kp = LaunchConfiguration("vision_yaw_kp")
    vision_yaw_kd = LaunchConfiguration("vision_yaw_kd")
    vision_command_slew = LaunchConfiguration("vision_command_slew")
    vision_horizontal_fov_deg = LaunchConfiguration("vision_horizontal_fov_deg")

    def control_nodes(context):
        nodes = []
        mux_enabled = _launch_bool(context, use_rc_mux)
        delegated_vision_enabled = (
            _launch_bool(context, use_observation_mission_fsm)
            and _launch_bool(context, use_vision_mission_controller)
        )
        if _launch_bool(context, use_mission_fsm) and _launch_bool(context, use_observation_mission_fsm):
            raise RuntimeError("ground-truth and observation mission FSMs cannot run together")
        if delegated_vision_enabled and not mux_enabled:
            raise RuntimeError(
                "use_vision_mission_controller requires use_rc_mux:=true so only the mux owns MAVROS"
            )
        if mux_enabled:
            nodes.append(Node(
                package="kmu26_pinger_homing",
                executable="rc_override_mux",
                name="rc_override_mux",
                output="screen",
                parameters=[{
                    "output_topic": rc_topic,
                    "mission_topic": mission_rc_topic,
                    "pinger_topic": pinger_rc_topic,
                    "joystick_topic": joystick_rc_topic,
                    "vision_topic": vision_rc_topic,
                    "stale_timeout_s": ParameterValue(rc_mux_stale_timeout, value_type=float),
                }],
            ))

        needs_hydrophone_estimator = (
            _launch_bool(context, use_observation_mission_fsm) or
            _launch_bool(context, use_pinger_homing)
        )
        if needs_hydrophone_estimator and _launch_bool(context, use_hydrophone_estimator):
            nodes.append(Node(
                package="audio_capture",
                executable="audio_phase_estimator",
                name="mission_audio_phase_estimator",
                output="screen",
                remappings=[
                    ("/audio_boosted", hydrophone_audio_topic),
                    ("/odometry/filtered", pose_topic),
                    ("/depth/pose", hydrophone_depth_topic),
                    ("/homing/direction", hydrophone_direction_topic),
                ],
                parameters=[{
                    "reference_frequency_hz": ParameterValue(
                        hydrophone_reference_frequency_hz, value_type=float),
                    "initial_demodulation_frequency_hz": ParameterValue(
                        hydrophone_reference_frequency_hz, value_type=float),
                    "enable_frequency_acquisition": False,
                }],
            ))

        if _launch_bool(context, use_observation_mission_fsm):
            nodes.append(Node(
                package="kmu26_vision_mission_fsm",
                executable="observation_mission_fsm",
                name="observation_mission_fsm",
                output="screen",
                parameters=[{
                    "observation_topic": yolo_detection_topic,
                    "collector_topic": collector_state_topic,
                    "odom_topic": pose_topic,
                    "state_topic": state_topic,
                    "status_json_path": mission_status_json,
                    "output_topic": mission_rc_topic if mux_enabled else rc_topic,
                    "delegate_vision_control": delegated_vision_enabled,
                    "vision_enable_topic": vision_enable_topic,
                    "vision_state_topic": vision_controller_state_topic,
                    "vision_search_handoff_s": ParameterValue(
                        vision_search_handoff_s, value_type=float),
                    "vision_complete_requires_detach_count": ParameterValue(
                        vision_complete_requires_detach_count, value_type=bool),
                    "enabled": ParameterValue(mission_enabled, value_type=bool),
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "require_armed": ParameterValue(wait_armed, value_type=bool),
                    "expected_detach_count": ParameterValue(expected_detach_count, value_type=int),
                    "expected_net_count": ParameterValue(expected_net_count, value_type=int),
                    "use_pinger_first": ParameterValue(
                        observation_use_pinger_first, value_type=bool),
                    "start_surface_phase": ParameterValue(
                        observation_start_surface, value_type=bool),
                    "hydrophone_direction_topic": hydrophone_direction_topic,
                    "hydrophone_direction_frame": "world",
                    "pinger_depth_z": ParameterValue(pinger_depth_z, value_type=float),
                    "pinger_acoustic_source_depth_z": ParameterValue(
                        pinger_acoustic_source_depth_z, value_type=float),
                    "pinger_transit_depth_z": ParameterValue(
                        pinger_transit_depth_z, value_type=float),
                    "pinger_depth_transition_range_m": ParameterValue(
                        pinger_depth_transition_range, value_type=float),
                    "pinger_heave_kp": ParameterValue(pinger_heave_gain, value_type=float),
                    "pinger_heave_limit": ParameterValue(pinger_heave_limit, value_type=float),
                    "pinger_vertical_direction_deadband": ParameterValue(
                        pinger_vertical_direction_deadband, value_type=float),
                    "pinger_vertical_alignment_tolerance": ParameterValue(
                        pinger_vertical_alignment_tolerance, value_type=float),
                    "pinger_vertical_forward_limit": ParameterValue(
                        pinger_vertical_forward_limit, value_type=float),
                    "pinger_vertical_yaw_limit": ParameterValue(
                        pinger_vertical_yaw_limit, value_type=float),
                    "pinger_acoustic_vertical_zero_range_m": ParameterValue(
                        pinger_acoustic_vertical_zero_range, value_type=float),
                    "pinger_acoustic_vertical_full_range_m": ParameterValue(
                        pinger_acoustic_vertical_full_range, value_type=float),
                    "pinger_camera_hfov_rad": ParameterValue(
                        pinger_camera_hfov_rad, value_type=float),
                    "pinger_capture_commit_range_m": ParameterValue(
                        pinger_capture_commit_range, value_type=float),
                    "pinger_rake_lane_blend_start_m": ParameterValue(
                        pinger_rake_lane_blend_start, value_type=float),
                    "pinger_rake_lane_full_range_m": ParameterValue(
                        pinger_rake_lane_full_range, value_type=float),
                    "underwater_target_depth_z": ParameterValue(
                        underwater_target_depth_z, value_type=float),
                    "pinger_forward_fast": ParameterValue(pinger_forward_fast, value_type=float),
                    "pinger_forward_turn": ParameterValue(pinger_forward_turn, value_type=float),
                    "pinger_heading_drive_tolerance_rad": ParameterValue(
                        pinger_heading_drive_tolerance, value_type=float),
                    "pinger_near_slow_range_m": ParameterValue(
                        pinger_near_slow_range, value_type=float),
                    "pinger_near_forward": ParameterValue(
                        pinger_near_forward, value_type=float),
                    "pinger_final_slow_range_m": ParameterValue(
                        pinger_final_slow_range, value_type=float),
                    "pinger_final_forward": ParameterValue(
                        pinger_final_forward, value_type=float),
                    "pinger_probe_forward": ParameterValue(
                        pinger_probe_forward, value_type=float),
                    "pinger_probe_yaw": ParameterValue(pinger_probe_yaw, value_type=float),
                    "pinger_min_probe_s": ParameterValue(pinger_min_probe_s, value_type=float),
                    "pinger_homing_leg_s": ParameterValue(
                        pinger_homing_leg_s, value_type=float),
                    "pinger_homing_sway_amplitude": ParameterValue(
                        pinger_homing_sway_amplitude, value_type=float),
                    "pinger_homing_sway_period_s": ParameterValue(
                        pinger_homing_sway_period_s, value_type=float),
                    "pinger_homing_yaw_dither_amplitude": ParameterValue(
                        pinger_homing_yaw_dither_amplitude, value_type=float),
                    "pinger_homing_yaw_dither_period_s": ParameterValue(
                        pinger_homing_yaw_dither_period_s, value_type=float),
                    "pinger_homing_drive_s": ParameterValue(
                        pinger_homing_drive_s, value_type=float),
                    "pinger_homing_pause_s": ParameterValue(
                        pinger_homing_pause_s, value_type=float),
                    "pinger_spin_rehome_yaw_rad": ParameterValue(
                        pinger_spin_rehome_yaw_rad, value_type=float),
                    "pinger_spin_rehome_max_translation_m": ParameterValue(
                        pinger_spin_rehome_max_translation, value_type=float),
                    "pinger_spin_rehome_stop_s": ParameterValue(
                        pinger_spin_rehome_stop_s, value_type=float),
                    "pinger_range_regression_margin_m": ParameterValue(
                        pinger_range_regression_margin, value_type=float),
                    "pinger_range_regression_hold_s": ParameterValue(
                        pinger_range_regression_hold, value_type=float),
                    "pinger_range_progress_grace_s": ParameterValue(
                        pinger_range_progress_grace, value_type=float),
                    "pinger_range_progress_check_s": ParameterValue(
                        pinger_range_progress_check, value_type=float),
                    "pinger_range_min_progress_m": ParameterValue(
                        pinger_range_min_progress, value_type=float),
                    "pinger_doppler_approach_delta_m": ParameterValue(
                        pinger_doppler_approach_delta, value_type=float),
                    "pinger_doppler_recede_delta_m": ParameterValue(
                        pinger_doppler_recede_delta, value_type=float),
                    "pinger_doppler_reversal_max_range_m": ParameterValue(
                        pinger_doppler_reversal_max_range, value_type=float),
                    "pinger_doppler_approach_samples": ParameterValue(
                        pinger_doppler_approach_samples, value_type=int),
                    "pinger_doppler_recede_samples": ParameterValue(
                        pinger_doppler_recede_samples, value_type=int),
                    "pinger_doppler_brake_s": ParameterValue(
                        pinger_doppler_brake_s, value_type=float),
                    "pinger_doppler_brake_reverse": ParameterValue(
                        pinger_doppler_brake_reverse, value_type=float),
                    "allow_internal_hydrophone_direction_fallback": ParameterValue(
                        allow_internal_hydrophone_direction_fallback, value_type=bool),
                    "prefer_upstream_hydrophone_direction": ParameterValue(
                        prefer_upstream_hydrophone_direction, value_type=bool),
                    "prefer_internal_hydrophone_direction": ParameterValue(
                        prefer_internal_hydrophone_direction, value_type=bool),
                    "pinger_position_fit_bearing_tolerance_rad": ParameterValue(
                        pinger_position_fit_bearing_tolerance, value_type=float),
                    "use_acoustic_position_fusion": ParameterValue(
                        use_acoustic_position_fusion, value_type=bool),
                    "pinger_position_fit_timeout_s": ParameterValue(
                        pinger_position_fit_timeout_s, value_type=float),
                    "pinger_acoustic_position_lock_range_m": ParameterValue(
                        pinger_acoustic_position_lock_range, value_type=float),
                    "pinger_acoustic_position_min_range_m": ParameterValue(
                        pinger_acoustic_position_min_range, value_type=float),
                    "use_phase_range_position_fusion": ParameterValue(
                        use_phase_range_position_fusion, value_type=bool),
                    "phase_range_position_timeout_s": ParameterValue(
                        phase_range_position_timeout_s, value_type=float),
                    "phase_range_min_fit_duration_s": ParameterValue(
                        phase_range_min_fit_duration_s, value_type=float),
                    "phase_range_measurement_delay_s": ParameterValue(
                        phase_range_measurement_delay_s, value_type=float),
                    "pinger_max_probe_s": ParameterValue(pinger_max_probe_s, value_type=float),
                    "pinger_yolo_acoustic_range_m": ParameterValue(
                        pinger_yolo_acoustic_range, value_type=float),
                    "pinger_yolo_min_bbox_ratio": ParameterValue(
                        pinger_yolo_min_bbox_ratio, value_type=float),
                    "require_pinger_yolo_for_capture": ParameterValue(
                        require_pinger_yolo_for_capture, value_type=bool),
                    "pinger_acoustic_crawl_forward": ParameterValue(
                        pinger_acoustic_crawl_forward, value_type=float),
                    "pinger_acoustic_crawl_bearing_rad": ParameterValue(
                        pinger_acoustic_crawl_bearing, value_type=float),
                    "pinger_acoustic_capture_range_m": ParameterValue(
                        pinger_acoustic_capture_range, value_type=float),
                    "pinger_acoustic_capture_bearing_rad": ParameterValue(
                        pinger_acoustic_capture_bearing, value_type=float),
                    "own_course": own_course,
                    "course_boundary_x": ParameterValue(course_boundary_x, value_type=float),
                    "course_boundary_margin": ParameterValue(course_boundary_margin, value_type=float),
                    "course_boundary_standoff": ParameterValue(course_boundary_standoff, value_type=float),
                    "detached_exclusion_radius_m": ParameterValue(
                        detached_exclusion_radius, value_type=float),
                    "search_area_center_x": ParameterValue(search_area_center_x, value_type=float),
                    "search_area_center_y": ParameterValue(search_area_center_y, value_type=float),
                    "search_area_width_m": ParameterValue(search_area_width, value_type=float),
                    "search_area_height_m": ParameterValue(search_area_height, value_type=float),
                    "search_lane_spacing_m": ParameterValue(search_lane_spacing, value_type=float),
                    "search_turn_yaw_limit": ParameterValue(
                        search_turn_yaw_limit, value_type=float),
                    "search_heading_tolerance_rad": ParameterValue(
                        search_heading_tolerance, value_type=float),
                    "search_escape_forward": ParameterValue(
                        search_escape_forward, value_type=float),
                    "search_accept_bbox_ratio": ParameterValue(
                        search_accept_bbox_ratio, value_type=float),
                    "score_zone_x": ParameterValue(score_zone_x, value_type=float),
                    "score_zone_y": ParameterValue(score_zone_y, value_type=float),
                    "score_zone_radius": ParameterValue(score_zone_radius, value_type=float),
                    "surface_collect_depth_z": ParameterValue(
                        surface_collect_depth_z, value_type=float),
                    "surface_collect_depth_tolerance_m": ParameterValue(
                        surface_collect_depth_tolerance, value_type=float),
                    "surface_yaw_kp": ParameterValue(surface_yaw_kp, value_type=float),
                    "surface_yaw_limit": ParameterValue(surface_yaw_limit, value_type=float),
                    "surface_forward": ParameterValue(surface_forward, value_type=float),
                    "surface_turn_forward": ParameterValue(
                        surface_turn_forward, value_type=float),
                    "surface_center_tolerance": ParameterValue(
                        surface_center_tolerance, value_type=float),
                    "surface_steer_timeout_s": ParameterValue(
                        surface_steer_timeout, value_type=float),
                    "odom_timeout_s": ParameterValue(odom_timeout, value_type=float),
                    "state_timeout_s": ParameterValue(state_timeout, value_type=float),
                    "vehicle_disconnect_grace_s": ParameterValue(
                        vehicle_disconnect_grace, value_type=float),
                    "target_class_name": vision_target_class,
                    "pinger_visual_class_names": ["buoy", "stick"],
                    "underwater_visual_class_names": ["stick", "buoy"],
                    "min_confidence": ParameterValue(vision_min_confidence, value_type=float),
                    "observation_timeout_s": ParameterValue(
                        vision_observation_timeout, value_type=float),
                    "pinger_visual_reacquire_timeout_s": ParameterValue(
                        pinger_visual_reacquire_timeout, value_type=float),
                    "alignment_hold_s": ParameterValue(vision_alignment_hold, value_type=float),
                    "fine_bbox_ratio": ParameterValue(vision_fine_height_ratio, value_type=float),
                    "capture_bbox_ratio": ParameterValue(vision_capture_height_ratio, value_type=float),
                    "capture_drive_s": ParameterValue(vision_capture_drive_s, value_type=float),
                    "capture_insert_forward": ParameterValue(
                        vision_capture_insert_forward, value_type=float),
                    "capture_backoff_s": ParameterValue(
                        vision_capture_backoff_s, value_type=float),
                    "capture_backoff_forward": ParameterValue(
                        vision_capture_backoff_forward, value_type=float),
                    "capture_heading_kp": ParameterValue(
                        vision_capture_heading_kp, value_type=float),
                    "capture_heading_kd": ParameterValue(
                        vision_capture_heading_kd, value_type=float),
                    "capture_heading_yaw_limit": ParameterValue(
                        vision_capture_heading_yaw_limit, value_type=float),
                    "capture_center_tolerance_x": ParameterValue(
                        vision_capture_center_tolerance_x, value_type=float),
                    "capture_center_tolerance_y": ParameterValue(
                        vision_capture_center_tolerance_y, value_type=float),
                    "capture_alignment_hold_s": ParameterValue(
                        vision_capture_alignment_hold_s, value_type=float),
                    "capture_aim_offset_x": ParameterValue(
                        vision_capture_aim_offset_x, value_type=float),
                    "capture_aim_offset_y": ParameterValue(
                        vision_capture_aim_offset_y, value_type=float),
                    "center_tolerance": ParameterValue(vision_center_tolerance, value_type=float),
                    "yaw_kp": ParameterValue(vision_yaw_kp, value_type=float),
                    "yaw_kd": ParameterValue(vision_yaw_kd, value_type=float),
                    "command_slew_per_s": ParameterValue(vision_command_slew, value_type=float),
                }],
            ))

        if delegated_vision_enabled:
            nodes.append(Node(
                package="kmu26_vision_mission_fsm",
                executable="mission_state_machine_node",
                name="vision_mission_controller",
                output="screen",
                parameters=[{
                    "bbox_topic": vision_bbox_topic,
                    "depth_pose_topic": hydrophone_depth_topic,
                    "depth_odom_topic": pose_topic,
                    "depth_odom_scale": -1.0,
                    "depth_pose_scale": ParameterValue(
                        vision_depth_pose_scale, value_type=float),
                    "enable_topic": vision_enable_topic,
                    "state_topic": vision_controller_state_topic,
                    "status_topic": vision_controller_status_topic,
                    "vehicle_state_topic": state_topic,
                    "rc_override_topic": vision_rc_topic,
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "require_armed": ParameterValue(wait_armed, value_type=bool),
                    "vehicle_state_timeout_sec": ParameterValue(
                        vision_vehicle_state_timeout_s, value_type=float),
                    "work_depth_m": ParameterValue(vision_work_depth_m, value_type=float),
                    "surface_depth_m": ParameterValue(vision_surface_depth_m, value_type=float),
                    "max_depth_m": ParameterValue(vision_max_depth_m, value_type=float),
                    "depth_kp_pwm_per_m": ParameterValue(
                        vision_depth_kp_pwm_per_m, value_type=float),
                    "expected_target_count": ParameterValue(
                        vision_expected_target_count, value_type=int),
                    "yaw_invert": True,
                    "vertical_positive_is_up": True,
                }],
            ))

        if _launch_bool(context, use_mission_fsm):
            if not _launch_bool(context, allow_ground_truth_controller):
                raise RuntimeError(
                    "use_mission_fsm starts the simulation-only ground_truth_buoy_fsm. "
                    "It is blocked for real-vehicle launch; set "
                    "allow_ground_truth_controller:=true only in an isolated simulator.")

            mission_fsm_args = [
                "--scene", scene.perform(context),
                "--course", course.perform(context),
                "--own-course", own_course.perform(context),
                "--rate-hz", rate_hz.perform(context),
                "--transport", transport.perform(context),
                "--pose-topic", pose_topic.perform(context),
                "--pose-type", pose_type.perform(context),
                "--buoy-status-topic", buoy_status_topic.perform(context),
                "--yolo-detection-topic", yolo_detection_topic.perform(context),
                "--state-topic", state_topic.perform(context),
                "--rc-topic", (mission_rc_topic if mux_enabled else rc_topic).perform(context),
                "--manual-topic", manual_topic.perform(context),
                "--command-override-topic", command_override_topic.perform(context),
                "--mission-log", mission_log.perform(context),
                "--status-json", mission_status_json.perform(context),
                "--course-boundary-x", course_boundary_x.perform(context),
                "--course-boundary-margin", course_boundary_margin.perform(context),
                "--course-boundary-standoff", course_boundary_standoff.perform(context),
            ]
            mission_fsm_args.append(
                "--require-live-status" if _launch_bool(context, require_live_status) else "--allow-static-fallback")
            mission_fsm_args.append(
                "--surface-collect-yolo" if _launch_bool(context, surface_collect_yolo) else "--no-surface-collect-yolo")
            mission_fsm_args.append(
                "--pinger-hydrophone" if _launch_bool(context, pinger_hydrophone) else "--no-pinger-hydrophone")
            if _launch_bool(context, dry_run):
                mission_fsm_args.append("--dry-run")
            mission_fsm_args.append("--wait-armed" if _launch_bool(context, wait_armed) else "--no-wait-armed")
            if _launch_bool(context, no_pinger):
                mission_fsm_args.append("--no-pinger")
            if _launch_bool(context, nearest_first):
                mission_fsm_args.append("--nearest-first")

            nodes.append(Node(
                package="kmu26_vision_mission_fsm",
                executable="ground_truth_buoy_fsm",
                name="ground_truth_buoy_fsm",
                output="screen",
                arguments=mission_fsm_args,
            ))

        if _launch_bool(context, use_pinger_homing):
            controller_kind = pinger_controller.perform(context).strip().lower()
            if controller_kind == "active_range":
                nodes.append(Node(
                    package="kmu26_pinger_homing",
                    executable="single_hydrophone_homing_controller.py",
                    name="single_hydrophone_homing_controller",
                    output="screen",
                    parameters=[{
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "odometry_topic": pose_topic,
                        "vehicle_state_topic": state_topic,
                        "direction_input_topic": hydrophone_direction_topic,
                        "rc_output_topic": pinger_rc_topic if mux_enabled else rc_topic,
                        "rate_hz": ParameterValue(rate_hz, value_type=float),
                        "forward_max": ParameterValue(pinger_forward_fast, value_type=float),
                        "yaw_gain": ParameterValue(pinger_yaw_gain, value_type=float),
                        "tank_max_depth_m": ParameterValue(tank_max_depth_m, value_type=float),
                        "success_range_m": ParameterValue(
                            pinger_success_range_m, value_type=float),
                        "success_hold_s": ParameterValue(
                            pinger_success_hold_s, value_type=float),
                        "max_runtime_s": ParameterValue(
                            pinger_max_runtime_s, value_type=float),
                        "vehicle_disconnect_grace_s": ParameterValue(
                            vehicle_disconnect_grace, value_type=float),
                    }],
                ))
            elif controller_kind == "direction":
                nodes.append(Node(
                    package="kmu26_pinger_homing",
                    executable="pinger_homing_controller",
                    name="pinger_homing_controller",
                    output="screen",
                    parameters=[{
                        "transport": transport,
                        "direction_topic": hydrophone_direction_topic,
                        "direction_frame": "world",
                        "odom_topic": pose_topic,
                        "yolo_topic": yolo_detection_topic,
                        "state_topic": state_topic,
                        "command_override_topic": command_override_topic,
                        "rc_override_topic": pinger_rc_topic if mux_enabled else rc_topic,
                        "mode": pinger_mode,
                        "rate_hz": ParameterValue(rate_hz, value_type=float),
                        "forward_fast": ParameterValue(pinger_forward_fast, value_type=float),
                        "yaw_gain": ParameterValue(pinger_yaw_gain, value_type=float),
                        "yolo_min_confidence": ParameterValue(
                            pinger_yolo_min_confidence, value_type=float),
                        "yolo_min_bbox_height_ratio": ParameterValue(
                            pinger_yolo_min_bbox_height_ratio, value_type=float),
                        "auto_arm": ParameterValue(pinger_auto_arm, value_type=bool),
                        "auto_mode": ParameterValue(pinger_auto_mode, value_type=bool),
                        "use_yolo_final": ParameterValue(pinger_use_yolo_final, value_type=bool),
                    }],
                ))
            else:
                raise RuntimeError(
                    "pinger_controller must be 'active_range' or 'direction', got "
                    + repr(controller_kind)
                )
        return nodes

    return LaunchDescription(args + [
        OpaqueFunction(function=control_nodes),
        Node(
            package="kmu26_vision_mission_fsm",
            executable="run_yolo_buoy_detector",
            name="mission_yolo_buoy_detector",
            output="screen",
            parameters=[{
                "image_topic": camera_compressed_topic,
                "bbox_topic": vision_bbox_topic,
                "observation_topic": yolo_detection_topic,
                "status_topic": vision_status_topic,
                "model_path": vision_model_path,
                "target_class_id": -1,
                "target_class_name": vision_target_class,
                "pinger_target_class_names": ["buoy", "stick"],
                "underwater_target_class_names": ["stick", "buoy"],
                "alignment_preferred_class_name": "stick",
                "publish_per_class": True,
                "min_vertical_aspect": 0.50,
                "mission_gated": True,
                "cpu_threads": 2,
                "cpu_affinity_cores": 3,
                "limit_cpu_affinity": False,
                "confidence_threshold": ParameterValue(
                    vision_detector_confidence, value_type=float),
                "inference_hz": ParameterValue(
                    vision_detector_inference_hz, value_type=float),
                "imgsz": ParameterValue(vision_detector_imgsz, value_type=int),
                "preprocess_enabled": ParameterValue(
                    vision_detector_preprocess, value_type=bool),
                "track_hold_seconds": ParameterValue(
                    vision_detector_track_hold, value_type=float),
                "max_inference_result_age_s": ParameterValue(
                    vision_detector_max_result_age, value_type=float),
                "pinger_white_min_ratio": ParameterValue(
                    pinger_white_min_ratio, value_type=float),
                "pinger_bearing_tolerance_rad": ParameterValue(
                    pinger_bearing_tolerance, value_type=float),
                "horizontal_fov_deg": ParameterValue(
                    vision_horizontal_fov_deg, value_type=float),
            }],
            additional_env={
                "OMP_NUM_THREADS": "2",
                "MKL_NUM_THREADS": "2",
                "OPENBLAS_NUM_THREADS": "2",
                "NUMEXPR_NUM_THREADS": "2",
            },
            condition=IfCondition(use_vision_detector),
        ),
        Node(
            package="kmu26_vision_mission_fsm",
            executable="mission_rviz_visualizer",
            name="mission_rviz_visualizer",
            output="screen",
            parameters=[{
                "mission_status_json": mission_status_json,
                "marker_topic": marker_topic,
                "marker_frame": marker_frame,
                "pose_topic": pose_topic,
                "pose_type": pose_type,
                "yolo_detection_topic": vision_status_topic,
                "observation_topic": yolo_detection_topic,
                "fsm_status_topic": fsm_status_topic,
                "hydrophone_body_topic": "/mission/hydrophone/direction_body",
                "own_course": own_course,
                "course_boundary_x_m": ParameterValue(course_boundary_x, value_type=float),
                "course_boundary_margin_m": ParameterValue(course_boundary_margin, value_type=float),
                "course_boundary_standoff_m": ParameterValue(course_boundary_standoff, value_type=float),
            }],
            condition=IfCondition(use_mission_rviz_visualizer),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mission_fsm_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
