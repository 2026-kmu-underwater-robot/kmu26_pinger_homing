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
    share_dir = get_package_share_directory("kmu26_mission_fsm")
    default_scene = os.path.join(share_dir, "config", "tank_current_scene.xml")
    default_rviz = os.path.join(share_dir, "rviz", "mission_fsm.rviz")

    args = [
        DeclareLaunchArgument("use_mission_fsm", default_value="false"),
        DeclareLaunchArgument("use_pinger_homing", default_value="false"),
        DeclareLaunchArgument("use_mission_rviz_visualizer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("mission_status_json", default_value="/tmp/kmu26_mission_fsm_status.json"),
        DeclareLaunchArgument("scene", default_value=default_scene),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("manual_topic", default_value="/mavros/manual_control/send"),
        DeclareLaunchArgument("command_override_topic", default_value="/uuv_mujoco/sitl/command_override"),
        DeclareLaunchArgument("buoy_status_topic", default_value="/mujoco/course_buoys/status"),
        DeclareLaunchArgument("yolo_detection_topic", default_value="/uuv_mujoco/yolo_buoy_detections"),
        DeclareLaunchArgument("hydrophone_direction_topic", default_value="/mujoco/hydrophone/direction"),
        DeclareLaunchArgument("hydrophone_status_topic", default_value="/mujoco/hydrophone/status"),
        DeclareLaunchArgument("marker_topic", default_value="/mission/rviz_markers"),
        DeclareLaunchArgument("marker_frame", default_value="map"),
        DeclareLaunchArgument("course", default_value="all"),
        DeclareLaunchArgument("own_course", default_value="a"),
        DeclareLaunchArgument("transport", default_value="rc_override"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("wait_armed", default_value="true"),
        DeclareLaunchArgument("no_pinger", default_value="false"),
        DeclareLaunchArgument("nearest_first", default_value="true"),
        DeclareLaunchArgument("mission_log", default_value="auto"),
        DeclareLaunchArgument("course_boundary_x", default_value="0.0"),
        DeclareLaunchArgument("course_boundary_margin", default_value="0.8"),
        DeclareLaunchArgument("course_boundary_standoff", default_value="0.7"),
        DeclareLaunchArgument("pinger_mode", default_value="MANUAL"),
        DeclareLaunchArgument("pinger_forward_fast", default_value="0.90"),
        DeclareLaunchArgument("pinger_yaw_gain", default_value="1.15"),
        DeclareLaunchArgument("pinger_yolo_switch_range_m", default_value="0.30"),
        DeclareLaunchArgument("pinger_auto_arm", default_value="true"),
        DeclareLaunchArgument("pinger_auto_mode", default_value="true"),
        DeclareLaunchArgument("pinger_use_yolo_final", default_value="true"),
    ]

    use_mission_fsm = LaunchConfiguration("use_mission_fsm")
    use_pinger_homing = LaunchConfiguration("use_pinger_homing")
    use_mission_rviz_visualizer = LaunchConfiguration("use_mission_rviz_visualizer")
    use_rviz = LaunchConfiguration("use_rviz")
    mission_status_json = LaunchConfiguration("mission_status_json")
    scene = LaunchConfiguration("scene")
    rviz_config = LaunchConfiguration("rviz_config")
    pose_topic = LaunchConfiguration("pose_topic")
    state_topic = LaunchConfiguration("state_topic")
    rc_topic = LaunchConfiguration("rc_topic")
    manual_topic = LaunchConfiguration("manual_topic")
    command_override_topic = LaunchConfiguration("command_override_topic")
    buoy_status_topic = LaunchConfiguration("buoy_status_topic")
    yolo_detection_topic = LaunchConfiguration("yolo_detection_topic")
    hydrophone_direction_topic = LaunchConfiguration("hydrophone_direction_topic")
    hydrophone_status_topic = LaunchConfiguration("hydrophone_status_topic")
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
    mission_log = LaunchConfiguration("mission_log")
    course_boundary_x = LaunchConfiguration("course_boundary_x")
    course_boundary_margin = LaunchConfiguration("course_boundary_margin")
    course_boundary_standoff = LaunchConfiguration("course_boundary_standoff")
    pinger_mode = LaunchConfiguration("pinger_mode")
    pinger_forward_fast = LaunchConfiguration("pinger_forward_fast")
    pinger_yaw_gain = LaunchConfiguration("pinger_yaw_gain")
    pinger_yolo_switch_range_m = LaunchConfiguration("pinger_yolo_switch_range_m")
    pinger_auto_arm = LaunchConfiguration("pinger_auto_arm")
    pinger_auto_mode = LaunchConfiguration("pinger_auto_mode")
    pinger_use_yolo_final = LaunchConfiguration("pinger_use_yolo_final")

    def mission_fsm_node(context):
        if not _launch_bool(context, use_mission_fsm):
            return []

        mission_fsm_args = [
            "--scene", scene.perform(context),
            "--course", course.perform(context),
            "--own-course", own_course.perform(context),
            "--rate-hz", rate_hz.perform(context),
            "--transport", transport.perform(context),
            "--pose-topic", pose_topic.perform(context),
            "--buoy-status-topic", buoy_status_topic.perform(context),
            "--yolo-detection-topic", yolo_detection_topic.perform(context),
            "--state-topic", state_topic.perform(context),
            "--rc-topic", rc_topic.perform(context),
            "--manual-topic", manual_topic.perform(context),
            "--command-override-topic", command_override_topic.perform(context),
            "--mission-log", mission_log.perform(context),
            "--status-json", mission_status_json.perform(context),
            "--course-boundary-x", course_boundary_x.perform(context),
            "--course-boundary-margin", course_boundary_margin.perform(context),
            "--course-boundary-standoff", course_boundary_standoff.perform(context),
            "--require-live-status",
            "--surface-collect-yolo",
            "--pinger-hydrophone",
        ]
        if _launch_bool(context, dry_run):
            mission_fsm_args.append("--dry-run")
        mission_fsm_args.append("--wait-armed" if _launch_bool(context, wait_armed) else "--no-wait-armed")
        if _launch_bool(context, no_pinger):
            mission_fsm_args.append("--no-pinger")
        if _launch_bool(context, nearest_first):
            mission_fsm_args.append("--nearest-first")

        return [Node(
            package="kmu26_mission_fsm",
            executable="ground_truth_buoy_fsm",
            name="ground_truth_buoy_fsm",
            output="screen",
            arguments=mission_fsm_args,
        )]

    return LaunchDescription(args + [
        OpaqueFunction(function=mission_fsm_node),
        Node(
            package="kmu26_mission_fsm",
            executable="pinger_homing_controller",
            name="pinger_homing_controller",
            output="screen",
            parameters=[{
                "transport": transport,
                "direction_topic": hydrophone_direction_topic,
                "hydrophone_status_topic": hydrophone_status_topic,
                "yolo_topic": yolo_detection_topic,
                "command_override_topic": command_override_topic,
                "rc_override_topic": rc_topic,
                "mode": pinger_mode,
                "rate_hz": ParameterValue(rate_hz, value_type=float),
                "forward_fast": ParameterValue(pinger_forward_fast, value_type=float),
                "yaw_gain": ParameterValue(pinger_yaw_gain, value_type=float),
                "yolo_switch_range_m": ParameterValue(pinger_yolo_switch_range_m, value_type=float),
                "auto_arm": ParameterValue(pinger_auto_arm, value_type=bool),
                "auto_mode": ParameterValue(pinger_auto_mode, value_type=bool),
                "use_yolo_final": ParameterValue(pinger_use_yolo_final, value_type=bool),
            }],
            condition=IfCondition(use_pinger_homing),
        ),
        Node(
            package="kmu26_mission_fsm",
            executable="mission_rviz_visualizer",
            name="mission_rviz_visualizer",
            output="screen",
            parameters=[{
                "mission_status_json": mission_status_json,
                "marker_topic": marker_topic,
                "marker_frame": marker_frame,
                "pose_topic": pose_topic,
                "yolo_detection_topic": yolo_detection_topic,
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
