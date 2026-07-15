#!/usr/bin/env python3
"""Launch the dedicated KMU26 mission FSM web GUI."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_bool(context, value: LaunchConfiguration) -> bool:
    return value.perform(context).strip().lower() in {"1", "true", "yes", "on"}


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8890"),
        DeclareLaunchArgument("mission_status_json", default_value="/tmp/kmu26_mission_fsm_status.json"),
        DeclareLaunchArgument("config_json", default_value="/tmp/kmu26_mission_fsm_gui_config.json"),
        DeclareLaunchArgument("pose_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("pose_type", default_value="odometry"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("manual_topic", default_value="/mavros/manual_control/send"),
        DeclareLaunchArgument("dvl_twist_topic", default_value="/dvl/twist"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("battery_topic", default_value="/battery"),
        DeclareLaunchArgument("camera_compressed_topic", default_value="/camera/camera/color/image_raw/compressed"),
        DeclareLaunchArgument(
            "camera_annotated_topic", default_value="/vision/buoy/image_annotated/compressed"
        ),
        DeclareLaunchArgument("camera_raw_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("vision_status_topic", default_value="/vision/buoy/status"),
        DeclareLaunchArgument("fsm_status_topic", default_value="/mission/fsm/status"),
        DeclareLaunchArgument("pinger_status_topic", default_value="/pinger_homing/status"),
        DeclareLaunchArgument("camera_on", default_value="false"),
        DeclareLaunchArgument("allow_rc_send", default_value="false"),
    ]

    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    mission_status_json = LaunchConfiguration("mission_status_json")
    config_json = LaunchConfiguration("config_json")
    pose_topic = LaunchConfiguration("pose_topic")
    pose_type = LaunchConfiguration("pose_type")
    state_topic = LaunchConfiguration("state_topic")
    rc_topic = LaunchConfiguration("rc_topic")
    manual_topic = LaunchConfiguration("manual_topic")
    dvl_twist_topic = LaunchConfiguration("dvl_twist_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    joy_topic = LaunchConfiguration("joy_topic")
    battery_topic = LaunchConfiguration("battery_topic")
    camera_compressed_topic = LaunchConfiguration("camera_compressed_topic")
    camera_annotated_topic = LaunchConfiguration("camera_annotated_topic")
    camera_raw_topic = LaunchConfiguration("camera_raw_topic")
    vision_status_topic = LaunchConfiguration("vision_status_topic")
    fsm_status_topic = LaunchConfiguration("fsm_status_topic")
    pinger_status_topic = LaunchConfiguration("pinger_status_topic")
    camera_on = LaunchConfiguration("camera_on")
    allow_rc_send = LaunchConfiguration("allow_rc_send")

    def gui_node(context):
        argv = [
            "--host", host.perform(context),
            "--port", port.perform(context),
            "--mission-status-json", mission_status_json.perform(context),
            "--config-json", config_json.perform(context),
            "--pose-topic", pose_topic.perform(context),
            "--pose-type", pose_type.perform(context),
            "--state-topic", state_topic.perform(context),
            "--rc-topic", rc_topic.perform(context),
            "--manual-topic", manual_topic.perform(context),
            "--dvl-twist-topic", dvl_twist_topic.perform(context),
            "--depth-topic", depth_topic.perform(context),
            "--imu-topic", imu_topic.perform(context),
            "--joy-topic", joy_topic.perform(context),
            "--battery-topic", battery_topic.perform(context),
            "--camera-compressed-topic", camera_compressed_topic.perform(context),
            "--camera-annotated-topic", camera_annotated_topic.perform(context),
            "--camera-raw-topic", camera_raw_topic.perform(context),
            "--vision-status-topic", vision_status_topic.perform(context),
            "--fsm-status-topic", fsm_status_topic.perform(context),
            "--pinger-status-topic", pinger_status_topic.perform(context),
        ]
        if _launch_bool(context, camera_on):
            argv.append("--camera-on")
        if _launch_bool(context, allow_rc_send):
            argv.append("--allow-rc-send")
        return [Node(
            package="kmu26_vision_mission_fsm",
            executable="fsm_web_gui.py",
            name="mission_fsm_web_gui",
            output="screen",
            arguments=argv,
        )]

    return LaunchDescription(args + [OpaqueFunction(function=gui_node)])
