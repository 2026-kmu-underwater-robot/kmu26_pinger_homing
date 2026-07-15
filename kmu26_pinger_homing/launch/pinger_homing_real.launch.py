#!/usr/bin/env python3
"""Standalone, real-vehicle bringup for the KMU26 pinger homing controller."""

from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_audio_capture = LaunchConfiguration("use_audio_capture")
    use_hydrophone_estimator = LaunchConfiguration("use_hydrophone_estimator")
    use_rc_mux = LaunchConfiguration("use_rc_mux")
    dry_run = LaunchConfiguration("dry_run")

    audio_topic = LaunchConfiguration("audio_topic")
    odometry_topic = LaunchConfiguration("odometry_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    state_topic = LaunchConfiguration("state_topic")
    delta_range_topic = LaunchConfiguration("delta_range_topic")
    iq_magnitude_topic = LaunchConfiguration("iq_magnitude_topic")
    direction_topic = LaunchConfiguration("direction_topic")
    collector_topic = LaunchConfiguration("collector_topic")
    status_topic = LaunchConfiguration("status_topic")
    direction_output_topic = LaunchConfiguration("direction_output_topic")
    pinger_rc_topic = LaunchConfiguration("pinger_rc_topic")
    rc_topic = LaunchConfiguration("rc_topic")

    arguments = [
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("use_audio_capture", default_value="false"),
        DeclareLaunchArgument("use_hydrophone_estimator", default_value="true"),
        DeclareLaunchArgument("use_rc_mux", default_value="true"),
        DeclareLaunchArgument("audio_device", default_value=""),
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("audio_channels", default_value="2"),
        DeclareLaunchArgument("audio_sample_rate", default_value="96000"),
        DeclareLaunchArgument("audio_sample_format", default_value="S32LE"),
        DeclareLaunchArgument("reference_frequency_hz", default_value="21164.0"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument(
            "delta_range_topic", default_value="/audio_phase_estimator/delta_range_m"
        ),
        DeclareLaunchArgument(
            "iq_magnitude_topic", default_value="/audio_phase_estimator/iq_magnitude"
        ),
        DeclareLaunchArgument("direction_topic", default_value="/homing/direction"),
        DeclareLaunchArgument("collector_topic", default_value="/collector/state"),
        DeclareLaunchArgument("status_topic", default_value="/pinger_homing/status"),
        DeclareLaunchArgument(
            "direction_output_topic", default_value="/pinger_homing/direction_body"
        ),
        DeclareLaunchArgument(
            "pinger_rc_topic", default_value="/control/pinger/rc_override"
        ),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument("forward_max", default_value="0.48"),
        DeclareLaunchArgument("yaw_gain", default_value="0.85"),
        DeclareLaunchArgument("yaw_command_limit", default_value="0.42"),
        DeclareLaunchArgument("tank_max_depth_m", default_value="11.0"),
        DeclareLaunchArgument("success_range_m", default_value="0.0"),
        DeclareLaunchArgument("success_hold_s", default_value="0.8"),
        DeclareLaunchArgument("max_runtime_s", default_value="0.0"),
        DeclareLaunchArgument(
            "amplitude_range_constant",
            default_value="0.0",
            description=(
                "Physical IQ-to-range calibration constant. Zero disables uncalibrated "
                "absolute range; simulator calibration is 0.325."
            ),
        ),
        DeclareLaunchArgument("rc_mux_stale_timeout", default_value="0.35"),
    ]

    capture_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("audio_capture") + "/launch/capture.launch.py"
        ),
        launch_arguments={
            "device": LaunchConfiguration("audio_device"),
            "audio_topic": audio_topic,
            "channels": LaunchConfiguration("audio_channels"),
            "sample_rate": LaunchConfiguration("audio_sample_rate"),
            "sample_format": LaunchConfiguration("audio_sample_format"),
            "ns": "",
        }.items(),
        condition=IfCondition(use_audio_capture),
    )

    hydrophone_estimator = Node(
        package="audio_capture",
        executable="audio_phase_estimator",
        name="pinger_audio_phase_estimator",
        output="screen",
        remappings=[
            ("/audio_boosted", audio_topic),
            ("/odometry/filtered", odometry_topic),
            ("/depth/pose", depth_topic),
            ("/audio_phase_estimator/delta_range_m", delta_range_topic),
            ("/audio_phase_estimator/iq_magnitude", iq_magnitude_topic),
            ("/homing/direction", direction_topic),
        ],
        parameters=[{
            "reference_frequency_hz": ParameterValue(
                LaunchConfiguration("reference_frequency_hz"), value_type=float
            ),
            "initial_demodulation_frequency_hz": ParameterValue(
                LaunchConfiguration("reference_frequency_hz"), value_type=float
            ),
            "enable_frequency_acquisition": False,
        }],
        condition=IfCondition(use_hydrophone_estimator),
    )

    controller = Node(
        package="kmu26_pinger_homing",
        executable="single_hydrophone_homing_controller.py",
        name="single_hydrophone_homing_controller",
        output="screen",
        parameters=[{
            "dry_run": ParameterValue(dry_run, value_type=bool),
            "odometry_topic": odometry_topic,
            "vehicle_state_topic": state_topic,
            "delta_range_topic": delta_range_topic,
            "iq_magnitude_topic": iq_magnitude_topic,
            "direction_input_topic": direction_topic,
            "collector_topic": collector_topic,
            "direction_output_topic": direction_output_topic,
            "status_topic": status_topic,
            "rc_output_topic": pinger_rc_topic,
            "rate_hz": ParameterValue(LaunchConfiguration("rate_hz"), value_type=float),
            "forward_max": ParameterValue(
                LaunchConfiguration("forward_max"), value_type=float
            ),
            "yaw_gain": ParameterValue(LaunchConfiguration("yaw_gain"), value_type=float),
            "yaw_command_limit": ParameterValue(
                LaunchConfiguration("yaw_command_limit"), value_type=float
            ),
            "tank_max_depth_m": ParameterValue(
                LaunchConfiguration("tank_max_depth_m"), value_type=float
            ),
            "success_range_m": ParameterValue(
                LaunchConfiguration("success_range_m"), value_type=float
            ),
            "success_hold_s": ParameterValue(
                LaunchConfiguration("success_hold_s"), value_type=float
            ),
            "max_runtime_s": ParameterValue(
                LaunchConfiguration("max_runtime_s"), value_type=float
            ),
            "amplitude_range_constant": ParameterValue(
                LaunchConfiguration("amplitude_range_constant"), value_type=float
            ),
        }],
    )

    rc_mux = Node(
        package="kmu26_pinger_homing",
        executable="rc_override_mux",
        name="pinger_rc_override_mux",
        output="screen",
        parameters=[{
            "output_topic": rc_topic,
            "pinger_topic": pinger_rc_topic,
            # Standalone homing must never relay an already-running mission,
            # joystick, or vision controller through a second MAVROS publisher.
            "joystick_topic": "/pinger_homing/disabled/joystick_rc_override",
            "mission_topic": "/pinger_homing/disabled/mission_rc_override",
            "vision_topic": "/pinger_homing/disabled/vision_rc_override",
            "require_exclusive_output": True,
            "output_discovery_grace_s": 1.0,
            "stale_timeout_s": ParameterValue(
                LaunchConfiguration("rc_mux_stale_timeout"), value_type=float
            ),
        }],
        condition=IfCondition(use_rc_mux),
    )

    return LaunchDescription(
        arguments + [capture_launch, hydrophone_estimator, controller, rc_mux]
    )
