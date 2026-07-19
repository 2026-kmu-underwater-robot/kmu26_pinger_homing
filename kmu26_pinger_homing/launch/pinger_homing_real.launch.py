#!/usr/bin/env python3
"""Standalone physical-vehicle bringup for canonical C++ Phase homing.

The hydrophone estimator remains the separately maintained ``audio_capture``
implementation.  For terminal scan-and-select use
``pinger_homing_real_interactive.launch.py``; this fixed-frequency launch is
also kept for the Web GUI, which already supplies an explicit selected
frequency.  Both routes start the estimator at that immutable frequency and
therefore preserve the hydrophone algorithm.
"""

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
    imu_topic = LaunchConfiguration("imu_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    state_topic = LaunchConfiguration("state_topic")
    delta_range_topic = LaunchConfiguration("delta_range_topic")
    iq_magnitude_topic = LaunchConfiguration("iq_magnitude_topic")
    direction_topic = LaunchConfiguration("direction_topic")
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
        DeclareLaunchArgument(
            "audio_input_latency_s",
            default_value="0.0",
            description="Known capture-to-receive latency; use a small positive value for batched simulator audio.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock for simulator audio, estimator, controller, and mux timing.",
        ),
        DeclareLaunchArgument("reference_frequency_hz", default_value="21164.0"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument(
            "delta_range_topic", default_value="/audio_phase_estimator/delta_range_m"
        ),
        DeclareLaunchArgument(
            "iq_magnitude_topic", default_value="/audio_phase_estimator/iq_magnitude"
        ),
        DeclareLaunchArgument("direction_topic", default_value="/homing/direction"),
        DeclareLaunchArgument("status_topic", default_value="/pinger_homing/status"),
        DeclareLaunchArgument(
            "direction_output_topic", default_value="/pinger_homing/direction_body"
        ),
        DeclareLaunchArgument(
            "pinger_rc_topic", default_value="/control/pinger/rc_override"
        ),
        DeclareLaunchArgument("rc_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("rate_hz", default_value="30.0"),
        DeclareLaunchArgument(
            "mode",
            default_value="ALT_HOLD",
            description="Required physical ArduSub mode; the C++ controller never drives in MANUAL.",
        ),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_mode", default_value="false"),
        DeclareLaunchArgument("forward_max", default_value="0.48"),
        DeclareLaunchArgument("yaw_gain", default_value="0.85"),
        DeclareLaunchArgument("yaw_command_limit", default_value="0.42"),
        DeclareLaunchArgument("tank_max_depth_m", default_value="11.0"),
        DeclareLaunchArgument("success_range_m", default_value="0.0"),
        DeclareLaunchArgument("success_hold_s", default_value="0.8"),
        DeclareLaunchArgument("arrival_radius_m", default_value="1.5"),
        DeclareLaunchArgument("arrival_hold_s", default_value="1.0"),
        DeclareLaunchArgument("max_runtime_s", default_value="180.0"),
        DeclareLaunchArgument(
            "amplitude_range_constant",
            default_value="0.0",
            description=(
                "Physical IQ-to-range calibration constant. Zero disables uncalibrated "
                "absolute range; simulator calibration is 0.325."
            ),
        ),
        DeclareLaunchArgument("rc_mux_stale_timeout", default_value="0.35"),
        # Physical Phase profile.  These are PWM deltas from 1500, rather
        # than a simulator-normalized command.  Keep the initial values small
        # enough for tethered commissioning and expose every motion timing
        # needed to tune a particular vehicle.
        DeclareLaunchArgument("rc_pwm_span", default_value="400.0"),
        DeclareLaunchArgument("probe_pwm_delta", default_value="20"),
        DeclareLaunchArgument("approach_pwm_delta", default_value="25"),
        DeclareLaunchArgument("probe_leg_s", default_value="1.5"),
        DeclareLaunchArgument("probe_neutral_s", default_value="0.50"),
        DeclareLaunchArgument("probe_settle_s", default_value="0.80"),
        DeclareLaunchArgument("probe_sample_delay_s", default_value="0.45"),
        DeclareLaunchArgument("approach_duration_s", default_value="4.0"),
        DeclareLaunchArgument("initial_confirmation_probes", default_value="2"),
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
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "audio_input_latency_s": ParameterValue(
                LaunchConfiguration("audio_input_latency_s"), value_type=float
            ),
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
        executable="pinger_homing_controller",
        name="pinger_homing_controller",
        output="screen",
        parameters=[{
            # This is the canonical C++ state machine verified in the test
            # tank.  The real profile retains its conservative timing and
            # requires the vehicle's actual ALT_HOLD state before live RC.
            "controller_mode": "active_range",
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "navigation_mode": "no_odom_phase",
            "acoustic_estimator_mode": "phase",
            "controller_profile": "real",
            "transport": "rc_override",
            "legacy_python_sequence": True,
            "dry_run": ParameterValue(dry_run, value_type=bool),
            "auto_arm": ParameterValue(LaunchConfiguration("auto_arm"), value_type=bool),
            "auto_mode": ParameterValue(LaunchConfiguration("auto_mode"), value_type=bool),
            "mode": LaunchConfiguration("mode"),
            "odometry_topic": odometry_topic,
            "imu_topic": imu_topic,
            "depth_pose_topic": depth_topic,
            "vehicle_state_topic": state_topic,
            "delta_range_topic": delta_range_topic,
            "iq_magnitude_topic": iq_magnitude_topic,
            "direction_input_topic": direction_topic,
            "direction_output_topic": direction_output_topic,
            "status_topic": status_topic,
            "rc_output_topic": pinger_rc_topic,
            "rate_hz": ParameterValue(LaunchConfiguration("rate_hz"), value_type=float),
            "rc_pwm_span": ParameterValue(LaunchConfiguration("rc_pwm_span"), value_type=float),
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
            # No localization is used as a Phase control input. ALT_HOLD owns
            # vertical control while the C++ ABBA fit excites only XY.
            "no_odom_horizontal_only": True,
            "no_odom_vertical_control_enabled": False,
            "no_odom_probe_pwm_delta": ParameterValue(
                LaunchConfiguration("probe_pwm_delta"), value_type=int
            ),
            "no_odom_approach_pwm_delta": ParameterValue(
                LaunchConfiguration("approach_pwm_delta"), value_type=int
            ),
            "no_odom_probe_leg_s": ParameterValue(
                LaunchConfiguration("probe_leg_s"), value_type=float
            ),
            "no_odom_probe_neutral_s": ParameterValue(
                LaunchConfiguration("probe_neutral_s"), value_type=float
            ),
            "no_odom_probe_settle_s": ParameterValue(
                LaunchConfiguration("probe_settle_s"), value_type=float
            ),
            "no_odom_probe_sample_delay_s": ParameterValue(
                LaunchConfiguration("probe_sample_delay_s"), value_type=float
            ),
            "no_odom_forward_duration_s": ParameterValue(
                LaunchConfiguration("approach_duration_s"), value_type=float
            ),
            "no_odom_initial_confirmation_probes": ParameterValue(
                LaunchConfiguration("initial_confirmation_probes"), value_type=int
            ),
            "no_odom_terminal_brake_enabled": True,
            "success_range_m": ParameterValue(
                LaunchConfiguration("success_range_m"), value_type=float
            ),
            "success_hold_s": ParameterValue(
                LaunchConfiguration("success_hold_s"), value_type=float
            ),
            "arrival_radius_m": ParameterValue(
                LaunchConfiguration("arrival_radius_m"), value_type=float
            ),
            "arrival_hold_s": ParameterValue(
                LaunchConfiguration("arrival_hold_s"), value_type=float
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
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "output_topic": rc_topic,
            "pinger_topic": pinger_rc_topic,
            # The physical joystick has higher mux priority than autonomous
            # pinger homing so the operator can take over immediately.
            "joystick_topic": "/control/joystick/rc_override",
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
