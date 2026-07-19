"""Test-tank Phase homing with the canonical C++ controller.

The selector and IQ/Phase estimator keep their existing topic contract.  The
controller is deliberately no-odometry, XY-only and ALT_HOLD: it uses only
the selected audio stream, MAVROS IMU yaw and the RC response it commands.
Ground truth is never wired into this launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("imu_topic", default_value="/mavros/imu/data"),
        DeclareLaunchArgument("depth_topic", default_value="/depth/pose"),
        DeclareLaunchArgument("state_topic", default_value="/mavros/state"),
        DeclareLaunchArgument("mode", default_value="ALT_HOLD"),
        DeclareLaunchArgument("estimator_mode", default_value="phase"),
        DeclareLaunchArgument("rc_output_topic", default_value="/mavros/rc/override"),
        DeclareLaunchArgument("auto_select_top", default_value="false"),
        DeclareLaunchArgument("require_frequency_selection", default_value="true"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_mode", default_value="true"),
        DeclareLaunchArgument("success_range_m", default_value="1.2"),
        DeclareLaunchArgument("success_hold_s", default_value="0.8"),
        DeclareLaunchArgument("amplitude_range_constant", default_value="0.325"),
        DeclareLaunchArgument("max_runtime_s", default_value="180.0"),
        DeclareLaunchArgument("rc_pwm_span", default_value="400.0"),
        DeclareLaunchArgument("probe_pwm_delta", default_value="90"),
        DeclareLaunchArgument("approach_pwm_delta", default_value="120"),
        DeclareLaunchArgument("probe_leg_s", default_value="0.65"),
        DeclareLaunchArgument("probe_neutral_s", default_value="0.20"),
        DeclareLaunchArgument("probe_settle_s", default_value="0.45"),
        DeclareLaunchArgument("probe_sample_delay_s", default_value="0.22"),
        DeclareLaunchArgument("approach_duration_s", default_value="2.2"),
        DeclareLaunchArgument("initial_confirmation_probes", default_value="1"),
        Node(
            package="kmu26_pinger_homing",
            executable="pinger_frequency_selector",
            name="pinger_frequency_selector",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": True,
                "audio_topic": LaunchConfiguration("audio_topic"),
                "auto_select_top": ParameterValue(
                    LaunchConfiguration("auto_select_top"), value_type=bool),
            }],
        ),
        Node(
            package="kmu26_pinger_homing",
            executable="pinger_audio_estimator",
            name="pinger_audio_estimator",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "audio_topic": LaunchConfiguration("audio_topic"),
                "require_frequency_selection": ParameterValue(
                    LaunchConfiguration("require_frequency_selection"), value_type=bool),
            }],
        ),
        Node(
            package="kmu26_pinger_homing",
            executable="pinger_homing_controller",
            name="pinger_homing_controller",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "controller_mode": "active_range",
                "navigation_mode": "no_odom_phase",
                "acoustic_estimator_mode": LaunchConfiguration("estimator_mode"),
                "controller_profile": "sim_fast",
                "transport": "rc_override",
                "mode": LaunchConfiguration("mode"),
                "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                "auto_arm": ParameterValue(LaunchConfiguration("auto_arm"), value_type=bool),
                "auto_mode": ParameterValue(LaunchConfiguration("auto_mode"), value_type=bool),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "depth_pose_topic": LaunchConfiguration("depth_topic"),
                "vehicle_state_topic": LaunchConfiguration("state_topic"),
                "delta_range_topic": "/pinger_homing/delta_range_m",
                "iq_magnitude_topic": "/pinger_homing/iq_magnitude",
                "direction_output_topic": "/pinger_homing/direction_body",
                "control_direction_output_topic": "/pinger_homing/control_direction_body",
                "status_topic": "/pinger_homing/status",
                "rc_output_topic": LaunchConfiguration("rc_output_topic"),
                "rate_hz": 40.0,
                # Test-tank profile: ALT_HOLD owns Z; no map/ground-truth or
                # /odometry/filtered input participates in the Phase fit.
                "tank_max_depth_m": 0.0,
                "no_odom_horizontal_only": True,
                "no_odom_vertical_control_enabled": False,
                "no_odom_probe_pwm_delta": ParameterValue(
                    LaunchConfiguration("probe_pwm_delta"), value_type=int),
                "no_odom_approach_pwm_delta": ParameterValue(
                    LaunchConfiguration("approach_pwm_delta"), value_type=int),
                "no_odom_probe_leg_s": ParameterValue(
                    LaunchConfiguration("probe_leg_s"), value_type=float),
                "no_odom_probe_neutral_s": ParameterValue(
                    LaunchConfiguration("probe_neutral_s"), value_type=float),
                "no_odom_probe_settle_s": ParameterValue(
                    LaunchConfiguration("probe_settle_s"), value_type=float),
                "no_odom_probe_sample_delay_s": ParameterValue(
                    LaunchConfiguration("probe_sample_delay_s"), value_type=float),
                "no_odom_forward_duration_s": ParameterValue(
                    LaunchConfiguration("approach_duration_s"), value_type=float),
                "no_odom_initial_confirmation_probes": ParameterValue(
                    LaunchConfiguration("initial_confirmation_probes"), value_type=int),
                "no_odom_terminal_brake_enabled": True,
                "success_range_m": ParameterValue(
                    LaunchConfiguration("success_range_m"), value_type=float),
                "success_hold_s": ParameterValue(
                    LaunchConfiguration("success_hold_s"), value_type=float),
                "amplitude_range_constant": ParameterValue(
                    LaunchConfiguration("amplitude_range_constant"), value_type=float),
                "max_runtime_s": ParameterValue(
                    LaunchConfiguration("max_runtime_s"), value_type=float),
                "rc_pwm_span": ParameterValue(
                    LaunchConfiguration("rc_pwm_span"), value_type=float),
            }],
        ),
    ])
