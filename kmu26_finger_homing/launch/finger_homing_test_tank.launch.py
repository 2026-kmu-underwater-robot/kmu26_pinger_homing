from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("audio_topic", default_value="/audio"),
        DeclareLaunchArgument("odometry_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("mode", default_value="ALT_HOLD"),
        DeclareLaunchArgument("estimator_mode", default_value="phase"),
        DeclareLaunchArgument("rc_output_topic", default_value="/control/finger_homing/rc_override"),
        DeclareLaunchArgument("auto_select_top", default_value="false"),
        DeclareLaunchArgument("require_frequency_selection", default_value="true"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("success_range_m", default_value="1.2"),
        DeclareLaunchArgument("max_runtime_s", default_value="180.0"),
        DeclareLaunchArgument("rc_pwm_span", default_value="400.0"),
        DeclareLaunchArgument("probe_command", default_value="0.20"),
        DeclareLaunchArgument("forward_command", default_value="0.28"),
        DeclareLaunchArgument("lateral_command", default_value="0.22"),
        DeclareLaunchArgument("probe_leg_s", default_value="0.8"),
        DeclareLaunchArgument("probe_neutral_s", default_value="0.3"),
        DeclareLaunchArgument("auto_arm", default_value="false"),
        DeclareLaunchArgument("auto_mode", default_value="false"),
        Node(
            package="kmu26_finger_homing",
            executable="finger_frequency_selector",
            name="finger_frequency_selector",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": True,
                "audio_topic": LaunchConfiguration("audio_topic"),
                "auto_select_top": LaunchConfiguration("auto_select_top"),
            }],
        ),
        Node(
            package="kmu26_finger_homing",
            executable="finger_audio_estimator",
            name="finger_audio_estimator",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "audio_topic": LaunchConfiguration("audio_topic"),
                "require_frequency_selection": LaunchConfiguration("require_frequency_selection"),
            }],
        ),
        Node(
            package="kmu26_finger_homing",
            executable="finger_homing_controller",
            name="finger_homing_controller",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "mode": LaunchConfiguration("mode"),
                "estimator_mode": LaunchConfiguration("estimator_mode"),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "rc_output_topic": LaunchConfiguration("rc_output_topic"),
                "dry_run": LaunchConfiguration("dry_run"),
                "success_range_m": LaunchConfiguration("success_range_m"),
                "max_runtime_s": LaunchConfiguration("max_runtime_s"),
                "rc_pwm_span": LaunchConfiguration("rc_pwm_span"),
                "probe_command": LaunchConfiguration("probe_command"),
                "forward_command": LaunchConfiguration("forward_command"),
                "lateral_command": LaunchConfiguration("lateral_command"),
                "probe_leg_s": LaunchConfiguration("probe_leg_s"),
                "probe_neutral_s": LaunchConfiguration("probe_neutral_s"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "auto_mode": LaunchConfiguration("auto_mode"),
            }],
        ),
    ])
