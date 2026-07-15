from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bbox_topic", default_value="/vision/buoy_bbox"),
            DeclareLaunchArgument("rc_override_topic", default_value="/mavros/rc/override"),
            DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("lost_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "lost_behavior",
                default_value="neutral",
                description="neutral keeps controlled axes at 1500; release sends CHAN_RELEASE=0.",
            ),
            DeclareLaunchArgument("pitch_channel", default_value="1"),
            DeclareLaunchArgument("roll_channel", default_value="2"),
            DeclareLaunchArgument("throttle_channel", default_value="3"),
            DeclareLaunchArgument("yaw_channel", default_value="4"),
            DeclareLaunchArgument("forward_channel", default_value="5"),
            DeclareLaunchArgument("lateral_channel", default_value="6"),
            DeclareLaunchArgument("neutral_pwm", default_value="1500"),
            DeclareLaunchArgument("min_pwm", default_value="1100"),
            DeclareLaunchArgument("max_pwm", default_value="1900"),
            DeclareLaunchArgument("max_yaw_delta", default_value="250"),
            DeclareLaunchArgument("max_throttle_delta", default_value="150"),
            DeclareLaunchArgument("forward_pwm", default_value="1600"),
            DeclareLaunchArgument("aligned_deadband_x", default_value="0.12"),
            DeclareLaunchArgument("aligned_deadband_y", default_value="0.18"),
            DeclareLaunchArgument("yaw_invert", default_value="false"),
            DeclareLaunchArgument("vertical_positive_is_up", default_value="true"),
            Node(
                package="kmu26_vision_mission_fsm",
                executable="bbox_controller_node",
                name="bbox_controller_node",
                output="screen",
                parameters=[
                    {
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                        "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
                        "lost_timeout_sec": ParameterValue(LaunchConfiguration("lost_timeout_sec"), value_type=float),
                        "lost_behavior": LaunchConfiguration("lost_behavior"),
                        "pitch_channel": ParameterValue(LaunchConfiguration("pitch_channel"), value_type=int),
                        "roll_channel": ParameterValue(LaunchConfiguration("roll_channel"), value_type=int),
                        "throttle_channel": ParameterValue(LaunchConfiguration("throttle_channel"), value_type=int),
                        "yaw_channel": ParameterValue(LaunchConfiguration("yaw_channel"), value_type=int),
                        "forward_channel": ParameterValue(LaunchConfiguration("forward_channel"), value_type=int),
                        "lateral_channel": ParameterValue(LaunchConfiguration("lateral_channel"), value_type=int),
                        "neutral_pwm": ParameterValue(LaunchConfiguration("neutral_pwm"), value_type=int),
                        "min_pwm": ParameterValue(LaunchConfiguration("min_pwm"), value_type=int),
                        "max_pwm": ParameterValue(LaunchConfiguration("max_pwm"), value_type=int),
                        "max_yaw_delta": ParameterValue(LaunchConfiguration("max_yaw_delta"), value_type=int),
                        "max_throttle_delta": ParameterValue(
                            LaunchConfiguration("max_throttle_delta"),
                            value_type=int,
                        ),
                        "forward_pwm": ParameterValue(LaunchConfiguration("forward_pwm"), value_type=int),
                        "aligned_deadband_x": ParameterValue(
                            LaunchConfiguration("aligned_deadband_x"),
                            value_type=float,
                        ),
                        "aligned_deadband_y": ParameterValue(
                            LaunchConfiguration("aligned_deadband_y"),
                            value_type=float,
                        ),
                        "yaw_invert": ParameterValue(LaunchConfiguration("yaw_invert"), value_type=bool),
                        "vertical_positive_is_up": ParameterValue(
                            LaunchConfiguration("vertical_positive_is_up"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
