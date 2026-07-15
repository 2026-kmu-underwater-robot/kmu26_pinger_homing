from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bbox_topic", default_value="/vision/buoy_bbox"),
            DeclareLaunchArgument("depth_topic", default_value="/auv/depth"),
            DeclareLaunchArgument("depth_pose_topic", default_value="/depth/pose"),
            DeclareLaunchArgument("depth_pose_scale", default_value="1.0"),
            DeclareLaunchArgument("depth_pose_offset_m", default_value="0.0"),
            DeclareLaunchArgument("depth_odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("depth_odom_scale", default_value="-1.0"),
            DeclareLaunchArgument("depth_odom_offset_m", default_value="0.0"),
            DeclareLaunchArgument("enable_topic", default_value="/control/vision/enable"),
            DeclareLaunchArgument("state_topic", default_value="/control/vision/state"),
            DeclareLaunchArgument("status_topic", default_value="/control/vision/status"),
            DeclareLaunchArgument("vehicle_state_topic", default_value="/mavros/state"),
            DeclareLaunchArgument("rc_override_topic", default_value="/control/vision/rc_override"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("require_armed", default_value="true"),
            DeclareLaunchArgument("vehicle_state_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("throttle_channel", default_value="3"),
            DeclareLaunchArgument("yaw_channel", default_value="4"),
            DeclareLaunchArgument("forward_channel", default_value="5"),
            DeclareLaunchArgument("neutral_pwm", default_value="1500"),
            DeclareLaunchArgument(
                "min_pwm",
                default_value="1300",
                description="Minimum thruster PWM command.",
            ),
            DeclareLaunchArgument(
                "max_pwm",
                default_value="1700",
                description="Maximum thruster PWM command.",
            ),
            DeclareLaunchArgument("max_yaw_delta", default_value="180"),
            DeclareLaunchArgument("forward_pwm", default_value="1600"),
            DeclareLaunchArgument("yaw_invert", default_value="false"),
            DeclareLaunchArgument("vertical_positive_is_up", default_value="true"),
            DeclareLaunchArgument("work_depth_m", default_value="8.5"),
            DeclareLaunchArgument("surface_depth_m", default_value="0.4"),
            DeclareLaunchArgument("max_depth_m", default_value="10.8"),
            DeclareLaunchArgument("depth_kp_pwm_per_m", default_value="120.0"),
            DeclareLaunchArgument("expected_target_count", default_value="0"),
            DeclareLaunchArgument("buoy_class_id", default_value="0"),
            DeclareLaunchArgument("stick_class_id", default_value="1"),
            DeclareLaunchArgument("approach_area_ratio", default_value="0.12"),
            DeclareLaunchArgument("fork_target_x", default_value="0.5"),
            DeclareLaunchArgument("fork_target_y", default_value="0.5"),
            DeclareLaunchArgument("stick_deadband_x", default_value="0.06"),
            DeclareLaunchArgument("stick_deadband_y", default_value="0.08"),
            DeclareLaunchArgument("align_stable_sec", default_value="0.7"),
            DeclareLaunchArgument("insert_pwm", default_value="1560"),
            DeclareLaunchArgument("insert_duration_sec", default_value="0.8"),
            DeclareLaunchArgument("detach_pwm", default_value="1620"),
            DeclareLaunchArgument("detach_duration_sec", default_value="0.3"),
            DeclareLaunchArgument("backoff_pwm", default_value="1420"),
            DeclareLaunchArgument("backoff_duration_sec", default_value="0.5"),
            DeclareLaunchArgument("search_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("area_verify_sec", default_value="12.0"),
            Node(
                package="kmu26_vision_mission_fsm",
                executable="mission_state_machine_node",
                name="mission_state_machine_node",
                output="screen",
                parameters=[
                    {
                        "bbox_topic": LaunchConfiguration("bbox_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "depth_pose_topic": LaunchConfiguration("depth_pose_topic"),
                        "depth_pose_scale": ParameterValue(
                            LaunchConfiguration("depth_pose_scale"), value_type=float
                        ),
                        "depth_pose_offset_m": ParameterValue(
                            LaunchConfiguration("depth_pose_offset_m"), value_type=float
                        ),
                        "depth_odom_topic": LaunchConfiguration("depth_odom_topic"),
                        "depth_odom_scale": ParameterValue(
                            LaunchConfiguration("depth_odom_scale"), value_type=float
                        ),
                        "depth_odom_offset_m": ParameterValue(
                            LaunchConfiguration("depth_odom_offset_m"), value_type=float
                        ),
                        "enable_topic": LaunchConfiguration("enable_topic"),
                        "state_topic": LaunchConfiguration("state_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "vehicle_state_topic": LaunchConfiguration("vehicle_state_topic"),
                        "rc_override_topic": LaunchConfiguration("rc_override_topic"),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "require_armed": ParameterValue(
                            LaunchConfiguration("require_armed"), value_type=bool
                        ),
                        "vehicle_state_timeout_sec": ParameterValue(
                            LaunchConfiguration("vehicle_state_timeout_sec"), value_type=float
                        ),
                        "control_rate_hz": ParameterValue(
                            LaunchConfiguration("control_rate_hz"), value_type=float
                        ),
                        "work_depth_m": ParameterValue(
                            LaunchConfiguration("work_depth_m"), value_type=float
                        ),
                        "surface_depth_m": ParameterValue(
                            LaunchConfiguration("surface_depth_m"), value_type=float
                        ),
                        "max_depth_m": ParameterValue(
                            LaunchConfiguration("max_depth_m"), value_type=float
                        ),
                        "depth_kp_pwm_per_m": ParameterValue(
                            LaunchConfiguration("depth_kp_pwm_per_m"), value_type=float
                        ),
                        "expected_target_count": ParameterValue(
                            LaunchConfiguration("expected_target_count"), value_type=int
                        ),
                        "buoy_class_id": ParameterValue(
                            LaunchConfiguration("buoy_class_id"), value_type=int
                        ),
                        "stick_class_id": ParameterValue(
                            LaunchConfiguration("stick_class_id"), value_type=int
                        ),
                        "approach_area_ratio": ParameterValue(
                            LaunchConfiguration("approach_area_ratio"), value_type=float
                        ),
                        "fork_target_x": ParameterValue(
                            LaunchConfiguration("fork_target_x"), value_type=float
                        ),
                        "fork_target_y": ParameterValue(
                            LaunchConfiguration("fork_target_y"), value_type=float
                        ),
                        "stick_deadband_x": ParameterValue(
                            LaunchConfiguration("stick_deadband_x"), value_type=float
                        ),
                        "stick_deadband_y": ParameterValue(
                            LaunchConfiguration("stick_deadband_y"), value_type=float
                        ),
                        "align_stable_sec": ParameterValue(
                            LaunchConfiguration("align_stable_sec"), value_type=float
                        ),
                        "insert_pwm": ParameterValue(
                            LaunchConfiguration("insert_pwm"), value_type=int
                        ),
                        "insert_duration_sec": ParameterValue(
                            LaunchConfiguration("insert_duration_sec"), value_type=float
                        ),
                        "detach_pwm": ParameterValue(
                            LaunchConfiguration("detach_pwm"), value_type=int
                        ),
                        "detach_duration_sec": ParameterValue(
                            LaunchConfiguration("detach_duration_sec"), value_type=float
                        ),
                        "backoff_pwm": ParameterValue(
                            LaunchConfiguration("backoff_pwm"), value_type=int
                        ),
                        "backoff_duration_sec": ParameterValue(
                            LaunchConfiguration("backoff_duration_sec"), value_type=float
                        ),
                        "search_timeout_sec": ParameterValue(
                            LaunchConfiguration("search_timeout_sec"), value_type=float
                        ),
                        "area_verify_sec": ParameterValue(
                            LaunchConfiguration("area_verify_sec"), value_type=float
                        ),
                        "throttle_channel": ParameterValue(
                            LaunchConfiguration("throttle_channel"), value_type=int
                        ),
                        "yaw_channel": ParameterValue(
                            LaunchConfiguration("yaw_channel"), value_type=int
                        ),
                        "forward_channel": ParameterValue(
                            LaunchConfiguration("forward_channel"), value_type=int
                        ),
                        "neutral_pwm": ParameterValue(
                            LaunchConfiguration("neutral_pwm"), value_type=int
                        ),
                        "min_pwm": ParameterValue(
                            LaunchConfiguration("min_pwm"), value_type=int
                        ),
                        "max_pwm": ParameterValue(
                            LaunchConfiguration("max_pwm"), value_type=int
                        ),
                        "max_yaw_delta": ParameterValue(
                            LaunchConfiguration("max_yaw_delta"), value_type=int
                        ),
                        "approach_forward_pwm": ParameterValue(
                            LaunchConfiguration("forward_pwm"), value_type=int
                        ),
                        "yaw_invert": ParameterValue(
                            LaunchConfiguration("yaw_invert"), value_type=bool
                        ),
                        "vertical_positive_is_up": ParameterValue(
                            LaunchConfiguration("vertical_positive_is_up"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
