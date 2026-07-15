#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    robot_package = LaunchConfiguration("robot_package")
    robot_launch = LaunchConfiguration("robot_launch")
    pinger_package = LaunchConfiguration("pinger_package")
    pinger_launch = LaunchConfiguration("pinger_launch")
    odom_topic = LaunchConfiguration("odom_topic")
    mavros_state_topic = LaunchConfiguration("mavros_state_topic")
    pinger_homing_status_topic = LaunchConfiguration("pinger_homing_status_topic")
    hydrophone_direction_topic = LaunchConfiguration("hydrophone_direction_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8878"),
            DeclareLaunchArgument("robot_package", default_value="hit25_auv_ros2"),
            DeclareLaunchArgument("robot_launch", default_value="localization_test.launch.py"),
            DeclareLaunchArgument("pinger_package", default_value="kmu26_pinger_homing"),
            DeclareLaunchArgument("pinger_launch", default_value="pinger_homing_real.launch.py"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
            DeclareLaunchArgument("mavros_state_topic", default_value="/mavros/state"),
            DeclareLaunchArgument("pinger_homing_status_topic", default_value="/pinger_homing/status"),
            DeclareLaunchArgument("hydrophone_direction_topic", default_value="/homing/direction"),
            Node(
                package="kmu26_pinger_homing",
                executable="pinger_web_gui",
                name="kmu26_pinger_homing_web_gui",
                output="screen",
                arguments=[
                    "--host",
                    host,
                    "--port",
                    port,
                    "--robot-package",
                    robot_package,
                    "--robot-launch",
                    robot_launch,
                    "--pinger-package",
                    pinger_package,
                    "--pinger-launch",
                    pinger_launch,
                    "--odom-topic",
                    odom_topic,
                    "--mavros-state-topic",
                    mavros_state_topic,
                    "--pinger-homing-status-topic",
                    pinger_homing_status_topic,
                    "--hydrophone-direction-topic",
                    hydrophone_direction_topic,
                ],
            ),
        ]
    )
