#!/usr/bin/env python3
"""Isolated no-navigation robot_localization EKF + lidar localization test."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    directory = os.path.dirname(os.path.realpath(__file__))
    adapter_params = os.path.join(directory, "ekf_sensor_adapter.yaml")
    ekf_params = os.path.join(directory, "ekf_odom_imu_test.yaml")
    lidar_params = os.path.join(directory, "lslidar_n10_uart.yaml")
    slam_params = os.path.join(
        directory,
        "slam_toolbox_localization_ekf_test.yaml",
    )
    rviz_config = os.path.join(
        directory,
        "ekf_lidar_localization_test.rviz",
    )
    hardware = LaunchConfiguration("hardware")
    rviz = LaunchConfiguration("rviz")
    turn_commands = LaunchConfiguration("turn_commands")
    ackermann_command_topic = LaunchConfiguration("ackermann_command_topic")

    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # No motion publisher is launched in the ordinary EKF test.
        DeclareLaunchArgument("turn_commands", default_value="false"),
        DeclareLaunchArgument(
            "ackermann_command_topic",
            default_value="/ackermann_cmd_ekf_test_disabled",
        ),
        Node(
            condition=IfCondition(hardware),
            package="origincar_base",
            executable="origincar_base_node",
            name="origincar_base_ekf_test",
            output="screen",
            parameters=[{
                "usart_port_name": "/dev/ttyACM0",
                "serial_baud_rate": 115200,
                "robot_frame_id": "base_footprint",
                "odom_frame_id": "odom_raw",
                "gyro_frame_id": "gyro_link",
                "cmd_vel": "/cmd_vel_ekf_test_disabled",
                "akm_cmd_vel": ackermann_command_topic,
            }],
        ),
        ExecuteProcess(
            condition=IfCondition(hardware),
            cmd=[
                "python3",
                "/root/rdk_imu-main/bmi088_mahony_imu_node.py",
                "--topic", "/imu/data",
                "--frame-id", "gyro_link",
                "--rate", "40.0",
                "--log-period", "1.0",
            ],
            cwd="/root/rdk_imu-main",
            output="screen",
        ),
        Node(
            condition=IfCondition(hardware),
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            namespace="x10",
            output="screen",
            parameters=[lidar_params],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf_ekf_test",
            arguments=[
                "--x", "-0.03", "--y", "0.01", "--z", "0.20",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "laser_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_gyro_tf_ekf_test",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "gyro_link",
            ],
        ),
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(directory, "ekf_sensor_adapter.py"),
                "--ros-args",
                "--params-file", adapter_params,
            ],
            cwd=directory,
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_ekf_test",
            output="screen",
            parameters=[ekf_params],
            remappings=[
                ("odometry/filtered", "/odom_ekf_test"),
            ],
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(directory, "ekf_test_monitor.py")],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(turn_commands),
            cmd=[
                "python3", os.path.join(directory, "ackermann_converter.py"),
                "--ros-args",
                "-r", "/cmd_vel:=/cmd_vel_turn_test",
                "-r", "/ackermann_cmd:=/ackermann_cmd_turn_test",
                "-r",
                "/cruise/emergency_stop:=/turn_test/emergency_stop_unused",
            ],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            # Start scan matching only after the EKF is the sole live authority
            # for odom_ekf_test -> base_footprint.
            cmd=[
                "python3", os.path.join(directory, "wait_for_fused_tf.py"),
                "--enabled", hardware,
                "--parent", "odom_ekf_test",
                "--child", "base_footprint",
                "--",
                "/opt/ros/humble/lib/slam_toolbox/"
                "localization_slam_toolbox_node",
                "--ros-args",
                "-r", "__node:=slam_toolbox",
                "--params-file", slam_params,
                "-p", "use_sim_time:=false",
            ],
            cwd=directory,
            output="screen",
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(hardware),
                    cmd=[
                        "python3",
                        os.path.join(directory, "initial_pose_once.py"),
                        "--config", slam_params,
                    ],
                    cwd=directory,
                    output="screen",
                ),
            ],
        ),
        Node(
            condition=IfCondition(rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_ekf_localization_test",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
