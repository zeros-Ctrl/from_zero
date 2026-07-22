#!/usr/bin/env python3
"""Bring up the complete one-shot slam_toolbox + Nav2 race stack."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    directory = os.path.dirname(os.path.realpath(__file__))
    nav2_params = os.path.join(directory, "nav2_params.yaml")
    collision_params = os.path.join(directory, "collision_monitor.yaml")
    adapter_params = os.path.join(directory, "ekf_sensor_adapter.yaml")
    ekf_params = os.path.join(directory, "ekf_odom_imu_test.yaml")
    lidar_params = os.path.join(directory, "lslidar_n10_uart.yaml")
    slam_params = os.path.join(directory, "slam_toolbox_localization.yaml")

    hardware = LaunchConfiguration("hardware")
    camera = LaunchConfiguration("camera")
    mission = LaunchConfiguration("mission")

    actions = [
        DeclareLaunchArgument("hardware", default_value="true"),
        # The camera costs more than one CPU core on this board. The mission
        # starts it on demand while stopped at WP1, so race navigation leaves
        # this diagnostic override disabled.
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("mission", default_value="true"),

        Node(
            condition=IfCondition(hardware),
            package="origincar_base",
            executable="origincar_base_node",
            name="origincar_base",
            output="screen",
            parameters=[{
                "usart_port_name": "/dev/ttyACM0",
                "serial_baud_rate": 115200,
                # The chassis publishes raw odometry but no odom->base TF.
                # robot_localization is the only odom->base TF authority.
                "robot_frame_id": "base_footprint",
                "odom_frame_id": "odom_raw",
                "gyro_frame_id": "gyro_link",
                "cmd_vel": "/cmd_vel_unused",
                "akm_cmd_vel": "/ackermann_cmd",
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
        ExecuteProcess(
            cmd=[
                "python3",
                os.path.join(directory, "ekf_sensor_adapter.py"),
                "--ros-args", "--params-file", adapter_params,
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
        Node(
            condition=IfCondition(hardware),
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            namespace="x10",
            output="screen",
            parameters=[lidar_params],
        ),
        ExecuteProcess(
            condition=IfCondition(hardware),
            cmd=[
                "python3",
                os.path.join(directory, "scan_costmap_throttle.py"),
                "--ros-args",
                "-p", "input_topic:=/scan",
                "-p", "output_topic:=/scan_costmap",
                "-p", "publish_hz:=5.0",
                "-p", "source_timeout_s:=0.45",
            ],
            cwd=directory,
            output="screen",
        ),
        Node(
            condition=IfCondition(camera),
            package="origincar_camera",
            executable="camera_node",
            name="origincar_camera",
            output="screen",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=[
                "--x", "-0.03", "--y", "0.01", "--z", "0.20",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "laser_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_gyro_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", "base_footprint", "--child-frame-id", "gyro_link",
            ],
        ),

        ExecuteProcess(
            # RViz may use a different wall clock. Normalize /initialpose to
            # the robot's ROS clock before slam_toolbox or the mission sees it.
            cmd=["python3", os.path.join(directory, "initial_pose_time_relay.py")],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            # The wrapper replaces itself with slam_toolbox only after the
            # EKF has published odom_ekf_test->base_footprint.
            cmd=[
                "python3", os.path.join(directory, "wait_for_fused_tf.py"),
                "--enabled", hardware,
                "--parent", "odom_ekf_test",
                "--child", "base_footprint",
                "--",
                "/opt/ros/humble/lib/slam_toolbox/localization_slam_toolbox_node",
                "--ros-args",
                "-r", "__node:=slam_toolbox",
                "-r", "/initialpose:=/initialpose_robot_time",
                "--params-file", slam_params,
                "-p", "use_sim_time:=false",
            ],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", os.path.join(directory, "navigation_map_guard.py")],
            cwd=directory,
            output="screen",
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel_nav")],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
            remappings=[
                ("cmd_vel", "/cmd_vel_nav"),
                ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
            ],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[collision_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "autostart": True,
                "bond_timeout": 0.0,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "velocity_smoother",
                    "collision_monitor",
                ],
            }],
        ),
        ExecuteProcess(
            condition=IfCondition(mission),
            cmd=["python3", os.path.join(directory, "cruise_mission.py")],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(mission),
            cmd=[
                "python3",
                os.path.join(directory, "command_chain_diagnostic.py"),
            ],
            cwd=directory,
            output="screen",
        ),
        ExecuteProcess(
            condition=IfCondition(hardware),
            cmd=[
                "python3",
                os.path.join(directory, "ackermann_converter.py"),
                "--ros-args",
                "-p", "min_turning_radius:=0.20",
                "-p", "max_forward_speed:=0.65",
                "-p", "max_reverse_speed:=0.25",
            ],
            cwd=directory,
            output="screen",
        ),
    ]
    return LaunchDescription(actions)
