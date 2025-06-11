#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Grap bringup real robot launch
    grap_bringup_pkg = FindPackageShare('grap_bringup')
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([grap_bringup_pkg, 'launch', 'real_robot.launch.py'])
        )
    )

    # RealSense2 camera launch without TF publishing
    realsense_pkg = FindPackageShare('realsense2_camera')
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([realsense_pkg, 'launch', 'rs_launch.py'])
        ),
        launch_arguments={'publish_tf': 'false'}.items()
    )

    # ArUco ROS single launch
    aruco_pkg = FindPackageShare('aruco_ros')
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([aruco_pkg, 'launch', 'single.launch.py'])
        )
    )

    return LaunchDescription([
        real_robot_launch,
        rs_launch,
        aruco_launch
    ])
