import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch the ArUco follower node with simulated time enabled.
    """
    return LaunchDescription([
        Node(
            package='move_program',            # TODO: replace with your package name
            executable='aruco_follower',     # TODO: replace with your executable name
            name='aruco_follower',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ],
        )
    ])
