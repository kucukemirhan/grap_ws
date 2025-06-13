# File: grap_ws/src/grap_bringup/launch/run_cartesian_only.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # 1) Declare use_sim_time argument (default to true)
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="If true, nodes will subscribe to /clock"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 2) Launch only the cartesian_path node from move_program
    cartesian_node = Node(
        package="move_program",
        executable="cartesian_path",
        name="cartesian_path_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartesian_node,
    ])
