import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="If true, nodes will subscribe to /clock"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 2) Launch only the cartesian_path node from move_program
    cartesian_node = Node(
        package="move_program",
        executable="xyz_trajectory",
        name="cartesian_path_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Path to your joystick parameters file
    joy_params = os.path.join(
        get_package_share_directory('move_program'),
        'config',
        'joystick.yaml'
    )

    # Launch the joy_node with those parameters
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        # remappings=[('/cmd_vel', '/cmd_vel_joy')],
    )

    return LaunchDescription([
        use_sim_time_arg,
        cartesian_node,
        joy_node,
        teleop_node,
    ])
