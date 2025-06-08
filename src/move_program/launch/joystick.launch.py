import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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
        joy_node,
        teleop_node,
    ])
