import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("grap_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("grap_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "True", "is_cam_real": "False"}.items()
        )
    
    moveit = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("grap_moveit"),
                "launch",
                "moveit.launch.py"
            ),
            launch_arguments={"is_sim": "True"}.items()
        )
    
    # remote_interface = IncludeLaunchDescription(
    #         os.path.join(
    #             get_package_share_directory("grap_remote"),
    #             "launch",
    #             "remote_interface.launch.py"
    #         ),
    #     )
    
    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        #remote_interface,
    ])