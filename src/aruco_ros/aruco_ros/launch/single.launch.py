from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': 'camera_color_frame',
        'camera_frame': 'camera_color_frame',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[
            ('/camera_info', '/camera/color/camera_info'),
            ('/image',       '/camera/color/image_raw'),
        ],
    )

    return [aruco_single]


def generate_launch_description():
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='31',
        description='ID of the ArUco marker to detect.'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.1',
        description='Size of the ArUco marker (in meters).'
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame',
        default_value='aruco_marker_frame',
        description='Name of the TF frame in which the marker pose will be published.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='',
        description=(
            'Reference frame for the published pose. '
            'Leave empty to publish relative to the parent frame.'
        )
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement',
        default_value='LINES',
        description='Corner refinement algorithm for ArUco detection.',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame_arg)
    ld.add_action(corner_refinement_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
