from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare launch arguments
    global_frame_arg = DeclareLaunchArgument(
        'global_frame_id', default_value='base_link',
        description='Global frame ID for TF transformations'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha', default_value='0.005',
        description='Alpha value for mesh construction'
    )
    
    bbox_max_arg = DeclareLaunchArgument(
        'bbox_max', default_value='1.0',
        description='Bounding box max size for cropping point clouds'
    )
    local_bbox_max_arg = DeclareLaunchArgument(
        'local_bbox_max', default_value='0.0',
        description='Bounding box max size for cropping point clouds in the local coordinate system'
    )
    save_mesh_arg = DeclareLaunchArgument(
        'save_mesh', default_value='false',
        description='Flag to save the constructed mesh'
    )

    # Node configuration
    gap_detector = Node(
        package="lls_processing",
        executable="gap_detector",
        parameters=[
            {'global_frame_id': LaunchConfiguration('global_frame_id')},
            {'alpha': LaunchConfiguration('alpha')},
            {'bbox_max': LaunchConfiguration('bbox_max')},
            {'local_bbox_max': LaunchConfiguration('local_bbox_max')},
            {'save_pcl': LaunchConfiguration('save_mesh')}
        ]
    )

    # LaunchDescription object containing the node and launch arguments
    ld = LaunchDescription([
        global_frame_arg,
        alpha_arg,
        bbox_max_arg,
        local_bbox_max_arg,
        save_mesh_arg,
        gap_detector
    ])

    return ld
