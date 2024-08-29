from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = "lls_processing"

    gap_detector = Node(
        package=pkg,
        executable="gap_detector",
        )
    
    ld = LaunchDescription([gap_detector])
    return ld