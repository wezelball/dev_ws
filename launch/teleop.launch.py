from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('ugv_pkg'),
        'config',
        'teleop_twist_joy.yaml'
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file]
        )
    ])