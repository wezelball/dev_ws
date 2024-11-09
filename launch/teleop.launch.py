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
            name='joy_node',
            parameters=[{'dev': '/dev/input/js1',
                         'deadzone': 0.1,
                         'autorepeat_rate': 10.0
                         }]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_file]
        ),

        Node(
            package='ugv_pkg',
            executable='motor_control_node',
            name='motor_control_node'
        ),

        Node(
            package='ugv_pkg',
            executable='video_stream_subscriber_node',
            name='video_stream_subscriber_node'
        ),

        Node(
            package='ugv_pkg',
            executable='video_display_node',
            name='video_display_node'
        ),
        
        Node(
            package='ugv_pkg',
            executable='joystick_gimbal_control',
            name='joystick_gimbal_control',
            parameters=[config_file]
        ),

        Node(
            package='ugv_pkg',
            executable='gimbal_control_node',
            name='gimbal_control_node'
        ),

        Node(
            package='ugv_pkg',
            executable='lights_control_node',
            name='lights_control_node'
        ),

        Node(
            package='ugv_pkg',
            executable='lidar_data_publisher',
            name='lidar_data_publisher'
        )

    ])