from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_viewer = Node(
        package='drone_control',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'window_width': 1600,
            'window_height': 900,
            'camera_topics': [
                '/uav1/bluefox_down/image_raw',
                '/uav1/bluefox_reverse/image_raw',
                '/uav1/bluefox_front/image_raw',
            ],
        }],
    )

    return LaunchDescription([camera_viewer])
