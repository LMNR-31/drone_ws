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
        }],
    )

    return LaunchDescription([camera_viewer])
