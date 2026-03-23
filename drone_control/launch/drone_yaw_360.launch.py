from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drone_yaw_360 = Node(
        package='drone_control',
        executable='drone_yaw_360',
        name='drone_yaw_360',
        output='screen',
        parameters=[{
            'uav_ns': '/uav1',
            'z_hold': 2.0,
            'yaw_rate': 0.8,
            'angle': 6.283185307179586,  # 2 * pi
            'hz': 20.0,
        }],
    )

    return LaunchDescription([drone_yaw_360])
