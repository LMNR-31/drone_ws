from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('drone_control'),
        'config',
        'landing_params.yaml'
    )

    soft_land = Node(
        package='drone_control',
        executable='drone_soft_land',
        name='drone_soft_land',
        output='screen',
        parameters=[config]
    )

    activator = Node(
        package='drone_control',
        executable='drone_activator',
        name='drone_activator',
        output='screen'
    )

    forward = Node(
        package='drone_control',
        executable='drone_go_forward',
        name='drone_go_forward',
        output='screen'
    )

    delay_start = RegisterEventHandler(
        OnProcessExit(
            target_action=soft_land,
            on_exit=[
                TimerAction(
                    period=10.0,
                    actions=[
                        activator,
                        forward
                    ]
                )
            ],
        )
    )

    return LaunchDescription([
        soft_land,
        delay_start
    ])