from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    soft_land = Node(
        package='drone_control',
        executable='drone_soft_land',
        name='drone_soft_land',
        output='screen'
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

    camera_viewer = Node(
        package='drone_control',
        executable='camera_viewer',
        name='camera_viewer',
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
        camera_viewer,
        soft_land,
        delay_start
    ])