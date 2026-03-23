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
            'z_hold': -1.0,
            'yaw_rate': 0.8,
            'angle': 6.283185307179586,  # 2 * pi
            'hz': 20.0,
            'ccw': True,  # True = anti-horário (CCW), False = horário (CW)
            # Nome do nó controlador a pausar durante o giro (sem barra inicial)
            'controller_node': 'drone_controller_completo',
            # Se True, pausa o controller antes do giro e retoma ao terminar
            'auto_disable_controller': True,
        }],
    )

    return LaunchDescription([drone_yaw_360])
