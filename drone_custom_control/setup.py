from setuptools import find_packages, setup

package_name = 'drone_custom_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyrosrobot5',
    maintainer_email='icafs.30@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_activation_node = drone_custom_control.nodes.drone_activation_node:main',
            'hover_supervisor_node = drone_custom_control.nodes.hover_supervisor_node:main',
            'lpv_mpc_drone_node = drone_custom_control.nodes.lpv_mpc_drone_node:main',
            'mavros_takeoff_node = drone_custom_control.nodes.mavros_takeoff_node:main',
        ],
    },
)
