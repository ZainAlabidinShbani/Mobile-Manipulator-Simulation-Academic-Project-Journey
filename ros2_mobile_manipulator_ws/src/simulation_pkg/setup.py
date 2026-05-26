from glob import glob

from setuptools import setup

package_name = 'simulation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='essa',
    maintainer_email='essa@todo.todo',
    description='Simulation, robot model, and visualization support for the autonomous mobile manipulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_bridge_node = simulation_pkg.gazebo_bridge_node:main',
            'rviz_bridge_node = simulation_pkg.rviz_bridge_node:main',
        ],
    },
)
