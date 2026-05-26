from glob import glob

from setuptools import setup

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='essa',
    maintainer_email='essa@todo.todo',
    description='Navigation and localization support for the autonomous mobile manipulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization_node = navigation_pkg.localization_node:main',
            'slam_node = navigation_pkg.slam_node:main',
            'obstacle_avoidance_node = navigation_pkg.obstacle_avoidance_node:main',
            'nav2_bridge_node = navigation_pkg.nav2_bridge_node:main',
        ],
    },
)
