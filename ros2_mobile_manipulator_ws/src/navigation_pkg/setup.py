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
    maintainer='Zain Alabidin Shbani',
    maintainer_email='zain.alabidin.shbani@gmail.com',
    description='Navigation and localization for the autonomous mobile manipulator (Nav2 + SLAM Toolbox).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node        = navigation_pkg.navigation_node:main',
            'slam_node              = navigation_pkg.slam_node:main',
            'localization_node      = navigation_pkg.localization_node:main',
            'obstacle_avoidance_node = navigation_pkg.obstacle_avoidance_node:main',
            'nav2_bridge_node       = navigation_pkg.nav2_bridge_node:main',
        ],
    },
)
