from glob import glob

from setuptools import setup

package_name = 'perception_pkg'

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
    description='Perception pipeline for the autonomous mobile manipulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = perception_pkg.camera_node:main',
            'yolo_detector_node = perception_pkg.yolo_detector_node:main',
            'grasp_pose_estimator_node = perception_pkg.grasp_pose_estimator_node:main',
        ],
    },
)
