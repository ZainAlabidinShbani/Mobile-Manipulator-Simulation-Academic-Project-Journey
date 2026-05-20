from glob import glob

from setuptools import setup

package_name = 'task_manager_pkg'

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
    description='Finite-state task manager for autonomous pick-and-place orchestration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_task_manager_node = task_manager_pkg.fsm_task_manager_node:main',
        ],
    },
)
