import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='user@todo.todo',
    description='ROS2-native tool calling system for I_CAN Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Active nodes (ROS2-native tool calling)
            'tool_manager_node = ican_tools.tool_manager_node:main',
            'dice_service_node = ican_tools.dice_service_node:main',
            'move_robot_node = ican_tools.move_robot_node:main',
            'query_room_node = ican_tools.query_room_node:main',
        ],
    },
)
