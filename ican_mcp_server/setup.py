import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_mcp_server'

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
    description='MCP Server bridging ROS and LLM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_server = ican_mcp_server.behavior_server:main',
            'senses_server = ican_mcp_server.senses_server:main',
        ],
    },
)
