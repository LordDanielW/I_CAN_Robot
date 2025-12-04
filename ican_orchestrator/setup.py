import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # Include URDF/Worlds if they exist (recursive)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools', 'ollama'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='user@todo.todo',
    description='Part of I_CAN_Robot stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain_node = ican_orchestrator.brain_node:main',
            'ollama_node = ican_orchestrator.ollama_node:main',
            'ollama_chat = ican_orchestrator.ollama_chat:main',
            'ollama_mcp_chat = ican_orchestrator.ollama_mcp_chat:main',
        ],
    },
)
