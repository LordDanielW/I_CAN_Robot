import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_voice'

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
    install_requires=['setuptools', 'vosk', 'pyttsx3', 'pyaudio', 'faster-whisper'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='user@todo.todo',
    description='Part of I_CAN_Robot stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whisper_server_node = ican_voice.whisper_server_node:main',
            'vosk_node = ican_voice.vosk_node:main',
            'vosk_server_node = ican_voice.vosk_server_node:main',
            'audio_streamer_node = ican_voice.audio_streamer_node:main',
            'tts_node = ican_voice.tts_node:main',
        ],
    },
)
