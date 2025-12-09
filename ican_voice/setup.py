import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ican_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: ['*.wav'],  # Include WAV files in the package
    },
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
            'audio_streamer_node = ican_voice.audio_streamer_node:main',
            'audio_playback_node = ican_voice.audio_playback_node:main',
            'audio_playback_ffmpeg = ican_voice.audio_playback_ffmpeg:main',
            'tts_node = ican_voice.tts_node:main',
            # Test nodes
            'test_audio_streamer = ican_voice.test_audio_streamer:main',
            'test_audio_playback = ican_voice.test_audio_playback:main',
            'test_whisper_to_tts = ican_voice.test_whisper_to_tts:main',
            'test_tts_to_file = ican_voice.test_tts_to_file:main',
            'test_sound_node = ican_voice.test_sound_node:main',
            'test_ffmpeg_stream = ican_voice.test_ffmpeg_stream:main',
        ],
    },
)
