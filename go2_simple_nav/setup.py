from setuptools import setup

package_name = 'go2_simple_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joystick_goal_webrtc.launch.py', 'launch/qwen3_vl.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unknown',
    maintainer_email='unknown@example.com',
    description='Lightweight goal-seeking navigation for Unitree Go2 (no Nav2, no SLAM).',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_nav = go2_simple_nav.simple_nav_node:main',
            'send_nav_goal = go2_simple_nav.send_goal_once:main',
            'send_nav_joystick = go2_simple_nav.send_joystick_once:main',
            'joystick_goal = go2_simple_nav.joystick_goal:main',
            'joystick_goal_webrtc = go2_simple_nav.joystick_goal_webrtc:main',
            'qwen3_vl_node = go2_simple_nav.qwen3_vl_node:main',
        ],
    },
)
