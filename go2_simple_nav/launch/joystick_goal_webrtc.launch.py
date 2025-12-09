from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_simple_nav',
            executable='joystick_goal_webrtc',
            name='joystick_goal_webrtc',
            output='screen',
        ),
    ])
