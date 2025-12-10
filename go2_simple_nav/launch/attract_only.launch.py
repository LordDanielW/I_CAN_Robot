from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the attract-only navigator to the requested goal."""

    goal_x = 12.294527
    goal_y = -4.360512

    return LaunchDescription([
        Node(
            package='go2_simple_nav',
            # This node isn't installed as an entrypoint; run via python executable
            executable='python3',
            name='attract_only_goal_webrtc',
            output='screen',
            arguments=['-u',
                       # full path to the script
                       'src/go2_simple_nav/go2_simple_nav/attract_only_goal_webrtc.py',
                       '--ros-args',
                       '-p', f'goal_x:={goal_x}',
                       '-p', f'goal_y:={goal_y}',
                       '-p', 'obstacle_avoidance:=true',
                       ],
            emulate_tty=True,
        ),
    ])
