from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch joy node and teleop node for Xbox 360 controller."""
    
    return LaunchDescription([       
        # Teleop node - converts joy to cmd_vel
        Node(
            package='ican_move',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[{
                'linear_scale': 0.5,
                'angular_scale': 1.0,
                'slow_scale': 0.3,
                'turbo_scale': 2.0,
                'enable_button': 0,  # A button
                'deadzone': 0.1,
            }],
        ),
    ])
