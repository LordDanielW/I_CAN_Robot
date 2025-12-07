#!/usr/bin/env python3
"""
Teleop node for Xbox 360 controller to cmd_vel mapping.

Maps Xbox 360 controller inputs from /joy topic to /cmd_vel for robot control.

Controller mapping:
- Left stick Y-axis: Linear velocity (forward/backward)
- Right stick X-axis: Angular velocity (turn left/right)
- Left trigger (LT): Slow mode multiplier
- Right trigger (RT): Turbo mode multiplier
- A button: Enable/disable teleop (safety)

Author: LordDanielW
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopNode(Node):
    """Teleop node for Xbox 360 controller."""

    def __init__(self):
        super().__init__('teleop_node')
        
        # Parameters
        self.declare_parameter('linear_scale', 0.5)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('slow_scale', 0.3)
        self.declare_parameter('turbo_scale', 2.0)
        self.declare_parameter('enable_button', 0)  # A button on Xbox 360
        self.declare_parameter('deadzone', 0.1)
        
        # Get parameters
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.slow_scale = self.get_parameter('slow_scale').value
        self.turbo_scale = self.get_parameter('turbo_scale').value
        self.enable_button = self.get_parameter('enable_button').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Xbox 360 controller axis mapping
        # Axes: [LS_X, LS_Y, LT, RS_X, RS_Y, RT, DPAD_X, DPAD_Y]
        self.axis_linear = 1   # Left stick Y-axis (forward/backward)
        self.axis_angular = 3  # Right stick X-axis (turn left/right)
        self.axis_slow = 2     # Left trigger (LT) - returns -1.0 to 1.0
        self.axis_turbo = 5    # Right trigger (RT) - returns -1.0 to 1.0
        
        # State
        self.enabled = False
        self.last_joy_time = self.get_clock().now()
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Safety timer - stop robot if no joy messages received
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info('Teleop node started')
        self.get_logger().info(f'Press button {self.enable_button} (A) to enable teleop')
        self.get_logger().info('Left stick Y: forward/backward, Right stick X: turn')
        self.get_logger().info('LT: slow mode, RT: turbo mode')

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def normalize_trigger(self, trigger_value):
        """
        Normalize trigger value from [-1.0, 1.0] to [0.0, 1.0].
        Xbox triggers rest at -1.0 and go to 1.0 when pressed.
        """
        return (trigger_value + 1.0) / 2.0

    def joy_callback(self, msg: Joy):
        """Handle joystick messages."""
        self.last_joy_time = self.get_clock().now()
        
        # Check if we have enough buttons
        if len(msg.buttons) <= self.enable_button:
            self.get_logger().warn(f'Not enough buttons on controller (need at least {self.enable_button + 1})')
            return
        
        # Toggle enable state on button press
        if msg.buttons[self.enable_button] == 1:
            self.enabled = not self.enabled
            status = "ENABLED" if self.enabled else "DISABLED"
            self.get_logger().info(f'Teleop {status}')
        
        # Only send commands if enabled
        if not self.enabled:
            # Send stop command when disabled
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Check if we have enough axes
        required_axes = max(self.axis_linear, self.axis_angular, self.axis_slow, self.axis_turbo) + 1
        if len(msg.axes) < required_axes:
            self.get_logger().warn(f'Not enough axes on controller (need at least {required_axes})')
            return
        
        # Get joystick values with deadzone
        linear = self.apply_deadzone(msg.axes[self.axis_linear])
        angular = self.apply_deadzone(msg.axes[self.axis_angular])
        
        # Get trigger values (normalize from [-1, 1] to [0, 1])
        slow_trigger = self.normalize_trigger(msg.axes[self.axis_slow])
        turbo_trigger = self.normalize_trigger(msg.axes[self.axis_turbo])
        
        # Calculate speed multiplier based on triggers
        speed_multiplier = 1.0
        
        # Slow mode takes priority
        if slow_trigger > 0.1:
            speed_multiplier = self.slow_scale
        elif turbo_trigger > 0.1:
            speed_multiplier = self.turbo_scale
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear * self.linear_scale * speed_multiplier
        twist.angular.z = angular * self.angular_scale * speed_multiplier
        
        self.cmd_vel_pub.publish(twist)
        
        # Log when actually moving
        if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.01:
            mode = ""
            if slow_trigger > 0.1:
                mode = " [SLOW]"
            elif turbo_trigger > 0.1:
                mode = " [TURBO]"
            self.get_logger().debug(
                f'cmd_vel: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}{mode}'
            )

    def safety_check(self):
        """Safety check - stop robot if no joy messages received recently."""
        time_since_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        
        if time_since_joy > 1.0 and self.enabled:
            self.get_logger().warn('No joystick input - stopping robot for safety')
            self.enabled = False
            twist = Twist()
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
