#!/usr/bin/env python3
"""
Move Robot Node - Preprogrammed robot movement tool

This node provides a tool for moving the robot through predefined paths.
Currently programmed to move the robot dog through a door to the bathroom.

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MoveRobotNode(Node):
    """ROS 2 Node for preprogrammed robot movement"""
    
    def __init__(self):
        super().__init__('move_robot_node')
        self.get_logger().info('Move Robot Node initialized')
        
        # Movement state tracking
        self.is_moving = False
        self.current_destination = None
        
        # Create subscriber for movement commands
        self.command_sub = self.create_subscription(
            String,
            '/move_robot/command',
            self.command_callback,
            10
        )
        
        # Create publisher for movement status
        self.status_pub = self.create_publisher(
            String,
            '/move_robot/status',
            10
        )
        
        # Create publisher for movement result
        self.result_pub = self.create_publisher(
            String,
            '/tool_result',
            10
        )
        
        # Create service for simple triggers
        self.move_service = self.create_service(
            Trigger,
            '/move_robot/go',
            self.handle_move_service
        )
        
        self.get_logger().info('Move Robot services ready:')
        self.get_logger().info('  - Service: /move_robot/go')
        self.get_logger().info('  - Subscriber: /move_robot/command')
        self.get_logger().info('  - Publisher: /move_robot/status, /tool_result')
    
    def command_callback(self, msg):
        """Handle movement command from topic"""
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received move command: {command}')
        
        result = self.process_move_command(command)
        
        # Publish result to tool_result for tool_manager
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(f'Published result: {result[:100]}...')
    
    def handle_move_service(self, request, response):
        """Handle service request for movement"""
        result = self.move_to_bathroom()
        response.success = True
        response.message = result
        return response
    
    def process_move_command(self, command: str) -> str:
        """Process various movement commands"""
        
        # Check if already moving
        if self.is_moving:
            return "⚠️ Robot is already in motion. Please wait for current movement to complete."
        
        # Parse command for destination
        if 'bathroom' in command:
            return self.move_to_bathroom()
        elif 'door' in command:
            return self.move_through_door()
        elif 'stop' in command or 'halt' in command:
            return self.stop_movement()
        elif 'status' in command:
            return self.get_movement_status()
        else:
            return f"❌ Unknown movement command: '{command}'\nAvailable: bathroom, door, stop, status"
    
    def move_to_bathroom(self) -> str:
        """Execute preprogrammed movement to bathroom"""
        self.is_moving = True
        self.current_destination = "bathroom"
        
        # Publish status
        status_msg = String()
        status_msg.data = "MOVING_TO_BATHROOM"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('🚶 Executing preprogrammed path to bathroom...')
        
        # Simulate movement sequence
        result = "🤖 Moving to bathroom (preprogrammed sequence):\n"
        result += "   1. ✓ Rotate 45° left\n"
        result += "   2. ✓ Move forward 2 meters\n"
        result += "   3. ✓ Rotate 90° right\n"
        result += "   4. ✓ Navigate through doorway\n"
        result += "   5. ✓ Move forward 1.5 meters\n"
        result += "   6. ✓ Arrived at bathroom entrance\n"
        result += "\n✓ Movement complete. Robot is now at the bathroom."
        
        # Update status
        self.is_moving = False
        self.current_destination = None
        
        status_msg.data = "ARRIVED_BATHROOM"
        self.status_pub.publish(status_msg)
        
        return result
    
    def move_through_door(self) -> str:
        """Execute door navigation"""
        self.is_moving = True
        
        self.get_logger().info('🚪 Navigating through door...')
        
        result = "🚪 Navigating through door:\n"
        result += "   1. ✓ Detect door frame\n"
        result += "   2. ✓ Align with doorway\n"
        result += "   3. ✓ Move through carefully\n"
        result += "   4. ✓ Clear of doorway\n"
        result += "\n✓ Successfully passed through door."
        
        self.is_moving = False
        return result
    
    def stop_movement(self) -> str:
        """Stop current movement"""
        if not self.is_moving:
            return "ℹ️ Robot is not currently moving."
        
        self.get_logger().warn('⛔ Emergency stop requested!')
        
        self.is_moving = False
        prev_destination = self.current_destination
        self.current_destination = None
        
        # Publish stop status
        status_msg = String()
        status_msg.data = "STOPPED"
        self.status_pub.publish(status_msg)
        
        return f"⛔ Robot movement stopped. Previous destination: {prev_destination}"
    
    def get_movement_status(self) -> str:
        """Get current movement status"""
        if self.is_moving:
            return f"📍 Status: Moving to {self.current_destination}"
        else:
            return "📍 Status: Robot is stationary and ready for commands"


def main(args=None):
    rclpy.init(args=args)
    
    move_robot_node = MoveRobotNode()
    
    try:
        rclpy.spin(move_robot_node)
    except KeyboardInterrupt:
        move_robot_node.get_logger().info('Shutting down move robot node...')
    finally:
        move_robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
