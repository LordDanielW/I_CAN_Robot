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
from geometry_msgs.msg import Point


class MoveRobotNode(Node):
    """ROS 2 Node for preprogrammed robot movement"""
    
    def __init__(self):
        super().__init__('move_robot_node')
        self.get_logger().info('Move Robot Node initialized')
        
        # Movement state tracking
        self.is_moving = False
        self.current_destination = None
        
        # Publisher for navigation goals (to go2_simple_nav)
        self.goal_pub = self.create_publisher(
            Point,
            '/goal',
            10
        )
        
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
        self.get_logger().info('  - Publisher: /goal (to go2_simple_nav)')
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
            return self.move_to_door()
        else:
            return f"❌ Unknown movement command: '{command}'\nAvailable: bathroom, door"
    
    def move_to_bathroom(self) -> str:
        """Execute preprogrammed movement to bathroom"""
        self.is_moving = True
        self.current_destination = "bathroom"
        
        # Publish status
        status_msg = String()
        status_msg.data = "MOVING_TO_BATHROOM"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('🚶 Sending navigation goal to bathroom...')
        
        # Send goal to go2_simple_nav using coordinates from send_goal_once.py
        goal = Point()
        goal.x = 1.440033
        goal.y = -1.425135
        goal.z = 0.382595
        
        self.goal_pub.publish(goal)
        
        result = "🤖 Navigation goal sent to go2_simple_nav:\n"
        result += f"   Target: Bathroom\n"
        result += f"   Coordinates: ({goal.x:.3f}, {goal.y:.3f}, {goal.z:.3f})\n"
        result += "   go2_simple_nav will handle the navigation\n"
        result += "\n✓ Goal published to /goal topic"
        
        # Update status
        self.is_moving = False
        self.current_destination = None
        
        status_msg.data = "GOAL_SENT"
        self.status_pub.publish(status_msg)
        
        return result
    
    def move_to_door(self) -> str:
        """Execute preprogrammed movement to door"""
        self.is_moving = True
        self.current_destination = "door"
        
        # Publish status
        status_msg = String()
        status_msg.data = "MOVING_TO_DOOR"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('🍳 Sending navigation goal to door...')
        
        # Send goal to go2_simple_nav with door coordinates
        goal = Point()
        goal.x = 2.5
        goal.y = 1.0
        goal.z = 0.0
        
        self.goal_pub.publish(goal)
        
        result = "🤖 Navigation goal sent to go2_simple_nav:\n"
        result += f"   Target: door\n"
        result += f"   Coordinates: ({goal.x:.3f}, {goal.y:.3f}, {goal.z:.3f})\n"
        result += "   go2_simple_nav will handle the navigation\n"
        result += "\n✓ Goal published to /goal topic"
        
        # Update status
        self.is_moving = False
        self.current_destination = None
        
        status_msg.data = "GOAL_SENT"
        self.status_pub.publish(status_msg)
        
        return result
    
    def stop_movement(self) -> str:
        """Stop current movement by sending current position as goal"""
        self.get_logger().warn('⛔ Stop requested - sending zero velocity goal')
        
        # Send current position (0,0) as goal to stop
        goal = Point()
        goal.x = 0.0
        goal.y = 0.0
        goal.z = 0.0
        
        self.goal_pub.publish(goal)
        
        self.is_moving = False
        self.current_destination = None
        
        # Publish stop status
        status_msg = String()
        status_msg.data = "STOPPED"
        self.status_pub.publish(status_msg)
        
        return "⛔ Stop command sent to go2_simple_nav (goal at origin)"


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
