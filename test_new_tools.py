#!/usr/bin/env python3
"""
Quick Test Script for New Tools

Tests the move_robot and query_room tools independently.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class ToolTester(Node):
    def __init__(self):
        super().__init__('tool_tester')
        
        # Publishers for testing
        self.move_pub = self.create_publisher(String, '/move_robot/command', 10)
        self.query_pub = self.create_publisher(String, '/query_room/command', 10)
        
        # Subscribers for results
        self.create_subscription(String, '/tool_result', self.result_callback, 10)
        self.create_subscription(String, '/move_robot/status', self.move_status_callback, 10)
        self.create_subscription(String, '/query_room/status', self.query_status_callback, 10)
        self.create_subscription(String, '/tts/speak', self.tts_callback, 10)
        
        self.get_logger().info('Tool Tester Ready!')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  1 - Test move to bathroom')
        self.get_logger().info('  2 - Test move through door')
        self.get_logger().info('  3 - Test describe room (vision)')
        self.get_logger().info('  4 - Test check safety (vision)')
        self.get_logger().info('  0 - Exit')
        
        # Start interactive loop
        self.timer = self.create_timer(0.1, self.menu_loop)
        self.waiting_for_input = False
    
    def result_callback(self, msg):
        print(f'\n📦 TOOL RESULT:\n{msg.data}\n')
    
    def move_status_callback(self, msg):
        print(f'🤖 MOVE STATUS: {msg.data}')
    
    def query_status_callback(self, msg):
        print(f'👁️  QUERY STATUS: {msg.data}')
    
    def tts_callback(self, msg):
        print(f'\n🔊 TTS SPEAKS: {msg.data}\n')
    
    def menu_loop(self):
        if self.waiting_for_input:
            return
        
        self.waiting_for_input = True
        
        try:
            choice = input('\nEnter command (1-4, 0 to exit): ').strip()
            
            if choice == '0':
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                return
            elif choice == '1':
                self.test_move_bathroom()
            elif choice == '2':
                self.test_move_door()
            elif choice == '3':
                self.test_describe_room()
            elif choice == '4':
                self.test_check_safety()
            else:
                print('Invalid choice!')
        
        except KeyboardInterrupt:
            rclpy.shutdown()
        finally:
            self.waiting_for_input = False
    
    def test_move_bathroom(self):
        print('\n🧪 Testing: Move to bathroom...')
        msg = String()
        msg.data = 'bathroom'
        self.move_pub.publish(msg)
    
    def test_move_door(self):
        print('\n🧪 Testing: Move through door...')
        msg = String()
        msg.data = 'door'
        self.move_pub.publish(msg)
    
    def test_describe_room(self):
        print('\n🧪 Testing: Describe room (vision)...')
        msg = String()
        msg.data = 'describe room'
        self.query_pub.publish(msg)
    
    def test_check_safety(self):
        print('\n🧪 Testing: Check safety (vision)...')
        msg = String()
        msg.data = 'check safety'
        self.query_pub.publish(msg)


def main():
    rclpy.init()
    tester = ToolTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
