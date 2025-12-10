#!/usr/bin/env python3
"""
Debug Monitor - Full message display for all robot topics

Use this to see complete messages without truncation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DebugMonitor(Node):
    def __init__(self):
        super().__init__('debug_monitor')
        
        # Subscribe to all major topics
        self.create_subscription(String, '/speech_text', self.speech_callback, 10)
        self.create_subscription(String, '/llm_prompt', self.prompt_callback, 10)
        self.create_subscription(String, '/llm_response', self.response_callback, 10)
        self.create_subscription(String, '/tool_result', self.tool_callback, 10)
        self.create_subscription(String, '/dice/command', self.dice_cmd_callback, 10)
        self.create_subscription(String, '/dice/result', self.dice_result_callback, 10)
        self.create_subscription(String, '/prompt_status', self.status_callback, 10)
        self.create_subscription(String, '/tts/speak', self.tts_callback, 10)
        self.create_subscription(String, '/llm_vision_query', self.vision_query_callback, 10)
        
        self.get_logger().info('='*80)
        self.get_logger().info('DEBUG MONITOR STARTED - Showing full messages')
        self.get_logger().info('='*80)
    
    def speech_callback(self, msg):
        self.print_message('SPEECH', msg.data)
    
    def prompt_callback(self, msg):
        self.print_message('LLM PROMPT', msg.data)
    
    def response_callback(self, msg):
        self.print_message('LLM RESPONSE', msg.data)
    
    def tool_callback(self, msg):
        self.print_message('TOOL RESULT', msg.data)
    
    def dice_cmd_callback(self, msg):
        self.print_message('DICE COMMAND', msg.data)
    
    def dice_result_callback(self, msg):
        self.print_message('DICE RESULT', msg.data)
    
    def status_callback(self, msg):
        self.print_message('STATUS', msg.data)
    
    def tts_callback(self, msg):
        self.print_message('TTS SPEAK', msg.data)
    
    def vision_query_callback(self, msg):
        self.print_message('VISION QUERY', msg.data)
    
    def print_message(self, topic_name, content):
        """Print full message with formatting"""
        print('\n' + '='*80)
        print(f'[{topic_name}]')
        print('-'*80)
        print(content)
        print('='*80 + '\n')


def main(args=None):
    rclpy.init(args=args)
    monitor = DebugMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
