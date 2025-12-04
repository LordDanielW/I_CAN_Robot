#!/usr/bin/env python3
"""
Ollama Chat with MCP - Test client for dice_mcp_node

This node sends prompts to the LLM asking it to use dice rolling tools.
It demonstrates how to test MCP server functionality.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OllamaMCPChatNode(Node):
    def __init__(self):
        super().__init__('ollama_mcp_chat')
        
        # Publisher to send prompts to the LLM
        self.prompt_publisher = self.create_publisher(
            String,
            '/llm_prompt',
            10
        )
        
        # Subscriber to receive responses from the LLM
        self.response_subscriber = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )
        
        self.get_logger().info('OllamaMCPChatNode initialized.')
        self.get_logger().info('Publishing to /llm_prompt')
        self.get_logger().info('Subscribing to /llm_response')
        
        # Test prompts for dice rolling MCP server
        self.test_prompts = [
            "Roll a d20 for me",
            "Roll 3d6 and tell me the total",
            "I need to roll 2d10+5 for my attack roll",
            "Generate character stats using 4d6 drop lowest method",
            "Flip a coin for me",
            "Roll percentile dice (d100)",
            "Show me the last 5 dice rolls from history",
            "Roll 4d6-2 for my damage",
        ]
        
        self.current_prompt_index = 0
        
        # Create a timer to send test prompts every 15 seconds
        self.timer = self.create_timer(15.0, self.send_next_test_prompt)
        
        self.get_logger().info(f'Will test {len(self.test_prompts)} different dice rolling scenarios')
        
    def response_callback(self, msg):
        """Callback function for receiving LLM responses"""
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'Received response:\n{msg.data}')
        self.get_logger().info('=' * 80)
        
    def send_next_test_prompt(self):
        """Send the next test prompt in sequence"""
        if self.current_prompt_index >= len(self.test_prompts):
            # Loop back to start
            self.current_prompt_index = 0
        
        prompt_text = self.test_prompts[self.current_prompt_index]
        self.send_prompt(prompt_text)
        
        self.current_prompt_index += 1
        
    def send_prompt(self, prompt_text):
        """Public method to send a custom prompt"""
        prompt_msg = String()
        prompt_msg.data = prompt_text
        self.prompt_publisher.publish(prompt_msg)
        self.get_logger().info(f'\n📤 Sent prompt: "{prompt_text}"')


def main(args=None):
    rclpy.init(args=args)
    node = OllamaMCPChatNode()
    
    # Send first prompt immediately
    node.send_next_test_prompt()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
