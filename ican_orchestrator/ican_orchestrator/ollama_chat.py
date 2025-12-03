#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OllamaChatNode(Node):
    def __init__(self):
        super().__init__('ollama_chat')
        
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
        
        self.get_logger().info('OllamaChatNode initialized.')
        self.get_logger().info('Publishing to /llm_prompt')
        self.get_logger().info('Subscribing to /llm_response')
        
        # Create a timer to send periodic prompts (for testing)
        # Remove or comment this out if you want to send prompts manually
        self.timer = self.create_timer(10.0, self.send_test_prompt)
        self.prompt_count = 0
        
    def response_callback(self, msg):
        """Callback function for receiving LLM responses"""
        self.get_logger().info(f'Received response: {msg.data}')
        
    def send_test_prompt(self):
        """Send a test prompt to the LLM"""
        self.prompt_count += 1
        prompt_msg = String()
        prompt_msg.data = f"Tell me a short fact about robotics (#{self.prompt_count})"
        
        self.prompt_publisher.publish(prompt_msg)
        self.get_logger().info(f'Sent prompt: {prompt_msg.data}')
        
    def send_prompt(self, prompt_text):
        """Public method to send a custom prompt"""
        prompt_msg = String()
        prompt_msg.data = prompt_text
        self.prompt_publisher.publish(prompt_msg)
        self.get_logger().info(f'Sent prompt: {prompt_text}')


def main(args=None):
    rclpy.init(args=args)
    node = OllamaChatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
