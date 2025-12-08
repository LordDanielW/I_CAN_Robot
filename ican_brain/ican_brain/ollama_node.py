#!/usr/bin/env python3
"""
Ollama Tool Node - LLM with ROS2-based tool calling

This node integrates Ollama LLM with ROS2 services/topics for tool calling.
Instead of MCP stdio communication, it uses ROS2 native communication.

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
import threading
import time


class OllamaToolNode(Node):
    """ROS 2 Node that interfaces with Ollama LLM"""

    def __init__(self):
        super().__init__('ollama_tool_node')
        
        # Configuration
        self.declare_parameter('llm_model', 'qwen2.5-vl:8b')
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        
        self.get_logger().info(f'Ollama Tool Node initialized. Model: {self.llm_model}')
        
        self.client = ollama
        
        # Publishers/Subscribers
        self.response_pub = self.create_publisher(String, '/llm_response', 10)
        self.prompt_sub = self.create_subscription(String, '/llm_prompt', self.prompt_callback, 10)
        
        self.lock = threading.Lock()
        
        self.get_logger().info('Publishers/Subscribers ready:')
        self.get_logger().info('  - Subscribed to: /llm_prompt')
        self.get_logger().info('  - Publishing to: /llm_response')
    
    def prompt_callback(self, msg):
        """Handle incoming prompts"""
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn('Prompt ignored: LLM is busy')
            return
        
        thread = threading.Thread(target=self.process_prompt, args=(msg.data,))
        thread.start()
    
    def process_prompt(self, prompt_text):
        """Process prompt and generate response"""
        self.get_logger().info(f'Processing: "{prompt_text[:50]}..."')
        
        try:
            start_time = time.time()
            
            # Generate LLM response
            response = self.client.generate(
                model=self.llm_model,
                prompt=prompt_text,
                options={"temperature": 0.7, "num_ctx": 4096}
            )
            response_text = response['response'].strip()
            
            # Publish response
            response_msg = String()
            response_msg.data = response_text
            self.response_pub.publish(response_msg)
            
            duration = time.time() - start_time
            self.get_logger().info(f'Response published in {duration:.2f}s')
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            error_msg = String()
            error_msg.data = f"ERROR: {str(e)}"
            self.response_pub.publish(error_msg)
        finally:
            self.lock.release()


def main(args=None):
    rclpy.init(args=args)
    
    ollama_tool_node = OllamaToolNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(ollama_tool_node, executor=executor)
    
    ollama_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
