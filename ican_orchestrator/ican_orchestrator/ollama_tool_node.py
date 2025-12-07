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
import re


class OllamaToolNode(Node):
    """ROS 2 Node that interfaces with Ollama LLM and ROS2 tool services"""

    def __init__(self):
        super().__init__('ollama_tool_node')
        
        # Configuration
        self.declare_parameter('llm_model', 'qwen2.5:7b')
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        
        self.get_logger().info(f'Ollama Tool Node initialized. Model: {self.llm_model}')
        
        self.client = ollama
        
        # Publishers/Subscribers
        self.response_pub = self.create_publisher(String, '/llm_response', 10)
        self.prompt_sub = self.create_subscription(String, '/llm_prompt', self.prompt_callback, 10)
        
        # Dice tool integration
        self.dice_command_pub = self.create_publisher(String, '/dice/command', 10)
        self.dice_result_sub = self.create_subscription(String, '/dice/result', self.dice_result_callback, 10)
        
        self.dice_result = None
        self.waiting_for_dice = False
        
        self.lock = threading.Lock()
        
        self.get_logger().info('Publishers/Subscribers ready:')
        self.get_logger().info('  - Subscribed to: /llm_prompt')
        self.get_logger().info('  - Publishing to: /llm_response')
        self.get_logger().info('  - Tool command: /dice/command')
        self.get_logger().info('  - Tool result: /dice/result')
    
    def dice_result_callback(self, msg):
        """Receive dice roll results"""
        self.dice_result = msg.data
        self.waiting_for_dice = False
        self.get_logger().info(f'Received dice result: {msg.data[:50]}...')
    
    def prompt_callback(self, msg):
        """Handle incoming prompts"""
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn('Prompt ignored: LLM is busy')
            return
        
        thread = threading.Thread(target=self.process_prompt, args=(msg.data,))
        thread.start()
    
    def process_prompt(self, prompt_text):
        """Process prompt with tool detection"""
        self.get_logger().info(f'Processing: "{prompt_text[:50]}..."')
        
        try:
            start_time = time.time()
            
            # Check if this is a dice-related query
            if self._is_dice_query(prompt_text):
                response_text = self._process_with_dice_tool(prompt_text)
            else:
                # Regular LLM response
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
    
    def _is_dice_query(self, text: str) -> bool:
        """Detect if query is about dice rolling"""
        dice_keywords = [
            'roll', 'dice', 'd20', 'd6', 'd8', 'd10', 'd12', 'd100',
            'coin flip', 'flip a coin', 'character stats', 'ability scores',
            'percentile', 'roll history'
        ]
        text_lower = text.lower()
        return any(keyword in text_lower for keyword in dice_keywords)
    
    def _extract_dice_command(self, text: str) -> str:
        """Extract dice notation from text"""
        # Look for dice notation patterns
        dice_pattern = r'\d*d\d+[\+\-]?\d*'
        match = re.search(dice_pattern, text.lower())
        
        if match:
            return match.group(0)
        elif 'stats' in text.lower() or 'ability' in text.lower():
            return 'stats'
        elif 'coin' in text.lower() or 'flip' in text.lower():
            return 'coin flip'
        elif 'history' in text.lower():
            return 'history'
        else:
            # Default to d20
            return 'd20'
    
    def _process_with_dice_tool(self, prompt_text: str) -> str:
        """Process prompt using dice tool"""
        self.get_logger().info('Detected dice query - calling dice tool')
        
        # Extract dice command
        dice_command = self._extract_dice_command(prompt_text)
        self.get_logger().info(f'Dice command: {dice_command}')
        
        # Send dice command
        cmd_msg = String()
        cmd_msg.data = dice_command
        self.dice_command_pub.publish(cmd_msg)
        
        # Wait for result (with timeout)
        self.waiting_for_dice = True
        self.dice_result = None
        
        timeout = 5.0
        start_wait = time.time()
        while self.waiting_for_dice and (time.time() - start_wait) < timeout:
            time.sleep(0.1)
        
        if self.dice_result is None:
            return "❌ Error: Dice service did not respond. Is dice_service_node running?"
        
        # Generate natural language response with dice result
        context_prompt = f"""The user asked: "{prompt_text}"

The dice rolling tool returned this result:
{self.dice_result}

Respond naturally to the user, incorporating the dice roll result. Be friendly and concise."""
        
        response = self.client.generate(
            model=self.llm_model,
            prompt=context_prompt,
            options={"temperature": 0.7, "num_ctx": 4096}
        )
        
        return response['response'].strip()


def main(args=None):
    rclpy.init(args=args)
    
    ollama_tool_node = OllamaToolNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(ollama_tool_node, executor=executor)
    
    ollama_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
