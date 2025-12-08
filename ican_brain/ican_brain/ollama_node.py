#!/usr/bin/env python3
"""
Ollama Tool Node - LLM/VLM with ROS2-based tool calling

This node integrates Ollama LLM/VLM with ROS2 services/topics.
Handles two separate paths:
1. Text prompts (from prompt_node) → responses to tool_manager
2. Vision prompts (from query_room_node) → responses to tts_node

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama
import threading
import time
import base64


class OllamaToolNode(Node):
    """ROS 2 Node that interfaces with Ollama LLM/VLM for dual-path processing"""

    def __init__(self):
        super().__init__('ollama_tool_node')
        
        # Configuration - single model for both text and vision
        self.declare_parameter('model', 'qwen3-vl:8b')
        
        self.model = self.get_parameter('model').get_parameter_value().string_value
        
        self.get_logger().info(f'Ollama Tool Node initialized.')
        self.get_logger().info(f'  Model: {self.model} (handles both text and vision)')
        
        self.client = ollama
        
        # Publishers/Subscribers for TEXT PATH
        self.text_response_pub = self.create_publisher(String, '/llm_response', 10)
        self.text_prompt_sub = self.create_subscription(String, '/llm_prompt', self.text_prompt_callback, 10)
        
        # Publishers/Subscribers for VISION PATH
        self.vision_response_pub = self.create_publisher(String, '/tts/speak', 10)
        self.vision_query_sub = self.create_subscription(String, '/llm_vision_query', self.vision_query_callback, 10)
        
        # Separate locks for text and vision processing
        self.text_lock = threading.Lock()
        self.vision_lock = threading.Lock()
        
        self.get_logger().info('Publishers/Subscribers ready:')
        self.get_logger().info('  TEXT PATH:')
        self.get_logger().info('    - Subscribed to: /llm_prompt')
        self.get_logger().info('    - Publishing to: /llm_response')
        self.get_logger().info('  VISION PATH:')
        self.get_logger().info('    - Subscribed to: /llm_vision_query')
        self.get_logger().info('    - Publishing to: /tts/speak')
    
    def text_prompt_callback(self, msg):
        """Handle incoming text prompts (Path 1: whisper → prompt_node → ollama → tool_manager)"""
        if not self.text_lock.acquire(blocking=False):
            self.get_logger().warn('Text prompt ignored: LLM is busy with text processing')
            return
        
        thread = threading.Thread(target=self.process_text_prompt, args=(msg.data,))
        thread.start()
    
    def vision_query_callback(self, msg):
        """Handle incoming vision queries (Path 2: cam2image → query_room → ollama → tts)"""
        if not self.vision_lock.acquire(blocking=False):
            self.get_logger().warn('Vision query ignored: VLM is busy with vision processing')
            return
        
        thread = threading.Thread(target=self.process_vision_query, args=(msg.data,))
        thread.start()
    
    def process_text_prompt(self, prompt_text):
        """Process text prompt and generate response for tool_manager"""
        self.get_logger().info(f'[TEXT] Processing: "{prompt_text[:50]}..."')
        
        try:
            start_time = time.time()
            
            # Generate LLM response
            response = self.client.generate(
                model=self.model,
                prompt=prompt_text,
                options={"temperature": 0.7, "num_ctx": 4096}
            )
            response_text = response['response'].strip()
            
            # Publish response to tool_manager
            response_msg = String()
            response_msg.data = response_text
            self.text_response_pub.publish(response_msg)
            
            duration = time.time() - start_time
            self.get_logger().info(f'[TEXT] Response published in {duration:.2f}s')
            
        except Exception as e:
            self.get_logger().error(f'[TEXT] Error: {e}')
            error_msg = String()
            error_msg.data = f"ERROR: {str(e)}"
            self.text_response_pub.publish(error_msg)
        finally:
            self.text_lock.release()
    
    def process_vision_query(self, vision_message):
        """Process vision query and send response directly to TTS
        
        Expected format: VISION_QUERY|<prompt_text>|<base64_image>
        """
        self.get_logger().info(f'[VISION] Processing vision query...')
        
        try:
            # Parse vision message
            if not vision_message.startswith('VISION_QUERY|'):
                raise ValueError('Invalid vision query format')
            
            parts = vision_message.split('|', 2)
            if len(parts) != 3:
                raise ValueError('Vision query must have format: VISION_QUERY|<prompt>|<image_b64>')
            
            _, prompt_text, image_b64 = parts
            
            self.get_logger().info(f'[VISION] Prompt: "{prompt_text[:50]}..."')
            self.get_logger().info(f'[VISION] Image size: {len(image_b64)} bytes (base64)')
            
            start_time = time.time()
            
            # Call VLM with image using Ollama's chat API format
            response = self.client.chat(
                model=self.model,
                messages=[
                    {
                        'role': 'user',
                        'content': prompt_text,
                        'images': [image_b64]  # Ollama expects base64 image in 'images' field
                    }
                ],
                options={"temperature": 0.7, "num_ctx": 4096}
            )
            
            # Extract response text
            response_text = response['message']['content'].strip()
            
            if not response_text:
                response_text = "(VLM returned an empty response)"
            
            # Publish response directly to TTS
            tts_msg = String()
            tts_msg.data = response_text
            self.vision_response_pub.publish(tts_msg)
            
            duration = time.time() - start_time
            self.get_logger().info(f'[VISION] Response sent to TTS in {duration:.2f}s')
            self.get_logger().info(f'[VISION] VLM said: "{response_text[:100]}..."')
            
        except Exception as e:
            self.get_logger().error(f'[VISION] Error: {e}')
            # Send error to TTS so user knows something went wrong
            error_msg = String()
            error_msg.data = f"Sorry, I had trouble analyzing the image. {str(e)}"
            self.vision_response_pub.publish(error_msg)
        finally:
            self.vision_lock.release()


def main(args=None):
    rclpy.init(args=args)
    
    ollama_tool_node = OllamaToolNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(ollama_tool_node, executor=executor)
    
    ollama_tool_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
