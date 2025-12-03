#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama # Import the Ollama Python library
import threading
import time

class OllamaNode(Node):
    """
    ROS 2 Node that interfaces with a local Ollama LLM server.

    It subscribes to '/llm_prompt' for user text input and publishes the
    AI response to '/llm_response'.
    """

    def __init__(self):
        super().__init__('ollama_node')
        
        # --- Configuration Parameters ---
        self.declare_parameter('llm_model', 'qwen2.5:7b')
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        self.get_logger().info(f'OllamaNode initialized. Using model: {self.llm_model}')

        # --- Ollama Client Setup ---
        # Ollama usually runs on localhost:11434. The client handles the API calls.
        self.client = ollama
        
        # --- ROS 2 Communication ---
        
        # 1. Publisher for the LLM's response
        self.publisher_ = self.create_publisher(
            String, 
            '/llm_response', 
            10
        )
        self.get_logger().info('Publishing to /llm_response (std_msgs/String)')
        
        # 2. Subscriber for the user's prompt/query
        self.subscription = self.create_subscription(
            String,
            '/llm_prompt',
            self.prompt_callback,
            10
        )
        self.get_logger().info('Subscribing to /llm_prompt (std_msgs/String)')
        
        # Lock to prevent multiple simultaneous model calls, as LLM calls can be slow
        self.lock = threading.Lock()


    def prompt_callback(self, msg):
        """Callback function executed when a new prompt is received."""
        
        # If the lock is held, a request is already being processed.
        if not self.lock.acquire(blocking=False):
            self.get_logger().warn('Prompt ignored: LLM is currently busy processing another request.')
            return

        # Start a new thread for the LLM query to avoid blocking the ROS 2 executor thread
        thread = threading.Thread(target=self.process_llm_query, args=(msg.data,))
        thread.start()

    def process_llm_query(self, prompt_text):
        """Separate thread function to handle the synchronous Ollama API call."""
        
        self.get_logger().info(f'Received prompt: "{prompt_text[:50]}..."')
        
        response_msg = String()
        
        try:
            # Synchronous call to Ollama generate function
            start_time = time.time()
            
            # Using ollama.generate for a single, non-conversational prompt
            response = self.client.generate(
                model=self.llm_model, 
                prompt=prompt_text,
                # Set options for faster inference (optional)
                options={
                    "temperature": 0.5, # Slightly creative
                    "num_ctx": 4096     # Context window size
                }
            )

            # Extract the generated text
            llm_text = response['response'].strip()
            
            end_time = time.time()
            duration = end_time - start_time

            response_msg.data = llm_text
            self.publisher_.publish(response_msg)
            
            self.get_logger().info(f'Response published in {duration:.2f}s.')
            self.get_logger().info(f'AI Response: "{llm_text[:50]}..."')

        except ollama.ResponseError as e:
            error_msg = f"LLM API Error: {e}. Is Ollama running and is the model installed?"
            self.get_logger().error(error_msg)
            
            # Publish an error message to the response topic
            response_msg.data = f"ERROR: LLM API Failure: {error_msg}"
            self.publisher_.publish(response_msg)

        finally:
            # Release the lock so the node can process the next request
            self.lock.release()

def main(args=None):
    rclpy.init(args=args)
    
    ollama_node = OllamaNode()
    
    # Use a MultiThreadedExecutor to handle the main ROS loop 
    # while the Ollama thread is running.
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(ollama_node, executor=executor)

    ollama_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()