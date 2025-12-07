#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama # Import the Ollama Python library
import threading
import time
import asyncio
import json
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

class OllamaNode(Node):
    """
    ROS 2 Node that interfaces with a local Ollama LLM server with MCP tool support.

    It subscribes to '/llm_prompt' for user text input and publishes the
    AI response to '/llm_response'. Supports tool calling via MCP servers.
    """

    def __init__(self):
        super().__init__('ollama_node')
        
        # --- Configuration Parameters ---
        self.declare_parameter('llm_model', 'qwen2.5:7b')
        self.declare_parameter('enable_mcp', True)
        self.declare_parameter('mcp_server_path', 
            '/home/fire/ros2_ws/src/I_CAN_Robot/ican_tools/ican_tools/dice_mcp_node.py')
        
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        self.enable_mcp = self.get_parameter('enable_mcp').get_parameter_value().bool_value
        self.mcp_server_path = self.get_parameter('mcp_server_path').get_parameter_value().string_value
        
        self.get_logger().info(f'OllamaNode initialized. Using model: {self.llm_model}')
        self.get_logger().info(f'MCP tool calling: {"ENABLED" if self.enable_mcp else "DISABLED"}')

        # --- Ollama Client Setup ---
        # Ollama usually runs on localhost:11434. The client handles the API calls.
        self.client = ollama
        
        # --- MCP Setup ---
        self.mcp_session = None
        self.mcp_tools = []
        self.ollama_tools = []
        
        if self.enable_mcp:
            # Start async event loop in separate thread for MCP
            self.loop = asyncio.new_event_loop()
            self.mcp_thread = threading.Thread(target=self._start_event_loop, daemon=True)
            self.mcp_thread.start()
            
            # Initialize MCP connection
            future = asyncio.run_coroutine_threadsafe(self._setup_mcp(), self.loop)
            try:
                future.result(timeout=10)
                self.get_logger().info(f'MCP initialized with {len(self.ollama_tools)} tools')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize MCP: {e}')
                self.enable_mcp = False
        
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
        
    def _start_event_loop(self):
        """Run asyncio event loop in separate thread for MCP operations"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    async def _setup_mcp(self):
        """Initialize MCP client and connect to server"""
        try:
            # Get ROS2 environment variables for the MCP server subprocess
            import os
            mcp_env = os.environ.copy()
            
            # Ensure PYTHONPATH includes ROS2
            ros2_python_path = '/opt/ros/jazzy/lib/python3.12/site-packages'
            if 'PYTHONPATH' in mcp_env:
                if ros2_python_path not in mcp_env['PYTHONPATH']:
                    mcp_env['PYTHONPATH'] = f"{ros2_python_path}:{mcp_env['PYTHONPATH']}"
            else:
                mcp_env['PYTHONPATH'] = ros2_python_path
            
            server_params = StdioServerParameters(
                command="python3",
                args=[self.mcp_server_path],
                env=mcp_env
            )
            
            # Create persistent connection
            self.read, self.write = await stdio_client(server_params).__aenter__()
            self.mcp_session = ClientSession(self.read, self.write)
            await self.mcp_session.__aenter__()
            
            # Initialize session
            await self.mcp_session.initialize()
            
            # Get available tools
            tools_response = await self.mcp_session.list_tools()
            self.mcp_tools = tools_response.tools
            
            # Convert to Ollama format
            self.ollama_tools = self._mcp_tools_to_ollama_format(self.mcp_tools)
            
            self.get_logger().info(f'Connected to MCP server: {self.mcp_server_path}')
            for tool in self.mcp_tools:
                self.get_logger().info(f'  - {tool.name}: {tool.description}')
                
        except Exception as e:
            self.get_logger().error(f'MCP setup failed: {e}')
            raise
    
    def _mcp_tools_to_ollama_format(self, mcp_tools):
        """Convert MCP tool definitions to Ollama tool format"""
        ollama_tools = []
        for tool in mcp_tools:
            ollama_tools.append({
                'type': 'function',
                'function': {
                    'name': tool.name,
                    'description': tool.description,
                    'parameters': tool.inputSchema
                }
            })
        return ollama_tools


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
        """Process LLM query with MCP tool support"""
        
        self.get_logger().info(f'Received prompt: "{prompt_text[:50]}..."')
        
        response_msg = String()
        
        try:
            start_time = time.time()
            
            if self.enable_mcp and self.ollama_tools:
                # Use chat API with tools
                final_response = self._process_with_tools(prompt_text)
            else:
                # Fallback to simple generate
                response = self.client.generate(
                    model=self.llm_model, 
                    prompt=prompt_text,
                    options={
                        "temperature": 0.5,
                        "num_ctx": 4096
                    }
                )
                final_response = response['response'].strip()
            
            end_time = time.time()
            duration = end_time - start_time

            response_msg.data = final_response
            self.publisher_.publish(response_msg)
            
            self.get_logger().info(f'Response published in {duration:.2f}s.')
            self.get_logger().info(f'AI Response: "{final_response[:100]}..."')

        except ollama.ResponseError as e:
            error_msg = f"LLM API Error: {e}. Is Ollama running and is the model installed?"
            self.get_logger().error(error_msg)
            
            response_msg.data = f"ERROR: LLM API Failure: {error_msg}"
            self.publisher_.publish(response_msg)
        except Exception as e:
            error_msg = f"Unexpected error: {e}"
            self.get_logger().error(error_msg)
            
            response_msg.data = f"ERROR: {error_msg}"
            self.publisher_.publish(response_msg)

        finally:
            self.lock.release()
    
    def _process_with_tools(self, prompt_text):
        """Process prompt using Ollama chat API with tool calling"""
        
        messages = [{'role': 'user', 'content': prompt_text}]
        
        # Initial chat request with tools
        response = self.client.chat(
            model=self.llm_model,
            messages=messages,
            tools=self.ollama_tools
        )
        
        # Check if model wants to use tools
        if response['message'].get('tool_calls'):
            self.get_logger().info(f"Model requested {len(response['message']['tool_calls'])} tool call(s)")
            
            # Add assistant's message to conversation
            messages.append(response['message'])
            
            # Process each tool call
            for tool_call in response['message']['tool_calls']:
                function_name = tool_call['function']['name']
                function_args = tool_call['function']['arguments']
                
                self.get_logger().info(f"Calling tool: {function_name} with args: {function_args}")
                
                # Call MCP server
                try:
                    future = asyncio.run_coroutine_threadsafe(
                        self._call_mcp_tool(function_name, function_args),
                        self.loop
                    )
                    tool_result = future.result(timeout=10)
                    
                    self.get_logger().info(f"Tool result: {tool_result[:100]}...")
                    
                    # Add tool result to messages
                    messages.append({
                        'role': 'tool',
                        'content': tool_result
                    })
                    
                except Exception as e:
                    error_msg = f"Tool call failed: {e}"
                    self.get_logger().error(error_msg)
                    messages.append({
                        'role': 'tool',
                        'content': f"Error: {error_msg}"
                    })
            
            # Get final response from model with tool results
            final_response = self.client.chat(
                model=self.llm_model,
                messages=messages
            )
            
            return final_response['message']['content']
        else:
            # No tool calls, return direct response
            return response['message']['content']
    
    async def _call_mcp_tool(self, tool_name, arguments):
        """Call MCP tool and return result"""
        try:
            result = await self.mcp_session.call_tool(tool_name, arguments)
            
            # Extract content from result
            if hasattr(result, 'content') and result.content:
                # Get text content from first content item
                content_item = result.content[0]
                if hasattr(content_item, 'text'):
                    return content_item.text
                else:
                    return str(content_item)
            else:
                return str(result)
                
        except Exception as e:
            self.get_logger().error(f"MCP tool call error: {e}")
            return f"Error calling {tool_name}: {e}"
    
    def destroy_node(self):
        """Cleanup MCP connections on shutdown"""
        if self.enable_mcp and self.mcp_session:
            try:
                # Schedule cleanup in event loop
                asyncio.run_coroutine_threadsafe(self._cleanup_mcp(), self.loop)
            except:
                pass
        
        super().destroy_node()
    
    async def _cleanup_mcp(self):
        """Cleanup MCP session"""
        try:
            if self.mcp_session:
                await self.mcp_session.__aexit__(None, None, None)
        except:
            pass


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