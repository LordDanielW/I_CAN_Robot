#!/usr/bin/env python3
"""
Brain Node - MCP Host for Unitree Go2 Robot
Orchestrates robot behavior by consulting Qwen LLM and executing tools via MCP servers.
"""

import asyncio
import json
from contextlib import AsyncExitStack
from typing import List, Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from openai import AsyncOpenAI
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client


class BrainNode(Node):
    """
    MCP Host node that orchestrates robot behavior.
    Connects to two MCP servers (behaviors and senses) and uses Qwen LLM for decision making.
    """

    def __init__(self):
        super().__init__('brain_node')
        
        # Create a reentrant callback group for async operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize OpenAI client for Ollama
        self.llm_client = AsyncOpenAI(
            base_url="http://localhost:11434/v1",
            api_key="ollama"  # Ollama doesn't need a real key
        )
        
        # MCP server configurations
        self.behavior_server_params = StdioServerParameters(
            command="ros2",
            args=["run", "ican_mcp_server", "behavior_server"]
        )
        
        self.senses_server_params = StdioServerParameters(
            command="ros2",
            args=["run", "ican_mcp_server", "senses_server"]
        )
        
        # MCP sessions and clients (will be initialized in setup)
        self.exit_stack: Optional[AsyncExitStack] = None
        self.behavior_session: Optional[ClientSession] = None
        self.senses_session: Optional[ClientSession] = None
        
        # System prompt for the LLM
        self.system_prompt = (
            "You are a Unitree Go2 robot dog. You control your body via the 'Behaviors' tools "
            "and check your status via 'Senses' tools. Always check your surroundings before moving. "
            "Be helpful, friendly, and cautious. Respond concisely and act appropriately based on "
            "the user's commands."
        )
        
        # Subscribe to human speech
        self.speech_subscription = self.create_subscription(
            String,
            '/human/speech',
            self.speech_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for robot responses (optional)
        self.response_publisher = self.create_publisher(
            String,
            '/robot/response',
            10
        )
        
        self.get_logger().info('Brain Node initialized. Waiting for MCP server connections...')
        
        # Flag to track initialization
        self.mcp_initialized = False

    async def initialize_mcp_connections(self):
        """Initialize connections to both MCP servers."""
        try:
            self.exit_stack = AsyncExitStack()
            
            # Connect to behavior server
            self.get_logger().info('Connecting to behavior MCP server...')
            behavior_stdio = await self.exit_stack.enter_async_context(
                stdio_client(self.behavior_server_params)
            )
            behavior_read, behavior_write = behavior_stdio
            self.behavior_session = await self.exit_stack.enter_async_context(
                ClientSession(behavior_read, behavior_write)
            )
            await self.behavior_session.initialize()
            self.get_logger().info('Connected to behavior MCP server')
            
            # Connect to senses server
            self.get_logger().info('Connecting to senses MCP server...')
            senses_stdio = await self.exit_stack.enter_async_context(
                stdio_client(self.senses_server_params)
            )
            senses_read, senses_write = senses_stdio
            self.senses_session = await self.exit_stack.enter_async_context(
                ClientSession(senses_read, senses_write)
            )
            await self.senses_session.initialize()
            self.get_logger().info('Connected to senses MCP server')
            
            self.mcp_initialized = True
            self.get_logger().info('All MCP connections established successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MCP connections: {e}')
            raise

    async def cleanup_mcp_connections(self):
        """Cleanup MCP server connections."""
        if self.exit_stack:
            try:
                await self.exit_stack.aclose()
                self.get_logger().info('MCP connections closed')
            except Exception as e:
                self.get_logger().error(f'Error closing MCP connections: {e}')

    async def get_all_tools(self) -> List[Dict[str, Any]]:
        """
        Aggregate tools from both MCP servers and convert to OpenAI format.
        
        Returns:
            List of tools in OpenAI function calling format
        """
        tools_openai_format = []
        
        try:
            # Get tools from behavior server
            if self.behavior_session:
                behavior_tools = await self.behavior_session.list_tools()
                for tool in behavior_tools.tools:
                    tools_openai_format.append(self._mcp_tool_to_openai(tool, "behavior"))
            
            # Get tools from senses server
            if self.senses_session:
                senses_tools = await self.senses_session.list_tools()
                for tool in senses_tools.tools:
                    tools_openai_format.append(self._mcp_tool_to_openai(tool, "senses"))
            
            self.get_logger().debug(f'Collected {len(tools_openai_format)} tools total')
            
        except Exception as e:
            self.get_logger().error(f'Error fetching tools: {e}')
        
        return tools_openai_format

    def _mcp_tool_to_openai(self, mcp_tool: Any, server_type: str) -> Dict[str, Any]:
        """
        Convert MCP tool format to OpenAI function calling format.
        
        Args:
            mcp_tool: Tool from MCP server
            server_type: Type of server ("behavior" or "senses")
            
        Returns:
            Tool in OpenAI format
        """
        # Build the function definition
        function_def = {
            "name": f"{server_type}_{mcp_tool.name}",
            "description": mcp_tool.description or f"Tool: {mcp_tool.name}",
        }
        
        # Convert input schema if available
        if hasattr(mcp_tool, 'inputSchema') and mcp_tool.inputSchema:
            function_def["parameters"] = mcp_tool.inputSchema
        else:
            # Default empty schema
            function_def["parameters"] = {
                "type": "object",
                "properties": {},
                "required": []
            }
        
        return {
            "type": "function",
            "function": function_def
        }

    async def execute_tool(self, tool_name: str, arguments: Dict[str, Any]) -> str:
        """
        Execute a tool on the appropriate MCP server.
        
        Args:
            tool_name: Name of the tool (with server prefix)
            arguments: Tool arguments
            
        Returns:
            Tool execution result as string
        """
        try:
            # Determine which server to use based on prefix
            if tool_name.startswith("behavior_"):
                actual_name = tool_name.replace("behavior_", "", 1)
                session = self.behavior_session
                server_type = "behavior"
            elif tool_name.startswith("senses_"):
                actual_name = tool_name.replace("senses_", "", 1)
                session = self.senses_session
                server_type = "senses"
            else:
                return f"Error: Unknown tool prefix in '{tool_name}'"
            
            if not session:
                return f"Error: {server_type} server not connected"
            
            self.get_logger().info(f'Executing tool: {actual_name} on {server_type} server')
            self.get_logger().debug(f'Arguments: {arguments}')
            
            # Call the tool on the MCP server
            result = await session.call_tool(actual_name, arguments)
            
            # Extract content from result
            if hasattr(result, 'content') and result.content:
                # Handle list of content items
                if isinstance(result.content, list):
                    content_parts = []
                    for item in result.content:
                        if hasattr(item, 'text'):
                            content_parts.append(item.text)
                        else:
                            content_parts.append(str(item))
                    return "\n".join(content_parts)
                else:
                    return str(result.content)
            else:
                return str(result)
                
        except Exception as e:
            error_msg = f"Error executing tool {tool_name}: {e}"
            self.get_logger().error(error_msg)
            return error_msg

    def speech_callback(self, msg: String):
        """
        Callback for human speech messages.
        This is a synchronous callback that launches async processing.
        """
        if not self.mcp_initialized:
            self.get_logger().warn('MCP servers not initialized yet. Ignoring speech input.')
            return
        
        speech_text = msg.data
        self.get_logger().info(f'Received speech: "{speech_text}"')
        
        # Process the speech asynchronously
        asyncio.run_coroutine_threadsafe(
            self.process_speech(speech_text),
            asyncio.get_event_loop()
        )

    async def process_speech(self, speech_text: str):
        """
        Process human speech by consulting LLM and executing tools.
        
        Args:
            speech_text: Text from human speech
        """
        try:
            # Get all available tools
            tools = await self.get_all_tools()
            
            # Prepare messages for LLM
            messages = [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": speech_text}
            ]
            
            self.get_logger().info('Consulting Qwen LLM...')
            
            # Call LLM with tools
            response = await self.llm_client.chat.completions.create(
                model="qwen2.5:latest",
                messages=messages,
                tools=tools if tools else None,
                tool_choice="auto" if tools else None,
                temperature=0.7,
                max_tokens=1000
            )
            
            response_message = response.choices[0].message
            
            # Check if LLM wants to call a tool
            if response_message.tool_calls:
                self.get_logger().info(f'LLM requested {len(response_message.tool_calls)} tool call(s)')
                
                # Execute each tool call
                for tool_call in response_message.tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)
                    
                    self.get_logger().info(f'Calling tool: {function_name}')
                    
                    # Execute the tool
                    tool_result = await self.execute_tool(function_name, function_args)
                    
                    self.get_logger().info(f'Tool result: {tool_result}')
                    
                    # Add tool call and result to messages
                    messages.append({
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [{
                            "id": tool_call.id,
                            "type": "function",
                            "function": {
                                "name": function_name,
                                "arguments": tool_call.function.arguments
                            }
                        }]
                    })
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "content": tool_result
                    })
                
                # Get final response from LLM after tool execution
                self.get_logger().info('Getting final response from LLM...')
                final_response = await self.llm_client.chat.completions.create(
                    model="qwen2.5:latest",
                    messages=messages,
                    temperature=0.7,
                    max_tokens=500
                )
                
                final_text = final_response.choices[0].message.content
                self.get_logger().info(f'Robot response: {final_text}')
                
                # Publish response
                response_msg = String()
                response_msg.data = final_text if final_text else "Action completed."
                self.response_publisher.publish(response_msg)
                
            else:
                # No tool calls, just use the LLM's response
                response_text = response_message.content
                self.get_logger().info(f'Robot response: {response_text}')
                
                # Publish response
                response_msg = String()
                response_msg.data = response_text if response_text else "I understand."
                self.response_publisher.publish(response_msg)
                
        except Exception as e:
            error_msg = f'Error processing speech: {e}'
            self.get_logger().error(error_msg)
            
            # Publish error response
            response_msg = String()
            response_msg.data = "Sorry, I encountered an error processing your request."
            self.response_publisher.publish(response_msg)


async def run_node():
    """Async function to run the ROS 2 node with MCP connections."""
    rclpy.init()
    
    brain_node = BrainNode()
    
    try:
        # Initialize MCP connections
        await brain_node.initialize_mcp_connections()
        
        # Create executor with multiple threads for async callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(brain_node)
        
        # Spin in a separate thread
        import threading
        spin_thread = threading.Thread(
            target=executor.spin,
            daemon=True
        )
        spin_thread.start()
        
        brain_node.get_logger().info('Brain Node is running. Listening for speech...')
        
        # Keep the async loop running
        while rclpy.ok():
            await asyncio.sleep(0.1)
            
    except KeyboardInterrupt:
        brain_node.get_logger().info('Keyboard interrupt received')
    except Exception as e:
        brain_node.get_logger().error(f'Error in node execution: {e}')
    finally:
        # Cleanup
        await brain_node.cleanup_mcp_connections()
        brain_node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Main entry point."""
    try:
        asyncio.run(run_node())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
