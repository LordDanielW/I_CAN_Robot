#!/usr/bin/env python3
"""
Tool Manager Node - Central tool registry and execution

This node manages all available tools in the system:
1. Tracks status of all tool nodes (dice, movement, vision, etc.)
2. Provides tool discovery service
3. Receives LLM output and extracts tool calls
4. Routes tool calls to appropriate nodes
5. Collects tool results and publishes them

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import re


class ToolManagerNode(Node):
    """Central manager for all robot tools"""
    
    def __init__(self):
        super().__init__('tool_manager_node')
        self.get_logger().info('Tool Manager initializing...')
        
        # Tool registry: {tool_name: {topic, description, pattern, status}}
        self.tools = {
            'roll': {
                'topic': '/dice/command',
                'description': 'roll(dice_notation) - Roll dice using RPG notation (e.g., d20, 3d6+5, 2d10-2)',
                'pattern': r'TOOL:roll\(([^)]+)\)',
                'status': 'unknown'
            },
            'roll_stats': {
                'topic': '/dice/command',
                'description': 'roll_stats() - Generate RPG character ability scores (4d6 drop lowest, 6 times)',
                'pattern': r'TOOL:roll_stats\(\)',
                'status': 'unknown'
            },
            'coin_flip': {
                'topic': '/dice/command',
                'description': 'coin_flip(count) - Flip one or more coins (e.g., coin_flip(1), coin_flip(3))',
                'pattern': r'TOOL:coin_flip\((\d+)\)',
                'status': 'unknown'
            },
            'roll_history': {
                'topic': '/dice/command',
                'description': 'roll_history(limit) - Show recent dice roll history (e.g., roll_history(5))',
                'pattern': r'TOOL:roll_history\((\d+)\)',
                'status': 'unknown'
            }
        }
        
        # Publishers for tool commands
        self.dice_command_pub = self.create_publisher(String, '/dice/command', 10)
        
        # Subscriber for LLM output
        self.llm_output_sub = self.create_subscription(
            String,
            '/llm_response',
            self.llm_output_callback,
            10
        )
        
        # Service to list available tools
        self.list_service = self.create_service(
            Trigger,
            '/tools/list',
            self.handle_list_tools
        )
        
        # Subscriber to check dice service status
        self.dice_result_sub = self.create_subscription(
            String,
            '/dice/result',
            self.dice_result_callback,
            10
        )
        
        # Timer to check tool status
        self.status_timer = self.create_timer(5.0, self.check_tool_status)
        
        self.get_logger().info('Tool Manager ready')
        self.get_logger().info(f'  Registered tools: {list(self.tools.keys())}')
        self.get_logger().info('  Subscribed to: /llm_response')
        self.get_logger().info('  Service: /tools/list')
        
        # Initial status check
        self.check_tool_status()
    
    def check_tool_status(self):
        """Check if tool nodes are available"""
        # For now, assume dice tools are available if we're running
        # In the future, this could ping service nodes
        for tool_name in self.tools:
            if 'dice' in self.tools[tool_name]['topic']:
                # Try to detect if dice_service_node is running
                # Simple heuristic: if we've seen any dice results, it's active
                self.tools[tool_name]['status'] = 'available'
    
    def handle_list_tools(self, request, response):
        """Service handler to list available tools"""
        tool_descriptions = []
        
        for tool_name, tool_info in self.tools.items():
            if tool_info['status'] == 'available' or tool_info['status'] == 'unknown':
                tool_descriptions.append(f"{tool_info['description']}")
        
        if tool_descriptions:
            response.success = True
            response.message = '; '.join(tool_descriptions)
            self.get_logger().info(f'Tool list requested: {len(tool_descriptions)} tools available')
        else:
            response.success = False
            response.message = "No tools available"
            self.get_logger().warn('Tool list requested but no tools available')
        
        return response
    
    def llm_output_callback(self, msg):
        """Parse LLM output for tool calls and execute them"""
        llm_text = msg.data
        
        self.get_logger().info(f'Checking LLM output for tool calls...')
        
        # Search for tool calls in the LLM output
        tool_calls_found = []
        
        for tool_name, tool_info in self.tools.items():
            pattern = tool_info['pattern']
            matches = re.findall(pattern, llm_text, re.IGNORECASE)
            
            if matches:
                for match in matches:
                    tool_calls_found.append({
                        'name': tool_name,
                        'params': match if match else None,
                        'topic': tool_info['topic']
                    })
        
        if tool_calls_found:
            self.get_logger().info(f'🔧 Found {len(tool_calls_found)} tool call(s)!')
            
            for tool_call in tool_calls_found:
                self.execute_tool(tool_call)
        else:
            self.get_logger().info('No tool calls found in LLM output')
    
    def execute_tool(self, tool_call):
        """Execute a tool call by publishing to appropriate topic"""
        tool_name = tool_call['name']
        params = tool_call['params']
        topic = tool_call['topic']
        
        self.get_logger().info(f'Executing: {tool_name}({params})')
        
        # Construct command based on tool
        if tool_name == 'roll':
            command = params  # dice notation
        elif tool_name == 'roll_stats':
            command = 'stats'
        elif tool_name == 'coin_flip':
            command = f'coin flip {params}'
        elif tool_name == 'roll_history':
            command = f'history {params}'
        else:
            self.get_logger().warn(f'Unknown tool: {tool_name}')
            return
        
        # Publish command to appropriate topic
        if '/dice/command' in topic:
            cmd_msg = String()
            cmd_msg.data = command
            self.dice_command_pub.publish(cmd_msg)
            self.get_logger().info(f'→ Sent to dice service: "{command}"')
    
    def dice_result_callback(self, msg):
        """Log dice results"""
        self.get_logger().info(f'Dice result: {msg.data[:100]}...')


def main(args=None):
    rclpy.init(args=args)
    
    tool_manager = ToolManagerNode()
    
    try:
        rclpy.spin(tool_manager)
    except KeyboardInterrupt:
        tool_manager.get_logger().info('Shutting down tool manager...')
    finally:
        tool_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
