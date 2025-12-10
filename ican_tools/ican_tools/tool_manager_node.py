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
            # Movement tools
            'move_to_bathroom': {
                'topic': '/move_robot/command',
                'description': 'move_to_bathroom() - Move robot through door to bathroom using go2_simple_nav',
                'pattern': r'TOOL:move_to_bathroom\(\)',
                'status': 'unknown'
            },
            'move_to_door': {
                'topic': '/move_robot/command',
                'description': 'move_to_door() - Move robot to the door using go2_simple_nav',
                'pattern': r'TOOL:move_to_door\(\)',
                'status': 'unknown'
            },
            # Vision tools
            'describe_room': {
                'topic': '/query_room/command',
                'description': 'describe_room() - Use camera to describe the current room to a blind person',
                'pattern': r'TOOL:describe_room\(\)',
                'status': 'unknown'
            },
            'check_safety': {
                'topic': '/query_room/command',
                'description': 'check_safety() - Analyze camera view for navigation hazards and obstacles',
                'pattern': r'TOOL:check_safety\(\)',
                'status': 'unknown'
            },
            'find_door': {
                'topic': '/query_room/command',
                'description': 'find_door() - Look for doors in the camera view',
                'pattern': r'TOOL:find_door\(\)',
                'status': 'unknown'
            }
        }
        
        # Publishers for tool commands
        self.move_command_pub = self.create_publisher(String, '/move_robot/command', 10)
        self.query_room_pub = self.create_publisher(String, '/query_room/command', 10)
        
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
        
        # Subscriber to check move robot status
        self.move_status_sub = self.create_subscription(
            String,
            '/move_robot/status',
            self.move_status_callback,
            10
        )
        
        # Subscriber to check query room status
        self.query_status_sub = self.create_subscription(
            String,
            '/query_room/status',
            self.query_status_callback,
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
        # For now, assume tools are available if we're running
        # In the future, this could ping service nodes
        for tool_name in self.tools:
            topic = self.tools[tool_name]['topic']
            if '/move_robot/command' in topic:
                self.tools[tool_name]['status'] = 'available'
            elif '/query_room/command' in topic:
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
        if tool_name == 'move_to_bathroom':
            command = 'bathroom'
        elif tool_name == 'move_to_door':
            command = 'door'
        # Vision tools
        elif tool_name == 'describe_room':
            command = 'describe room'
        elif tool_name == 'check_safety':
            command = 'check safety'
        elif tool_name == 'find_door':
            command = 'find door'
        else:
            self.get_logger().warn(f'Unknown tool: {tool_name}')
            return
        
        # Publish command to appropriate topic
        cmd_msg = String()
        cmd_msg.data = command
        
        if '/move_robot/command' in topic:
            self.move_command_pub.publish(cmd_msg)
            self.get_logger().info(f'→ Sent to move robot: "{command}"')
        elif '/query_room/command' in topic:
            self.query_room_pub.publish(cmd_msg)
            self.get_logger().info(f'→ Sent to query room: "{command}"')
    
    def move_status_callback(self, msg):
        """Log movement status"""
        self.get_logger().info(f'Movement status: {msg.data}')
    
    def query_status_callback(self, msg):
        """Log query room status"""
        self.get_logger().info(f'Query room status: {msg.data}')


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
