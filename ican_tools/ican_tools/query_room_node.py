#!/usr/bin/env python3
"""
Query Room Node - Vision-based room description tool

This node captures a camera frame and sends it to the VLM (via ollama_node)
to describe the room to a blind person. The VLM response goes directly to TTS.

Author: I_CAN Robot Project
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import cv2
import numpy as np


class QueryRoomNode(Node):
    """ROS 2 Node for vision-based room queries"""
    
    def __init__(self):
        super().__init__('query_room_node')
        self.get_logger().info('Query Room Node initialized')
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Latest camera frame
        self.latest_frame = None
        self.frame_available = False
        
        # Subscribe to camera feed (from camera nodes)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Camera nodes publish to /camera/image_raw
            self.image_callback,
            10
        )
        
        # Subscribe to query commands (from tool_manager)
        self.command_sub = self.create_subscription(
            String,
            '/query_room/command',
            self.command_callback,
            10
        )
        
        # Publisher for VLM vision queries (to ollama_node)
        self.vision_query_pub = self.create_publisher(
            String,
            '/llm_vision_query',
            10
        )
        
        # Publisher for status
        self.status_pub = self.create_publisher(
            String,
            '/query_room/status',
            10
        )
        
        # Publisher for results (to tool_manager for logging)
        self.result_pub = self.create_publisher(
            String,
            '/tool_result',
            10
        )
        
        self.get_logger().info('Query Room services ready:')
        self.get_logger().info('  - Subscriber: /camera/image_raw (camera)')
        self.get_logger().info('  - Subscriber: /query_room/command')
        self.get_logger().info('  - Publisher: /llm_vision_query (to ollama)')
        self.get_logger().info('  - Publisher: /query_room/status, /tool_result')
    
    def image_callback(self, msg):
        """Store latest camera frame"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv_image
            self.frame_available = True
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
    
    def command_callback(self, msg):
        """Handle query room command"""
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received query room command: {command}')
        
        result = self.process_query_command(command)
        
        # Publish result to tool_result for logging
        result_msg = String()
        result_msg.data = result
        self.result_pub.publish(result_msg)
    
    def process_query_command(self, command: str) -> str:
        """Process room query commands"""
        
        # Check if camera frame is available
        if not self.frame_available or self.latest_frame is None:
            error_msg = "❌ No camera frame available. Is cam2image running?"
            self.get_logger().error(error_msg)
            return error_msg
        
        # Publish status
        status_msg = String()
        status_msg.data = "CAPTURING_FRAME"
        self.status_pub.publish(status_msg)
        
        # Capture current frame
        frame = self.latest_frame.copy()
        
        self.get_logger().info('📸 Captured camera frame, encoding for VLM...')
        
        # Encode frame as JPEG then base64
        try:
            # Resize if too large (optional, for efficiency)
            height, width = frame.shape[:2]
            if width > 1280:
                scale = 1280 / width
                new_width = 1280
                new_height = int(height * scale)
                frame = cv2.resize(frame, (new_width, new_height))
            
            # Encode as JPEG
            success, jpeg_buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            if not success:
                error_msg = "❌ Failed to encode image as JPEG"
                self.get_logger().error(error_msg)
                return error_msg
            
            # Convert to base64
            image_b64 = base64.b64encode(jpeg_buffer.tobytes()).decode('utf-8')
            
            self.get_logger().info(f'✓ Image encoded ({len(image_b64)} bytes)')
            
        except Exception as e:
            error_msg = f"❌ Error encoding image: {e}"
            self.get_logger().error(error_msg)
            return error_msg
        
        # Construct vision prompt for VLM
        vision_prompt = self.construct_vision_prompt(command, image_b64)
        
        # Send to ollama_node via /llm_vision_query
        query_msg = String()
        query_msg.data = vision_prompt
        self.vision_query_pub.publish(query_msg)
        
        # Update status
        status_msg.data = "SENT_TO_VLM"
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('✓ Vision query sent to ollama_node')
        
        # Return confirmation (actual VLM response will go to TTS)
        return "✓ Room query sent to VLM. Description will be spoken via TTS."
    
    def construct_vision_prompt(self, command: str, image_b64: str) -> str:
        """Construct a vision prompt with image data for the VLM
        
        Format: VISION_QUERY|<prompt_text>|<base64_image>
        """
        
        # System context for the VLM
        system_context = """You are the vision system of Spot, a robotic seeing-eye guide dog.
Your purpose is to describe the environment to help a blind person navigate safely.

IMPORTANT GUIDELINES:
- Be concise but descriptive (2-4 sentences)
- Focus on spatial layout and navigation hazards
- Mention doors, furniture, obstacles, and room type
- Use clear, spoken-friendly language
- Prioritize safety-relevant information
"""
        
        # Parse command for specific requests
        if 'describe' in command or 'what' in command or 'where' in command:
            user_request = "Describe what you see in this image. What type of room is this? What objects and obstacles are present?"
        elif 'safe' in command or 'navigate' in command:
            user_request = "Analyze this image for navigation safety. What obstacles should I avoid? Is the path clear?"
        elif 'door' in command:
            user_request = "Look for doors in this image. Where are they located? Are they open or closed?"
        else:
            user_request = "Describe this room and environment for a blind person."
        
        # Combine system context and user request
        full_prompt = f"{system_context}\n\nUSER REQUEST: {user_request}"
        
        # Format as VISION_QUERY message with image
        # Format: VISION_QUERY|<text_prompt>|<base64_image>
        vision_message = f"VISION_QUERY|{full_prompt}|{image_b64}"
        
        return vision_message


def main(args=None):
    rclpy.init(args=args)
    
    query_room_node = QueryRoomNode()
    
    try:
        rclpy.spin(query_room_node)
    except KeyboardInterrupt:
        query_room_node.get_logger().info('Shutting down query room node...')
    finally:
        query_room_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
