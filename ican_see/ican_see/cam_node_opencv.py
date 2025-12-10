#!/usr/bin/env python3
"""
OpenCV Camera Node - Robust Camera Capture
Uses OpenCV VideoCapture with error recovery for Insta360 X5
Handles dropped frames gracefully without crashing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class OpenCVCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 5.0)  # Publish at 5Hz
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('skip_frames', 6)  # Skip 5 frames, publish every 6th
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.skip_frames = self.get_parameter('skip_frames').value
        
        # Publisher
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Video capture
        self.cap = None
        self.frame_count = 0
        self.error_count = 0
        self.max_errors = 10  # Max consecutive errors before reinit
        
        # Initialize camera
        self.init_camera()
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"OpenCV Camera Node initialized on device {self.device_id}")
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz (capturing at {self.fps} fps)")
    
    def init_camera(self):
        """Initialize or reinitialize the camera"""
        if self.cap is not None:
            self.cap.release()
        
        try:
            # Use V4L2 backend explicitly for better error handling
            self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera device {self.device_id}")
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer lag
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Use MJPEG explicitly
            
            # Verify settings
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f"Camera opened: {actual_width}x{actual_height} @ {actual_fps} fps")
            
            self.error_count = 0
            return True
            
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
            return False
    
    def timer_callback(self):
        """Capture and publish frames"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().warn("Camera not opened, attempting to reinitialize...")
            self.init_camera()
            return
        
        try:
            # Skip frames to reduce processing load
            for _ in range(self.skip_frames - 1):
                ret = self.cap.grab()
                if not ret:
                    break
            
            # Read the final frame (try multiple times for corrupted MJPEG)
            ret, frame = False, None
            for attempt in range(3):
                ret, frame = self.cap.read()
                if ret and frame is not None and frame.size > 0:
                    break
                if attempt < 2:
                    self.get_logger().debug(f"Frame read retry {attempt + 1}")
            
            if not ret or frame is None or frame.size == 0:
                self.error_count += 1
                self.get_logger().warn(f"Frame read failed (error count: {self.error_count})")
                
                # Reinitialize if too many errors
                if self.error_count >= self.max_errors:
                    self.get_logger().error("Too many consecutive errors, reinitializing camera...")
                    self.init_camera()
                return
            
            # Reset error count on successful read
            self.error_count = 0
            self.frame_count += 1
            
            # Convert to ROS Image message
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                
                self.pub_image.publish(msg)
                
                if self.frame_count % 50 == 0:
                    self.get_logger().info(f"Published frame {self.frame_count}")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to convert/publish frame: {e}")
                
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Frame capture exception: {e}")
            
            if self.error_count >= self.max_errors:
                self.get_logger().error("Too many exceptions, reinitializing camera...")
                self.init_camera()
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
