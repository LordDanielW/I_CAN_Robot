#!/usr/bin/env python3
"""
GStreamer Camera Node - Alternative for Insta360 X5
Uses GStreamer pipeline via OpenCV for robust MJPEG decoding
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 2880)
        self.declare_parameter('height', 1440)
        self.declare_parameter('fps', 30)
        self.declare_parameter('output_width', 1920)
        self.declare_parameter('output_height', 960)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Publisher
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Video capture
        self.cap = None
        self.frame_count = 0
        self.error_count = 0
        self.max_errors = 20
        
        # Initialize camera
        self.init_camera()
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"GStreamer Camera Node initialized on /dev/video{self.device_id}")
        self.get_logger().info(f"Capture: {self.width}x{self.height} @ {self.fps}fps")
        self.get_logger().info(f"Output: {self.output_width}x{self.output_height} @ {self.publish_rate}Hz")
    
    def init_camera(self):
        """Initialize camera with GStreamer pipeline"""
        if self.cap is not None:
            self.cap.release()
        
        try:
            # GStreamer pipeline for MJPEG capture with jpegdec for robustness
            gst_pipeline = (
                f'v4l2src device=/dev/video{self.device_id} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                'jpegdec ! '  # JPEG decoder handles corrupted frames better
                'videoconvert ! '
                f'videoscale ! video/x-raw,width={self.output_width},height={self.output_height} ! '
                'videoconvert ! appsink drop=true max-buffers=2'
            )
            
            self.get_logger().info(f"GStreamer pipeline: {gst_pipeline}")
            
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.cap.isOpened():
                self.get_logger().error("Failed to open GStreamer pipeline")
                return False
            
            self.get_logger().info("GStreamer camera opened successfully")
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
            # Read frame (GStreamer handles skipping to latest frame)
            ret, frame = self.cap.read()
            
            if not ret or frame is None or frame.size == 0:
                self.error_count += 1
                
                if self.error_count % 10 == 0:
                    self.get_logger().warn(f"Frame read failed (error count: {self.error_count})")
                
                # Reinitialize if too many errors
                if self.error_count >= self.max_errors:
                    self.get_logger().error("Too many consecutive errors, reinitializing camera...")
                    self.init_camera()
                return
            
            # Reset error count on successful read
            if self.error_count > 0:
                self.get_logger().info(f"Recovered after {self.error_count} errors")
            self.error_count = 0
            self.frame_count += 1
            
            # Convert to ROS Image message
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                
                self.pub_image.publish(msg)
                
                if self.frame_count % 100 == 0:
                    self.get_logger().info(f"Published frame {self.frame_count}")
                    
            except Exception as e:
                self.get_logger().error(f"Failed to convert/publish frame: {e}")
                
        except Exception as e:
            self.error_count += 1
            
            if self.error_count % 10 == 0:
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
    node = GStreamerCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
