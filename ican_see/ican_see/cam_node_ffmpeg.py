#!/usr/bin/env python3
"""
FFmpeg Camera Node - Robust Camera Capture
Uses FFmpeg subprocess for Insta360 X5
Handles dropped frames gracefully without crashing
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess
import numpy as np
import cv2
import threading
import time


class FFmpegCameraNode(Node):
    def __init__(self):
        super().__init__('ffmpeg_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 30)
        self.declare_parameter('publish_rate', 2.0)  # Publish at 2Hz for vision queries
        self.declare_parameter('input_format', 'v4l2')  # v4l2 for Linux
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.input_format = self.get_parameter('input_format').value
        
        # Publisher
        self.pub_image = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # FFmpeg process
        self.process = None
        self.running = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.frame_count = 0
        
        # Start FFmpeg capture thread
        self.init_ffmpeg()
        
        # Timer for publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"FFmpeg Camera Node initialized on device {self.device_id}")
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz (2 images/sec for vision queries)")
    
    def init_ffmpeg(self):
        """Initialize FFmpeg capture subprocess"""
        try:
            device_path = f'/dev/video{self.device_id}'
            
            # FFmpeg command to capture frames with error resilience
            cmd = [
                'ffmpeg',
                '-f', self.input_format,
                '-input_format', 'mjpeg',  # Many USB cameras use MJPEG
                '-video_size', f'{self.width}x{self.height}',
                '-framerate', str(self.fps),
                '-i', device_path,
                '-err_detect', 'ignore_err',  # Ignore decode errors from corrupted MJPEG
                '-fflags', '+genpts',  # Generate missing timestamps
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',  # OpenCV format
                '-vsync', '0',  # Don't drop/duplicate frames
                '-'
            ]
            
            self.get_logger().info(f"Starting FFmpeg: {' '.join(cmd)}")
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.width * self.height * 3
            )
            
            self.running = True
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            
            # Start stderr reader thread (for debugging)
            self.stderr_thread = threading.Thread(target=self.read_stderr, daemon=True)
            self.stderr_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Failed to start FFmpeg: {e}")
    
    def read_stderr(self):
        """Read FFmpeg stderr for debugging"""
        if self.process is None:
            return
        
        try:
            for line in iter(self.process.stderr.readline, b''):
                if not self.running:
                    break
                line_str = line.decode('utf-8', errors='ignore').strip()
                if line_str and 'frame=' not in line_str.lower():
                    self.get_logger().debug(f"FFmpeg: {line_str}")
        except Exception as e:
            self.get_logger().error(f"Error reading FFmpeg stderr: {e}")
    
    def capture_loop(self):
        """Background thread to continuously capture frames from FFmpeg"""
        frame_size = self.width * self.height * 3
        consecutive_errors = 0
        max_consecutive_errors = 100  # Allow many errors before restart
        
        while self.running:
            try:
                if self.process is None or self.process.poll() is not None:
                    self.get_logger().error("FFmpeg process died, attempting restart...")
                    time.sleep(1)
                    self.init_ffmpeg()
                    break
                
                # Read one frame
                raw_frame = self.process.stdout.read(frame_size)
                
                if len(raw_frame) == 0:
                    # EOF - process probably died
                    self.get_logger().error("EOF from FFmpeg, restarting...")
                    time.sleep(1)
                    self.init_ffmpeg()
                    break
                
                if len(raw_frame) != frame_size:
                    consecutive_errors += 1
                    # Don't warn on every corrupted frame - these are common with Insta360
                    if consecutive_errors % 10 == 0:
                        self.get_logger().debug(f"Skipping corrupted frames ({consecutive_errors} total)")
                    
                    if consecutive_errors > max_consecutive_errors:
                        self.get_logger().warn(f"Too many corrupted frames ({consecutive_errors}), restarting FFmpeg...")
                        self.init_ffmpeg()
                        break
                    continue
                
                # Reset error counter on successful read
                consecutive_errors = 0
                
                # Convert to numpy array
                frame = np.frombuffer(raw_frame, dtype=np.uint8)
                frame = frame.reshape((self.height, self.width, 3))
                
                # Update latest frame
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                    
            except Exception as e:
                if self.running:
                    consecutive_errors += 1
                    if consecutive_errors % 10 == 0:
                        self.get_logger().error(f"Frame capture error: {e}")
                    time.sleep(0.1)
    
    def timer_callback(self):
        """Publish the latest captured frame"""
        with self.frame_lock:
            if self.latest_frame is None:
                self.get_logger().warn("No frame available yet")
                return
            
            frame = self.latest_frame.copy()
        
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            self.pub_image.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 50 == 0:
                self.get_logger().info(f"Published frame {self.frame_count}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to publish frame: {e}")
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FFmpegCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
