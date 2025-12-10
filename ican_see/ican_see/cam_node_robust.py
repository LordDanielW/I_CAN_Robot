#!/usr/bin/env python3
"""
Robust Camera Node - Based on Working MediaMTX Configuration
Uses ffmpeg with the exact parameters that work for Insta360 X5
Handles corrupted MJPEG frames gracefully
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


class RobustCameraNode(Node):
    def __init__(self):
        super().__init__('robust_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 2880)  # Native Insta360 resolution
        self.declare_parameter('height', 1440)
        self.declare_parameter('output_width', 1920)  # Scaled output
        self.declare_parameter('output_height', 960)
        self.declare_parameter('fps', 10)
        self.declare_parameter('publish_rate', 2.0)  # Publish at 2Hz for vision queries
        
        # Get parameters
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.output_width = self.get_parameter('output_width').value
        self.output_height = self.get_parameter('output_height').value
        self.fps = self.get_parameter('fps').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
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
        
        self.get_logger().info(f"Robust Camera Node initialized on /dev/video{self.device_id}")
        self.get_logger().info(f"Capture: {self.width}x{self.height} @ {self.fps}fps")
        self.get_logger().info(f"Output: {self.output_width}x{self.output_height} @ {self.publish_rate}Hz (2 images/sec)")
    
    def init_ffmpeg(self):
        """Initialize FFmpeg capture subprocess using working MediaMTX parameters"""
        try:
            device_path = f'/dev/video{self.device_id}'
            
            # FFmpeg command based on working MediaMTX configuration
            # Uses exact flags that handle corrupted MJPEG from Insta360
            cmd = [
                'ffmpeg',
                '-fflags', 'nobuffer',
                '-flags', 'low_delay',
                '-probesize', '32',
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-thread_queue_size', '512',
                '-video_size', f'{self.width}x{self.height}',
                '-framerate', str(self.fps),
                '-use_wallclock_as_timestamps', '1',
                '-i', device_path,
                '-vf', f'fps={self.fps},scale={self.output_width}:{self.output_height}',
                '-f', 'rawvideo',
                '-pix_fmt', 'bgr24',
                '-'
            ]
            
            self.get_logger().info(f"Starting FFmpeg with working MediaMTX params")
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.output_width * self.output_height * 3
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
                # Only log important messages, not frame stats
                if line_str and 'frame=' not in line_str.lower() and 'fps=' not in line_str.lower():
                    if 'error' in line_str.lower() or 'warning' in line_str.lower():
                        self.get_logger().warn(f"FFmpeg: {line_str}")
                    else:
                        self.get_logger().debug(f"FFmpeg: {line_str}")
        except Exception as e:
            self.get_logger().error(f"Error reading FFmpeg stderr: {e}")
    
    def capture_loop(self):
        """Background thread to continuously capture frames from FFmpeg"""
        frame_size = self.output_width * self.output_height * 3
        consecutive_errors = 0
        max_consecutive_errors = 50
        good_frame_count = 0
        
        self.get_logger().info(f"Capture loop started, expecting {frame_size} bytes per frame")
        
        while self.running:
            try:
                if self.process is None or self.process.poll() is not None:
                    self.get_logger().error("FFmpeg process died, attempting restart...")
                    time.sleep(2)
                    consecutive_errors = 0
                    self.init_ffmpeg()
                    break
                
                # Read one frame
                raw_frame = self.process.stdout.read(frame_size)
                
                if len(raw_frame) == 0:
                    # EOF - process probably died
                    self.get_logger().error("EOF from FFmpeg, restarting...")
                    time.sleep(2)
                    consecutive_errors = 0
                    self.init_ffmpeg()
                    break
                
                if len(raw_frame) != frame_size:
                    consecutive_errors += 1
                    # Only log occasionally - corrupted frames are expected with Insta360
                    if consecutive_errors == 1 or consecutive_errors % 50 == 0:
                        self.get_logger().debug(
                            f"Incomplete frame: got {len(raw_frame)}/{frame_size} bytes "
                            f"({consecutive_errors} errors)"
                        )
                    
                    if consecutive_errors > max_consecutive_errors:
                        self.get_logger().error(
                            f"Too many consecutive errors ({consecutive_errors}), restarting FFmpeg..."
                        )
                        self.init_ffmpeg()
                        consecutive_errors = 0
                        break
                    continue
                
                # Reset error counter on successful read
                if consecutive_errors > 0:
                    self.get_logger().info(f"Recovered after {consecutive_errors} errors")
                consecutive_errors = 0
                good_frame_count += 1
                
                # Convert to numpy array
                frame = np.frombuffer(raw_frame, dtype=np.uint8)
                frame = frame.reshape((self.output_height, self.output_width, 3))
                
                # Update latest frame
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                
                # Log occasionally
                if good_frame_count % 100 == 0:
                    self.get_logger().info(f"Captured {good_frame_count} good frames")
                    
            except Exception as e:
                if self.running:
                    consecutive_errors += 1
                    if consecutive_errors % 10 == 0:
                        self.get_logger().error(f"Frame capture error: {e}")
                    
                    if consecutive_errors > max_consecutive_errors:
                        self.get_logger().error("Too many errors, restarting FFmpeg...")
                        self.init_ffmpeg()
                        consecutive_errors = 0
                        break
                    
                    time.sleep(0.1)
    
    def timer_callback(self):
        """Publish the latest captured frame"""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            
            frame = self.latest_frame.copy()
        
        try:
            # Convert to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            
            self.pub_image.publish(msg)
            
            self.frame_count += 1
            if self.frame_count % 100 == 0:
                self.get_logger().info(f"Published {self.frame_count} frames")
                
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
    node = RobustCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
