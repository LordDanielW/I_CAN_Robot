#!/usr/bin/env python3
"""
YOLO Server Node - Object detection using YOLOv8
Subscribes to camera images and publishes detection results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np


class YOLOServer(Node):
    def __init__(self):
        super().__init__('yolo_server')
        
        # Declare parameters
        self.declare_parameter('model', 'yolov8n.pt')  # Model size: n, s, m, l, x
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')  # 'cpu' or 'cuda'
        self.declare_parameter('image_topic', '/image')
        
        # Get parameters
        model_name = self.get_parameter('model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        image_topic = self.get_parameter('image_topic').value
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            'yolo_detections',
            10
        )
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_name} on {device}...')
        try:
            self.model = YOLO(model_name)
            if device == 'cuda':
                self.model.to('cuda')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise
        
        self.frame_count = 0
        self.get_logger().info(f'YOLO Server ready. Subscribed to {image_topic}')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            self.frame_count += 1
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = Header()
            detection_array.header.stamp = msg.header.stamp
            detection_array.header.frame_id = msg.header.frame_id
            
            # Process each detection
            for result in results:
                boxes = result.boxes
                
                for box in boxes:
                    # Get class info
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id]
                    confidence = float(box.conf[0])
                    
                    # Create Detection2D message
                    detection = Detection2D()
                    
                    # Get bounding box coordinates (xyxy format)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Set bounding box center and size
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    # Create ObjectHypothesisWithPose for class and confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(cls_id)
                    hypothesis.hypothesis.score = confidence
                    
                    detection.results.append(hypothesis)
                    
                    # Use the ID field to store the class name
                    detection.id = cls_name
                    
                    # Add detection to array
                    detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Log periodically
            if self.frame_count % 30 == 0:
                detection_summary = ', '.join([
                    f"{d.id}({d.results[0].hypothesis.score:.2f})"
                    for d in detection_array.detections if d.results
                ])
                self.get_logger().info(
                    f'Frame {self.frame_count}: {len(detection_array.detections)} detections - {detection_summary}'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
