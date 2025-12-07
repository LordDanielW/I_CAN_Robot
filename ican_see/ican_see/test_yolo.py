#!/usr/bin/env python3
"""
YOLO Test Script (No ROS)
Tests YOLOv8 or YOLOv13 object detection on webcam or test image

Supports:
- YOLOv8 (stable, default) - auto-downloads from Ultralytics
- YOLOv13 (latest) - requires manual download from iMoonLab

YOLOv13 features HyperACE and FullPAD for improved accuracy.
"""

import cv2
from ultralytics import YOLO
import time

def test_yolo_webcam():
    """Test YOLO with live webcam feed"""
    print("Testing YOLO with webcam...")
    
    # Load YOLO model (change to yolov13n.pt for YOLOv13)
    MODEL = "yolov8n.pt"  # yolov8: n/s/m/l/x, yolov13: n/s/m/l/x
    
    try:
        version = 'YOLOv13' if 'v13' in MODEL else 'YOLOv8'
        print(f"\nLoading {version} model: {MODEL}...")
        model = YOLO(MODEL)
        print(f"{version} model loaded successfully!")
        
        # Open webcam
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("✗ Could not open webcam")
            print("Trying test image instead...")
            return test_yolo_image(model)
        
        print("\n" + "=" * 60)
        print("🎥 YOLO LIVE DETECTION (Press 'q' to quit)")
        print("=" * 60 + "\n")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Run YOLO inference
            results = model(frame, verbose=False)
            
            # Draw results on frame
            annotated_frame = results[0].plot()
            
            # Calculate FPS
            frame_count += 1
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"FPS: {fps:.1f} | Detections: {len(results[0].boxes)}")
            
            # Display
            cv2.imshow('YOLO Detection', annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        print("\n✓ YOLO webcam test completed!")
        return True
        
    except Exception as e:
        print(f"\n✗ YOLO test failed: {e}")
        print("\nTroubleshooting:")
        print("  - Install ultralytics: pip install ultralytics")
        print("  - Install opencv: pip install opencv-python")
        print("  - YOLOv8 models auto-download from Ultralytics")
        print("  - For YOLOv13: git clone https://github.com/iMoonLab/yolov13.git")
        print("  - YOLOv13 models: https://github.com/iMoonLab/yolov13/releases")
        return False

def test_yolo_image(model=None):
    """Test YOLO on a test image"""
    print("\nTesting YOLO with test image...")
    
    try:
        if model is None:
            MODEL = "yolov8n.pt"
            version = 'YOLOv8'
            print(f"Loading {version} model: {MODEL}...")
            model = YOLO(MODEL)
        
        # Create a test image or use a URL
        test_url = "https://ultralytics.com/images/bus.jpg"
        
        print(f"Running inference on test image from: {test_url}")
        results = model(test_url)
        
        # Print detections
        print("\n" + "=" * 60)
        print("DETECTIONS:")
        print("=" * 60)
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls]
                print(f"  - {name}: {conf:.2f}")
        
        print("=" * 60)
        
        # Show result
        annotated = results[0].plot()
        cv2.imshow('YOLO Test Image', annotated)
        print("\nDisplaying result. Press any key to close...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        print("\n✓ YOLO image test successful!")
        return True
        
    except Exception as e:
        print(f"\n✗ YOLO test failed: {e}")
        return False

def print_yolo_info():
    """Print information about available YOLO models"""
    print("\nAvailable YOLOv8 Models (Stable, Auto-Download):")
    print("  yolov8n.pt - Nano (fastest, ~6MB)")
    print("  yolov8s.pt - Small (~22MB)")
    print("  yolov8m.pt - Medium (~52MB)")
    print("  yolov8l.pt - Large (~87MB)")
    print("  yolov8x.pt - Extra Large (slowest, most accurate, ~136MB)")
    print("\nAvailable YOLOv13 Models (Latest, Manual Download):")
    print("  yolov13n.pt - Nano (+3.0% mAP over YOLO11-N)")
    print("  yolov13s/m/l/x.pt - Small/Medium/Large/Extra Large")
    print("  Features: HyperACE, FullPAD (June 2025, iMoonLab/Tsinghua)")
    print("  Setup: git clone https://github.com/iMoonLab/yolov13.git")
    print("  Models: https://github.com/iMoonLab/yolov13/releases")

if __name__ == '__main__':
    print("=" * 60)
    print("YOLO Object Detection Test Script")
    print("=" * 60)
    print_yolo_info()
    print()
    
    test_yolo_webcam()
