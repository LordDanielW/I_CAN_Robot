# YOLO Vision Setup for I_CAN Robot

## Overview
The `ican_see` package provides YOLOv8-based object detection for the I_CAN Robot. It processes camera images and publishes detection results in ROS2.

**Note:** YOLOv13 doesn't exist yet. This implementation uses YOLOv8, which is the latest stable version from Ultralytics.

## Installation

### System Dependencies
```bash
sudo apt install -y python3-opencv
```

### Python Packages
```bash
# Install ultralytics (YOLO) and opencv in system Python
/usr/bin/python3 -m pip install --break-system-packages ultralytics opencv-python

# CRITICAL: If you get numpy version errors with cv_bridge:
# Remove user-installed numpy 2.x to use system numpy 1.26.4
rm -rf ~/.local/lib/python3.12/site-packages/numpy*

# Verify numpy version (should be 1.26.4 for ROS2 compatibility)
python3 -c "import numpy; print('NumPy:', numpy.__version__)"
```

This will install:
- `ultralytics` (YOLOv8 framework)
- `opencv-python` (image processing)
- `torch` and `torchvision` (deep learning backend)
- All required dependencies

## Testing YOLO

### Test Without ROS
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_see/ican_see
python3 test_yolo.py
```

This will:
1. Download the YOLOv8 nano model (~6MB) on first run
2. Open your webcam
3. Run real-time object detection
4. Display results with bounding boxes and labels
5. Press 'q' to quit

### Available Models
- `yolov8n.pt` - Nano (fastest, ~6MB)
- `yolov8s.pt` - Small (~22MB)
- `yolov8m.pt` - Medium (~52MB)
- `yolov8l.pt` - Large (~87MB)
- `yolov8x.pt` - Extra Large (slowest, most accurate, ~136MB)

Models are automatically downloaded to `~/.cache/ultralytics/` on first use.

## ROS2 Integration

### Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ican_see --symlink-install
source install/setup.bash
```

### Run YOLO Server Node

#### With Camera (cam2image)
```bash
# Terminal 1: Start camera
ros2 run image_tools cam2image --ros-args -p frequency:=5.0

# Terminal 2: Start YOLO server
ros2 run ican_see yolo_server_node --ros-args -p image_topic:=/image
```

#### With Launch File
```bash
ros2 launch ican_bringup local_nodes.launch.py
```

### Node Parameters

The `yolo_server_node` accepts these parameters:

```bash
ros2 run ican_see yolo_server_node --ros-args \
  -p model:=yolov8n.pt \
  -p confidence_threshold:=0.5 \
  -p device:=cpu \
  -p image_topic:=/image
```

Parameters:
- `model` - Which YOLO model to use (default: `yolov8n.pt`)
- `confidence_threshold` - Minimum detection confidence 0.0-1.0 (default: `0.5`)
- `device` - Use `cpu` or `cuda` for GPU acceleration (default: `cpu`)
- `image_topic` - Input image topic (default: `/image`)

### Topics

**Subscribed:**
- `/image` (or custom topic) - `sensor_msgs/Image` - RGB camera images

**Published:**
- `/yolo_detections` - `vision_msgs/Detection2DArray` - Detected objects with:
  - Bounding box center (x, y)
  - Bounding box size (width, height)
  - Class ID (COCO dataset labels)
  - Confidence score

### Monitor Detections
```bash
# View detection messages
ros2 topic echo /yolo_detections

# Check detection rate
ros2 topic hz /yolo_detections

# List all topics
ros2 topic list
```

## COCO Classes
YOLOv8 is trained on the COCO dataset with 80 classes including:

**Common objects:**
- person, bicycle, car, motorcycle, airplane, bus, train, truck
- traffic light, fire hydrant, stop sign, parking meter, bench
- bird, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe
- backpack, umbrella, handbag, tie, suitcase
- frisbee, skis, snowboard, sports ball, kite, baseball bat
- bottle, wine glass, cup, fork, knife, spoon, bowl
- banana, apple, sandwich, orange, broccoli, carrot, hot dog, pizza
- chair, couch, potted plant, bed, dining table, toilet, tv, laptop
- mouse, remote, keyboard, cell phone, microwave, oven, toaster
- sink, refrigerator, book, clock, vase, scissors, teddy bear
- hair drier, toothbrush

Full list: https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml

## GPU Acceleration

### Check CUDA Availability
```bash
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

### Use GPU
```bash
ros2 run ican_see yolo_server_node --ros-args -p device:=cuda
```

GPU acceleration can provide 5-10x faster inference depending on your hardware.

## Troubleshooting

### NumPy Version Conflict
If you see errors about "NumPy 1.x cannot run in NumPy 2.x":
```bash
# Remove user-installed numpy 2.x
rm -rf ~/.local/lib/python3.12/site-packages/numpy*

# Verify you're using system numpy 1.26.4
python3 -c "import numpy; print(numpy.__version__)"  # Should show 1.26.4

# Test all modules still work
python3 -c "from ultralytics import YOLO; from cv_bridge import CvBridge; print('OK')"
```

The issue occurs because:
- ROS2's `cv_bridge` is compiled against numpy 1.x
- Installing ultralytics with `--user` installs numpy 2.x
- System numpy 1.26.4 works with everything (YOLO, cv_bridge, whisper, ollama)

### Module Not Found
```bash
# Verify ultralytics is installed
/usr/bin/python3 -c "import ultralytics; print(ultralytics.__version__)"

# If not installed
/usr/bin/python3 -m pip install --break-system-packages ultralytics opencv-python
```

### Camera Not Working
```bash
# List video devices
ls -l /dev/video*

# Test camera with cam2image
ros2 run image_tools cam2image --ros-args -p device_id:=0
```

### Low FPS
- Use a smaller model (`yolov8n.pt` instead of `yolov8l.pt`)
- Reduce camera frequency
- Use GPU acceleration if available
- Increase confidence threshold to reduce detections

### Model Download Issues
```bash
# Models are cached in:
ls ~/.cache/ultralytics/

# Manually download if needed:
cd ~/.cache/ultralytics/
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

## Performance Tips

1. **Choose the right model**: Start with `yolov8n.pt` for real-time performance
2. **Adjust confidence**: Higher threshold = fewer false positives, faster processing
3. **Reduce camera rate**: 5 Hz is usually sufficient for robotics
4. **Use GPU**: Can provide 5-10x speedup
5. **Limit processing**: Add logic to skip frames if needed

## Example Usage

### Simple Detection Monitor
```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class DetectionMonitor(Node):
    def __init__(self):
        super().__init__('detection_monitor')
        self.sub = self.create_subscription(
            Detection2DArray,
            'yolo_detections',
            self.callback,
            10
        )
    
    def callback(self, msg):
        print(f"Detected {len(msg.detections)} objects:")
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                x = detection.bbox.center.position.x
                y = detection.bbox.center.position.y
                print(f"  Class {class_id}: {score:.2f} at ({x:.0f}, {y:.0f})")

rclpy.init()
node = DetectionMonitor()
rclpy.spin(node)
```

## Integration with Robot System

The YOLO server node is designed to work with:
- **Camera**: Receives images from `cam2image` or other camera nodes
- **Orchestrator**: Can send detection events to the brain for decision making
- **Voice**: Can announce detected objects via TTS
- **LLM**: Can describe what the robot sees using Ollama

### Complete Vision Pipeline
```bash
# Terminal 1: Camera
ros2 run image_tools cam2image --ros-args -p frequency:=5.0

# Terminal 2: YOLO Detection
ros2 run ican_see yolo_server_node

# Terminal 3: Monitor detections
ros2 topic echo /yolo_detections
```
