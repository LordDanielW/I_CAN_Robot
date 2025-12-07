# ican_see - YOLO Vision for I_CAN Robot

Object detection using YOLOv8 (stable) or YOLOv13 (latest) with ROS2 integration.

## Quick Start

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select ican_see --symlink-install
source install/setup.bash

# Launch with camera (uses YOLOv8 by default)
ros2 launch ican_bringup local_nodes.launch.py

# Or run standalone
ros2 run ican_see yolo_server_node
```

**To use YOLOv13 instead:** Edit `local_nodes.launch.py` and change `model: 'yolov8n.pt'` to `model: 'yolov13n.pt'`

## Supported Versions

### YOLOv8 (Recommended - Stable)
- **Provider:** Ultralytics (official)
- **Auto-download:** Models download automatically on first use
- **Stability:** Production-ready, well-tested
- **Models:** yolov8n/s/m/l/x.pt

### YOLOv13 (Optional - Latest)
- **Provider:** iMoonLab/Tsinghua University (June 2025)
- **Manual setup:** Requires cloning repository and downloading models
- **Features:** 
  - **HyperACE** - Hypergraph-based Adaptive Correlation Enhancement
  - **FullPAD** - Full-Pipeline Aggregation-and-Distribution
- **Performance:** +3.0% mAP over YOLO11-N
- **Models:** yolov13n/s/m/l/x.pt
- **Source:** https://github.com/iMoonLab/yolov13

## Installation

### Basic Dependencies (Required)
```bash
# Install Python packages (included in Robot_Remote_Install.sh)
/usr/bin/python3 -m pip install --break-system-packages ultralytics opencv-python torch torchvision
```

### YOLOv8 Setup (Default)
No additional setup needed - models auto-download from Ultralytics on first use.

### YOLOv13 Setup (Optional)
```bash
# Clone YOLOv13 repository
cd ~
git clone https://github.com/iMoonLab/yolov13.git

# Download pre-trained models
cd yolov13
# Download from: https://github.com/iMoonLab/yolov13/releases
wget https://github.com/iMoonLab/yolov13/releases/download/v1.0/yolov13n.pt
wget https://github.com/iMoonLab/yolov13/releases/download/v1.0/yolov13s.pt
# ... other models as needed
```

### NumPy Compatibility (CRITICAL)
```bash
# Remove user-installed numpy 2.x (conflicts with cv_bridge)
rm -rf ~/.local/lib/python3.12/site-packages/numpy*

# Verify system numpy 1.26.4
python3 -c "import numpy; print('NumPy:', numpy.__version__)"
```

## Available Models

### YOLOv8 Models (Auto-Download)
| Model | Size | Speed | Accuracy | Use Case |
|-------|------|-------|----------|----------|
| `yolov8n.pt` | ~6MB | Fastest | Good | Real-time robotics |
| `yolov8s.pt` | ~22MB | Fast | Better | Balanced performance |
| `yolov8m.pt` | ~52MB | Medium | High | Offline processing |
| `yolov8l.pt` | ~87MB | Slow | Very High | Research |
| `yolov8x.pt` | ~136MB | Slowest | Best | Maximum accuracy |

### YOLOv13 Models (Manual Download)
| Model | Size | Performance | Notes |
|-------|------|-------------|-------|
| `yolov13n.pt` | ~6MB | +3.0% mAP vs YOLO11-N | Fastest |
| `yolov13s/m/l/x.pt` | Various | Incrementally better | Slower |

## Testing

### Quick Test (No ROS)
```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_see/ican_see
python3 test_yolo.py
```
Opens webcam with live YOLO detection. Press 'q' to quit.
Uses YOLOv8n by default. Edit `test_yolo.py` to change model.

### Build ROS2 Package
```bash
cd ~/ros2_ws
colcon build --packages-select ican_see --symlink-install
source install/setup.bash
```

## ROS2 Integration

### Architecture
```
Camera (cam2image) → YOLO Server → /yolo_detections
                       ↓
                  vision_msgs/Detection2DArray
                  (bounding boxes, classes, confidence)
```

**Default:** Uses YOLOv8n.pt (stable, auto-download)
**Optional:** Set `model:=yolov13n.pt` for latest features

### Launch with Camera
```bash
# Terminal 1: Start local nodes (camera + YOLO)
ros2 launch ican_bringup local_nodes.launch.py

# Terminal 2: Monitor detections
ros2 topic echo /yolo_detections
```

### Manual Launch
```bash
# Terminal 1: Camera at 5Hz
ros2 run image_tools cam2image --ros-args -p frequency:=5.0

# Terminal 2: YOLO server (YOLOv8 - stable)
ros2 run ican_see yolo_server_node --ros-args \
  -p model:=yolov8n.pt \
  -p confidence_threshold:=0.5 \
  -p device:=cpu \
  -p image_topic:=/image

# Or use YOLOv13 (requires manual setup)
ros2 run ican_see yolo_server_node --ros-args \
  -p model:=yolov13n.pt \
  -p confidence_threshold:=0.5 \
  -p device:=cpu \
  -p image_topic:=/image
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model` | string | `yolov8n.pt` | Model to use (yolov8/v13 + n/s/m/l/x) |
| `confidence_threshold` | float | `0.5` | Minimum detection confidence (0.0-1.0) |
| `device` | string | `cpu` | Use `cpu` or `cuda` for GPU |
| `image_topic` | string | `/image` | Input camera topic |

## Topics

**Subscribed:**
- `/image` - `sensor_msgs/Image` - RGB camera images

**Published:**
- `/yolo_detections` - `vision_msgs/Detection2DArray` - Detected objects:
  - Bounding box center (x, y)
  - Bounding box size (width, height)
  - Class ID (COCO dataset: 80 classes)
  - Confidence score

## COCO Dataset Classes

Both YOLOv8 and YOLOv13 are trained on COCO with 80 object classes:

**Common:** person, bicycle, car, motorcycle, airplane, bus, train, truck, traffic light, stop sign, cat, dog, bird, horse, sheep, cow, elephant, bear, zebra, giraffe

**Items:** backpack, umbrella, handbag, tie, suitcase, bottle, cup, fork, knife, spoon, bowl, banana, apple, sandwich, orange, broccoli, carrot, pizza

**Furniture:** chair, couch, bed, dining table, toilet, tv, laptop, mouse, keyboard, cell phone, book, clock

**Full list:** https://github.com/ultralytics/ultralytics/blob/main/ultralytics/cfg/datasets/coco.yaml

## GPU Acceleration

### Check CUDA
```bash
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

### Enable GPU
```bash
ros2 run ican_see yolo_server_node --ros-args -p device:=cuda
```
GPU provides 5-10x faster inference.

## Troubleshooting

### NumPy Version Conflict
**Error:** "NumPy 1.x cannot run in NumPy 2.x"

**Solution:**
```bash
rm -rf ~/.local/lib/python3.12/site-packages/numpy*
python3 -c "import numpy; print(numpy.__version__)"  # Should be 1.26.4
```
ROS2's `cv_bridge` requires numpy 1.x. System numpy 1.26.4 works with YOLO, cv_bridge, and all dependencies.

### Module Not Found
```bash
/usr/bin/python3 -c "import ultralytics; print(ultralytics.__version__)"
/usr/bin/python3 -m pip install --break-system-packages ultralytics opencv-python
```

### Camera Issues
```bash
ls -l /dev/video*  # List cameras
ros2 run image_tools cam2image --ros-args -p device_id:=0
```

### Low FPS
- Use smaller model (`yolov8n.pt` instead of `yolov8l.pt`)
- Reduce camera frequency (5Hz recommended)
- Enable GPU acceleration
- Increase confidence threshold

### Model Download
```bash
# YOLOv8 models cached in:
ls ~/.cache/ultralytics/

# YOLOv13 models (manual):
cd ~/yolov13
wget https://github.com/iMoonLab/yolov13/releases/download/v1.0/yolov13n.pt
```

## Performance Tips

1. **Start small:** Use `yolov8n.pt` for real-time (30+ FPS on CPU)
2. **Tune confidence:** Higher threshold = fewer false positives, faster
3. **Camera rate:** 5Hz sufficient for robotics (no need for 30Hz)
4. **GPU recommended:** 5-10x speedup for larger models
5. **Resolution:** Reduce camera resolution if CPU-bound
6. **Try YOLOv13:** +3% mAP improvement if you need better accuracy

## Example Usage

### Detection Monitor Node
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
        for det in msg.detections:
            if det.results:
                cls = det.results[0].hypothesis.class_id
                score = det.results[0].hypothesis.score
                x = det.bbox.center.position.x
                y = det.bbox.center.position.y
                print(f"{det.id}: {score:.2f} at ({x:.0f}, {y:.0f})")

rclpy.init()
node = DetectionMonitor()
rclpy.spin(node)
```

## Integration with Robot System

The YOLO server integrates with:
- **Camera:** Receives images from `cam2image` or other camera nodes
- **Orchestrator:** Sends detection events to AI brain for decisions
- **Voice:** Announces detected objects via TTS
- **LLM:** Describes visual scenes using Ollama

### Complete Vision Pipeline
```bash
# All-in-one launch
ros2 launch ican_bringup local_nodes.launch.py

# Or manually:
ros2 run image_tools cam2image --ros-args -p frequency:=5.0
ros2 run ican_see yolo_server_node
ros2 topic echo /yolo_detections
```

## Development

### Package Structure
```
ican_see/
├── ican_see/
│   ├── yolo_server_node.py  # ROS2 detection server
│   └── test_yolo.py         # Standalone test script
├── setup.py                 # Package configuration
└── README.md               # This file
```

### Files to Delete
- `YOLO_SETUP.md` - Consolidated into this README

## Resources

- **YOLOv8 (Ultralytics):** https://github.com/ultralytics/ultralytics
- **YOLOv13 Paper:** https://arxiv.org/abs/2506.17733
- **YOLOv13 GitHub:** https://github.com/iMoonLab/yolov13
- **COCO Dataset:** https://cocodataset.org/

---

**Note:** 
- YOLOv8 is the recommended stable version with auto-download
- YOLOv13 is experimental, requires manual setup, +3% accuracy improvement
