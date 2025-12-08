# Quick Start Guide: New Tools

## What Was Added

### 1. **move_robot_node** - Robot Movement Tool
- Preprogrammed movement sequences
- Currently simulates movement to bathroom through door
- Will be replaced with real robot control later

### 2. **query_room_node** - Vision-Based Room Description
- Captures camera frame
- Sends to VLM (Ollama) for analysis
- Speaks description via TTS (bypasses tool_manager)

### 3. **Enhanced ollama_node** - Dual-Path Processing
- **Path 1**: Text prompts → tool_manager (existing)
- **Path 2**: Vision queries → TTS (new!)
- Single Ollama instance handles both

## Quick Build & Test

```bash
# 1. Build the updated packages
cd ~/ros2_ws
colcon build --packages-select ican_tools ican_brain
source install/setup.bash

# 2. Launch the system (3 terminals)

# Terminal 1: Brain nodes
ros2 launch ican_brain ican_brain_remote.launch.py

# Terminal 2: Tool nodes
ros2 launch ican_tools ican_tools_remote.launch.py

# Terminal 3: Local nodes (camera, TTS, etc)
ros2 launch ican_bringup local_nodes.launch.py

# 3. Test individual tools

# Test movement
ros2 topic pub --once /move_robot/command std_msgs/String "data: 'bathroom'"

# Test vision (requires camera)
ros2 topic pub --once /query_room/command std_msgs/String "data: 'describe room'"

# 4. Interactive tester
python3 /home/fire/ros2_ws/src/I_CAN_Robot/test_new_tools.py
```

## Voice Commands (through LLM)

Once the system is running, you can say:

### Movement Commands
- "Hey Spot, go to the bathroom"
- "Hey Spot, move through the door"
- "Hey Spot, stop moving"

### Vision Commands
- "Hey Spot, what room am I in?"
- "Hey Spot, is it safe to walk here?"
- "Hey Spot, where is the door?"

## Topic Architecture

```
PATH 1 (Text):
  Mic → prompt_node → ollama_node → tool_manager → tools

PATH 2 (Vision):
  Camera → query_room → ollama_node → TTS → Speaker
```

## Key Files Modified

1. `ican_tools/ican_tools/move_robot_node.py` (NEW)
2. `ican_tools/ican_tools/query_room_node.py` (NEW)
3. `ican_brain/ican_brain/ollama_node.py` (ENHANCED)
4. `ican_tools/ican_tools/tool_manager_node.py` (UPDATED)
5. `ican_tools/setup.py` (UPDATED)
6. `ican_tools/launch/ican_tools_remote.launch.py` (UPDATED)
7. `ican_brain/launch/ican_brain_remote.launch.py` (UPDATED)

## Monitoring

```bash
# See all messages
ros2 run ican_brain debug_monitor

# Watch specific topics
ros2 topic echo /llm_vision_query
ros2 topic echo /move_robot/status
ros2 topic echo /query_room/status
ros2 topic echo /tts/speak
```

## Troubleshooting

**Vision not working?**
- Check camera: `ros2 topic echo /image`
- Check Ollama VLM: `ollama list | grep qwen3-vl`
- Install dependencies: `pip install opencv-python`

**Movement not working?**
- Check move_robot_node: `ros2 node list | grep move_robot`
- Check topic: `ros2 topic info /move_robot/command`

**Tool calls not detected?**
- Use debug_monitor to see LLM output
- Check tool_manager logs for regex matches
- Verify LLM includes "TOOL:" prefix

## Architecture Highlights

✅ **Single Ollama Instance**: Efficient resource usage  
✅ **Dual Path Processing**: Text and vision independent  
✅ **Direct TTS Route**: Vision responses skip tool_manager  
✅ **Separate Thread Locks**: No blocking between paths  
✅ **Extensible**: Easy to add more tools

---

See `DUAL_PATH_ARCHITECTURE.md` for complete technical details.
