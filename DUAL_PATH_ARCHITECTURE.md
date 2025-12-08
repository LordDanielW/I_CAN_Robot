# Dual-Path Architecture: Text and Vision Processing

## Overview

The I_CAN Robot now supports **two independent processing paths** through a single Ollama VLM instance:

### Path 1: Text Processing (Voice Commands)
```
Whisper/Vosk → prompt_node → ollama_node → tool_manager → [tool execution]
```

### Path 2: Vision Processing (Room Description)
```
cam2image → query_room_node → ollama_node → tts_node → Audio Output
```

## Key Design Decision: Single Ollama Instance with Dual Topics

**Solution**: One `ollama_node` handles both paths using separate input/output topics.

### Why This Approach?

1. **Resource Efficiency**: Only one VLM instance runs in memory
2. **Clean Separation**: Different topics keep paths logically separate
3. **Independent Locking**: Separate threading locks prevent interference
4. **Flexible Routing**: Responses go to different destinations based on input type

## Architecture Details

### ollama_node.py - Dual-Path Handler

**Input Topics:**
- `/llm_prompt` - Text prompts from prompt_node (Path 1)
- `/llm_vision_query` - Vision queries from query_room_node (Path 2)

**Output Topics:**
- `/llm_response` - Text responses to tool_manager (Path 1)
- `/tts/speak` - Vision descriptions directly to TTS (Path 2)

**Threading:**
- `text_lock` - Prevents concurrent text processing
- `vision_lock` - Prevents concurrent vision processing
- Both can run simultaneously without blocking each other

**Models:**
- `model`: `qwen3-vl:8b`

### New Tools

#### 1. move_robot_node.py

**Purpose**: Preprogrammed robot movement sequences

**Features:**
- Simulated movement to bathroom through door
- Door navigation
- Emergency stop
- Status reporting

**Topics:**
- Subscribes: `/move_robot/command`
- Publishes: `/move_robot/status`, `/tool_result`

**Tool Functions:**
- `move_to_bathroom()` - Full bathroom navigation sequence
- `move_through_door()` - Door passage only
- `stop_robot()` - Emergency halt

#### 2. query_room_node.py

**Purpose**: Vision-based room description for blind users

**Features:**
- Captures single frame from camera
- Encodes as base64 JPEG
- Sends to VLM with specialized prompt
- Response goes directly to TTS

**Topics:**
- Subscribes: `/image` (from cam2image), `/query_room/command`
- Publishes: `/llm_vision_query` (to ollama), `/query_room/status`, `/tool_result`

**Tool Functions:**
- `describe_room()` - General room description
- `check_safety()` - Navigation hazard analysis
- `find_door()` - Door location detection

**Vision Query Format:**
```
VISION_QUERY|<prompt_text>|<base64_image>
```

### Updated tool_manager_node.py

**New Tool Registrations:**

```python
# Movement tools
'move_to_bathroom': moves robot to bathroom
'move_through_door': navigates through doorway
'stop_robot': emergency stop

# Vision tools
'describe_room': VLM room description
'check_safety': obstacle detection
'find_door': door finding
```

**New Publishers:**
- `/move_robot/command` - Movement commands
- `/query_room/command` - Vision queries

## Usage Examples

### Example 1: Voice Command for Movement

**User says:** "Hey Spot, go to the bathroom"

**Flow:**
1. Whisper → prompt_node detects wake word
2. prompt_node constructs prompt with tool list
3. ollama_node (text path) generates: "TOOL:move_to_bathroom()"
4. tool_manager extracts tool call
5. Publishes "bathroom" to `/move_robot/command`
6. move_robot_node executes sequence
7. Publishes status and result

### Example 2: Voice Command for Vision

**User says:** "Hey Spot, what room am I in?"

**Flow:**
1. Whisper → prompt_node detects wake word
2. prompt_node constructs prompt with tool list
3. ollama_node (text path) generates: "TOOL:describe_room()"
4. tool_manager extracts tool call
5. Publishes "describe room" to `/query_room/command`
6. query_room_node:
   - Captures camera frame
   - Encodes as base64
   - Publishes vision query to `/llm_vision_query`
7. ollama_node (vision path):
   - Receives vision query
   - Calls VLM with image
   - Publishes description to `/tts/speak`
8. TTS speaks the room description

## Topic Flow Diagram

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│  Microphone  │────▶│ prompt_node  │────▶│ ollama_node  │
└──────────────┘     └──────────────┘     │  (text path) │
                                           └──────┬───────┘
                                                  │
                     ┌────────────────────────────┘
                     │
                     ▼
              ┌──────────────┐
              │tool_manager  │
              └──────┬───────┘
                     │
         ┌───────────┼───────────┐
         │           │           │
         ▼           ▼           ▼
    ┌────────┐  ┌────────┐  ┌────────┐
    │  dice  │  │  move  │  │ query  │
    │service │  │ robot  │  │  room  │
    └────────┘  └────────┘  └───┬────┘
                                 │
                                 ▼
                          ┌──────────────┐
    ┌────────────┐        │ ollama_node  │
    │ cam2image  │───────▶│ (vision path)│
    └────────────┘        └──────┬───────┘
                                 │
                                 ▼
                          ┌──────────────┐
                          │  tts_node    │
                          └──────────────┘
```

## Building and Running

### 1. Build the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select ican_tools ican_brain
source install/setup.bash
```

### 2. Launch on Remote Computer

```bash
# Terminal 1: Brain nodes (ollama, prompt)
ros2 launch ican_brain ican_brain_remote.launch.py

# Terminal 2: Tool nodes
ros2 launch ican_tools ican_tools_remote.launch.py
```

### 3. Launch on Local Computer

```bash
# Terminal 3: Local nodes (camera, audio, TTS)
ros2 launch ican_bringup local_nodes.launch.py
```

### 4. Test the System

```bash
# Test movement tool
ros2 topic pub --once /move_robot/command std_msgs/String "data: 'bathroom'"

# Test vision tool (requires camera running)
ros2 topic pub --once /query_room/command std_msgs/String "data: 'describe room'"

# Monitor all activity
ros2 run ican_brain debug_monitor
```

## Dependencies

**query_room_node requires:**
```bash
pip install opencv-python cv-bridge
```

**Already included in workspace:**
- ollama (Python client)
- rclpy
- std_msgs
- sensor_msgs

## Performance Considerations

### Simultaneous Processing

The dual-lock architecture allows:
- ✅ Text processing while waiting for vision
- ✅ Vision processing while waiting for text
- ❌ Two text requests simultaneously (blocked)
- ❌ Two vision requests simultaneously (blocked)

### Memory Usage

- **Single VLM instance**: ~8GB VRAM (qwen3-vl:8b)
- **LLM + VLM loaded**: ~12GB VRAM total
- Models auto-load on first use
- Ollama manages model caching

### Latency

- **Text response**: 0.5-2s (depending on prompt)
- **Vision response**: 2-5s (image encoding + VLM inference)
- **Camera capture**: <0.1s (single frame)

## Troubleshooting

### Vision queries not working

1. Check camera is running: `ros2 topic echo /image`
2. Check ollama VLM is available: `ollama list | grep qwen3-vl`
3. Check query_room_node logs for errors

### TTS not speaking vision responses

1. Verify topic connection: `ros2 topic info /tts/speak`
2. Check TTS node is running: `ros2 node list | grep tts`
3. Monitor vision path: `ros2 topic echo /llm_vision_query`

### Tool calls not detected

1. Verify LLM includes "TOOL:" prefix in responses
2. Check tool_manager regex patterns match output
3. Use debug_monitor to see full LLM responses

## Future Enhancements

1. **Real Movement Integration**: Replace simulated movement with actual robot control
2. **Continuous Vision**: Stream descriptions during navigation
3. **Multi-Modal Fusion**: Combine vision + voice commands
4. **Tool Chaining**: Execute multiple tools in sequence
5. **Context Memory**: Remember previous room descriptions

---

**Author**: I_CAN Robot Project  
**License**: Apache-2.0  
**Last Updated**: December 8, 2025
