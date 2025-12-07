# ican_brain - AI Brain for I_CAN Robot

Voice-controlled AI orchestration using Ollama LLM with ROS2 tool calling.

## Quick Start

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select ican_brain --symlink-install
source install/setup.bash

# Test Ollama (no ROS2)
bash ~/ros2_ws/src/I_CAN_Robot/ican_brain/scripts/test_ollama.sh

# Launch complete system (remote computer)
ros2 launch ican_bringup remote_nodes.launch.py
```

## Architecture

```
Voice Input → Whisper → Prompt Node → Ollama Tool Node → Tool Manager → Tools
                            ↓              ↓                   ↓
                    /speech_text    /llm_prompt        /tool_result
                                        ↓
                                  /llm_response
```

### Current Nodes (Active)

1. **prompt_node** - Wake word detection & prompt construction
2. **ollama_tool_node** - LLM inference with tool detection
3. **debug_monitor** - Full message display (no truncation)

### Legacy Nodes (Kept for Testing)

- **ollama_node** - Original MCP implementation (not in pipeline)

## Node Details

### prompt_node

Intelligent prompt construction with wake word detection.

**Functionality:**
- Listens for wake word ("hey spot") with fuzzy matching
- Buffers speech text into complete sentences
- Queries tool_manager for available tools
- Constructs rich prompts with robot context
- Sends formatted prompts to LLM

**Topics:**
- Subscribes: `/speech_text` (String) - From whisper/vosk
- Publishes: `/llm_prompt` (String) - To ollama_tool_node
- Publishes: `/prompt_status` (String) - Status updates

**Parameters:**
```bash
ros2 run ican_brain prompt_node --ros-args \
  -p wake_word:='hey spot' \
  -p buffer_timeout:=2.0 \
  -p wake_word_threshold:=0.75
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wake_word` | string | `hey spot` | Wake phrase to activate listening |
| `buffer_timeout` | float | `2.0` | Seconds to wait before processing buffer |
| `wake_word_threshold` | float | `0.75` | Fuzzy match threshold (0.0-1.0) |

**Wake Word Variants:**
Fuzzy matching handles common misrecognitions:
- "hey spot", "hi spot"
- "hey spock", "hi spock" (Star Trek!)
- "hey scott", "hi scott"
- "hey spots", "a spot", "hay spot"

### ollama_tool_node

Ollama LLM with ROS2-based tool calling (no MCP stdio).

**Functionality:**
- Receives prompts from prompt_node
- Uses Ollama (qwen2.5:7b) for inference
- Detects dice roll queries automatically
- Calls dice service via ROS2 topics
- Returns natural language responses

**Topics:**
- Subscribes: `/llm_prompt` (String) - From prompt_node
- Publishes: `/llm_response` (String) - To system
- Publishes: `/dice/command` (String) - Tool commands
- Subscribes: `/dice/result` (String) - Tool results

**Parameters:**
```bash
ros2 run ican_brain ollama_tool_node --ros-args \
  -p llm_model:='qwen2.5:7b'
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `llm_model` | string | `qwen2.5:7b` | Ollama model to use |

**Tool Detection:**
Automatically detects queries containing:
- "roll", "dice", "d20", "d6", etc.
- "coin flip", "flip a coin"
- "character stats", "ability scores"

### debug_monitor

Full message display without truncation.

**Functionality:**
- Subscribes to all major topics
- Prints complete message content
- Formatted output with separators
- Useful for debugging long prompts/responses

**Topics Monitored:**
- `/speech_text` - Raw speech recognition
- `/llm_prompt` - Constructed prompts
- `/llm_response` - LLM responses
- `/tool_result` - Tool execution results
- `/dice/command` - Dice commands
- `/dice/result` - Dice results
- `/prompt_status` - Prompt node status

**Usage:**
```bash
ros2 run ican_brain debug_monitor
```

## Message Flow Example

```
1. Whisper recognizes: "hey spot roll a d20"
   → /speech_text: "hey spot roll a d20"

2. Prompt Node detects wake word, constructs prompt
   → /llm_prompt: "You are Spot, a helpful robot dog. Tools: roll_dice()..."
   → /prompt_status: "Wake word detected, listening..."

3. Ollama Tool Node processes prompt
   - Detects dice query
   - Publishes: /dice/command: "roll(d20)"
   - Waits for: /dice/result: "Rolled d20: 17"
   - Generates response: "I rolled a d20 and got 17!"
   → /llm_response: "I rolled a d20 and got 17!"

4. TTS speaks the response
```

## Testing

### Test Ollama (No ROS2)
```bash
bash ~/ros2_ws/src/I_CAN_Robot/ican_brain/scripts/test_ollama.sh
```

Checks:
- Ollama installed
- Service running
- qwen2.5:7b model available
- Simple query test

### Test Individual Nodes

**Prompt Node:**
```bash
# Terminal 1: Prompt node
ros2 run ican_brain prompt_node

# Terminal 2: Debug monitor
ros2 run ican_brain debug_monitor

# Terminal 3: Simulate speech input
ros2 topic pub /speech_text std_msgs/String "data: 'hey spot hello there'"
```

**Ollama Tool Node:**
```bash
# Terminal 1: Tool manager + dice service
ros2 run ican_tools tool_manager_node
ros2 run ican_tools dice_service_node

# Terminal 2: Ollama node
ros2 run ican_brain ollama_tool_node

# Terminal 3: Send prompt
ros2 topic pub /llm_prompt std_msgs/String "data: 'Roll a d20 for me'"
```

**Debug Monitor:**
```bash
# Launch system, then monitor
ros2 run ican_brain debug_monitor
```

## Complete System Launch

```bash
# Remote computer (powerful, runs AI)
ros2 launch ican_bringup remote_nodes.launch.py

# Local computer (robot, runs sensors)
ros2 launch ican_bringup local_nodes.launch.py
```

Remote nodes includes:
- whisper_server_node (speech recognition)
- prompt_node (wake word + context)
- ollama_tool_node (LLM)
- tool_manager_node (tool orchestration)
- dice_service_node (example tool)

## Dependencies

```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull model
ollama pull qwen2.5:7b

# Python packages (system)
/usr/bin/python3 -m pip install --break-system-packages ollama
```

## Troubleshooting

### Ollama Not Running
```bash
# Check if running
pgrep ollama

# Start manually
ollama serve &

# Test
ollama run qwen2.5:7b "Hello"
```

### Model Not Found
```bash
# List models
ollama list

# Pull qwen
ollama pull qwen2.5:7b
```

### Wake Word Not Detected
- Check fuzzy match threshold (lower = more permissive)
- View speech input: `ros2 topic echo /speech_text`
- Check status: `ros2 topic echo /prompt_status`
- Try exact phrase: "hey spot"

### Prompt Not Sent to LLM
- Verify prompt_node running: `ros2 node list`
- Monitor: `ros2 run ican_brain debug_monitor`
- Check buffer timeout (may need to wait 2 seconds)

### No Response from LLM
- Check ollama_tool_node running
- Verify Ollama service: `ollama list`
- Check logs: `ros2 node info /ollama_tool_node`
- Test Ollama: `bash scripts/test_ollama.sh`

## Development

### Package Structure
```
ican_brain/
├── ican_brain/
│   ├── prompt_node.py           # Wake word + prompt construction
│   ├── ollama_tool_node.py      # LLM with ROS2 tool calling
│   ├── debug_monitor.py         # Full message display
│   └── ollama_node.py          # Legacy (MCP version, kept for testing)
├── scripts/
│   └── test_ollama.sh          # Ollama test script
├── setup.py                     # Package configuration
└── README.md                    # This file
```

## Architecture Evolution

### Phase 1: MCP with OpenAI Client (Abandoned)
- Used OpenAI client → Ollama endpoint
- MCP stdio communication for tools
- Issues: async complexity, stdio buffering

### Phase 2: ROS2-Native Tool Calling (Current)
- Direct Ollama library integration
- ROS2 topics/services for tools
- Simpler, more reliable, ROS-native

### Future Enhancements
- Multi-tool support (beyond dice)
- Vision integration (YOLO detections)
- Motion commands (walk, turn, sit)
- Memory/conversation history
- Multi-robot coordination

## Resources

- **Ollama:** https://ollama.com/
- **Qwen 2.5:** https://ollama.com/library/qwen2.5
- **ROS2 Jazzy:** https://docs.ros.org/en/jazzy/

---

**Note:** This package provides the AI brain for I_CAN Robot. It processes voice commands, consults the LLM, and coordinates tool execution through ROS2.
