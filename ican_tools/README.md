# ican_tools - ROS2 Tool Calling System

ROS2-native tool registry and execution for I_CAN Robot voice control.

## Quick Start

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select ican_tools --symlink-install
source install/setup.bash

# Launch both nodes (part of remote_nodes.launch.py)
ros2 run ican_tools tool_manager_node
ros2 run ican_tools dice_service_node
```

## Architecture

```
Voice → Whisper → Prompt Node → Ollama → Tool Manager → Tool Nodes
                                    ↓           ↓            ↓
                              /llm_response  /dice/command  /dice/result
                                                  ↓
                                            /tool_result
```

### Current Nodes

1. **tool_manager_node** - Central tool registry and LLM output parser
2. **dice_service_node** - Example tool: RPG dice rolling service

## Node Details

### tool_manager_node

Central orchestrator for all robot tools.

**Functionality:**
- Maintains registry of available tools
- Parses LLM output for TOOL:function(params) format
- Routes tool calls to appropriate topics/services
- Collects and publishes tool results
- Provides tool discovery service

**Topics:**
- Subscribes: `/llm_response` (String) - From ollama_tool_node
- Publishes: `/tool_result` (String) - Back to system
- Publishes: `/dice/command` (String) - Dice tool commands
- Subscribes: `/dice/result` (String) - Dice tool results

**Services:**
- Provides: `/tools/list` (Trigger) - List available tools

**Tool Format:**
LLM outputs tool calls in the response text:
```
I'll roll a d20 for you! TOOL:roll(d20)
```

**Registered Tools:**
```python
'roll': 'roll(dice_notation) - Roll dice (e.g., d20, 3d6+5)'
'roll_stats': 'roll_stats() - Generate RPG character stats'
'coin_flip': 'coin_flip(count) - Flip coins'
'roll_history': 'roll_history(limit) - Show recent rolls'
```

**Usage:**
```bash
ros2 run ican_tools tool_manager_node

# Test tool discovery
ros2 service call /tools/list std_srvs/srv/Trigger

# Monitor tool calls
ros2 topic echo /tool_result
```

### dice_service_node

Comprehensive dice rolling tool via ROS2 topics.

**Functionality:**
- Roll dice using RPG notation (3d6, d20+5, etc.)
- Generate character ability scores (4d6 drop lowest)
- Coin flipping
- Roll history tracking (last 100 rolls)
- Percentile dice (d100)

**Topics:**
- Subscribes: `/dice/command` (String) - Commands from tool_manager
- Publishes: `/dice/result` (String) - Results back to tool_manager

**Services:**
- Provides: `/dice/roll` (Trigger) - Simple d20 roll service

**Command Format:**
```bash
# Standard dice notation
roll(d20)          # Roll one d20
roll(3d6)          # Roll three d6
roll(2d10+5)       # Roll two d10, add 5
roll(4d6-2)        # Roll four d6, subtract 2

# Special commands
roll_stats()       # Generate 6 ability scores (4d6 drop lowest)
coin_flip(3)       # Flip 3 coins
roll_history(5)    # Show last 5 rolls
percentile()       # Roll d100
```

**Example Output:**
```
Rolled 1d20: [17] = 17

Rolled 3d6: [4, 5, 3] = 12

Rolled 2d10+5: [7, 3] +5 = 15

Character Stats (4d6 drop lowest):
  Strength: 15 [6, 5, 3, 1] → dropped 1
  Dexterity: 13 [5, 4, 3, 1] → dropped 1
  ...
```

**Usage:**
```bash
ros2 run ican_tools dice_service_node

# Send command manually
ros2 topic pub --once /dice/command std_msgs/String "data: 'roll(d20)'"

# Monitor results
ros2 topic echo /dice/result
```

## Tool Call Flow Example

```
1. User: "Hey Spot, roll a d20"

2. Whisper recognizes: "hey spot roll a d20"
   → /speech_text

3. Prompt Node constructs prompt with tool context
   → /llm_prompt: "You are Spot... Available tools: roll()..."

4. Ollama generates response with tool call
   → /llm_response: "I'll roll a d20 for you! TOOL:roll(d20)"

5. Tool Manager parses and extracts
   - Detected: TOOL:roll(d20)
   - Routes to: /dice/command: "roll(d20)"

6. Dice Service executes
   → /dice/result: "Rolled 1d20: [17] = 17"

7. Tool Manager publishes result
   → /tool_result: "Rolled 1d20: [17] = 17"

8. System responds (TTS or further LLM processing)
```

## Testing

### Test Tool Manager

```bash
# Terminal 1: Tool manager
ros2 run ican_tools tool_manager_node

# Terminal 2: Dice service
ros2 run ican_tools dice_service_node

# Terminal 3: Simulate LLM output with tool call
ros2 topic pub --once /llm_response std_msgs/String \
  "data: 'I rolled for you! TOOL:roll(d20)'"

# Terminal 4: Monitor result
ros2 topic echo /tool_result
```

### Test Dice Service Directly

```bash
# Terminal 1: Dice service
ros2 run ican_tools dice_service_node

# Terminal 2: Send commands
ros2 topic pub --once /dice/command std_msgs/String "data: 'roll(3d6+2)'"
ros2 topic pub --once /dice/command std_msgs/String "data: 'roll_stats()'"
ros2 topic pub --once /dice/command std_msgs/String "data: 'coin_flip(5)'"

# Terminal 3: Monitor results
ros2 topic echo /dice/result
```

### Test Tool Discovery

```bash
ros2 service call /tools/list std_srvs/srv/Trigger
```

## Complete System Launch

```bash
# Launch all voice-tool nodes (includes tool_manager + dice_service)
ros2 launch ican_bringup remote_nodes.launch.py
```

Includes:
- whisper_server_node (speech recognition)
- prompt_node (wake word + context)
- ollama_tool_node (LLM)
- **tool_manager_node** (tool orchestration)
- **dice_service_node** (dice tool)

## Adding New Tools

### 1. Create Tool Node

Create a new ROS2 node that:
- Subscribes to a command topic (e.g., `/movement/command`)
- Publishes results (e.g., `/movement/result`)
- Implements tool functionality

### 2. Register in Tool Manager

Edit `tool_manager_node.py`:
```python
self.tools = {
    # ... existing tools ...
    'move_forward': {
        'topic': '/movement/command',
        'description': 'move_forward(distance) - Move robot forward',
        'pattern': r'TOOL:move_forward\(([^)]+)\)',
        'status': 'unknown'
    }
}
```

### 3. Create Publisher

Add publisher for new tool topic:
```python
self.movement_pub = self.create_publisher(String, '/movement/command', 10)
```

### 4. Add Execution Logic

Add case in `execute_tool()`:
```python
if tool_name == 'move_forward':
    self.movement_pub.publish(command_msg)
```

### 5. Update Prompt Node

Tool manager's `/tools/list` service automatically includes new tools in prompts.

## Tool Format Specification

### LLM Output Format
```
Regular text response... TOOL:function_name(parameters)... more text
```

### Parsing Rules
- Tool calls detected with regex: `TOOL:(\w+)\(([^)]*)\)`
- Multiple tools per response supported
- Tool calls can appear anywhere in response text
- Parameters extracted and passed to tool nodes

### Example Tool Calls
```
"Rolling! TOOL:roll(d20)"
"Let me check... TOOL:roll(3d6+5)"
"Generating stats... TOOL:roll_stats()"
"Flipping coins... TOOL:coin_flip(3)"
```

## Dependencies

```bash
# ROS2 packages (standard)
rclpy
std_msgs
std_srvs

# Python (built-in)
re
random
time
```

## Troubleshooting

### Tool Manager Not Detecting Tools
- Check `/llm_response` format: `ros2 topic echo /llm_response`
- Verify pattern matches: must be `TOOL:function(params)`
- Check tool_manager logs for parsing errors

### Dice Service Not Responding
- Verify node running: `ros2 node list | grep dice`
- Check topic connection: `ros2 topic info /dice/command`
- Monitor incoming commands: `ros2 topic echo /dice/command`

### Tools Not Available to LLM
- Check service: `ros2 service call /tools/list std_srvs/srv/Trigger`
- Verify prompt_node queries tool_manager on startup
- Check prompt_node logs for tool list

### No Tool Results
- Monitor `/tool_result`: `ros2 topic echo /tool_result`
- Check dice service logs for errors
- Verify tool_manager subscribed to `/dice/result`

## Development

### Package Structure
```
ican_tools/
├── ican_tools/
│   ├── tool_manager_node.py      # Tool registry & orchestration
│   ├── dice_service_node.py      # Dice rolling tool
│   └── __init__.py
├── setup.py                       # Package configuration
├── package.xml                    # ROS2 manifest
└── README.md                      # This file
```

### Files to Delete

**Obsolete (MCP-related, delete these):**
- `behavior_server.py` - MCP movement server (replaced by ROS2 topics)
- `senses_server.py` - MCP sensor server (not needed for tool calling)
- `dice_mcp_node.py` - MCP dice server (replaced by dice_service_node)
- `BEHAVIOR_SERVER_README.md` - MCP behavior docs
- `BEHAVIOR_SERVER_SUMMARY.md` - MCP behavior summary
- `MCP_INTEGRATION_GUIDE.md` - MCP integration (no longer using MCP)
- `QUICK_REFERENCE.md` - MCP quick reference
- `SENSES_SERVER_README.md` - MCP senses docs
- `README_old.md` - Old MCP-focused readme

```bash
cd ~/ros2_ws/src/I_CAN_Robot/ican_tools
rm ican_tools/behavior_server.py
rm ican_tools/senses_server.py
rm ican_tools/dice_mcp_node.py
rm BEHAVIOR_SERVER_README.md
rm BEHAVIOR_SERVER_SUMMARY.md
rm MCP_INTEGRATION_GUIDE.md
rm QUICK_REFERENCE.md
rm SENSES_SERVER_README.md
rm README_old.md

# Rebuild
cd ~/ros2_ws
colcon build --packages-select ican_tools --symlink-install
```

## Architecture Evolution

### Phase 1: MCP with stdio (Abandoned)
- FastMCP servers for tools
- stdio communication between processes
- Issues: async complexity, buffering problems

### Phase 2: ROS2-Native Tool Calling (Current)
- Tools as ROS2 nodes with topics/services
- Tool manager parses LLM output for TOOL:function()
- Direct ROS2 communication, simpler architecture

### Future Enhancements
- Movement tools (walk, turn, sit)
- Vision query tools (YOLO detections)
- Navigation tools (go to waypoint)
- Manipulation tools (gripper control)
- Multi-step tool chains
- Tool result validation

## Resources

- **ROS2 Jazzy:** https://docs.ros.org/en/jazzy/
- **I_CAN Robot:** Main repository documentation

---

**Note:** This package implements the tool layer of I_CAN Robot's voice control system. Tools are called by the LLM through tool_manager, which orchestrates execution and result collection.
