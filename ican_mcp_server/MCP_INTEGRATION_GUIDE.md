# MCP Integration Guide - Connecting Ollama to MCP Servers

## Overview

This guide explains how to connect the Ollama LLM to MCP servers for tool calling. Currently, **direct ROS2 integration is in development**. Here are the approaches:

## Current Status

### ⚠️ Important Note
Ollama's tool calling in the current `ollama_node.py` uses the basic `ollama.generate()` API which **does not support MCP tool calling directly**. To use MCP servers with Ollama, you need to use one of these approaches:

## Approach 1: Use Claude Desktop (Recommended for Testing)

Claude Desktop has built-in MCP support. You can test your MCP servers with Claude:

### 1. Install Claude Desktop
Download from: https://claude.ai/download

### 2. Configure MCP Server
Add to `~/Library/Application Support/Claude/claude_desktop_config.json` (Mac) or
`%APPDATA%\Claude\claude_desktop_config.json` (Windows):

```json
{
  "mcpServers": {
    "dice_roller": {
      "command": "python3",
      "args": [
        "/home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/dice_mcp_node.py"
      ],
      "env": {
        "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages"
      }
    }
  }
}
```

### 3. Test with Claude
- Restart Claude Desktop
- Ask: "Roll a d20 for me"
- Claude will use the dice_mcp_node to roll the die

## Approach 2: MCP Inspector (Development/Debugging)

Use the MCP Inspector to test your MCP servers:

```bash
# Test dice_mcp_node
mcp dev /home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/dice_mcp_node.py

# This opens a web interface to interact with the MCP server
```

## Approach 3: Ollama with Tool Calling (Future Implementation)

To make `ollama_node` work with MCP servers, we need to modify it to:

### Required Changes to ollama_node.py:

1. **Add MCP Client Support**
```python
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

# Connect to MCP server
server_params = StdioServerParameters(
    command="python3",
    args=["/path/to/dice_mcp_node.py"]
)

async def setup_mcp():
    async with stdio_client(server_params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            # Get available tools
            tools = await session.list_tools()
            return session, tools
```

2. **Convert MCP Tools to Ollama Format**
```python
def mcp_tools_to_ollama_format(mcp_tools):
    """Convert MCP tool definitions to Ollama tool format"""
    ollama_tools = []
    for tool in mcp_tools:
        ollama_tools.append({
            'type': 'function',
            'function': {
                'name': tool.name,
                'description': tool.description,
                'parameters': tool.inputSchema
            }
        })
    return ollama_tools
```

3. **Use Ollama Chat API with Tools**
```python
response = ollama.chat(
    model='qwen2.5:7b',
    messages=[{'role': 'user', 'content': prompt}],
    tools=ollama_tools  # Pass converted MCP tools
)

# Handle tool calls
if response['message'].get('tool_calls'):
    for tool_call in response['message']['tool_calls']:
        # Call MCP server
        result = await mcp_session.call_tool(
            tool_call['function']['name'],
            tool_call['function']['arguments']
        )
        # Send result back to Ollama
```

## Approach 4: Current Test Setup (Simple Integration)

For now, use the test client to simulate tool calling:

### Terminal 1: Start Dice MCP Server
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run ican_mcp_server dice_mcp_node
```

### Terminal 2: Start Ollama Node
```bash
ros2 run ican_orchestrator ollama_node
```

### Terminal 3: Send Test Prompts
```bash
# Use the MCP test chat
ros2 run ican_orchestrator ollama_mcp_chat

# Or manually
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Roll a d20 and tell me if it's a critical hit (20) or critical fail (1)'"
```

### Monitor Responses
```bash
ros2 topic echo /llm_response
```

## Available MCP Servers

### 1. dice_mcp_node
**Tools:**
- `roll(dice_notation)` - Roll dice (e.g., "3d6", "d20+5")
- `roll_multiple(dice_type, count)` - Roll multiple dice
- `roll_stats()` - Generate RPG character stats
- `coin_flip(count)` - Flip coins
- `roll_percentile()` - Roll d100
- `get_roll_history(limit)` - View recent rolls

**Example Prompts:**
- "Roll a d20 for initiative"
- "Generate character stats for me"
- "Flip 3 coins"
- "Roll 2d10+5 for damage"

### 2. behavior_server  
**Tools:**
- `move_robot(direction, duration, speed)` - Move robot
- `rotate_robot(direction, duration, angular_speed)` - Rotate robot
- `stop()` - Emergency stop
- `get_movement_capabilities()` - List capabilities

### 3. senses_server
**Tools:**
- Vision, audio, and sensor data reading
- (See SENSES_SERVER_README.md)

## Testing Without Full Integration

While we work on full MCP integration, you can:

1. **Test MCP Server Standalone:**
```bash
# Run dice server
python3 dice_mcp_node.py

# In another terminal, use mcp CLI
mcp dev dice_mcp_node.py
```

2. **Test with Simple Prompts:**
The LLM can respond to dice-related questions, even without tool calling:
```bash
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Imagine you rolled a d20. Tell me a creative result for rolling a 1 vs rolling a 20'"
```

## Building Full Integration (TODO)

To create a fully integrated system:

1. **Create `ollama_mcp_node.py`** - New node that:
   - Spawns MCP servers as subprocesses
   - Converts between MCP and Ollama tool formats
   - Handles async tool calling
   - Routes tool results back to Ollama

2. **Use Ollama's Chat API** - Instead of `generate()`, use `chat()` with tools

3. **Implement Tool Routing** - Map MCP tool calls to correct servers

4. **Add Error Handling** - Graceful degradation if tools fail

## Example: Manual Tool Calling Flow

Until full integration is ready:

```bash
# 1. LLM receives prompt
ros2 topic pub /llm_prompt std_msgs/String "data: 'Roll 2d6 for me'"

# 2. Human or bridge node calls MCP server directly
ros2 run ican_mcp_server dice_mcp_node
# (In MCP interface): call roll("2d6")

# 3. Get result: "🎲 Rolling 2d6: [4, 5] = 9"

# 4. Feed result back to LLM
ros2 topic pub /llm_prompt std_msgs/String \
  "data: 'The dice roll result was: 🎲 Rolling 2d6: [4, 5] = 9. What does this mean?'"
```

## Next Steps

- [ ] Implement async MCP client in ollama_node
- [ ] Add tool call parsing and routing
- [ ] Create unified MCP orchestrator node
- [ ] Add tool call history and logging
- [ ] Test with multiple MCP servers simultaneously

## Resources

- FastMCP Docs: https://github.com/jlowin/fastmcp
- Ollama Tool Calling: https://ollama.com/blog/tool-support
- MCP Protocol: https://modelcontextprotocol.io/

---

**Note:** This is a work in progress. The current setup allows testing MCP servers independently. Full LLM-to-MCP integration requires additional development.
