# Testing MCP Integration with Ollama Node

## Overview

The `ollama_node` now supports **MCP tool calling**! The LLM can automatically call tools from MCP servers based on user prompts.

## Features Implemented

✅ **MCP Client Integration** - Connects to MCP servers via stdio  
✅ **Tool Discovery** - Automatically fetches available tools from MCP server  
✅ **Ollama Chat API** - Uses `ollama.chat()` instead of `generate()` for tool support  
✅ **Tool Call Routing** - Intercepts tool calls and routes to MCP server  
✅ **Multi-turn Conversation** - Sends tool results back to LLM for final response  
✅ **Fallback Mode** - Works without MCP if disabled or unavailable  

## How It Works

```
User Prompt → Ollama LLM → Detects dice request → Calls MCP tool → Gets result → Returns natural language response
```

### Example Flow:
1. User: "Roll a d20 for me"
2. Ollama: *Tool call detected* → `roll("d20")`
3. MCP Server: Returns `🎲 Rolling d20: [15] = 15`
4. Ollama: "I rolled a d20 and got **15**!"

## Setup

### 1. Rebuild the Package
```bash
cd ~/ros2_ws
colcon build --packages-select ican_orchestrator --symlink-install
source install/setup.bash
```

### 2. Start the Ollama Node
```bash
# With MCP enabled (default)
ros2 run ican_orchestrator ollama_node

# OR with custom MCP server path
ros2 run ican_orchestrator ollama_node --ros-args \
  -p mcp_server_path:=/home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/dice_mcp_node.py

# Disable MCP tools (fallback to simple mode)
ros2 run ican_orchestrator ollama_node --ros-args -p enable_mcp:=false
```

### 3. Send Test Prompts

#### Terminal 2: Monitor responses
```bash
ros2 topic echo /llm_response
```

#### Terminal 3: Send prompts
```bash
# Simple dice roll
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Roll a d20 for me'"

# Complex dice notation
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Roll 3d6+5 for my attack damage'"

# Character stats
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Generate RPG character stats for me using 4d6 drop lowest'"

# Coin flip
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Flip a coin to decide if I should go left or right'"

# Check history
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'Show me the last 3 dice rolls'"

# Non-tool prompt (regular chat)
ros2 topic pub --once /llm_prompt std_msgs/String \
  "data: 'What is the capital of France?'"
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `llm_model` | string | `qwen2.5:7b` | Ollama model to use |
| `enable_mcp` | bool | `true` | Enable MCP tool calling |
| `mcp_server_path` | string | `/home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/dice_mcp_node.py` | Path to MCP server Python script |

## Expected Output

### On Startup:
```
[ollama_node]: OllamaNode initialized. Using model: qwen2.5:7b
[ollama_node]: MCP tool calling: ENABLED
[ollama_node]: Connected to MCP server: .../dice_mcp_node
[ollama_node]:   - roll: Roll dice using standard notation
[ollama_node]:   - roll_multiple: Roll multiple dice of the same type
[ollama_node]:   - roll_stats: Generate RPG character stats
[ollama_node]:   - coin_flip: Flip one or more coins
[ollama_node]:   - roll_percentile: Roll percentile dice (d100)
[ollama_node]:   - get_roll_history: Get recent dice roll history
[ollama_node]: MCP initialized with 6 tools
[ollama_node]: Publishing to /llm_response (std_msgs/String)
[ollama_node]: Subscribing to /llm_prompt (std_msgs/String)
```

### During Tool Call:
```
[ollama_node]: Received prompt: "Roll a d20 for me"
[ollama_node]: Model requested 1 tool call(s)
[ollama_node]: Calling tool: roll with args: {'dice_notation': 'd20'}
[ollama_node]: Tool result: 🎲 Rolling d20: [15] = 15
[ollama_node]: Response published in 3.45s.
[ollama_node]: AI Response: "I rolled a d20 and got 15!"
```

## Troubleshooting

### Error: "MCP setup failed"
**Problem:** Can't connect to MCP server  
**Solution:** 
1. Check the `mcp_server_path` parameter points to the correct executable
2. Verify the dice_mcp_node runs standalone: `python3 dice_mcp_node.py`
3. Check if fastmcp and mcp packages are installed

### Error: "Model requested 0 tool calls"
**Problem:** LLM isn't detecting tool opportunities  
**Solution:** 
1. Make your prompt more explicit: "Use the roll tool to roll a d20"
2. Check if tools were loaded: Look for "MCP initialized with X tools" in logs
3. Try a different model that supports tool calling better

### Error: "Tool call failed"
**Problem:** MCP server returned error  
**Solution:**
1. Check MCP server logs for errors
2. Verify the tool arguments match the expected schema
3. Test the tool directly with `mcp dev dice_mcp_node.py`

### Fallback Mode Activated
**Problem:** `enable_mcp:=false` or MCP failed to initialize  
**Solution:** Node works in simple mode (no tools). Check logs for MCP initialization errors.

## Architecture Details

### MCP Integration Components:

1. **Async Event Loop** - Separate thread runs asyncio for MCP operations
2. **Stdio Connection** - Spawns MCP server as subprocess with stdin/stdout communication
3. **ClientSession** - MCP protocol session for tool discovery and calling
4. **Tool Format Conversion** - Translates MCP tool schemas to Ollama format
5. **Multi-turn Chat** - Maintains message history through tool calls

### Code Flow:

```python
prompt_callback()
  └─> process_llm_query()
       └─> _process_with_tools()
            ├─> ollama.chat(tools=ollama_tools)  # First call
            ├─> _call_mcp_tool()  # For each tool call
            │    └─> mcp_session.call_tool()
            └─> ollama.chat(messages)  # Second call with results
```

## Testing Multiple MCP Servers

To add more MCP servers (e.g., behavior_server, senses_server):

### Option 1: Modify Code
Edit `ollama_node.py` to load multiple servers:
```python
self.mcp_servers = [
    '/path/to/dice_mcp_node',
    '/path/to/behavior_server',
    '/path/to/senses_server'
]
```

### Option 2: MCP Orchestrator (Future)
Create a dedicated `mcp_orchestrator_node` that:
- Manages multiple MCP servers
- Routes tool calls to appropriate servers
- Aggregates tools from all servers

## Next Steps

- [ ] Test with various dice rolling prompts
- [ ] Add behavior_server for robot movement tools
- [ ] Add senses_server for vision/audio tools
- [ ] Create MCP orchestrator for multi-server management
- [ ] Add conversation history for context awareness
- [ ] Implement tool call caching for performance

## Resources

- Ollama Tool Support: https://ollama.com/blog/tool-support
- FastMCP Documentation: https://github.com/jlowin/fastmcp
- MCP Protocol Spec: https://modelcontextprotocol.io/

---

**Status:** ✅ READY FOR TESTING

Run the setup commands above and start testing tool calling!
