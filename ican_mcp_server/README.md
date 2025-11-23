# ican_mcp_server Package

MCP (Model Context Protocol) Server package for I_CAN Robot - bridges ROS 2 and Large Language Models

## Overview

This package provides two complementary MCP servers that enable LLM integration with the Unitree robot:

1. **Senses Server** (`senses_server.py`) - Read-only sensor access 👁️
2. **Behavior Server** (`behavior_server.py`) - Write-access movement control 🤖

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    LLM / MCP Client                      │
│              (Claude Desktop, etc.)                      │
└────────────────────┬─────────────────┬───────────────────┘
                     │                 │
          ┌──────────▼─────────┐   ┌──▼────────────────┐
          │  Senses Server     │   │ Behavior Server   │
          │  (Read-Only)       │   │ (Write-Access)    │
          │                    │   │                   │
          │ Tools:             │   │ Tools:            │
          │ • check_obstacles  │   │ • move_robot      │
          │ • get_robot_status │   │ • rotate_robot    │
          │                    │   │ • stop            │
          └──────────┬─────────┘   └──┬────────────────┘
                     │                 │
                     │ Subscribe       │ Publish
                     │                 │
          ┌──────────▼─────────────────▼────────────────┐
          │           ROS 2 Middleware                  │
          │                                             │
          │  Topics:                                    │
          │  • /scan (LaserScan)                       │
          │  • /battery_state (BatteryState)           │
          │  • /odom (Odometry)                        │
          │  • /cmd_vel (Twist)                        │
          └─────────────────────────────────────────────┘
                              │
                    ┌─────────▼──────────┐
                    │   Unitree Robot    │
                    │   (Go2/B1/etc.)    │
                    └────────────────────┘
```

## Package Structure

```
ican_mcp_server/
├── ican_mcp_server/                # Python package
│   ├── __init__.py
│   ├── senses_server.py           # Read-only sensor MCP server
│   └── behavior_server.py         # Write-access control MCP server
├── launch/                         # Launch files (future)
├── config/                         # Configuration files (future)
├── resource/                       # ROS 2 resource marker
├── package.xml                     # ROS 2 package manifest
├── setup.py                        # Python package setup
├── SENSES_SERVER_README.md        # Detailed senses server docs
├── BEHAVIOR_SERVER_README.md      # Detailed behavior server docs
├── QUICK_REFERENCE.md             # Quick command reference
└── README.md                       # This file
```

## Features

### Senses Server

**Purpose:** Provides LLMs with safe, read-only access to robot sensor data

**Key Features:**
- 👁️ Real-time sensor monitoring
- 🔒 Thread-safe data caching
- ⏰ Data freshness tracking (5s threshold)
- 💬 Natural language responses
- 🎯 Obstacle detection and classification

**MCP Tools:**
- `get_robot_status()` - Battery and velocity information
- `check_obstacles()` - Nearest obstacle with safety assessment

**ROS 2 Subscriptions:**
- `/scan` (sensor_msgs/LaserScan)
- `/battery_state` (sensor_msgs/BatteryState)
- `/odom` (nav_msgs/Odometry)

### Behavior Server

**Purpose:** Enables LLMs to control robot movement with safety constraints

**Key Features:**
- 🤖 Linear and rotational movement control
- 🔄 Continuous command publishing (10 Hz)
- 🛑 Automatic stop after each command
- ⚠️ Speed and duration limits
- 🚨 Emergency stop function
- ✅ Input validation

**MCP Tools:**
- `move_robot(direction, duration, speed)` - Linear movement
- `rotate_robot(direction, duration, angular_speed)` - Rotation
- `stop()` - Emergency stop
- `get_movement_capabilities()` - Query available commands

**ROS 2 Publishers:**
- `/cmd_vel` (geometry_msgs/Twist)

## Installation

### Prerequisites

**ROS 2 Jazzy** must be installed and sourced.

**System Dependencies:**
```bash
# ROS 2 packages (usually included)
sudo apt update
sudo apt install ros-jazzy-sensor-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs
```

**Python Dependencies:**
```bash
# FastMCP library
pip install mcp
```

### Build Package

```bash
# Navigate to workspace
cd /home/fire/ros2_ws

# Build the package
colcon build --packages-select ican_mcp_server

# Source the workspace
source install/setup.bash
```

## Usage

### Running Individual Servers

**Senses Server:**
```bash
source /home/fire/ros2_ws/install/setup.bash
ros2 run ican_mcp_server senses_server
```

**Behavior Server:**
```bash
source /home/fire/ros2_ws/install/setup.bash
ros2 run ican_mcp_server behavior_server
```

### Running Both Servers

**Terminal 1 (Senses):**
```bash
ros2 run ican_mcp_server senses_server
```

**Terminal 2 (Behavior):**
```bash
ros2 run ican_mcp_server behavior_server
```

### Expected Output

**Senses Server:**
```
[INFO] [unitree_senses_node]: UnitreeSenses node initialized - subscribed to /scan, /battery_state, /odom
[INFO] [unitree_senses_node]: Starting MCP server for UnitreeSenses...
```

**Behavior Server:**
```
[INFO] [unitree_behavior_node]: UnitreeBehavior node initialized - publishing to /cmd_vel
[INFO] [unitree_behavior_node]: Starting MCP server for UnitreeBehaviors...
```

## MCP Client Configuration

### Claude Desktop Integration

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "unitree-senses": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "senses_server"],
      "env": {
        "ROS_DOMAIN_ID": "0",
        "ROS_LOCALHOST_ONLY": "1"
      }
    },
    "unitree-behaviors": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "behavior_server"],
      "env": {
        "ROS_DOMAIN_ID": "0",
        "ROS_LOCALHOST_ONLY": "1"
      }
    }
  }
}
```

### Generic MCP Client

```python
import mcp

# Connect to servers
senses_client = mcp.Client("unitree-senses")
behavior_client = mcp.Client("unitree-behaviors")

# Read sensors
status = senses_client.call_tool("get_robot_status")
obstacles = senses_client.call_tool("check_obstacles")

# Control movement (if safe)
if "Safe" in obstacles:
    result = behavior_client.call_tool(
        "move_robot",
        direction="forward",
        duration=2.0,
        speed=0.5
    )
```

## Testing

### Verify Topics

**Check Sensor Topics:**
```bash
# Check publishing rates
ros2 topic hz /scan
ros2 topic hz /battery_state
ros2 topic hz /odom

# Echo data
ros2 topic echo /scan --once
ros2 topic echo /battery_state --once
ros2 topic echo /odom --once
```

**Check Control Topics:**
```bash
# Monitor cmd_vel
ros2 topic echo /cmd_vel

# Check publishing rate (when moving)
ros2 topic hz /cmd_vel
```

### Verify Nodes

```bash
# List active nodes
ros2 node list

# Check node details
ros2 node info /unitree_senses_node
ros2 node info /unitree_behavior_node
```

## Example LLM Workflow

### Safe Navigation Example

```
1. LLM: Check environment
   → check_obstacles()
   ← "✅ Safe: Nearest obstacle is 3.5m away"

2. LLM: Check robot status
   → get_robot_status()
   ← "Battery: 92.1% | Speed: Stationary (0.00 m/s)"

3. LLM: Move forward (safe to proceed)
   → move_robot("forward", duration=2.0, speed=0.5)
   ← "✅ Moved forward at 0.50 m/s for 2.00s (~1.00m traveled)"

4. LLM: Check for new obstacles
   → check_obstacles()
   ← "⚠️ WARNING: Close obstacle at 0.45m"

5. LLM: Emergency stop
   → stop()
   ← "🛑 Robot stopped - all velocities set to zero"
```

### Exploration Pattern

```
1. LLM: Scan area by rotating
   → rotate_robot("left", duration=8.0, angular_speed=0.785)
   ← "✅ Rotated left at 0.78 rad/s for 8.00s (~360.0° rotation)"

2. LLM: Check all directions during rotation
   → check_obstacles() (called multiple times)
   ← Various readings at different angles

3. LLM: Move toward clearest direction
   → move_robot("forward", duration=3.0, speed=0.4)
```

## Safety Features

### Built-in Safeguards

**Senses Server:**
- ✅ Read-only access (no control)
- ✅ Thread-safe data access
- ✅ Stale data warnings (>5s)
- ✅ Invalid reading filtering

**Behavior Server:**
- ✅ Automatic stop after every command
- ✅ Duration limits (max 30s)
- ✅ Speed limits (max 2.0 m/s)
- ✅ Input validation
- ✅ Emergency stop function
- ✅ Error handling with auto-stop
- ✅ Continuous publishing (prevents timeout)

### Recommended LLM Safety Logic

```python
# Always check before moving
status = check_obstacles()
if "CRITICAL" in status or "WARNING" in status:
    # Don't move or use very slow speed
    pass
elif "Safe" in status:
    # Safe to move
    move_robot("forward", 2.0, 0.5)
```

## Troubleshooting

### No Data Available

**Problem:** Senses server reports "No data available"

**Solution:**
1. Check if robot/simulator is running
2. Verify topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic hz /scan
   ```
3. Check ROS_DOMAIN_ID matches

### Robot Not Moving

**Problem:** Behavior server commands don't move robot

**Solution:**
1. Verify `/cmd_vel` is being published:
   ```bash
   ros2 topic hz /cmd_vel
   ```
2. Check robot is subscribed to `/cmd_vel`:
   ```bash
   ros2 topic info /cmd_vel
   ```
3. Verify no emergency stop is engaged
4. Check robot controller is running

### Import Errors

**Problem:** `ModuleNotFoundError: No module named 'mcp'`

**Solution:**
```bash
pip install mcp
```

**Problem:** `No module named 'geometry_msgs'`

**Solution:**
```bash
source /home/fire/ros2_ws/install/setup.bash
```

### Stale Data Warnings

**Problem:** "Stale data" messages in responses

**Solution:**
1. Check sensor nodes are running
2. Verify topic publishing rates
3. Ensure no network issues (if using multi-machine setup)

## Dependencies

### ROS 2 Packages
- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard message definitions
- `sensor_msgs` - Sensor message definitions (LaserScan, BatteryState)
- `nav_msgs` - Navigation messages (Odometry)
- `geometry_msgs` - Geometric messages (Twist)

### Python Packages
- `mcp` - FastMCP library for MCP server implementation
- `threading` - Standard library (concurrency)
- `time` - Standard library (timing)

## Performance

### Senses Server
- **Sensor Update Rate**: Depends on ROS 2 topics (typically 10-20 Hz)
- **Data Freshness Threshold**: 5 seconds
- **Query Response Time**: < 10ms (from cache)

### Behavior Server
- **Command Publishing Rate**: 10 Hz during movement
- **Command Latency**: < 50ms
- **Stop Response**: Immediate (next publish cycle)

## Future Enhancements

### Senses Server
- [ ] Camera feed integration
- [ ] IMU orientation data
- [ ] GPS position (if available)
- [ ] Historical data trends
- [ ] Configurable topic names

### Behavior Server
- [ ] Path planning integration
- [ ] Obstacle avoidance during movement
- [ ] Speed ramping (smooth acceleration)
- [ ] Movement presets (patterns)
- [ ] Odometry-based position tracking
- [ ] Movement cancellation (async stop)

### Package-wide
- [ ] Launch files for both servers
- [ ] Configuration files (YAML)
- [ ] ROS 2 parameters for customization
- [ ] Integration tests
- [ ] Simulation support (Gazebo)

## Contributing

This package is part of the I_CAN Robot project. Contributions are welcome!

### Development Guidelines
1. Follow ROS 2 Python style guide
2. Add type hints to functions
3. Include docstrings for all public methods
4. Test with actual robot or simulation
5. Update documentation

## License

Apache-2.0

## Authors

I_CAN Robot Project Team

## Support

For detailed documentation:
- Senses Server: See `SENSES_SERVER_README.md`
- Behavior Server: See `BEHAVIOR_SERVER_README.md`
- Quick Reference: See `QUICK_REFERENCE.md`

## Version History

- **0.0.0** (Current)
  - Initial implementation
  - Senses server with sensor monitoring
  - Behavior server with movement control
  - FastMCP integration
  - Thread-safe operation
