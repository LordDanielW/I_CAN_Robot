# Senses Server - MCP Server for Robot Sensors

## Overview

The `senses_server.py` is an MCP (Model Context Protocol) server that provides **read-only** access to the Unitree robot's sensor data for LLM integration. It bridges ROS 2 sensor topics with the MCP protocol, allowing AI assistants to query the robot's current state.

## Architecture

```
┌─────────────────────────────────────────┐
│         MCP Server (Main Thread)        │
│                                         │
│  ┌─────────────────────────────────┐   │
│  │  MCP Tools (LLM Interface)     │   │
│  │  • get_robot_status()           │   │
│  │  • check_obstacles()            │   │
│  └─────────────────────────────────┘   │
│              ↓ ↑                        │
│  ┌─────────────────────────────────┐   │
│  │    Thread-Safe Sensor Cache     │   │
│  │  • Battery Percentage           │   │
│  │  • Closest Obstacle Distance    │   │
│  │  • Linear Velocity              │   │
│  └─────────────────────────────────┘   │
│              ↓ ↑                        │
└──────────────┼─┼────────────────────────┘
               ↓ ↑
┌──────────────┼─┼────────────────────────┐
│ ROS 2 Node (Background Thread)          │
│                                          │
│  Subscriptions:                          │
│  • /scan (sensor_msgs/LaserScan)        │
│  • /battery_state (BatteryState)        │
│  • /odom (nav_msgs/Odometry)            │
└──────────────────────────────────────────┘
```

## Key Features

### 1. **Thread-Safe Sensor Caching**
- `SensorCache` class provides thread-safe storage for sensor data
- Timestamps track data freshness
- Lock-based synchronization prevents race conditions

### 2. **Background ROS 2 Processing**
- ROS 2 node runs in a separate daemon thread
- Non-blocking sensor subscriptions
- Continuous data updates while MCP server runs

### 3. **MCP Tool Interface**
Two MCP tools are exposed for LLM queries:

#### `get_robot_status()` → str
Returns formatted status including:
- Battery percentage with freshness indicator
- Current linear velocity (m/s and km/h)
- Data staleness warnings

**Example Output:**
```
Battery: 85.3% | Speed: 0.45 m/s (1.62 km/h)
```

#### `check_obstacles()` → str
Returns natural language obstacle description:
- Distance to nearest obstacle
- Safety classification (Critical/Warning/Caution/Safe)
- Visual indicators (emoji-based alerts)

**Example Output:**
```
⚠️ WARNING: Close obstacle at 0.45m - proceed with caution
```

## ROS 2 Subscriptions

### `/scan` (sensor_msgs/LaserScan)
- **Callback:** `scan_callback()`
- **Processing:** Filters invalid readings (inf, nan, out-of-range), finds minimum distance
- **Cached:** Closest obstacle distance

### `/battery_state` (sensor_msgs/BatteryState)
- **Callback:** `battery_callback()`
- **Processing:** Extracts percentage field
- **Cached:** Battery percentage (0-100)

### `/odom` (nav_msgs/Odometry)
- **Callback:** `odom_callback()`
- **Processing:** Extracts linear.x velocity
- **Cached:** Current forward/backward velocity

## Safety Features

### Data Freshness Monitoring
- All cached data includes timestamp
- Warnings issued if data > 5 seconds old
- Prevents acting on stale information

### Obstacle Classification
| Distance | Classification | Icon | Action |
|----------|---------------|------|--------|
| < 0.3m   | CRITICAL      | 🛑   | Immediate stop |
| < 0.5m   | WARNING       | ⚠️   | Proceed with caution |
| < 1.0m   | CAUTION       | ⚡   | Reduce speed |
| < 2.0m   | NOTE          | ℹ️   | Relatively clear |
| ≥ 2.0m   | SAFE          | ✅   | Clear path |
| inf      | ALL CLEAR     | ✅   | No obstacles |

## Installation

### Dependencies

The package requires the following ROS 2 packages (specified in `package.xml`):
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
```

### Python Dependencies

Install FastMCP via pip:
```bash
pip install mcp
```

### Build
```bash
cd /home/fire/ros2_ws
colcon build --packages-select ican_mcp_server
source install/setup.bash
```

## Usage

### Running the Server

```bash
# Source your ROS 2 workspace
source /home/fire/ros2_ws/install/setup.bash

# Run the senses server
ros2 run ican_mcp_server senses_server
```

### Expected Output
```
[INFO] [unitree_senses_node]: UnitreeSenses node initialized - subscribed to /scan, /battery_state, /odom
[INFO] [unitree_senses_node]: Starting MCP server for UnitreeSenses...
```

### Testing with MCP Client

The server exposes two tools that can be called by any MCP client:

```python
# Example MCP client interaction (pseudo-code)
response = mcp_client.call_tool("get_robot_status")
# Returns: "Battery: 92.1% | Speed: Stationary (0.00 m/s)"

response = mcp_client.call_tool("check_obstacles")  
# Returns: "✅ Safe: Nearest obstacle is 2.35m away - clear path ahead"
```

## Integration with LLM

The senses server is designed to be used with Claude Desktop or other MCP-compatible clients. Add to your MCP configuration:

```json
{
  "mcpServers": {
    "unitree-senses": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "senses_server"],
      "env": {
        "ROS_DOMAIN_ID": "0"
      }
    }
  }
}
```

## Code Structure

### Classes

- **`SensorCache`**: Thread-safe data storage with locking mechanisms
- **`UnitreeSensesNode`**: ROS 2 node handling sensor subscriptions

### Functions

- **`ros_spin_thread()`**: Background thread function for ROS 2 executor
- **`main()`**: Entry point coordinating ROS 2 and MCP server

### MCP Tools

- **`@mcp.tool() get_robot_status()`**: Battery and velocity query
- **`@mcp.tool() check_obstacles()`**: Obstacle detection query

## Concurrency Model

1. **Main Thread**: Runs `mcp.run()` - handles MCP requests
2. **Background Thread**: Runs `rclpy.spin()` - processes ROS 2 callbacks
3. **Synchronization**: `threading.Lock` in `SensorCache` ensures safe access

## Error Handling

- Invalid laser scan readings filtered (inf, nan, out-of-range)
- Null checks before accessing cached data
- Timeout warnings for stale data
- Graceful shutdown on KeyboardInterrupt

## Troubleshooting

### No Data Available
```
Battery: No data available | Speed: No data available
```
**Solution:** Ensure sensor topics are publishing:
```bash
ros2 topic list
ros2 topic echo /battery_state
ros2 topic echo /scan
ros2 topic echo /odom
```

### Stale Data Warnings
```
Battery: 85.0% (stale data, 12s old)
```
**Solution:** Check if sensor nodes are still running and topics are active

### Import Error: mcp.server.fastmcp
**Solution:** Install FastMCP:
```bash
pip install mcp
```

## Future Enhancements

- [ ] Add camera feed access (image summaries)
- [ ] IMU orientation data
- [ ] GPS position (if available)
- [ ] Configurable topic names via parameters
- [ ] Historical data trends (e.g., battery drain rate)

## License

Apache-2.0

## Author

I_CAN Robot Project Team
