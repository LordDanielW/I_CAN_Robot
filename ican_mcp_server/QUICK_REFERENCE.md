# I_CAN MCP Server - Quick Reference

## Overview
Two MCP servers for LLM-Robot integration:
- **Senses Server**: Read-only sensor access 👁️
- **Behavior Server**: Write-access movement control 🤖

---

## 1️⃣ Senses Server (Read-Only)

### File Location
```
/home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/senses_server.py
```

### Run Command
```bash
ros2 run ican_mcp_server senses_server
```

### MCP Tools Exposed

#### `get_robot_status()`
**Returns:** Battery level and current speed
```
Battery: 85.3% | Speed: 0.45 m/s (1.62 km/h)
```

#### `check_obstacles()`
**Returns:** Nearest obstacle distance with safety assessment
```
✅ Safe: Nearest obstacle is 2.35m away - clear path ahead
```

### ROS 2 Topics Monitored

| Topic | Message Type | Data Extracted |
|-------|-------------|----------------|
| `/scan` | `sensor_msgs/LaserScan` | Closest obstacle distance |
| `/battery_state` | `sensor_msgs/BatteryState` | Battery percentage |
| `/odom` | `nav_msgs/Odometry` | Linear velocity |

### Safety Thresholds

- 🛑 **< 0.3m**: CRITICAL - Immediate stop
- ⚠️ **< 0.5m**: WARNING - Proceed with caution  
- ⚡ **< 1.0m**: CAUTION - Reduce speed
- ℹ️ **< 2.0m**: NOTE - Relatively clear
- ✅ **≥ 2.0m**: SAFE - Clear path

---

## 2️⃣ Behavior Server (Write-Access)

### File Location
```
/home/fire/ros2_ws/src/I_CAN_Robot/ican_mcp_server/ican_mcp_server/behavior_server.py
```

### Run Command
```bash
ros2 run ican_mcp_server behavior_server
```

### MCP Tools Exposed

#### `move_robot(direction, duration, speed)`
**Moves robot linearly**
```python
move_robot("forward", duration=2.0, speed=0.5)
# Returns: "✅ Moved forward at 0.50 m/s for 2.00s (~1.00m traveled). Robot stopped."
```

**Parameters:**
- `direction`: `"forward"` | `"backward"` | `"left"` | `"right"`
- `duration`: 0.1 - 30.0 seconds (default: 1.0)
- `speed`: 0.0 - 2.0 m/s (default: 0.5)

#### `rotate_robot(direction, duration, angular_speed)`
**Rotates robot in place**
```python
rotate_robot("left", duration=2.0, angular_speed=0.785)
# Returns: "✅ Rotated left at 0.78 rad/s for 2.00s (~90.0° rotation). Robot stopped."
```

**Parameters:**
- `direction`: `"left"` (CCW) | `"right"` (CW)
- `duration`: 0.1 - 30.0 seconds (default: 1.0)
- `angular_speed`: 0.0 - 2.0 rad/s (default: 0.5)

#### `stop()`
**Emergency stop**
```python
stop()
# Returns: "🛑 Robot stopped - all velocities set to zero"
```

#### `get_movement_capabilities()`
**Query available commands**

### ROS 2 Topics Published

| Topic | Message Type | Publish Rate |
|-------|-------------|--------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz (during movement) |

### Movement Quick Reference

**Recommended Speeds:**
- Exploration: 0.2 - 0.3 m/s
- Normal: 0.4 - 0.6 m/s
- Fast: 0.7 - 1.0 m/s

**Rotation Angles:**
- 45°: ~1.0s at 0.785 rad/s
- 90°: ~2.0s at 0.785 rad/s
- 180°: ~4.0s at 0.785 rad/s
- 360°: ~8.0s at 0.785 rad/s

---

## Common Setup

### Dependencies

**ROS 2 Packages:**
- `rclpy`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`

**Python Packages:**
```bash
pip install mcp
```

### Build
```bash
cd /home/fire/ros2_ws
colcon build --packages-select ican_mcp_server
source install/setup.bash
```

### Run Both Servers
```bash
# Terminal 1 - Senses (Read)
ros2 run ican_mcp_server senses_server

# Terminal 2 - Behavior (Write)
ros2 run ican_mcp_server behavior_server
```

---

## Architecture

```
┌─────────────────────────────────────┐
│         LLM / MCP Client            │
└─────────────┬───────────┬───────────┘
              │           │
    ┌─────────┘           └─────────┐
    │                               │
┌───▼──────────────┐    ┌───────────▼────────┐
│ Senses Server    │    │ Behavior Server    │
│ (Read-Only)      │    │ (Write-Access)     │
│                  │    │                    │
│ • check_obstacles│    │ • move_robot       │
│ • get_status     │    │ • rotate_robot     │
└───┬──────────────┘    │ • stop             │
    │                   └───┬────────────────┘
    │                       │
┌───▼───────────────────────▼────────────┐
│         ROS 2 Middleware               │
│                                        │
│  Topics:                               │
│  /scan ──→ [read]                     │
│  /battery_state ──→ [read]            │
│  /odom ──→ [read]                     │
│  /cmd_vel ←── [write]                 │
└────────────────────────────────────────┘
```

---

## Safety Features

**Senses Server:**
✅ Read-only access  
✅ Data freshness monitoring (5s)  
✅ Thread-safe caching

**Behavior Server:**
✅ Auto-stop after every command  
✅ Duration limits (max 30s)  
✅ Speed limits (max 2.0 m/s)  
✅ Emergency stop function  
✅ Input validation  
✅ Continuous 10 Hz publishing

---

## Testing

### Check Topics
```bash
# Senses Server Topics
ros2 topic hz /scan
ros2 topic hz /battery_state  
ros2 topic hz /odom

# Behavior Server Topics
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
```

### Monitor Nodes
```bash
ros2 node list
ros2 node info /unitree_senses_node
ros2 node info /unitree_behavior_node
```

---

## MCP Configuration

Add to Claude Desktop or MCP client config:

```json
{
  "mcpServers": {
    "unitree-senses": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "senses_server"],
      "env": {"ROS_DOMAIN_ID": "0"}
    },
    "unitree-behaviors": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "behavior_server"],
      "env": {"ROS_DOMAIN_ID": "0"}
    }
  }
}
```

---

## Example LLM Workflow

```
1. Check surroundings:
   check_obstacles() → "✅ Safe: 3.5m clear"
   
2. Check battery:
   get_robot_status() → "Battery: 92% | Speed: 0.00 m/s"
   
3. Move forward if safe:
   move_robot("forward", 2.0, 0.5) → "✅ Moved forward..."
   
4. Emergency if needed:
   stop() → "🛑 Robot stopped"
```
