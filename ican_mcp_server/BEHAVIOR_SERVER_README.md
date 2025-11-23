# Behavior Server - MCP Server for Robot Movement Control

## Overview

The `behavior_server.py` is an MCP (Model Context Protocol) server that provides **write-access** to control the Unitree robot's movement. It bridges ROS 2 motion control with the MCP protocol, allowing AI assistants to command the robot safely.

## Architecture

```
┌─────────────────────────────────────────┐
│         MCP Server (Main Thread)        │
│                                         │
│  ┌─────────────────────────────────┐   │
│  │  MCP Tools (LLM Interface)     │   │
│  │  • move_robot()                 │   │
│  │  • rotate_robot()               │   │
│  │  • stop()                        │   │
│  │  • get_movement_capabilities()  │   │
│  └─────────────────────────────────┘   │
│              ↓                          │
│  ┌─────────────────────────────────┐   │
│  │  UnitreeBehaviorNode            │   │
│  │  • publish_twist()              │   │
│  │  • publish_continuous()         │   │
│  │  • stop_robot()                 │   │
│  └─────────────────────────────────┘   │
│              ↓                          │
└──────────────┼──────────────────────────┘
               ↓
┌──────────────┼──────────────────────────┐
│         /cmd_vel Publisher               │
│     (geometry_msgs/Twist)                │
│                                          │
│  Publishes at 10 Hz during movement     │
└──────────────────────────────────────────┘
```

## Key Features

### 1. **Continuous Command Publishing**
- Commands are published at 10 Hz throughout the movement duration
- Prevents robot from stopping due to command timeout
- Automatic stop command sent after duration expires

### 2. **Thread-Safe Publishing**
- Lock-based synchronization for concurrent access
- Background ROS 2 spinner thread
- Safe for multi-threaded MCP server environment

### 3. **Safety Features**
- Duration limits (0.1 - 30 seconds)
- Speed limits (0 - 2.0 m/s)
- Automatic stop after every movement
- Emergency stop function
- Input validation and error handling

### 4. **MCP Tool Interface**
Four MCP tools exposed for LLM control:

#### `move_robot(direction, duration, speed)` → str
Moves the robot linearly in a specified direction.

**Parameters:**
- `direction`: `"forward"` | `"backward"` | `"left"` | `"right"`
- `duration`: Movement time in seconds (default: 1.0, max: 30.0)
- `speed`: Linear velocity in m/s (default: 0.5, max: 2.0)

**Example:**
```python
move_robot("forward", duration=2.0, speed=0.5)
# Returns: "✅ Moved forward at 0.50 m/s for 2.00s (~1.00m traveled). Robot stopped."
```

#### `rotate_robot(direction, duration, angular_speed)` → str
Rotates the robot in place.

**Parameters:**
- `direction`: `"left"` (CCW) | `"right"` (CW)
- `duration`: Rotation time in seconds (default: 1.0, max: 30.0)
- `angular_speed`: Angular velocity in rad/s (default: 0.5, max: 2.0)

**Example:**
```python
rotate_robot("left", duration=2.0, angular_speed=0.785)
# Returns: "✅ Rotated left (counter-clockwise) at 0.78 rad/s for 2.00s (~90.0° rotation). Robot stopped."
```

#### `stop()` → str
Emergency stop - immediately halts all movement.

**Example:**
```python
stop()
# Returns: "🛑 Robot stopped - all velocities set to zero"
```

#### `get_movement_capabilities()` → str
Returns detailed information about available commands and capabilities.

## ROS 2 Publishing

### `/cmd_vel` (geometry_msgs/Twist)
**Publisher Configuration:**
- Queue size: 10
- Publishing rate: 10 Hz (during movement)
- Thread-safe with locking

**Twist Message Structure:**
```python
geometry_msgs.msg.Twist(
    linear=Vector3(x, y, z),    # m/s
    angular=Vector3(x, y, z)    # rad/s
)
```

## Movement Mapping

### Linear Movement
| Direction | linear.x | linear.y | Description |
|-----------|----------|----------|-------------|
| forward   | +speed   | 0.0      | Move forward |
| backward  | -speed   | 0.0      | Move backward |
| left      | 0.0      | +speed   | Strafe left (if supported) |
| right     | 0.0      | -speed   | Strafe right (if supported) |

### Rotational Movement
| Direction | angular.z | Description |
|-----------|-----------|-------------|
| left      | +speed    | Counter-clockwise rotation |
| right     | -speed    | Clockwise rotation |

## Safety Parameters

### Speed Limits
```python
LINEAR_SPEED_RANGE = (0.0, 2.0)      # m/s
ANGULAR_SPEED_RANGE = (0.0, 2.0)     # rad/s
RECOMMENDED_LINEAR = (0.3, 0.8)      # m/s
RECOMMENDED_ANGULAR = (0.3, 0.8)     # rad/s
```

### Duration Limits
```python
DURATION_RANGE = (0.1, 30.0)         # seconds
```

### Automatic Safety Features
- ✅ All movements automatically stop after duration
- ✅ Zero velocity command sent after every movement
- ✅ Input validation prevents invalid commands
- ✅ Emergency stop available at all times
- ✅ Error handling with automatic stop on failure

## Installation

### Dependencies

Already included in `package.xml`:
```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
```

### Python Dependencies
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

# Run the behavior server
ros2 run ican_mcp_server behavior_server
```

### Expected Output
```
[INFO] [unitree_behavior_node]: UnitreeBehavior node initialized - publishing to /cmd_vel
[INFO] [unitree_behavior_node]: Starting MCP server for UnitreeBehaviors...
```

### Testing Commands

#### Test Movement
```bash
# In another terminal, check if cmd_vel is being published
ros2 topic echo /cmd_vel

# Check publishing rate
ros2 topic hz /cmd_vel
```

#### Monitor Robot
```bash
# Watch the robot's state
ros2 topic echo /odom

# Check for any issues
ros2 node info /unitree_behavior_node
```

## Integration with LLM

Add to your MCP configuration (e.g., Claude Desktop):

```json
{
  "mcpServers": {
    "unitree-behaviors": {
      "command": "ros2",
      "args": ["run", "ican_mcp_server", "behavior_server"],
      "env": {
        "ROS_DOMAIN_ID": "0"
      }
    }
  }
}
```

## Example LLM Interactions

### Basic Movement Sequence
```
LLM: "Move forward 1 meter, then turn left 90 degrees"

Executes:
1. move_robot("forward", duration=2.0, speed=0.5)
2. rotate_robot("left", duration=2.0, angular_speed=0.785)
```

### Exploration Pattern
```
LLM: "Scan the area by rotating 360 degrees slowly"

Executes:
1. rotate_robot("left", duration=8.0, angular_speed=0.785)
   # 8 seconds * 0.785 rad/s ≈ 360 degrees
```

### Emergency Response
```
LLM: "Stop immediately!"

Executes:
1. stop()
```

## Code Structure

### Classes

**`UnitreeBehaviorNode`**: ROS 2 node for movement control
- `publish_twist()`: Single Twist message publication
- `publish_continuous()`: Continuous publishing for duration
- `stop_robot()`: Emergency stop function

### Functions

**`ros_spin_thread()`**: Background thread for ROS 2 executor
**`main()`**: Entry point coordinating ROS 2 and MCP server

### MCP Tools

- **`@mcp.tool() move_robot()`**: Linear movement control
- **`@mcp.tool() rotate_robot()`**: Rotational movement control
- **`@mcp.tool() stop()`**: Emergency stop
- **`@mcp.tool() get_movement_capabilities()`**: Capability query

## Concurrency Model

1. **Main Thread**: Runs `mcp.run()` - handles MCP requests
2. **Background Thread**: Runs `rclpy.spin()` - maintains ROS 2 node
3. **Synchronization**: `threading.Lock` in `publish_twist()` ensures safe publishing

## Algorithm: Continuous Publishing

```python
def publish_continuous(linear_x, angular_z, duration, rate=10Hz):
    iterations = duration * rate
    sleep_time = 1 / rate
    
    for i in range(iterations):
        publish_twist(linear_x, angular_z)
        sleep(sleep_time)
    
    # Auto-stop
    publish_twist(0, 0)
```

This ensures the robot receives constant velocity commands, as most robots will timeout and stop if they don't receive regular updates.

## Error Handling

### Input Validation
```python
# Duration check
if duration <= 0 or duration > 30:
    return "❌ Error: Duration must be between 0 and 30 seconds"

# Speed check
if speed <= 0 or speed > 2.0:
    return "❌ Error: Speed must be between 0 and 2.0 m/s"

# Direction check
if direction not in ["forward", "backward", "left", "right"]:
    return "❌ Error: Invalid direction"
```

### Exception Handling
```python
try:
    behavior_node.publish_continuous(...)
    return "✅ Success message"
except Exception as e:
    behavior_node.stop_robot()  # Safety stop
    return f"❌ Error: {str(e)}"
```

## Troubleshooting

### Robot Not Moving
**Problem:** Commands sent but robot doesn't move

**Solutions:**
1. Check if `/cmd_vel` is being published:
   ```bash
   ros2 topic hz /cmd_vel
   ```

2. Verify robot is listening to `/cmd_vel`:
   ```bash
   ros2 node list
   ros2 node info /robot_base_controller
   ```

3. Check for emergency stop or safety system engaged

### Commands Timing Out
**Problem:** Robot stops mid-movement

**Solution:** The `publish_continuous()` function already handles this by publishing at 10 Hz. If issues persist, increase the rate:
```python
publish_rate = 20.0  # Hz
```

### Import Errors
**Problem:** `ModuleNotFoundError: No module named 'mcp'`

**Solution:**
```bash
pip install mcp
```

**Problem:** `ModuleNotFoundError: No module named 'geometry_msgs'`

**Solution:** Ensure ROS 2 workspace is sourced:
```bash
source /home/fire/ros2_ws/install/setup.bash
```

## Best Practices for LLM Integration

### 1. Speed Selection
- **Slow exploration**: 0.2 - 0.3 m/s
- **Normal movement**: 0.4 - 0.6 m/s
- **Fast movement**: 0.7 - 1.0 m/s
- **Maximum**: 1.0+ m/s (use with caution)

### 2. Duration Estimation
```python
# Distance-based
desired_distance = 2.0  # meters
speed = 0.5  # m/s
duration = desired_distance / speed  # 4.0 seconds

# Angle-based rotation
desired_angle = 90  # degrees
angle_rad = desired_angle * 3.14159 / 180  # 1.57 rad
angular_speed = 0.785  # rad/s
duration = angle_rad / angular_speed  # 2.0 seconds
```

### 3. Safe Movement Patterns
```python
# Always check obstacles first (use senses_server)
obstacle_status = check_obstacles()
if "Safe" in obstacle_status:
    move_robot("forward", 2.0, 0.5)
else:
    # Don't move or choose alternate path
```

### 4. Sequential Movements
```python
# Allow time between commands
move_robot("forward", 2.0, 0.5)
# Robot auto-stops, safe to issue next command
rotate_robot("left", 1.5, 0.5)
```

## Performance Characteristics

- **Publishing Rate**: 10 Hz (100ms intervals)
- **Command Latency**: < 50ms
- **Stop Response Time**: Immediate (next publish cycle)
- **Max Duration**: 30 seconds (safety limit)

## Future Enhancements

- [ ] Path planning integration
- [ ] Obstacle avoidance during movement
- [ ] Speed ramping (acceleration/deceleration)
- [ ] Movement presets (square pattern, figure-8, etc.)
- [ ] Odometry-based position tracking
- [ ] Configurable publishing rate
- [ ] Movement history logging

## License

Apache-2.0

## Author

I_CAN Robot Project Team
