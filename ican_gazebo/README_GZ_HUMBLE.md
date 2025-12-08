# Gazebo Humble Launch Files

## Overview

Two launch configurations for the I_CAN Robot with Gazebo Humble:

1. **gz_humble_local_launch.py** - Local robot communication (no visualizers)
2. **gz_humble_remote_launch.py** - Remote digital twin with full visualization suite

## Launch Files

### Local Launch (Robot Communication Only)

**Purpose**: Run on computer connected to physical robot. No debug visualizers.

**What it runs**:
- Robot State Publisher
- Go2 Driver Node (robot communication)
- Joystick control
- Teleop Twist Joy
- Twist Mux (velocity command manager)
- Lidar processing nodes
- Pointcloud to Laserscan converter
- Image compression

**Usage**:
```bash
# Set environment variables
export ROBOT_IP="192.168.123.161"
export ROBOT_TOKEN="your_token_here"
export CONN_TYPE="webrtc"

# Launch
ros2 launch ican_gazebo gz_humble_local_launch.py
```

### Remote Launch (Digital Twin + Visualizers)

**Purpose**: Run on visualization computer. Does NOT connect to physical robot.

**What it runs**:
- Gazebo Classic (empty world)
- Robot spawned as digital twin
- RViz2 (with go2_ros2_sdk config)
- Foxglove Bridge (web visualization)
- rqt_graph (node/topic graph)
- TF static transforms
- Joint State Publisher
- Robot State Publisher

**Usage**:
```bash
ros2 launch ican_gazebo gz_humble_remote_launch.py
```

**Optional arguments**:
```bash
ros2 launch ican_gazebo gz_humble_remote_launch.py \
    world_init_x:=1.0 \
    world_init_y:=2.0 \
    world_init_z:=0.5 \
    world_init_heading:=1.57
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select ican_gazebo
source install/setup.bash
```

## Architecture

### Local (Robot Communication)
```
Physical Robot <--WebRTC--> go2_driver_node
                                  |
                            Robot Topics
                            (no visualizers)
```

### Remote (Digital Twin)
```
Gazebo Sim <--> Robot State Publisher <--> Visualizers:
                                              - RViz2
                                              - Foxglove
                                              - rqt_graph
```

## Dependencies

**Local requires**:
- go2_robot_sdk
- lidar_processor_cpp
- pointcloud_to_laserscan
- image_transport
- joy
- teleop_twist_joy
- twist_mux

**Remote requires**:
- gazebo_ros
- rviz2
- foxglove_bridge
- rqt_graph
- joint_state_publisher
- tf2_ros

## Typical Use Case

Run both simultaneously for development/testing:

**Terminal 1 (Local - connected to robot)**:
```bash
export ROBOT_IP="192.168.123.161"
ros2 launch ican_gazebo gz_humble_local_launch.py
```

**Terminal 2 (Remote - visualization)**:
```bash
ros2 launch ican_gazebo gz_humble_remote_launch.py
```

This setup allows you to:
- Control physical robot from local computer
- Visualize robot state on remote computer
- See digital twin mirror robot movements
- Debug with RViz, Foxglove, and rqt_graph

## Notes

- **gz_jazzy_launch.py** is for a different repository and uses Gazebo Sim (not Gazebo Classic)
- Local launch uses `use_sim_time:=false` (physical robot time)
- Remote launch uses `use_sim_time:=true` (simulation time)
- Foxglove Bridge runs on port 8765 (access via http://localhost:8765)

## Troubleshooting

**Local launch can't find robot**:
- Check `ROBOT_IP` environment variable
- Verify robot is powered on and connected
- Test connection: `ping $ROBOT_IP`

**Remote launch Gazebo doesn't start**:
- Verify Gazebo Classic is installed: `gazebo --version`
- Check world file exists: `ls $(ros2 pkg prefix ican_gazebo)/share/ican_gazebo/worlds/empty.world`

**RViz config not found**:
- Ensure go2_robot_sdk is built and sourced
- Check: `ros2 pkg prefix go2_robot_sdk`
