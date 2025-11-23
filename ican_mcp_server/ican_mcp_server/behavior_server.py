#!/usr/bin/env python3
"""
Behavior Server - MCP Server for Robot Movement Control

This node acts as an MCP Server that gives an LLM write-access to control the robot's movement.
It publishes velocity commands to /cmd_vel and exposes movement controls through MCP tools.

Architecture:
- ROS 2 node with cmd_vel publisher
- FastMCP server as the main process
- Movement commands with duration control
- Safety features including automatic stop

Author: I_CAN Robot Project
License: Apache-2.0
"""

import threading
import time
from typing import Literal

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist

from mcp.server.fastmcp import FastMCP


class UnitreeBehaviorNode(Node):
    """ROS 2 Node for publishing movement commands"""
    
    def __init__(self):
        super().__init__('unitree_behavior_node')
        
        # Create publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Lock for thread-safe publishing
        self.publish_lock = threading.Lock()
        
        self.get_logger().info('UnitreeBehavior node initialized - publishing to /cmd_vel')
    
    def publish_twist(self, linear_x: float = 0.0, linear_y: float = 0.0, 
                      linear_z: float = 0.0, angular_x: float = 0.0, 
                      angular_y: float = 0.0, angular_z: float = 0.0):
        """
        Publish a Twist message with specified velocities.
        Thread-safe publishing.
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = linear_z
        twist.angular.x = angular_x
        twist.angular.y = angular_y
        twist.angular.z = angular_z
        
        with self.publish_lock:
            self.cmd_vel_pub.publish(twist)
            self.get_logger().debug(
                f'Published: linear=({linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}), '
                f'angular=({angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f})'
            )
    
    def publish_continuous(self, linear_x: float = 0.0, linear_y: float = 0.0,
                          angular_z: float = 0.0, duration: float = 1.0,
                          publish_rate: float = 10.0) -> None:
        """
        Publish velocity commands continuously for a specified duration.
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s) - for mecanum/omni robots
            angular_z: Rotation velocity (rad/s)
            duration: How long to publish the command (seconds)
            publish_rate: Publishing frequency (Hz)
        """
        if duration <= 0:
            self.get_logger().warning('Duration must be positive, skipping movement')
            return
        
        sleep_time = 1.0 / publish_rate
        num_iterations = int(duration * publish_rate)
        
        self.get_logger().info(
            f'Starting continuous movement: linear_x={linear_x:.2f}, '
            f'linear_y={linear_y:.2f}, angular_z={angular_z:.2f} for {duration:.2f}s'
        )
        
        # Publish commands continuously
        for i in range(num_iterations):
            self.publish_twist(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
            time.sleep(sleep_time)
        
        # Send stop command
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Movement completed, stop command sent')
    
    def stop_robot(self):
        """Immediately stop the robot by publishing zero velocities"""
        self.publish_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.get_logger().info('Emergency stop executed')


def ros_spin_thread(node: Node, executor: SingleThreadedExecutor):
    """Thread function to spin the ROS 2 node"""
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f'Error in ROS spin thread: {e}')


# Global node instance (initialized in main)
behavior_node: UnitreeBehaviorNode = None

# Initialize FastMCP server
mcp = FastMCP("UnitreeBehaviors")


@mcp.tool()
def move_robot(
    direction: Literal["forward", "backward", "left", "right"],
    duration: float = 1.0,
    speed: float = 0.5
) -> str:
    """
    Move the robot in a specified direction for a given duration.
    
    Args:
        direction: Direction to move - "forward", "backward", "left", or "right"
        duration: How long to move in seconds (default: 1.0)
        speed: Movement speed in m/s (default: 0.5, max recommended: 1.0)
    
    Returns:
        Status message describing the movement executed
    """
    if behavior_node is None:
        return "❌ Error: Behavior node not initialized"
    
    # Validate inputs
    if duration <= 0 or duration > 30:
        return f"❌ Error: Duration must be between 0 and 30 seconds (got {duration}s)"
    
    if speed <= 0 or speed > 2.0:
        return f"❌ Error: Speed must be between 0 and 2.0 m/s (got {speed} m/s)"
    
    # Map direction to velocity components
    direction = direction.lower()
    linear_x = 0.0
    linear_y = 0.0
    
    if direction == "forward":
        linear_x = speed
        direction_desc = "forward"
    elif direction == "backward":
        linear_x = -speed
        direction_desc = "backward"
    elif direction == "left":
        linear_y = speed
        direction_desc = "left"
    elif direction == "right":
        linear_y = -speed
        direction_desc = "right"
    else:
        return f"❌ Error: Invalid direction '{direction}'. Must be 'forward', 'backward', 'left', or 'right'"
    
    try:
        # Execute movement
        behavior_node.publish_continuous(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=0.0,
            duration=duration
        )
        
        distance_estimate = speed * duration
        return (
            f"✅ Moved {direction_desc} at {speed:.2f} m/s for {duration:.2f}s "
            f"(~{distance_estimate:.2f}m traveled). Robot stopped."
        )
    except Exception as e:
        behavior_node.get_logger().error(f'Error during movement: {e}')
        behavior_node.stop_robot()
        return f"❌ Error during movement: {str(e)}. Emergency stop executed."


@mcp.tool()
def rotate_robot(
    direction: Literal["left", "right"],
    duration: float = 1.0,
    angular_speed: float = 0.5
) -> str:
    """
    Rotate the robot in place in a specified direction.
    
    Args:
        direction: Rotation direction - "left" (counter-clockwise) or "right" (clockwise)
        duration: How long to rotate in seconds (default: 1.0)
        angular_speed: Rotation speed in rad/s (default: 0.5, max recommended: 1.0)
    
    Returns:
        Status message describing the rotation executed
    """
    if behavior_node is None:
        return "❌ Error: Behavior node not initialized"
    
    # Validate inputs
    if duration <= 0 or duration > 30:
        return f"❌ Error: Duration must be between 0 and 30 seconds (got {duration}s)"
    
    if angular_speed <= 0 or angular_speed > 2.0:
        return f"❌ Error: Angular speed must be between 0 and 2.0 rad/s (got {angular_speed} rad/s)"
    
    # Map direction to angular velocity
    direction = direction.lower()
    
    if direction == "left":
        angular_z = angular_speed  # Counter-clockwise (positive)
        direction_desc = "left (counter-clockwise)"
    elif direction == "right":
        angular_z = -angular_speed  # Clockwise (negative)
        direction_desc = "right (clockwise)"
    else:
        return f"❌ Error: Invalid direction '{direction}'. Must be 'left' or 'right'"
    
    try:
        # Execute rotation
        behavior_node.publish_continuous(
            linear_x=0.0,
            linear_y=0.0,
            angular_z=angular_z,
            duration=duration
        )
        
        # Calculate approximate angle rotated (in degrees)
        angle_radians = angular_speed * duration
        angle_degrees = angle_radians * 180.0 / 3.14159
        
        return (
            f"✅ Rotated {direction_desc} at {angular_speed:.2f} rad/s for {duration:.2f}s "
            f"(~{angle_degrees:.1f}° rotation). Robot stopped."
        )
    except Exception as e:
        behavior_node.get_logger().error(f'Error during rotation: {e}')
        behavior_node.stop_robot()
        return f"❌ Error during rotation: {str(e)}. Emergency stop executed."


@mcp.tool()
def stop() -> str:
    """
    Immediately stop the robot by sending zero velocity commands.
    
    This is an emergency stop function that halts all movement.
    
    Returns:
        Status message confirming the stop
    """
    if behavior_node is None:
        return "❌ Error: Behavior node not initialized"
    
    try:
        behavior_node.stop_robot()
        return "🛑 Robot stopped - all velocities set to zero"
    except Exception as e:
        behavior_node.get_logger().error(f'Error during stop: {e}')
        return f"❌ Error executing stop command: {str(e)}"


@mcp.tool()
def get_movement_capabilities() -> str:
    """
    Get information about the robot's movement capabilities and available commands.
    
    Returns:
        Description of available movement commands and their parameters
    """
    return """
🤖 Unitree Robot Movement Capabilities:

📍 Linear Movement (move_robot):
   • Directions: forward, backward, left, right
   • Speed range: 0.0 - 2.0 m/s (recommended: 0.3 - 0.8 m/s)
   • Duration range: 0.1 - 30.0 seconds
   • Example: move_robot("forward", duration=2.0, speed=0.5)

🔄 Rotation (rotate_robot):
   • Directions: left (CCW), right (CW)
   • Angular speed: 0.0 - 2.0 rad/s (recommended: 0.3 - 0.8 rad/s)
   • Duration range: 0.1 - 30.0 seconds
   • Note: ~1.57 rad/s for 2s ≈ 180° turn
   • Example: rotate_robot("left", duration=1.5, angular_speed=0.5)

🛑 Emergency Stop (stop):
   • Immediately halts all movement
   • No parameters required
   • Example: stop()

⚠️ Safety Notes:
   • All commands automatically stop the robot after duration expires
   • Always check surroundings before commanding movement
   • Use stop() if unexpected obstacles appear
   • Start with low speeds when testing
"""


def main():
    """Main entry point for the behavior server"""
    global behavior_node
    
    # Initialize ROS 2
    rclpy.init()
    
    # Create the ROS 2 node
    behavior_node = UnitreeBehaviorNode()
    
    # Create executor for the ROS 2 node
    executor = SingleThreadedExecutor()
    executor.add_node(behavior_node)
    
    # Start ROS 2 spinning in a background thread
    ros_thread = threading.Thread(
        target=ros_spin_thread,
        args=(behavior_node, executor),
        daemon=True
    )
    ros_thread.start()
    
    behavior_node.get_logger().info('Starting MCP server for UnitreeBehaviors...')
    
    try:
        # Run the MCP server (this blocks)
        mcp.run()
    except KeyboardInterrupt:
        behavior_node.get_logger().info('Shutting down behavior server...')
    finally:
        # Ensure robot is stopped before shutdown
        behavior_node.stop_robot()
        
        # Cleanup
        executor.shutdown()
        behavior_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
