#!/usr/bin/env python3
"""
Senses Server - MCP Server for Robot Sensor Data

This node acts as an MCP Server that gives an LLM read-only access to the robot's sensors.
It subscribes to various sensor topics and exposes them through MCP tools.

Architecture:
- ROS 2 node that runs in a background thread
- FastMCP server as the main process
- Caches sensor data for quick access by MCP tools

Author: I_CAN Robot Project
License: Apache-2.0
"""

import threading
import time
from typing import Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry

from mcp.server.fastmcp import FastMCP


class SensorCache:
    """Thread-safe cache for sensor data"""
    
    def __init__(self):
        self._lock = threading.Lock()
        self._battery_percentage: Optional[float] = None
        self._closest_obstacle: Optional[float] = None
        self._linear_velocity: Optional[float] = None
        self._last_scan_time: Optional[float] = None
        self._last_battery_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
    
    def update_battery(self, percentage: float):
        with self._lock:
            self._battery_percentage = percentage
            self._last_battery_time = time.time()
    
    def update_obstacle(self, distance: float):
        with self._lock:
            self._closest_obstacle = distance
            self._last_scan_time = time.time()
    
    def update_velocity(self, velocity: float):
        with self._lock:
            self._linear_velocity = velocity
            self._last_odom_time = time.time()
    
    def get_battery(self) -> tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self._battery_percentage, self._last_battery_time
    
    def get_obstacle(self) -> tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self._closest_obstacle, self._last_scan_time
    
    def get_velocity(self) -> tuple[Optional[float], Optional[float]]:
        with self._lock:
            return self._linear_velocity, self._last_odom_time


class UnitreeSensesNode(Node):
    """ROS 2 Node for subscribing to robot sensors"""
    
    def __init__(self, sensor_cache: SensorCache):
        super().__init__('unitree_senses_node')
        self.sensor_cache = sensor_cache
        
        # Create subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('UnitreeSenses node initialized - subscribed to /scan, /battery_state, /odom')
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan to find closest obstacle"""
        # Filter out invalid readings (inf, nan, out of range)
        valid_ranges = [
            r for r in msg.ranges 
            if not math.isinf(r) and not math.isnan(r) and msg.range_min <= r <= msg.range_max
        ]
        
        if valid_ranges:
            closest = min(valid_ranges)
            self.sensor_cache.update_obstacle(closest)
            self.get_logger().debug(f'Closest obstacle: {closest:.2f}m')
        else:
            # No valid readings - could mean clear area or sensor issue
            self.sensor_cache.update_obstacle(float('inf'))
            self.get_logger().debug('No valid obstacle readings')
    
    def battery_callback(self, msg: BatteryState):
        """Cache battery percentage"""
        # BatteryState.percentage is typically 0-100
        percentage = msg.percentage
        self.sensor_cache.update_battery(percentage)
        self.get_logger().debug(f'Battery: {percentage:.1f}%')
    
    def odom_callback(self, msg: Odometry):
        """Cache current linear velocity"""
        # Extract linear velocity from odometry
        linear_vel = msg.twist.twist.linear.x
        self.sensor_cache.update_velocity(linear_vel)
        self.get_logger().debug(f'Velocity: {linear_vel:.2f} m/s')


def ros_spin_thread(node: Node, executor: SingleThreadedExecutor):
    """Thread function to spin the ROS 2 node"""
    try:
        executor.spin()
    except Exception as e:
        node.get_logger().error(f'Error in ROS spin thread: {e}')


# Initialize global sensor cache
sensor_cache = SensorCache()

# Initialize FastMCP server
mcp = FastMCP("UnitreeSenses")


@mcp.tool()
def get_robot_status() -> str:
    """
    Get the current status of the robot including battery level and speed.
    
    Returns:
        A formatted string with battery percentage and current linear velocity.
    """
    battery, battery_time = sensor_cache.get_battery()
    velocity, velocity_time = sensor_cache.get_velocity()
    
    current_time = time.time()
    status_parts = []
    
    # Battery status
    if battery is not None:
        age = current_time - battery_time if battery_time else float('inf')
        if age < 5.0:  # Data is fresh (< 5 seconds old)
            status_parts.append(f"Battery: {battery:.1f}%")
        else:
            status_parts.append(f"Battery: {battery:.1f}% (stale data, {age:.0f}s old)")
    else:
        status_parts.append("Battery: No data available")
    
    # Velocity status
    if velocity is not None:
        age = current_time - velocity_time if velocity_time else float('inf')
        if age < 5.0:
            if abs(velocity) < 0.01:
                status_parts.append("Speed: Stationary (0.00 m/s)")
            else:
                status_parts.append(f"Speed: {velocity:.2f} m/s ({abs(velocity * 3.6):.2f} km/h)")
        else:
            status_parts.append(f"Speed: {velocity:.2f} m/s (stale data, {age:.0f}s old)")
    else:
        status_parts.append("Speed: No data available")
    
    return " | ".join(status_parts)


@mcp.tool()
def check_obstacles() -> str:
    """
    Check for obstacles around the robot using laser scan data.
    
    Returns:
        A natural language description of the nearest obstacle and safety status.
    """
    obstacle_dist, scan_time = sensor_cache.get_obstacle()
    
    if obstacle_dist is None:
        return "⚠️ No obstacle data available - laser scan not receiving data"
    
    current_time = time.time()
    age = current_time - scan_time if scan_time else float('inf')
    
    # Check data freshness
    if age > 5.0:
        return f"⚠️ Obstacle data is stale ({age:.0f}s old) - last reading: {obstacle_dist:.2f}m"
    
    # Classify obstacle distance
    if math.isinf(obstacle_dist):
        return "✅ All clear - no obstacles detected within sensor range"
    elif obstacle_dist < 0.3:
        return f"🛑 CRITICAL: Obstacle detected at {obstacle_dist:.2f}m - immediate stop recommended!"
    elif obstacle_dist < 0.5:
        return f"⚠️ WARNING: Close obstacle at {obstacle_dist:.2f}m - proceed with caution"
    elif obstacle_dist < 1.0:
        return f"⚡ CAUTION: Obstacle detected at {obstacle_dist:.2f}m - reduce speed"
    elif obstacle_dist < 2.0:
        return f"ℹ️ Note: Nearest obstacle at {obstacle_dist:.2f}m - path is relatively clear"
    else:
        return f"✅ Safe: Nearest obstacle is {obstacle_dist:.2f}m away - clear path ahead"


def main():
    """Main entry point for the senses server"""
    # Initialize ROS 2
    rclpy.init()
    
    # Create the ROS 2 node with shared sensor cache
    node = UnitreeSensesNode(sensor_cache)
    
    # Create executor for the ROS 2 node
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Start ROS 2 spinning in a background thread
    ros_thread = threading.Thread(
        target=ros_spin_thread,
        args=(node, executor),
        daemon=True
    )
    ros_thread.start()
    
    node.get_logger().info('Starting MCP server for UnitreeSenses...')
    
    try:
        # Run the MCP server (this blocks)
        mcp.run()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down senses server...')
    finally:
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
