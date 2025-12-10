#!/usr/bin/env python3
"""Publish High-Level motion WebRTC requests to drive to a goal using SDK API.

This node computes a simple controller to drive the robot to a goal, but instead
of publishing raw velocities it emits `go2_interfaces/msg/WebRtcReq` messages
that the `go2_driver_node` will forward through the SDK's high-level command
path. This preserves obstacle avoidance and gait handling.

Parameters:
- goal_x, goal_y: target position
- goal_tolerance: distance to consider goal reached
- max_speed, max_turn: caps for velocities
- rate: publish frequency
- robot_id: string id forwarded to driver (default '0')
- obstacle_avoidance: bool (default True) - use obstacle-aware API

Behavior: publishes `WebRtcReq` messages (api_id + parameter JSON + topic)
until within tolerance, then sends a zero-motion request and exits.
"""

from typing import Optional
import math
import json

import rclpy
import signal
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from go2_interfaces.msg import WebRtcReq


from go2_robot_sdk.application.utils.command_generator import gen_mov_command
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

from .utils import quaternion_to_yaw, wrap_angle, clamp

SPORT_TOPIC = 'rt/api/sport/request'
OBSTACLE_TOPIC = 'rt/api/obstacles_avoid/request'
API_SPORT_MOVE = 1008
API_OA_MOVE = 1003

class JoystickGoalWebRTC(Node):
    def __init__(self) -> None:
        super().__init__('joystick_goal_webrtc')
        # Visualization publisher for RViz2
        self.marker_pub = self.create_publisher(Marker, 'goal_marker', 1)
        self.vector_pub = self.create_publisher(Vector3, 'nav_vector', 1)
        # Parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn', 0.6)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('robot_id', '0')
        self.declare_parameter('obstacle_avoidance', True)
        self.declare_parameter('auto_enable_oa', True)
        self.declare_parameter('webrtc_priority', 1)
        self.declare_parameter('repulsion_radius', 0.7)
        self.declare_parameter('repulsion_scale', 0.7)
        self.declare_parameter('attraction_scale', 4.0)  # Increase to prioritize goal more

        self.goal_x = float(self.get_parameter('goal_x').get_parameter_value().double_value)
        self.goal_y = float(self.get_parameter('goal_y').get_parameter_value().double_value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self.max_speed = float(self.get_parameter('max_speed').get_parameter_value().double_value)
        self.max_turn = float(self.get_parameter('max_turn').get_parameter_value().double_value)
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.robot_id = str(self.get_parameter('robot_id').get_parameter_value().string_value)
        self.obstacle_avoidance = bool(self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value)
        self.auto_enable_oa = bool(self.get_parameter('auto_enable_oa').get_parameter_value().bool_value)
        self.webrtc_priority = int(self.get_parameter('webrtc_priority').get_parameter_value().integer_value)
        self.repulsion_radius = float(self.get_parameter('repulsion_radius').get_parameter_value().double_value)
        self.repulsion_scale = float(self.get_parameter('repulsion_scale').get_parameter_value().double_value)
        self.attraction_scale = float(self.get_parameter('attraction_scale').get_parameter_value().double_value)

        self.pose: Optional[tuple] = None
        self.scan: Optional[LaserScan] = None
        self.pose = None  # type: ignore
        self.scan = None  # type: ignore

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        scan_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.pub = self.create_publisher(WebRtcReq, 'webrtc_req', 10)
        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self._loop)

        # One-shot timer to publish OA-enable request after startup (give driver time to subscribe)
        if self.auto_enable_oa and self.obstacle_avoidance:
            self._enable_timer = self.create_timer(0.5, self._send_enable_oa)

        self.get_logger().info(f"JoystickGoalWebRTC ready. Goal=({self.goal_x:.3f}, {self.goal_y:.3f}), robot_id={self.robot_id}, OA={self.obstacle_avoidance}")

    def _publish_goal_marker(self):
        # Publish goal as a sphere marker
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.2
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.marker_pub.publish(marker)

    def _publish_vector(self, vec_x, vec_y):
        # Publish navigation vector as a Vector3 message
        vector = Vector3()
        vector.x = vec_x
        vector.y = vec_y
        vector.z = 0.0
        self.vector_pub.publish(vector)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_turn', 0.6)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('robot_id', '0')
        self.declare_parameter('obstacle_avoidance', True)
        self.declare_parameter('auto_enable_oa', True)
        self.declare_parameter('webrtc_priority', 1)
        self.declare_parameter('repulsion_radius', 0.7)
        self.declare_parameter('repulsion_scale', 0.7)
        self.declare_parameter('attraction_scale', 4.0)  # Increase to prioritize goal more

        self.goal_x = float(self.get_parameter('goal_x').get_parameter_value().double_value)
        self.goal_y = float(self.get_parameter('goal_y').get_parameter_value().double_value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').get_parameter_value().double_value)
        self.max_speed = float(self.get_parameter('max_speed').get_parameter_value().double_value)
        self.max_turn = float(self.get_parameter('max_turn').get_parameter_value().double_value)
        self.rate = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.robot_id = str(self.get_parameter('robot_id').get_parameter_value().string_value)
        self.obstacle_avoidance = bool(self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value)
        self.auto_enable_oa = bool(self.get_parameter('auto_enable_oa').get_parameter_value().bool_value)
        self.webrtc_priority = int(self.get_parameter('webrtc_priority').get_parameter_value().integer_value)
        self.repulsion_radius = float(self.get_parameter('repulsion_radius').get_parameter_value().double_value)
        self.repulsion_scale = float(self.get_parameter('repulsion_scale').get_parameter_value().double_value)
        self.attraction_scale = float(self.get_parameter('attraction_scale').get_parameter_value().double_value)

        self.pose: Optional[tuple] = None
        self.scan: Optional[LaserScan] = None
        self.pose = None  # type: ignore
        self.scan = None  # type: ignore

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        scan_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.pub = self.create_publisher(WebRtcReq, 'webrtc_req', 10)
        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self._loop)

        # One-shot timer to publish OA-enable request after startup (give driver time to subscribe)
        if self.auto_enable_oa and self.obstacle_avoidance:
            self._enable_timer = self.create_timer(0.5, self._send_enable_oa)

        self.get_logger().info(f"JoystickGoalWebRTC ready. Goal=({self.goal_x:.3f}, {self.goal_y:.3f}), robot_id={self.robot_id}, OA={self.obstacle_avoidance}")

    def _scan_cb(self, msg: LaserScan) -> None:
        self.scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pose = (pos.x, pos.y, yaw)

    def _loop(self) -> None:
        self._publish_goal_marker()
        # periodic loop: publish simulated joystick and send high-level commands
        if self.pose is None:
            self.get_logger().warn('No odometry yet.')
            return
        if self.scan is None:
            self.get_logger().warn('No scan data yet.')
            return

        x, y, yaw = self.pose
        dx = self.goal_x - x
        dy = self.goal_y - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            # publish zero-motion and exit using gen_mov_command to get canonical id/parameter
            try:
                cmd_json = gen_mov_command(0.0, 0.0, 0.0, self.obstacle_avoidance)
                cmd = json.loads(cmd_json)
                zero_req = WebRtcReq()
                zero_req.id = int(cmd['data']['header']['identity']['id'])
                zero_req.topic = cmd.get('topic', OBSTACLE_TOPIC if self.obstacle_avoidance else SPORT_TOPIC)
                zero_req.api_id = int(cmd['data']['header']['identity']['api_id'])
                zero_req.parameter = cmd['data'].get('parameter', '')
                zero_req.priority = int(self.webrtc_priority)
            except Exception:
                zero_req = WebRtcReq()
                zero_req.id = 0
                zero_req.topic = OBSTACLE_TOPIC if self.obstacle_avoidance else SPORT_TOPIC
                zero_req.api_id = API_OA_MOVE if self.obstacle_avoidance else API_SPORT_MOVE
                if self.obstacle_avoidance:
                    params = {"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}
                else:
                    params = {"x": 0.0, "y": 0.0, "z": 0.0}
                zero_req.parameter = json.dumps(params)
                zero_req.priority = int(self.webrtc_priority)

            self.pub.publish(zero_req)
            self.get_logger().info("Goal reached; sent zero high-level motion and exiting.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        # Attractive vector to goal (robot frame)
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(target_heading - yaw)
        att_x = self.attraction_scale * math.cos(heading_error)
        att_y = self.attraction_scale * math.sin(heading_error)

        # Repulsive vector from obstacles (robot frame)
        rep_x, rep_y = 0.0, 0.0
        scan = self.scan
        for i, r in enumerate(scan.ranges):
            if r < self.repulsion_radius and r > scan.range_min:
                angle = scan.angle_min + i * scan.angle_increment
                # Repulsion in robot frame
                strength = self.repulsion_scale / (r * r)
                rep_x -= strength * math.cos(angle)
                rep_y -= strength * math.sin(angle)

        # Combine vectors
        vec_x = att_x + rep_x
        vec_y = att_y + rep_y
        v = clamp(self.max_speed * vec_x, -self.max_speed, self.max_speed)
        w = clamp(self.max_turn * vec_y, -self.max_turn, self.max_turn)

        # Prepare WebRtcReq using gen_mov_command to ensure canonical structure
        try:
            cmd_json = gen_mov_command(round(v, 2), round(0.0, 2), round(w, 2), self.obstacle_avoidance)
            cmd = json.loads(cmd_json)
            req = WebRtcReq()
            req.id = int(cmd['data']['header']['identity']['id'])
            req.topic = cmd.get('topic', OBSTACLE_TOPIC if self.obstacle_avoidance else SPORT_TOPIC)
            req.api_id = int(cmd['data']['header']['identity']['api_id'])
            req.parameter = cmd['data'].get('parameter', '')
            req.priority = int(self.webrtc_priority)
            params = json.loads(req.parameter) if req.parameter else {}
        except Exception:
            # fallback to previous behavior
            req = WebRtcReq()
            req.id = 0
            if self.obstacle_avoidance:
                req.topic = OBSTACLE_TOPIC
                req.api_id = API_OA_MOVE
                params = {"x": round(v, 2), "y": round(0.0, 2), "yaw": round(w, 2), "mode": 0}
            else:
                req.topic = SPORT_TOPIC
                req.api_id = API_SPORT_MOVE
                params = {"x": round(v, 2), "y": round(0.0, 2), "z": round(w, 2)}
            req.parameter = json.dumps(params)
            req.priority = int(self.webrtc_priority)

        self.pub.publish(req)
        self.get_logger().debug(f"Published high-level move: {params} (dist={dist:.2f})")

    def _send_enable_oa(self) -> None:
        """Send a one-off request to enable remote OA commands on the robot."""
        try:
            enable_req = WebRtcReq()
            enable_req.id = 0
            enable_req.topic = OBSTACLE_TOPIC
            enable_req.api_id = 1004
            enable_req.parameter = json.dumps({"is_remote_commands_from_api": True})
            enable_req.priority = int(self.webrtc_priority)
            self.pub.publish(enable_req)
            self.get_logger().info('Published obstacle-avoidance enable request (api_id=1004)')
        except Exception as e:
            self.get_logger().error(f'Failed to publish OA enable request: {e}')
        finally:
            try:
                self._enable_timer.cancel()
            except Exception:
                pass
        


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoystickGoalWebRTC()

    def send_zero_cmd(signum, frame):
        try:
            # Build zero OA command
            zero_req = WebRtcReq()
            zero_req.id = 0
            zero_req.topic = OBSTACLE_TOPIC
            zero_req.api_id = API_OA_MOVE
            zero_req.parameter = '{"x": 0.0, "y": 0.0, "yaw": 0.0, "mode": 0}'
            zero_req.priority = int(getattr(node, 'webrtc_priority', 1))
            node.pub.publish(zero_req)
            node.get_logger().info('Sent zero OA command on interrupt.')
        except Exception as e:
            node.get_logger().error(f'Failed to send zero OA command: {e}')
        finally:
            rclpy.shutdown()

    signal.signal(signal.SIGINT, send_zero_cmd)

    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
