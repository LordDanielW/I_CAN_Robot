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
from rclpy.node import Node
from nav_msgs.msg import Odometry
from go2_interfaces.msg import WebRtcReq

from go2_robot_sdk.application.utils.command_generator import gen_mov_command

from .utils import quaternion_to_yaw, wrap_angle, clamp

SPORT_TOPIC = 'rt/api/sport/request'
OBSTACLE_TOPIC = 'rt/api/obstacles_avoid/request'
API_SPORT_MOVE = 1008
API_OA_MOVE = 1003


class JoystickGoalWebRTC(Node):
    def __init__(self) -> None:
        super().__init__('joystick_goal_webrtc')

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

        self.pose: Optional[tuple] = None

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.pub = self.create_publisher(WebRtcReq, 'webrtc_req', 10)
        self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self._loop)

        # One-shot timer to publish OA-enable request after startup (give driver time to subscribe)
        if self.auto_enable_oa and self.obstacle_avoidance:
            self._enable_timer = self.create_timer(0.5, self._send_enable_oa)

        self.get_logger().info(f"JoystickGoalWebRTC ready. Goal=({self.goal_x:.3f}, {self.goal_y:.3f}), robot_id={self.robot_id}, OA={self.obstacle_avoidance}")

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pose = (pos.x, pos.y, yaw)

    def _loop(self) -> None:
        # periodic loop: publish simulated joystick and send high-level commands
        if self.pose is None:
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

        target_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(target_heading - yaw)

        # simple proportional controllers
        k_lin = 1.0
        k_ang = 1.5

        v = clamp(k_lin * dist, -self.max_speed, self.max_speed)
        w = clamp(k_ang * heading_error, -self.max_turn, self.max_turn)

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
