#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


DEFAULT_X = 1.440033
DEFAULT_Y = -1.425135
DEFAULT_Z = 0.382595  # SimpleNav ignores z; kept for traceability


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node('simple_nav_goal_sender')

    goal_topic = node.declare_parameter('goal_topic', '/goal').value
    x = float(node.declare_parameter('x', DEFAULT_X).value)
    y = float(node.declare_parameter('y', DEFAULT_Y).value)
    z = float(node.declare_parameter('z', DEFAULT_Z).value)

    pub = node.create_publisher(Point, goal_topic, 10)
    msg = Point(x=x, y=y, z=z)

    # Publish a few times to ensure the goal is received even if the nav node starts slightly later.
    publishes = {'count': 0}

    def _tick() -> None:
        publishes['count'] += 1
        pub.publish(msg)
        node.get_logger().info(
            f"Published goal ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) to {goal_topic} [{publishes['count']}/5]"
        )
        if publishes['count'] >= 5:
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()

    timer = node.create_timer(0.25, _tick)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
