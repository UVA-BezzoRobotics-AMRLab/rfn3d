#!/usr/bin/env python3
"""Publish a single PointStamped goal and exit.

Usage: publish_goal.py <topic> <frame_id> <"x y z">
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped


def main():
    topic = sys.argv[1]
    frame = sys.argv[2]
    x, y, z = [float(v) for v in sys.argv[3].split()]

    rclpy.init()
    node = rclpy.create_node("_initial_goal")
    pub = node.create_publisher(PointStamped, topic, 10)

    time.sleep(0.5)

    msg = PointStamped()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x = x
    msg.point.y = y
    msg.point.z = z

    pub.publish(msg)
    node.get_logger().info(f"Published initial goal: ({x}, {y}, {z}) on {topic}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
