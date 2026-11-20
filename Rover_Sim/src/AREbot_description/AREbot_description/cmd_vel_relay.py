#!/usr/bin/env python3
"""Relay Twist commands from /cmd_vel to the diff_drive_controller input topic.

Replaces `topic_tools relay`, which in this environment subscribes with
TRANSIENT_LOCAL durability and is therefore incompatible with Nav2's VOLATILE
/cmd_vel publishers (messages are silently dropped, so the robot never moves).

This relay subscribes with the default VOLATILE durability, which is compatible
with both VOLATILE publishers (Nav2, teleop) and TRANSIENT_LOCAL publishers
(e.g. auto_localize).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter(
            'output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(Twist, out_topic, qos)
        self.sub = self.create_subscription(Twist, in_topic, self._cb, qos)
        self.get_logger().info(f'cmd_vel_relay: {in_topic} -> {out_topic}')

    def _cb(self, msg):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
