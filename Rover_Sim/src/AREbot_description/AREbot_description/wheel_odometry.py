#!/usr/bin/env python3
"""Differential-drive wheel odometry from ESP32 encoder ticks.

Subscribes to ``/wheel_telemetry`` (Float64MultiArray ``[tickA, tickB,
rpmA, rpmB]`` published by esp32_bridge) and integrates the wheel ticks into
``nav_msgs/Odometry`` on ``/odom`` plus the ``odom -> base_footprint`` TF.

The robot's Motor A / encoder A is on the RIGHT side (``motor_a_is_right``),
so tickA is the right wheel and tickB is the left wheel.

Run:
    ros2 run AREbot_description wheel_odometry
"""
import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


DEFAULTS = {
    'telemetry_topic': '/wheel_telemetry',
    'odom_topic': '/odom',
    'odom_frame': 'odom',
    'base_frame': 'base_footprint',
    'wheel_radius': 0.0335,
    'wheel_separation': 0.20,
    'ticks_per_rev_left': 374.0,
    'ticks_per_rev_right': 374.0,
    'left_tick_sign': 1.0,
    'right_tick_sign': 1.0,
    'motor_a_is_right': True,
    'publish_tf': True,
    'publish_rate': 25.0,
    'max_tick_jump': 1000.0,
    'cmd_vel_topic': '/cmd_vel',
    'arm_on_cmd_vel': True,
}


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self._declare_parameters()
        self._read_parameters()

        # With arm_on_cmd_vel, the odometry stays at the origin (re-baselining
        # the encoder ticks) until the first non-zero /cmd_vel arrives, so a
        # boot/serial-open twitch does not shift the odometry origin.
        self.armed = not self.arm_on_cmd_vel

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0

        self.prev_left = None
        self.prev_right = None
        self.last_time = None
        self.data_stale_logged = False

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = (
            TransformBroadcaster(self) if self.publish_tf else None)

        self.create_subscription(
            Float64MultiArray, self.telemetry_topic, self._telemetry_cb, 10)
        self.create_subscription(
            Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

        self.get_logger().info(
            f'wheel_odometry: {self.telemetry_topic} -> {self.odom_topic} '
            f'(r={self.wheel_radius}, sep={self.wheel_separation}, '
            f'tpr=({self.ticks_per_rev_left},{self.ticks_per_rev_right}), '
            f'motor_a_is_right={self.motor_a_is_right}, '
            f'publish_tf={self.publish_tf}, rate={self.publish_rate}Hz, '
            f'arm_on_cmd_vel={self.arm_on_cmd_vel})')

    # ------------------------------------------------------------------
    def _declare_parameters(self):
        yaml_params = self._load_yaml_defaults()
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, yaml_params.get(name, default))

    def _load_yaml_defaults(self):
        if yaml is None:
            return {}
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(
                get_package_share_directory('AREbot_description'),
                'config', 'wheel_odometry.yaml')
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            return (data or {}).get('wheel_odometry', {}).get(
                'ros__parameters', {}) or {}
        except Exception:
            return {}

    def _read_parameters(self):
        self.telemetry_topic = self.get_parameter('telemetry_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.ticks_per_rev_left = float(
            self.get_parameter('ticks_per_rev_left').value)
        self.ticks_per_rev_right = float(
            self.get_parameter('ticks_per_rev_right').value)
        self.left_tick_sign = float(
            self.get_parameter('left_tick_sign').value)
        self.right_tick_sign = float(
            self.get_parameter('right_tick_sign').value)
        self.motor_a_is_right = bool(
            self.get_parameter('motor_a_is_right').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.max_tick_jump = float(self.get_parameter('max_tick_jump').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arm_on_cmd_vel = bool(
            self.get_parameter('arm_on_cmd_vel').value)

    # ------------------------------------------------------------------
    def _telemetry_cb(self, msg):
        if len(msg.data) < 4:
            return
        tick_a = float(msg.data[0])
        tick_b = float(msg.data[1])

        if self.motor_a_is_right:
            tick_right, tick_left = tick_a, tick_b
        else:
            tick_right, tick_left = tick_b, tick_a

        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        if self.prev_left is None:
            self.prev_left = tick_left
            self.prev_right = tick_right
            self.last_time = now_s
            return

        if self.arm_on_cmd_vel and not self.armed:
            # No motion command yet: keep re-baselining so boot/serial-open
            # twitches do not move the odometry origin.
            self.prev_left = tick_left
            self.prev_right = tick_right
            self.last_time = now_s
            self.vx = 0.0
            self.wz = 0.0
            return

        d_ticks_left = tick_left - self.prev_left
        d_ticks_right = tick_right - self.prev_right

        # Detect ESP32 reset / reconnect (absurd jump): re-seed and skip.
        if (abs(d_ticks_left) > self.max_tick_jump
                or abs(d_ticks_right) > self.max_tick_jump):
            self.get_logger().warn(
                'Lompatan tick besar terdeteksi (reset/reconnect ESP32); '
                'odometri di-reset ke tick saat ini.')
            self.prev_left = tick_left
            self.prev_right = tick_right
            self.last_time = now_s
            return

        dt = now_s - self.last_time
        if dt <= 0.0:
            dt = 1.0 / self.publish_rate
        if dt > 1.0:
            # Avoid huge integration steps after a long pause.
            dt = 1.0

        mpt_left = 2.0 * math.pi * self.wheel_radius / max(
            self.ticks_per_rev_left, 1.0)
        mpt_right = 2.0 * math.pi * self.wheel_radius / max(
            self.ticks_per_rev_right, 1.0)

        d_left = d_ticks_left * self.left_tick_sign * mpt_left
        d_right = d_ticks_right * self.right_tick_sign * mpt_right

        d = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_separation

        self.x += d * math.cos(self.theta + d_theta / 2.0)
        self.y += d * math.sin(self.theta + d_theta / 2.0)
        self.theta = normalize_angle(self.theta + d_theta)

        self.vx = d / dt
        self.wz = d_theta / dt

        self.prev_left = tick_left
        self.prev_right = tick_right
        self.last_time = now_s
        self.data_stale_logged = False

    # ------------------------------------------------------------------
    def _cmd_vel_cb(self, msg):
        if self.armed:
            return
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.armed = True
            self.prev_left = None
            self.get_logger().info(
                'wheel_odometry: armed on first cmd_vel; '
                'integrating encoder ticks')

    def _publish(self):
        if self.prev_left is None:
            return

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.theta)

        pose_cov = [0.0] * 36
        pose_cov[0] = 0.02
        pose_cov[7] = 0.02
        pose_cov[14] = 1e6
        pose_cov[21] = 1e6
        pose_cov[28] = 1e6
        pose_cov[35] = 0.05
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.01
        twist_cov[7] = 1e6
        twist_cov[14] = 1e6
        twist_cov[21] = 1e6
        twist_cov[28] = 1e6
        twist_cov[35] = 0.02
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = yaw_to_quat(self.theta)
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
