#!/usr/bin/env python3
"""
Dashboard Telemetry Aggregator Node for DESPRO Rover.

Subscribes to relevant ROS 2 topics:
  - /odom (nav_msgs/msg/Odometry)
  - /amcl_pose (geometry_msgs/msg/PoseWithCovarianceStamped)
  - /wheel_telemetry (std_msgs/msg/Float64MultiArray)
  - /scan (sensor_msgs/msg/LaserScan)
  - /plan (nav_msgs/msg/Path)
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /map (nav_msgs/msg/OccupancyGrid)
  - /emergency_stop (std_msgs/msg/Bool)
  - /rover/mode (std_msgs/msg/String)

Aggregates the telemetry and publishes a unified, lightweight JSON string on:
  - /rover/dashboard_telemetry (std_msgs/msg/String)
"""

import json
import math
import os
import threading
import time

try:
    import psutil
except ImportError:
    psutil = None

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float64MultiArray


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion into yaw angle in radians [-pi, pi]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_pi_cpu_temp() -> float:
    """Read CPU temperature from Raspberry Pi thermal zone, fallback to 0.0."""
    try:
        thermal_path = '/sys/class/thermal/thermal_zone0/temp'
        if os.path.exists(thermal_path):
            with open(thermal_path, 'r') as f:
                return round(float(f.read().strip()) / 1000.0, 1)
    except Exception:
        pass
    return 0.0


class DashboardTelemetryNode(Node):
    def __init__(self):
        super().__init__('dashboard_telemetry_node')

        self._lock = threading.Lock()
        self._start_time = time.time()

        # Configurable parameters
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.declare_parameter('telemetry_topic', '/rover/dashboard_telemetry')
        self.declare_parameter('max_scan_samples', 72)
        self.declare_parameter('max_path_samples', 40)

        pub_rate = float(self.get_parameter('publish_rate').value)
        self.telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        self.max_scan_samples = int(self.get_parameter('max_scan_samples').value)
        self.max_path_samples = int(self.get_parameter('max_path_samples').value)

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        latched_map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Internal State Storage
        self.state = {
            'mode': 'STANDBY',
            'estop': False,
            'source_pose': 'none',
            'pose': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0,
                'yaw_deg': 0.0,
                'frame': 'odom',
            },
            'amcl_pose': {
                'active': False,
                'x': 0.0,
                'y': 0.0,
                'yaw': 0.0,
                'yaw_deg': 0.0,
                'covariance_xy': 0.0,
                'last_time': 0.0,
            },
            'motion': {
                'linear_vel': 0.0,
                'angular_vel': 0.0,
                'cmd_linear': 0.0,
                'cmd_angular': 0.0,
            },
            'wheels': {
                'connected': False,
                'tick_left': 0.0,
                'tick_right': 0.0,
                'rpm_left': 0.0,
                'rpm_right': 0.0,
                'last_time': 0.0,
            },
            'lidar': {
                'active': False,
                'min_range': 0.0,
                'front_distance': 0.0,
                'left_distance': 0.0,
                'right_distance': 0.0,
                'points_count': 0,
                'points': [],  # [[x, y], ...] in laser/base frame
                'last_time': 0.0,
            },
            'map': {
                'received': False,
                'width': 0,
                'height': 0,
                'resolution': 0.05,
                'origin_x': 0.0,
                'origin_y': 0.0,
                'explored_area_m2': 0.0,
            },
            'nav': {
                'is_navigating': False,
                'path_length': 0.0,
                'distance_to_goal': 0.0,
                'waypoints_count': 0,
                'waypoints': [],  # [[x, y], ...]
            },
            'system': {
                'cpu_temp_c': 0.0,
                'cpu_usage_pct': 0.0,
                'ram_usage_pct': 0.0,
                'uptime_sec': 0,
            },
        }

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, sensor_qos)
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, reliable_qos)
        self.sub_telemetry = self.create_subscription(
            Float64MultiArray, '/wheel_telemetry', self._wheel_telemetry_cb, reliable_qos)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.sub_plan = self.create_subscription(
            Path, '/plan', self._plan_cb, reliable_qos)
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, reliable_qos)
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latched_map_qos)
        self.sub_estop = self.create_subscription(
            Bool, '/emergency_stop', self._estop_cb, reliable_qos)
        self.sub_mode = self.create_subscription(
            String, '/rover/mode', self._mode_cb, reliable_qos)

        # Publisher
        self.pub_dashboard = self.create_publisher(
            String, self.telemetry_topic, reliable_qos)

        # Timer for periodic publish
        self.timer = self.create_timer(1.0 / pub_rate, self._publish_telemetry)
        self.get_logger().info(
            f"Dashboard Telemetry Node started on '{self.telemetry_topic}' at {pub_rate} Hz.")

    # ----------------- Callbacks -----------------

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

            # If AMCL is not active, use odom for primary pose
            if not self.state['amcl_pose']['active']:
                self.state['pose']['x'] = round(pos.x, 3)
                self.state['pose']['y'] = round(pos.y, 3)
                self.state['pose']['z'] = round(pos.z, 3)
                self.state['pose']['yaw'] = round(yaw, 3)
                self.state['pose']['yaw_deg'] = round(math.degrees(yaw), 1)
                self.state['pose']['frame'] = msg.header.frame_id or 'odom'
                self.state['source_pose'] = 'odom'

            self.state['motion']['linear_vel'] = round(msg.twist.twist.linear.x, 3)
            self.state['motion']['angular_vel'] = round(msg.twist.twist.angular.z, 3)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        with self._lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
            cov_xy = math.sqrt(abs(msg.pose.covariance[0]) + abs(msg.pose.covariance[7]))

            self.state['amcl_pose']['active'] = True
            self.state['amcl_pose']['x'] = round(pos.x, 3)
            self.state['amcl_pose']['y'] = round(pos.y, 3)
            self.state['amcl_pose']['yaw'] = round(yaw, 3)
            self.state['amcl_pose']['yaw_deg'] = round(math.degrees(yaw), 1)
            self.state['amcl_pose']['covariance_xy'] = round(cov_xy, 3)
            self.state['amcl_pose']['last_time'] = time.time()

            # Overwrite primary pose with AMCL when available
            self.state['pose']['x'] = round(pos.x, 3)
            self.state['pose']['y'] = round(pos.y, 3)
            self.state['pose']['z'] = round(pos.z, 3)
            self.state['pose']['yaw'] = round(yaw, 3)
            self.state['pose']['yaw_deg'] = round(math.degrees(yaw), 1)
            self.state['pose']['frame'] = msg.header.frame_id or 'map'
            self.state['source_pose'] = 'amcl'

    def _wheel_telemetry_cb(self, msg: Float64MultiArray):
        # [tickA, tickB, rpmA, rpmB]
        # In this rover: Motor A = Right, Motor B = Left
        if len(msg.data) >= 4:
            with self._lock:
                self.state['wheels']['tick_right'] = round(float(msg.data[0]), 0)
                self.state['wheels']['tick_left'] = round(float(msg.data[1]), 0)
                self.state['wheels']['rpm_right'] = round(float(msg.data[2]), 1)
                self.state['wheels']['rpm_left'] = round(float(msg.data[3]), 1)
                self.state['wheels']['connected'] = True
                self.state['wheels']['last_time'] = time.time()

    def _scan_cb(self, msg: LaserScan):
        ranges = msg.ranges
        if not ranges:
            return

        valid_ranges = []
        front_ranges = []
        left_ranges = []
        right_ranges = []
        cartesian_points = []

        angle = msg.angle_min
        step = msg.angle_increment
        total_rays = len(ranges)
        subsample_step = max(1, total_rays // self.max_scan_samples)

        for i in range(0, total_rays):
            r = ranges[i]
            cur_angle = angle + i * step
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r):
                valid_ranges.append(r)

                deg = math.degrees(cur_angle)
                # Front sector: -20 to +20 deg
                if -20.0 <= deg <= 20.0:
                    front_ranges.append(r)
                elif 20.0 < deg <= 80.0:
                    left_ranges.append(r)
                elif -80.0 <= deg < -20.0:
                    right_ranges.append(r)

                # Subsampled points for web canvas visualization
                if i % subsample_step == 0:
                    px = round(r * math.cos(cur_angle), 3)
                    py = round(r * math.sin(cur_angle), 3)
                    cartesian_points.append([px, py])

        min_range = min(valid_ranges) if valid_ranges else 0.0
        front_dist = min(front_ranges) if front_ranges else min_range
        left_dist = min(left_ranges) if left_ranges else min_range
        right_dist = min(right_ranges) if right_ranges else min_range

        with self._lock:
            self.state['lidar']['active'] = True
            self.state['lidar']['min_range'] = round(min_range, 2)
            self.state['lidar']['front_distance'] = round(front_dist, 2)
            self.state['lidar']['left_distance'] = round(left_dist, 2)
            self.state['lidar']['right_distance'] = round(right_dist, 2)
            self.state['lidar']['points_count'] = len(valid_ranges)
            self.state['lidar']['points'] = cartesian_points
            self.state['lidar']['last_time'] = time.time()

    def _plan_cb(self, msg: Path):
        with self._lock:
            poses = msg.poses
            count = len(poses)
            self.state['nav']['is_navigating'] = count > 0
            self.state['nav']['waypoints_count'] = count

            if count > 1:
                total_len = 0.0
                pts = []
                step = max(1, count // self.max_path_samples)
                for idx, p in enumerate(poses):
                    if idx % step == 0 or idx == count - 1:
                        pts.append([round(p.pose.position.x, 3), round(p.pose.position.y, 3)])
                    if idx > 0:
                        p_prev = poses[idx - 1].pose.position
                        p_curr = p.pose.position
                        total_len += math.hypot(p_curr.x - p_prev.x, p_curr.y - p_prev.y)

                self.state['nav']['path_length'] = round(total_len, 2)
                self.state['nav']['waypoints'] = pts

                # Distance from current pose to goal
                goal_p = poses[-1].pose.position
                cur_x = self.state['pose']['x']
                cur_y = self.state['pose']['y']
                self.state['nav']['distance_to_goal'] = round(
                    math.hypot(goal_p.x - cur_x, goal_p.y - cur_y), 2)
            else:
                self.state['nav']['path_length'] = 0.0
                self.state['nav']['distance_to_goal'] = 0.0
                self.state['nav']['waypoints'] = []

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self.state['motion']['cmd_linear'] = round(msg.linear.x, 3)
            self.state['motion']['cmd_angular'] = round(msg.angular.z, 3)

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            # Estimate explored area by counting cells not equal to -1 (unknown)
            explored_cells = sum(1 for val in msg.data if val >= 0)
            explored_area = explored_cells * (res ** 2)

            self.state['map']['received'] = True
            self.state['map']['width'] = w
            self.state['map']['height'] = h
            self.state['map']['resolution'] = round(res, 4)
            self.state['map']['origin_x'] = round(ox, 3)
            self.state['map']['origin_y'] = round(oy, 3)
            self.state['map']['explored_area_m2'] = round(explored_area, 1)

    def _estop_cb(self, msg: Bool):
        with self._lock:
            self.state['estop'] = bool(msg.data)
            if self.state['estop']:
                self.state['mode'] = 'ESTOP'

    def _mode_cb(self, msg: String):
        with self._lock:
            self.state['mode'] = msg.data.upper()

    # ----------------- Publisher Loop -----------------

    def _publish_telemetry(self):
        now = time.time()

        with self._lock:
            # Check timeouts for active sensors
            if now - self.state['wheels']['last_time'] > 2.5:
                self.state['wheels']['connected'] = False
            if now - self.state['lidar']['last_time'] > 2.5:
                self.state['lidar']['active'] = False
            if now - self.state['amcl_pose']['last_time'] > 3.0:
                self.state['amcl_pose']['active'] = False

            # Update system metrics
            cpu_temp = read_pi_cpu_temp()
            cpu_pct = psutil.cpu_percent() if psutil else 0.0
            ram_pct = psutil.virtual_memory().percent if psutil else 0.0
            uptime = int(now - self._start_time)

            self.state['system']['cpu_temp_c'] = cpu_temp
            self.state['system']['cpu_usage_pct'] = round(cpu_pct, 1)
            self.state['system']['ram_usage_pct'] = round(ram_pct, 1)
            self.state['system']['uptime_sec'] = uptime

            payload_data = dict(self.state)
            payload_data['timestamp'] = now

        msg = String()
        msg.data = json.dumps(payload_data)
        self.pub_dashboard.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
