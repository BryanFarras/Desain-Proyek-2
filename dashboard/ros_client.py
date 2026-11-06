#!/usr/bin/env python3
"""
ROS 2 Client Interface for the Web Dashboard.

Runs in a background thread using rclpy:
- Subscribes to /rover/dashboard_telemetry (std_msgs/msg/String)
- Subscribes to /map (nav_msgs/msg/OccupancyGrid)
- Publishes /cmd_vel (geometry_msgs/msg/Twist)
- Publishes /emergency_stop (std_msgs/msg/Bool)
- Publishes /rover/mode (std_msgs/msg/String)
- Publishes /goal_pose (geometry_msgs/msg/PoseStamped)
"""

import json
import math
import os
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        DurabilityPolicy,
        HistoryPolicy,
    )
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String, Bool
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False
    Node = object


def yaw_to_quaternion(yaw: float):
    """Convert a planar yaw angle into a geometry_msgs quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0),
    }


class DashboardROSClient(Node if RCLPY_AVAILABLE else object):
    def __init__(self):
        if not RCLPY_AVAILABLE:
            raise RuntimeError("rclpy is not installed or available.")

        super().__init__('web_dashboard_client')

        self._lock = threading.Lock()
        self._last_telemetry_time = 0.0
        self._last_map_time = 0.0

        # Latest Telemetry Cache
        self.telemetry = {
            'ros_connected': True,
            'mode': 'STANDBY',
            'estop': False,
            'source_pose': 'none',
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0, 'frame': 'odom'},
            'amcl_pose': {'active': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0, 'covariance_xy': 0.0},
            'motion': {'linear_vel': 0.0, 'angular_vel': 0.0, 'cmd_linear': 0.0, 'cmd_angular': 0.0},
            'wheels': {'connected': False, 'tick_left': 0.0, 'tick_right': 0.0, 'rpm_left': 0.0, 'rpm_right': 0.0},
            'lidar': {'active': False, 'min_range': 0.0, 'front_distance': 0.0, 'left_distance': 0.0, 'right_distance': 0.0, 'points_count': 0, 'points': []},
            'map': {'received': False, 'width': 0, 'height': 0, 'resolution': 0.05, 'origin_x': 0.0, 'origin_y': 0.0, 'explored_area_m2': 0.0},
            'nav': {'is_navigating': False, 'path_length': 0.0, 'distance_to_goal': 0.0, 'waypoints_count': 0, 'waypoints': []},
            'system': {'cpu_temp_c': 0.0, 'cpu_usage_pct': 0.0, 'ram_usage_pct': 0.0, 'uptime_sec': 0},
            'timestamp': time.time(),
        }

        # Latest Map Cache
        self.map_data = {
            'received': False,
            'width': 0,
            'height': 0,
            'resolution': 0.05,
            'origin': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
            'data': [],
            'timestamp': 0.0,
        }

        # QoS Profiles
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

        # Subscribers
        self.sub_telemetry = self.create_subscription(
            String, '/rover/dashboard_telemetry', self._telemetry_cb, reliable_qos)
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latched_map_qos)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', reliable_qos)
        self.pub_mode = self.create_publisher(String, '/rover/mode', reliable_qos)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', reliable_qos)

        self.get_logger().info("Dashboard ROS 2 client initialized successfully.")

    # ----------------- Subscribers -----------------

    def _telemetry_cb(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            with self._lock:
                self.telemetry = parsed
                self.telemetry['ros_connected'] = True
                self._last_telemetry_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Error parsing telemetry JSON: {e}")

    def _map_cb(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y
            oz = msg.info.origin.orientation.z
            ow = msg.info.origin.orientation.w
            yaw = 2.0 * math.atan2(oz, ow)

            # Convert int8 OccupancyGrid data into list
            raw_data = list(msg.data)

            with self._lock:
                self.map_data = {
                    'received': True,
                    'width': w,
                    'height': h,
                    'resolution': res,
                    'origin': {
                        'x': ox,
                        'y': oy,
                        'yaw': yaw,
                    },
                    'data': raw_data,
                    'timestamp': time.time(),
                }
                self._last_map_time = time.time()
                self.telemetry['map']['received'] = True
                self.telemetry['map']['width'] = w
                self.telemetry['map']['height'] = h
                self.telemetry['map']['resolution'] = res
                self.telemetry['map']['origin_x'] = ox
                self.telemetry['map']['origin_y'] = oy
        except Exception as e:
            self.get_logger().error(f"Error caching occupancy grid map: {e}")

    # ----------------- Commands -----------------

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        """Publish velocity command to /cmd_vel."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

    def send_emergency_stop(self):
        """Immediately trigger E-STOP and publish zero velocities."""
        msg_estop = Bool()
        msg_estop.data = True
        self.pub_estop.publish(msg_estop)

        # Halt wheels immediately
        self.send_cmd_vel(0.0, 0.0)

        # Set mode to ESTOP
        self.set_mode('ESTOP')

    def set_mode(self, mode: str):
        """Set robot operating mode (STANDBY, MANUAL, AUTONOMOUS, ESTOP)."""
        mode = mode.upper()
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)
        with self._lock:
            self.telemetry['mode'] = mode
            if mode != 'ESTOP':
                self.telemetry['estop'] = False

    def send_goal_pose(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'):
        """Publish a 2D Navigation Goal to /goal_pose."""
        q = yaw_to_quaternion(yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = q['x']
        msg.pose.orientation.y = q['y']
        msg.pose.orientation.z = q['z']
        msg.pose.orientation.w = q['w']
        self.pub_goal.publish(msg)
        self.get_logger().info(f"Published Nav2 goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)")

    # ----------------- Getters -----------------

    def get_telemetry(self):
        with self._lock:
            data = dict(self.telemetry)
            # Check if telemetry is still streaming
            is_recent = (time.time() - self._last_telemetry_time) < 2.5
            data['telemetry_streaming'] = is_recent
            return data

    def get_map_data(self):
        with self._lock:
            return dict(self.map_data)


# Global Singleton Client
_ros_client = None
_ros_thread = None


def start_ros_client():
    global _ros_client, _ros_thread
    if not RCLPY_AVAILABLE:
        print("[WARN] rclpy is not available. Running in standalone fallback mode.")
        return None

    if _ros_client is not None:
        return _ros_client

    rclpy.init(args=None)
    _ros_client = DashboardROSClient()

    def _spin():
        try:
            rclpy.spin(_ros_client)
        except Exception as e:
            print(f"[ROS Thread] Spin ended: {e}")
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    _ros_thread = threading.Thread(target=_spin, daemon=True)
    _ros_thread.start()
    return _ros_client


def get_ros_client():
    global _ros_client
    if _ros_client is None and RCLPY_AVAILABLE:
        _ros_client = start_ros_client()
    return _ros_client
