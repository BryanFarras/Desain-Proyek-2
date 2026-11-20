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
    from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Bool
    from tf2_ros import Buffer, TransformListener
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
            'pose': {
                'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'yaw_deg': 0.0, 'frame': 'odom'
            },
            'amcl_pose': {
                'active': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
                'yaw_deg': 0.0, 'covariance_xy': 0.0
            },
            'motion': {
                'linear_vel': 0.0, 'angular_vel': 0.0, 'cmd_linear': 0.0, 'cmd_angular': 0.0
            },
            'wheels': {
                'connected': False, 'tick_left': 0.0, 'tick_right': 0.0,
                'rpm_left': 0.0, 'rpm_right': 0.0
            },
            'lidar': {
                'active': False, 'min_range': 0.0, 'front_distance': 0.0,
                'left_distance': 0.0, 'right_distance': 0.0, 'points_count': 0, 'points': []
            },
            'map': {
                'received': False, 'width': 0, 'height': 0, 'resolution': 0.05,
                'origin_x': 0.0, 'origin_y': 0.0, 'explored_area_m2': 0.0
            },
            'nav': {
                'is_navigating': False, 'path_length': 0.0, 'distance_to_goal': 0.0,
                'waypoints_count': 0, 'waypoints': []
            },
            'system': {
                'cpu_temp_c': 0.0, 'cpu_usage_pct': 0.0, 'ram_usage_pct': 0.0, 'uptime_sec': 0
            },
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

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers
        self.sub_telemetry = self.create_subscription(
            String, '/rover/dashboard_telemetry', self._telemetry_cb, reliable_qos)
        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, latched_map_qos)
        # Direct /scan and /odom subscribers so dashboard always updates LiDAR & motion
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, sensor_qos)

        # TF2 Buffer & Listener for real-time map-frame tracking during SLAM/AMCL
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self._tf_pose_update_cb)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.pub_estop = self.create_publisher(Bool, '/emergency_stop', reliable_qos)
        self.pub_mode = self.create_publisher(String, '/rover/mode', reliable_qos)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', reliable_qos)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', reliable_qos)

        # Safety watchdog for manual driving (stops robot if client disconnects)
        self._last_cmd_vel_time = 0.0
        self._manual_motion_active = False
        self.watchdog_timer = self.create_timer(0.1, self._manual_watchdog_cb)

        self.get_logger().info("Dashboard ROS 2 client initialized successfully.")

    # ----------------- Subscribers -----------------

    def _telemetry_cb(self, msg: String):
        try:
            parsed = json.loads(msg.data)
            with self._lock:
                # Retain live lidar data if our direct /scan subscription is fresh
                cur_lidar = self.telemetry.get('lidar', {})
                is_live_fresh = (
                    cur_lidar.get('active') and
                    (time.time() - cur_lidar.get('last_time', 0.0) < 1.5)
                )
                if is_live_fresh:
                    live_lidar = cur_lidar
                else:
                    live_lidar = parsed.get('lidar', cur_lidar)

                # Retain map-frame pose if we have it from TF
                cur_pose = self.telemetry.get('pose', {})
                cur_source = self.telemetry.get('source_pose', 'none')
                if cur_source in ('slam', 'amcl') and cur_pose.get('frame') == 'map':
                    live_pose = cur_pose
                    live_source = cur_source
                else:
                    live_pose = parsed.get('pose', cur_pose)
                    live_source = parsed.get('source_pose', cur_source)

                self.telemetry = parsed
                self.telemetry['lidar'] = live_lidar
                self.telemetry['pose'] = live_pose
                self.telemetry['source_pose'] = live_source
                self.telemetry['ros_connected'] = True
                self._last_telemetry_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Error parsing telemetry JSON: {e}")

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
        subsample_step = max(1, total_rays // 180)

        for i in range(0, total_rays):
            r = ranges[i]
            cur_angle = angle + i * step
            if msg.range_min <= r <= msg.range_max and not math.isinf(r) and not math.isnan(r):
                valid_ranges.append(r)

                deg = math.degrees(cur_angle)
                deg = (deg + 180) % 360 - 180

                # Front sector: -25 to +25 deg
                if -25.0 <= deg <= 25.0:
                    front_ranges.append(r)
                elif 25.0 < deg <= 90.0:
                    left_ranges.append(r)
                elif -90.0 <= deg < -25.0:
                    right_ranges.append(r)

                if i % subsample_step == 0:
                    px = round(r * math.cos(cur_angle), 3)
                    py = round(r * math.sin(cur_angle), 3)
                    cartesian_points.append([px, py])

        min_range = min(valid_ranges) if valid_ranges else 0.0
        front_dist = min(front_ranges) if front_ranges else min_range
        left_dist = min(left_ranges) if left_ranges else min_range
        right_dist = min(right_ranges) if right_ranges else min_range

        with self._lock:
            self.telemetry['lidar'] = {
                'active': True,
                'min_range': round(min_range, 2),
                'front_distance': round(front_dist, 2),
                'left_distance': round(left_dist, 2),
                'right_distance': round(right_dist, 2),
                'points_count': len(valid_ranges),
                'points': cartesian_points,
                'last_time': time.time(),
            }

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            # Fallback to odom if map frame is not yet active
            if self.telemetry['source_pose'] not in ('slam', 'amcl'):
                pos = msg.pose.pose.position
                ori = msg.pose.pose.orientation
                yaw = 2.0 * math.atan2(ori.z, ori.w)
                self.telemetry['pose']['x'] = round(pos.x, 3)
                self.telemetry['pose']['y'] = round(pos.y, 3)
                self.telemetry['pose']['z'] = round(pos.z, 3)
                self.telemetry['pose']['yaw'] = round(yaw, 3)
                self.telemetry['pose']['yaw_deg'] = round(math.degrees(yaw), 1)
                self.telemetry['pose']['frame'] = msg.header.frame_id or 'odom'
                self.telemetry['source_pose'] = 'odom'

            self.telemetry['motion']['linear_vel'] = round(msg.twist.twist.linear.x, 3)
            self.telemetry['motion']['angular_vel'] = round(msg.twist.twist.angular.z, 3)

    def _tf_pose_update_cb(self):
        try:
            # Query true pose in map frame (active during SLAM mapping or AMCL navigation)
            now_tf = rclpy.time.Time()
            if self.tf_buffer.can_transform(
                'map', 'base_footprint', now_tf, timeout=rclpy.duration.Duration(seconds=0.03)
            ):
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', now_tf)
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                q = t.transform.rotation
                yaw = 2.0 * math.atan2(q.z, q.w)
                with self._lock:
                    self.telemetry['pose']['x'] = round(tx, 3)
                    self.telemetry['pose']['y'] = round(ty, 3)
                    self.telemetry['pose']['yaw'] = round(yaw, 3)
                    self.telemetry['pose']['yaw_deg'] = round(math.degrees(yaw), 1)
                    self.telemetry['pose']['frame'] = 'map'
                    is_amcl = self.telemetry.get('amcl_pose', {}).get('active', False)
                    self.telemetry['source_pose'] = 'amcl' if is_amcl else 'slam'
        except Exception:
            pass

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

    def _manual_watchdog_cb(self):
        """Safety watchdog: if MANUAL mode is active and no cmd_vel arrived for > 0.4s, stop."""
        with self._lock:
            is_estop = self.telemetry.get('estop', False)
            motion_active = self._manual_motion_active
            last_cmd = self._last_cmd_vel_time

        now = time.time()
        if (motion_active or is_estop) and (now - last_cmd > 0.40):
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub_cmd_vel.publish(msg)
            with self._lock:
                self._manual_motion_active = False

    # ----------------- Commands -----------------

    def send_cmd_vel(self, linear_x: float, angular_z: float, force: bool = False):
        """Publish velocity command to /cmd_vel with mode and watchdog protection."""
        with self._lock:
            mode = self.telemetry.get('mode', 'STANDBY')
            is_estop = self.telemetry.get('estop', False)

        # Always allow stopping (0, 0)
        is_stop = (abs(linear_x) < 1e-4 and abs(angular_z) < 1e-4)

        if is_estop:
            linear_x = 0.0
            angular_z = 0.0
        elif not is_stop and not force:
            if mode == 'STANDBY':
                self.get_logger().warn("Manual driving rejected: Rover is in STANDBY mode.")
                return False
            elif mode == 'AUTONOMOUS':
                self.get_logger().warn(
                    "Manual driving rejected: Rover is in AUTONOMOUS mode (Nav2 is driving)."
                )
                return False

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd_vel.publish(msg)

        now = time.time()
        with self._lock:
            self._last_cmd_vel_time = now
            self._manual_motion_active = not is_stop
        return True

    def send_emergency_stop(self):
        """Immediately trigger E-STOP and publish zero velocities."""
        msg_estop = Bool()
        msg_estop.data = True
        self.pub_estop.publish(msg_estop)

        # Halt wheels immediately
        self.send_cmd_vel(0.0, 0.0, force=True)

        # Set mode to ESTOP
        self.set_mode('ESTOP')

    def set_mode(self, mode: str):
        """Set robot operating mode (STANDBY, MANUAL, AUTONOMOUS, ESTOP)."""
        mode = mode.upper()
        msg = String()
        msg.data = mode
        self.pub_mode.publish(msg)

        # When leaving MANUAL or entering STANDBY/ESTOP/AUTONOMOUS, halt manual motion
        if mode in ['STANDBY', 'ESTOP', 'AUTONOMOUS']:
            self.send_cmd_vel(0.0, 0.0, force=True)

        with self._lock:
            self.telemetry['mode'] = mode
            if mode != 'ESTOP':
                self.telemetry['estop'] = False

    def send_initial_pose(self, x: float, y: float, yaw: float = 0.0, frame_id: str = 'map'):
        """Publish a 2D Initial Pose Estimate to /initialpose for AMCL."""
        q = yaw_to_quaternion(yaw)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q['x']
        msg.pose.pose.orientation.y = q['y']
        msg.pose.pose.orientation.z = q['z']
        msg.pose.pose.orientation.w = q['w']

        cov = [0.0] * 36
        cov[0] = 0.25   # x covariance
        cov[7] = 0.25   # y covariance
        cov[35] = 0.06  # yaw covariance
        msg.pose.covariance = cov

        self.pub_initialpose.publish(msg)
        self.get_logger().info(
            f"Published AMCL initial pose: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)"
        )

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
            lidar_data = data.get('lidar', {})
            # If lidar has not reported for > 2.5s, mark inactive
            if time.time() - lidar_data.get('last_time', 0.0) > 2.5:
                lidar_data['active'] = False

            # Telemetry streaming is true if either dashboard_telemetry or live /scan is active
            is_recent = (time.time() - self._last_telemetry_time < 2.5) or \
                        (lidar_data.get('active', False))
            data['telemetry_streaming'] = is_recent
            return data

    def get_map_data(self):
        with self._lock:
            return dict(self.map_data)

    def set_custom_map(
        self, width: int, height: int, resolution: float, origin: dict, data: list
    ):
        """Manually load/override map data (e.g. from a saved .pgm/.yaml file)."""
        with self._lock:
            self.map_data = {
                'received': True,
                'width': int(width),
                'height': int(height),
                'resolution': float(resolution),
                'origin': {
                    'x': float(origin.get('x', 0.0)),
                    'y': float(origin.get('y', 0.0)),
                    'yaw': float(origin.get('yaw', 0.0)),
                },
                'data': data,
                'timestamp': time.time(),
            }
            self.telemetry['map']['received'] = True
            self.telemetry['map']['width'] = int(width)
            self.telemetry['map']['height'] = int(height)
            self.telemetry['map']['resolution'] = float(resolution)
            self.telemetry['map']['origin_x'] = float(origin.get('x', 0.0))
            self.telemetry['map']['origin_y'] = float(origin.get('y', 0.0))
            self._last_map_time = time.time()


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
