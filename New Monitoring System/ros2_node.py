import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from threading import Thread, Lock
import time
import json

# ROS 2 Message Types
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path, Map
from sensor_msgs.msg import LaserScan, BatteryState, Imu
from std_msgs.msg import String, Bool, Float32
from slam_toolbox.msg import SerializePoseGraph

# Custom message types (if any)
# from aer_msgs.msg import RoverStatus, MissionState


class RoverROS2Node(Node):
    """
    ROS 2 Node for Autonomous Evacuation Rover
    Handles communication with SLAM, navigation, and sensor nodes
    """

    def __init__(self, shared_state):
        super().__init__('aer_dashboard_node')

        self.shared_state = shared_state
        self.state_lock = Lock()

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================== SUBSCRIBERS ====================

        # Robot pose (from SLAM/Localization)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            reliable_qos
        )

        # Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        # Laser scan (for obstacle detection)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        # Map (from SLAM)
        self.map_sub = self.create_subscription(
            Map,
            '/map',
            self.map_callback,
            reliable_qos
        )

        # Battery status
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_callback,
            reliable_qos
        )

        # IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Navigation path
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            reliable_qos
        )

        # Mission status
        self.mission_sub = self.create_subscription(
            String,
            '/mission_status',
            self.mission_callback,
            reliable_qos
        )

        # ==================== PUBLISHERS ====================

        # Velocity command (for manual control)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            reliable_qos
        )

        # Emergency stop
        self.estop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            reliable_qos
        )

        # Mission command
        self.mission_cmd_pub = self.create_publisher(
            String,
            '/mission_command',
            reliable_qos
        )

        # ==================== STATE INITIALIZATION ====================

        self._init_state()

        self.get_logger().info('AER Dashboard ROS 2 Node initialized')

    def _init_state(self):
        """Initialize shared state with default values"""
        with self.state_lock:
            self.shared_state.update({
                'mode': 'STANDBY',
                'battery': 100.0,
                'battery_voltage': 12.6,
                'cpu_temp': 45.0,
                'ros_connected': True,
                'camera_active': True,
                'uptime_seconds': 0,

                # Position (from SLAM)
                'lat': 0.0,
                'lng': 0.0,
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0,

                # Velocity
                'linear_vel': 0.0,
                'angular_vel': 0.0,

                # SLAM status
                'slam_active': False,
                'map_received': False,
                'localization_ok': False,

                # Sensors
                'min_obstacle_distance': float('inf'),
                'imu_ok': True,

                # Navigation
                'navigation_active': False,
                'goal_reached': False,
                'path_length': 0.0,

                # Mission
                'mission_state': 'IDLE',
                'mission_progress': 0.0,

                # Timestamps
                'last_pose_update': 0.0,
                'last_scan_update': 0.0,
                'last_map_update': 0.0,
            })

    # ==================== CALLBACKS ====================

    def pose_callback(self, msg: PoseStamped):
        """Handle robot pose updates from SLAM/Localization"""
        with self.state_lock:
            self.shared_state['x'] = msg.pose.position.x
            self.shared_state['y'] = msg.pose.position.y
            self.shared_state['z'] = msg.pose.position.z

            # Convert quaternion to yaw
            q = msg.pose.orientation
            yaw = self._quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.shared_state['yaw'] = yaw

            self.shared_state['last_pose_update'] = time.time()
            self.shared_state['localization_ok'] = True

    def odom_callback(self, msg: Odometry):
        """Handle odometry data"""
        with self.state_lock:
            self.shared_state['linear_vel'] = msg.twist.twist.linear.x
            self.shared_state['angular_vel'] = msg.twist.twist.angular.z

    def scan_callback(self, msg: LaserScan):
        """Handle laser scan data for obstacle detection"""
        with self.state_lock:
            # Find minimum distance (closest obstacle)
            valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
            if valid_ranges:
                self.shared_state['min_obstacle_distance'] = min(valid_ranges)
            self.shared_state['last_scan_update'] = time.time()

    def map_callback(self, msg: Map):
        """Handle map updates from SLAM"""
        with self.state_lock:
            self.shared_state['map_received'] = True
            self.shared_state['slam_active'] = True
            self.shared_state['last_map_update'] = time.time()

            # Store map metadata
            self.shared_state['map_width'] = msg.info.width
            self.shared_state['map_height'] = msg.info.height
            self.shared_state['map_resolution'] = msg.info.resolution

    def battery_callback(self, msg: BatteryState):
        """Handle battery status updates"""
        with self.state_lock:
            self.shared_state['battery'] = msg.percentage * 100.0
            self.shared_state['battery_voltage'] = msg.voltage

    def imu_callback(self, msg: Imu):
        """Handle IMU data"""
        with self.state_lock:
            self.shared_state['imu_ok'] = True

    def path_callback(self, msg: Path):
        """Handle navigation path updates"""
        with self.state_lock:
            self.shared_state['navigation_active'] = len(msg.poses) > 0

            # Calculate path length
            if len(msg.poses) > 1:
                total_length = 0.0
                for i in range(len(msg.poses) - 1):
                    p1 = msg.poses[i].pose.position
                    p2 = msg.poses[i+1].pose.position
                    dist = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
                    total_length += dist
                self.shared_state['path_length'] = total_length

    def mission_callback(self, msg: String):
        """Handle mission status updates"""
        with self.state_lock:
            try:
                mission_data = json.loads(msg.data)
                self.shared_state['mission_state'] = mission_data.get('state', 'IDLE')
                self.shared_state['mission_progress'] = mission_data.get('progress', 0.0)
            except json.JSONDecodeError:
                self.shared_state['mission_state'] = msg.data

    # ==================== COMMAND METHODS ====================

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to robot"""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Published cmd_vel: linear={linear_x}, angular={angular_z}')

    def send_emergency_stop(self):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = True
        self.estop_pub.publish(msg)

        # Also send zero velocity
        self.send_velocity_command(0.0, 0.0)

        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def send_mission_command(self, command: str):
        """Send mission command (start, pause, resume, abort)"""
        msg = String()
        msg.data = command
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(f'Mission command sent: {command}')

    def set_mode(self, mode: str):
        """Set robot operating mode"""
        with self.state_lock:
            self.shared_state['mode'] = mode
        self.get_logger().info(f'Mode changed to: {mode}')

    # ==================== UTILITY METHODS ====================

    def _quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle (radians)"""
        import math
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_state(self):
        """Get current robot state (thread-safe)"""
        with self.state_lock:
            return self.shared_state.copy()

    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down ROS 2 node...')
        self.send_velocity_command(0.0, 0.0)  # Ensure robot stops
        self.destroy_node()


def run_ros2_node(shared_state):
    """
    Run ROS 2 node in a separate thread
    This allows Flask to run concurrently with ROS 2 communication
    """
    rclpy.init()
    node = RoverROS2Node(shared_state)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    # Test standalone
    shared_state = {}
    run_ros2_node(shared_state)