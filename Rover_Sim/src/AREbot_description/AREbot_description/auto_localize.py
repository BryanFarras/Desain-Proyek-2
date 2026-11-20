#!/usr/bin/env python3
"""Automatic global localization + initial pose for AMCL.

Flow:
  1. Wait until the map, AMCL services, /amcl_pose and TF (odom <-> base) are ready.
  2. Call /reinitialize_global_localization (spreads particles over the whole map).
  3. Spin the robot in place (default 2 full turns) so the laser scan lets the
     particle filter disambiguate the pose.
  4. Stop, request a no-motion update and read the converged /amcl_pose.
  5. Publish that pose on /initialpose to lock it in -- the same thing RViz's
     "2D Pose Estimate" does, but automatic.

Run standalone:
    ros2 run AREbot_description auto_localize

It is also invoked automatically by `ros2 run AREbot_description run_amcl`.
"""
import math
import signal

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class AutoLocalize(Node):
    WAIT, SEED, GLOBAL, SPIN, SETTLE, PUBLISH, HOLD, DONE, FAIL = range(9)

    def __init__(self):
        super().__init__('auto_localize')

        self.declare_parameter('spin_speed', 0.5)
        self.declare_parameter('turns', 2.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('initpose_topic', '/initialpose')
        self.declare_parameter('global_loc_service', '/reinitialize_global_localization')
        self.declare_parameter('nomotion_service', '/request_nomotion_update')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_initial_pose', True)
        self.declare_parameter('ready_timeout', 60.0)
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('pose_timeout', 15.0)
        # Keep the node alive after publishing the initial pose and keep
        # requesting /request_nomotion_update (like run_amcl.py does).
        self.declare_parameter('keep_nomotion_updates', False)
        self.declare_parameter('nomotion_update_rate', 2.0)
        # Bias the initial belief towards the map origin (0, 0) with a Gaussian
        # prior instead of spreading particles uniformly over the whole map.
        self.declare_parameter('use_origin_prior', True)
        self.declare_parameter('prior_x', 0.0)
        self.declare_parameter('prior_y', 0.0)
        self.declare_parameter('prior_yaw', 0.0)
        self.declare_parameter('prior_xy_std', 0.75)
        self.declare_parameter('prior_yaw_std', 0.5)
        self.declare_parameter('seed_delay', 0.75)

        self.spin_speed = abs(self.get_parameter('spin_speed').value)
        self.turns = abs(self.get_parameter('turns').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.initpose_topic = self.get_parameter('initpose_topic').value
        self.global_loc_service = self.get_parameter('global_loc_service').value
        self.nomotion_service = self.get_parameter('nomotion_service').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_initial_pose = self.get_parameter('publish_initial_pose').value
        self.ready_timeout = self.get_parameter('ready_timeout').value
        self.settle_time = self.get_parameter('settle_time').value
        self.pose_timeout = self.get_parameter('pose_timeout').value
        self.keep_nomotion_updates = bool(
            self.get_parameter('keep_nomotion_updates').value)
        self.nomotion_update_rate = max(
            float(self.get_parameter('nomotion_update_rate').value), 0.1)
        self.use_origin_prior = self.get_parameter('use_origin_prior').value
        self.prior_x = self.get_parameter('prior_x').value
        self.prior_y = self.get_parameter('prior_y').value
        self.prior_yaw = self.get_parameter('prior_yaw').value
        self.prior_xy_std = self.get_parameter('prior_xy_std').value
        self.prior_yaw_std = self.get_parameter('prior_yaw_std').value
        self.seed_delay = self.get_parameter('seed_delay').value

        self.target_rotation = self.turns * 2.0 * math.pi
        self.spin_timeout = 20.0 + 2.0 * (self.target_rotation / max(self.spin_speed, 0.05))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # cmd_vel_relay (topic_tools) subscribes with TRANSIENT_LOCAL, so the
        # publisher must offer at least that durability or messages are dropped.
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, cmd_qos)
        self.initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initpose_topic, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._pose_cb, 10)

        self.glob_cli = self.create_client(Empty, self.global_loc_service)
        self.nomotion_cli = self.create_client(Empty, self.nomotion_service)

        self.map_ok = False
        self.pose_msg = None
        self.result_pose = None
        self.last_yaw = None
        self.rotated = 0.0

        self.state = self.WAIT
        self.state_t = self.get_clock().now()
        self.last_wait_log = self.state_t
        self.glob_future = None
        self.seeded = False
        self.finished = False
        self.success = False

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'auto_localize: spin_speed={self.spin_speed} rad/s, '
            f'turns={self.turns} ({math.degrees(self.target_rotation):.0f} deg), '
            f'cmd_vel={self.cmd_vel_topic}, '
            f'origin_prior={self.use_origin_prior} '
            f'({self.prior_x},{self.prior_y},{self.prior_yaw}) '
            f'std=({self.prior_xy_std},{self.prior_yaw_std})')

    # ------------------------------------------------------------------
    def _map_cb(self, msg):
        if not self.map_ok:
            self.get_logger().info('auto_localize: map received')
        self.map_ok = True

    def _pose_cb(self, msg):
        self.pose_msg = msg

    def _elapsed(self):
        return (self.get_clock().now() - self.state_t).nanoseconds * 1e-9

    def _enter(self, state):
        if state == self.SEED:
            self.seeded = False
        if state == self.HOLD:
            self.last_nomotion_call = self.get_clock().now()
        self.state = state
        self.state_t = self.get_clock().now()
        self.get_logger().info(
            f'auto_localize: state -> {self._state_name(state)}')

    @staticmethod
    def _state_name(state):
        return {
            AutoLocalize.WAIT: 'WAIT',
            AutoLocalize.SEED: 'SEED',
            AutoLocalize.GLOBAL: 'GLOBAL',
            AutoLocalize.SPIN: 'SPIN',
            AutoLocalize.SETTLE: 'SETTLE',
            AutoLocalize.PUBLISH: 'PUBLISH',
            AutoLocalize.HOLD: 'HOLD',
            AutoLocalize.DONE: 'DONE',
            AutoLocalize.FAIL: 'FAIL',
        }[state]

    def _yaw_in_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except TransformException:
            return None
        return quat_to_yaw(tf.transform.rotation)

    def _pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        return tf

    def _odom_stamp(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except TransformException:
            return None
        return tf.header.stamp

    def _publish_prior(self):
        """Seed AMCL with a Gaussian prior centred on the map origin."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        stamp = self._odom_stamp()
        if stamp is not None:
            msg.header.stamp = stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.prior_x
        msg.pose.pose.position.y = self.prior_y
        msg.pose.pose.orientation = yaw_to_quat(self.prior_yaw)
        cov = [0.0] * 36
        cov[0] = self.prior_xy_std ** 2
        cov[7] = self.prior_xy_std ** 2
        cov[35] = self.prior_yaw_std ** 2
        msg.pose.covariance = cov
        self.initpose_pub.publish(msg)

    def _publish_velocity(self, angular_z):
        msg = Twist()
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        if not rclpy.ok():
            return
        for _ in range(5):
            try:
                self._publish_velocity(0.0)
            except Exception:
                return

    # ------------------------------------------------------------------
    def _tick(self):
        if self.state == self.WAIT:
            self._tick_wait()
        elif self.state == self.SEED:
            self._tick_seed()
        elif self.state == self.GLOBAL:
            self._tick_global()
        elif self.state == self.SPIN:
            self._tick_spin()
        elif self.state == self.SETTLE:
            self._tick_settle()
        elif self.state == self.PUBLISH:
            self._tick_publish()
        elif self.state == self.HOLD:
            self._tick_hold()

    def _tick_wait(self):
        if self._elapsed() > self.ready_timeout:
            self.get_logger().error(
                'auto_localize: timed out waiting for map/AMCL/TF. '
                f'(map={self.map_ok}, global_srv={self.glob_cli.service_is_ready()}, '
                f'nomotion_srv={self.nomotion_cli.service_is_ready()}, '
                f'amcl_pose={self.pose_msg is not None}, '
                f'tf={self._yaw_in_odom() is not None})')
            self._fail()
            return

        # NOTE: /amcl_pose is intentionally NOT required here. AMCL withholds it
        # until the initial pose is known, which is exactly what the global
        # localization call below provides -- requiring it here would deadlock.
        pending = []
        if not self.map_ok:
            pending.append('map')
        if not self.use_origin_prior and not self.glob_cli.service_is_ready():
            pending.append('global_loc_srv')
        if not self.nomotion_cli.service_is_ready():
            pending.append('nomotion_srv')
        if self._yaw_in_odom() is None:
            pending.append('tf odom->base')

        if pending:
            if (self.get_clock().now() - self.last_wait_log).nanoseconds * 1e-9 > 5.0:
                self.get_logger().info(
                    'auto_localize: waiting for ' + ', '.join(pending))
                self.last_wait_log = self.get_clock().now()
            return

        if self.use_origin_prior:
            self.get_logger().info(
                f'auto_localize: all ready, seeding prior at ('
                f'{self.prior_x}, {self.prior_y}, {self.prior_yaw})')
            self._enter(self.SEED)
        else:
            self.get_logger().info(
                'auto_localize: all ready, requesting global localization')
            self.glob_future = self.glob_cli.call_async(Empty.Request())
            self._enter(self.GLOBAL)

    def _tick_seed(self):
        if not self.seeded:
            self._publish_prior()
            self.seeded = True
            self.get_logger().info('auto_localize: prior published to /initialpose')
            return
        if self._elapsed() >= self.seed_delay:
            self.last_yaw = self._yaw_in_odom()
            self.rotated = 0.0
            self._enter(self.SPIN)

    def _tick_global(self):
        if self.glob_future is not None and self.glob_future.done():
            self.last_yaw = self._yaw_in_odom()
            self.rotated = 0.0
            self._enter(self.SPIN)
        elif self._elapsed() > 10.0:
            self.get_logger().warn('auto_localize: global loc service slow, starting spin anyway')
            self.last_yaw = self._yaw_in_odom()
            self.rotated = 0.0
            self._enter(self.SPIN)

    def _tick_spin(self):
        cur_yaw = self._yaw_in_odom()
        if cur_yaw is not None and self.last_yaw is not None:
            delta = angle_diff(cur_yaw, self.last_yaw)
            self.rotated += abs(delta)
            self.last_yaw = cur_yaw

        if self.rotated >= self.target_rotation:
            self.stop_robot()
            self.get_logger().info(
                f'auto_localize: spin done ({math.degrees(self.rotated):.0f} deg)')
            self._enter(self.SETTLE)
            return

        if self._elapsed() > self.spin_timeout:
            self.get_logger().warn(
                f'auto_localize: spin timeout (rotated {math.degrees(self.rotated):.0f} deg), '
                'stopping and using best estimate')
            self.stop_robot()
            self._enter(self.SETTLE)
            return

        self._publish_velocity(self.spin_speed)

    def _tick_settle(self):
        self.stop_robot()
        if self._elapsed() < 0.5:
            # ask AMCL for a fresh pose while stationary
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            return
        # After global localization AMCL starts publishing /amcl_pose (and the
        # map->base TF). Wait for a pose before locking it in.
        have_pose = self.pose_msg is not None or self._pose_in_map() is not None
        if self._elapsed() >= self.settle_time and have_pose:
            self._enter(self.PUBLISH)
            return
        if self._elapsed() > self.settle_time + self.pose_timeout:
            self.get_logger().error(
                'auto_localize: no /amcl_pose or map->base TF after global '
                'localization; giving up')
            self._fail()

    def _tick_publish(self):
        pose = None
        if self.pose_msg is not None:
            pose = self.pose_msg

        if pose is None:
            tf = self._pose_in_map()
            if tf is not None:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = self.map_frame
                pose.pose.pose.position.x = tf.transform.translation.x
                pose.pose.pose.position.y = tf.transform.translation.y
                pose.pose.pose.orientation = tf.transform.rotation

        if pose is None:
            self.get_logger().error('auto_localize: no pose available to publish')
            self._fail()
            return

        x = pose.pose.pose.position.x
        y = pose.pose.pose.position.y
        yaw = quat_to_yaw(pose.pose.pose.orientation)
        self.result_pose = (x, y, yaw)

        if self.publish_initial_pose:
            msg = PoseWithCovarianceStamped()
            # Reuse the pose header (sim-time stamp + map frame) so it is
            # consistent with AMCL's clock.
            msg.header = pose.header
            msg.header.frame_id = self.map_frame
            msg.pose.pose = pose.pose.pose
            cov = [0.0] * 36
            cov[0] = 0.05
            cov[7] = 0.05
            cov[35] = 0.02
            msg.pose.covariance = cov
            self.initpose_pub.publish(msg)

        self.get_logger().info(
            f'auto_localize: estimated pose x={x:.3f} y={y:.3f} '
            f'yaw={math.degrees(yaw):.1f} deg'
            + (' (published to /initialpose)' if self.publish_initial_pose else ''))
        self.success = True
        if self.keep_nomotion_updates:
            self._enter(self.HOLD)
            self.get_logger().info(
                'auto_localize: keeping nomotion updates at '
                f'{self.nomotion_update_rate} Hz')
        else:
            self._enter(self.DONE)
            self.finished = True

    def _tick_hold(self):
        now = self.get_clock().now()
        interval = 1.0 / self.nomotion_update_rate
        if (now - self.last_nomotion_call).nanoseconds * 1e-9 >= interval:
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            self.last_nomotion_call = now

    def _fail(self):
        self.stop_robot()
        self.success = False
        self._enter(self.FAIL)
        self.finished = True


def _raise_keyboard_interrupt(_signum, _frame):
    raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = AutoLocalize()
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
