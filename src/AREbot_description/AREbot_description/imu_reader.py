#!/usr/bin/env python3
"""
Convert the Gazebo IMU quaternion to roll/pitch/yaw in degrees.

Subscribes to ``/imu_sensor_broadcaster/imu`` and publishes a
``geometry_msgs/Vector3`` on ``/imu`` (x=roll, y=pitch, z=yaw, in degrees).
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class ImuToEuler(Node):
    def __init__(self):
        super().__init__('imu_to_euler_converter')

        # --- KONFIGURASI ---
        self.input_topic = '/imu_sensor_broadcaster/imu'
        self.output_topic = '/imu'

        # 1. Subscriber (Dengar data mentah)
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.listener_callback,
            10)

        # 2. Publisher (Kirim data matang)
        self.publisher_ = self.create_publisher(
            Vector3, self.output_topic, 10)

    def listener_callback(self, msg):
        # Ambil Quaternion
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w

        # === RUMUS MATEMATIKA (Quaternion -> Euler) ===

        # 1. Roll (Miring Kiri/Kanan) - Sumbu X
        t0 = +2.0 * (q_w * q_x + q_y * q_z)
        t1 = +1.0 - 2.0 * (q_x * q_x + q_y * q_y)
        roll_x = math.atan2(t0, t1)

        # 2. Pitch (Nunduk/Dangak) - Sumbu Y
        t2 = +2.0 * (q_w * q_y - q_z * q_x)
        # Safety clamp biar gak error matematik
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        # 3. Yaw (Hadap Kompas) - Sumbu Z
        t3 = +2.0 * (q_w * q_z + q_x * q_y)
        t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
        yaw_z = math.atan2(t3, t4)

        # === PUBLISH DATA ===
        vec_msg = Vector3()

        # Kita kirim dalam satuan DERAJAT (Degrees) biar gampang dibaca
        # manusia & debug. Kalau butuh Radian untuk kontrol PID, hapus
        # math.degrees().
        vec_msg.x = math.degrees(roll_x)  # Roll
        vec_msg.y = math.degrees(pitch_y)  # Pitch
        vec_msg.z = math.degrees(yaw_z)  # Yaw

        self.publisher_.publish(vec_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuToEuler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
