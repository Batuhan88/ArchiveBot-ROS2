#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts


class ArchiveBotAutonomy(Node):

    def __init__(self):
        super().__init__('archivebot_autonomy_v2')

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Imu, '/archivebot/imu',
            self.imu_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Contacts, '/archivebot/bumper/contacts',
            self.contact_callback, 10
        )

        self.front = 0.0
        self.left = 0.0
        self.right = 0.0

        self.yaw = None
        self.last_scan = 0.0
        self.last_imu = 0.0

        self.state = 'BEKLE'
        self.start_yaw = None
        self.turn_direction = 1.0
        self.turn_start_time = 0.0
        self.turn_target = math.radians(60)

        self.contact_detected = False

        self.create_timer(0.2, self.control)

        self.get_logger().info(
            'ArchiveBot V2 - IMU destekli otonom sistem hazir.'
        )

    def scan_callback(self, msg):
        front = []
        left = []
        right = []

        for i, distance in enumerate(msg.ranges):

            if math.isnan(distance) or distance <= 0:
                continue

            if math.isinf(distance):
                distance = msg.range_max

            angle = msg.angle_min + i * msg.angle_increment

            angle = math.atan2(
                math.sin(angle), math.cos(angle)
            )

            if abs(angle) < math.radians(30):
                front.append(distance)

            elif math.radians(30) <= angle < math.radians(100):
                left.append(distance)

            elif -math.radians(100) < angle <= -math.radians(30):
                right.append(distance)

        self.front = min(front, default=0.0)
        self.left = min(left, default=0.0)
        self.right = min(right, default=0.0)

        self.last_scan = time.monotonic()

    def imu_callback(self, msg):
        q = msg.orientation

        # Quaternion -> yaw (radyan)
        siny = 2.0 * (
            q.w * q.z + q.x * q.y
        )
        cosy = 1.0 - 2.0 * (
            q.y * q.y + q.z * q.z
        )

        self.yaw = math.atan2(siny, cosy)
        self.last_imu = time.monotonic()

    def contact_callback(self, msg):
        for contact in msg.contacts:
            name1 = contact.collision1.name.lower()
            name2 = contact.collision2.name.lower()

            # Ignore floor contact
            if 'floor' in name1 or 'floor' in name2:
                continue

            if self.state != 'DONUS':
                self.contact_detected = True
            break

    def change_state(self, new_state):
        if self.state != new_state:
            self.state = new_state
            self.get_logger().info(
                f'Durum: {new_state} | '
                f'On: {self.front:.2f} m | '
                f'Sol: {self.left:.2f} m | '
                f'Sag: {self.right:.2f} m'
            )

    def start_turn(self):
        self.turn_direction = (
            1.0 if self.left >= self.right else -1.0
        )

        self.start_yaw = self.yaw
        self.turn_start_time = time.monotonic()

        self.change_state('DONUS')

    def control(self):
        now = time.monotonic()
        cmd = Twist()

        # Fail-safe: stop when sensor data is missing
        if (
            self.yaw is None
            or now - self.last_scan > 5.0
            or now - self.last_imu > 5.0
        ):
            self.change_state('SENSOR_BEKLENIYOR')
            self.publisher.publish(cmd)
            return

        if self.contact_detected:
            self.contact_detected = False

            self.get_logger().warn(
                'Tampon temasi! Robot yon degistiriyor.'
            )

            self.start_turn()

        if self.state == 'DONUS':
            difference = math.atan2(
                math.sin(self.yaw - self.start_yaw),
                math.cos(self.yaw - self.start_yaw)
            )

            turned = self.turn_direction * difference

            if turned >= self.turn_target:
                self.change_state('ILERI')

            elif now - self.turn_start_time > 25.0:
                self.change_state('DONUS_HATASI')

            else:
                cmd.angular.z = 0.4 * self.turn_direction

            self.publisher.publish(cmd)
            return

        if self.state == 'DONUS_HATASI':
            self.publisher.publish(cmd)
            return

        if self.front < 0.70:
            self.start_turn()
            cmd.angular.z = 0.4 * self.turn_direction

        else:
            self.change_state('ILERI')
            cmd.linear.x = 0.10

        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = ArchiveBotAutonomy()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if rclpy.ok():
                node.publisher.publish(Twist())
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
