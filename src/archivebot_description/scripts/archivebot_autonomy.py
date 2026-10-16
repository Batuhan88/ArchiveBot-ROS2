#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.msg import Contacts


class ArchiveBotAutonomy(Node):

    def __init__(self):
        super().__init__('archivebot_autonomy')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Contacts,
            '/archivebot/bumper/contacts',
            self.contact_callback,
            10
        )

        self.front = 0.0
        self.left = 0.0
        self.right = 0.0

        self.last_scan = 0.0
        self.turn_until = 0.0
        self.turn_direction = 1.0
        self.state = ""

        self.create_timer(0.2, self.control)

        self.get_logger().info(
            "ArchiveBot otonom kontrol hazir."
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
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= math.radians(30):
                front.append(distance)

            elif math.radians(30) < angle < math.radians(90):
                left.append(distance)

            elif -math.radians(90) < angle < -math.radians(30):
                right.append(distance)

        self.front = min(front, default=0.0)
        self.left = min(left, default=0.0)
        self.right = min(right, default=0.0)

        self.last_scan = time.monotonic()

    def contact_callback(self, msg):

        for contact in msg.contacts:
            name1 = contact.collision1.name.lower()
            name2 = contact.collision2.name.lower()

            # Ignore floor contacts
            if "floor" in name1 or "floor" in name2:
                continue

            self.turn_direction = (
                1.0 if self.left >= self.right else -1.0
            )

            self.turn_until = time.monotonic() + 1.5

            self.get_logger().warn(
                "Tampon temasi! Yon degistiriliyor."
            )
            break

    def control(self):

        cmd = Twist()
        now = time.monotonic()

        if now - self.last_scan > 5.0:
            state = "SENSOR_BEKLENIYOR"

        elif now < self.turn_until:
            state = "DONUS"
            cmd.angular.z = 0.4 * self.turn_direction

        elif self.front < 0.60:
            self.turn_direction = (
                1.0 if self.left >= self.right else -1.0
            )

            self.turn_until = now + 1.5
            state = "ENGELDEN_KACINMA"
            cmd.angular.z = 0.4 * self.turn_direction

        else:
            state = "ILERI"
            cmd.linear.x = 0.10

        self.cmd_pub.publish(cmd)

        if state != self.state:
            self.state = state
            self.get_logger().info(
                f"Durum: {state} | "
                f"On: {self.front:.2f} m | "
                f"Sol: {self.left:.2f} m | "
                f"Sag: {self.right:.2f} m"
            )


def main():
    rclpy.init()
    node = ArchiveBotAutonomy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
