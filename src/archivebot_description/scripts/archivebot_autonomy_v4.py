#!/usr/bin/env python3

import math
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


class ArchiveBotV3(Node):

    def __init__(self):
        super().__init__('archivebot_autonomy_v4')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            LaserScan, '/scan', self.scan_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/archivebot/imu', self.imu_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Contacts, '/archivebot/bumper/contacts',
            self.contact_cb, 10
        )

        self.create_subscription(
            String, '/archivebot/checkpoint_status',
            self.checkpoint_cb, 10
        )
        self.report_pub = self.create_publisher(
            String, '/archivebot/inspection_report', 10
        )
        self.last_color = 'NONE'
        self.color_streak = 0
        self.reported_colors = set()

        # Safe first test: no autonomous movement
        self.inspection_only = False

        self.front = None
        self.left = None
        self.right = None
        self.rear = None
        self.yaw = None
        self.position = None
        self.tipped = False
        self.hit = False

        self.last_scan = 0
        self.last_imu = 0
        self.last_odom = 0

        self.mode = 'BASLATILIYOR'
        self.direction = 1.0
        self.target_angle = math.radians(75)
        self.start_yaw = 0
        self.turn_started = 0

        self.back_started = 0
        self.back_position = None
        self.forward_started = 0
        self.forward_position = None

        self.create_timer(0.2, self.control)
        self.get_logger().info('ArchiveBot V4 otonom denetim hazir.')

    def scan_cb(self, msg):
        sectors = {'front': [], 'left': [],
                   'right': [], 'rear': []}

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or distance < msg.range_min:
                continue

            if math.isinf(distance):
                if distance < 0:
                    continue
                distance = msg.range_max

            if distance > msg.range_max:
                distance = msg.range_max

            a = msg.angle_min + i * msg.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            deg = math.degrees(a)

            if abs(deg) <= 25:
                sectors['front'].append(distance)
            elif 35 <= deg <= 100:
                sectors['left'].append(distance)
            elif -100 <= deg <= -35:
                sectors['right'].append(distance)
            elif abs(deg) >= 155:
                sectors['rear'].append(distance)

        for key in sectors:
            setattr(
                self, key,
                min(sectors[key]) if sectors[key] else None
            )

        self.last_scan = time.monotonic()

    def imu_cb(self, msg):
        q = msg.orientation

        self.yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        roll = math.atan2(
            2 * (q.w*q.x + q.y*q.z),
            1 - 2 * (q.x*q.x + q.y*q.y)
        )

        pitch = math.asin(max(
            -1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))
        ))

        if abs(roll) > 0.7 or abs(pitch) > 0.7:
            self.tipped = True

        self.last_imu = time.monotonic()

    def odom_cb(self, msg):
        p = msg.pose.pose.position

        if math.isfinite(p.x) and math.isfinite(p.y):
            self.position = (p.x, p.y)
            self.last_odom = time.monotonic()

    def contact_cb(self, msg):
        for c in msg.contacts:
            a = c.collision1.name.lower()
            b = c.collision2.name.lower()

            if 'floor' in a or 'floor' in b:
                continue

            self.hit = True
            break

    def distance_from(self, p):
        if p is None or self.position is None:
            return 0.0

        return math.hypot(
            self.position[0]-p[0],
            self.position[1]-p[1]
        )

    def checkpoint_cb(self, msg):
        color = msg.data.strip().upper()

        if color not in ('RED', 'GREEN', 'NONE'):
            return

        if color == self.last_color:
            self.color_streak += 1
        else:
            self.last_color = color
            self.color_streak = 1

        if (
            color in ('RED', 'GREEN')
            and self.color_streak >= 5
            and color not in self.reported_colors
            and self.position is not None
            and time.monotonic() - self.last_odom < 5.0
        ):
            self.reported_colors.add(color)

            if self.position is not None:
                x, y = self.position
                location = f"x={x:.2f}, y={y:.2f}"
            else:
                location = "konum bilinmiyor"

            report = f"{color} CHECKPOINT | {location}"

            self.report_pub.publish(String(data=report))
            self.get_logger().info(
                "ARSIV DENETIMI: " + report
            )

    def set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.get_logger().info('Durum: ' + mode)

            if mode == 'ILERI':
                self.forward_position = self.position
                self.forward_started = time.monotonic()

    def start_turn(self, degrees=75):
        self.direction = (
            1.0 if self.left >= self.right else -1.0
        )
        self.start_yaw = self.yaw
        self.target_angle = math.radians(degrees)
        self.turn_started = time.monotonic()
        self.set_mode('DONUS')

    def start_recovery(self):
        if self.rear is not None and self.rear > 0.85:
            self.back_position = self.position
            self.back_started = time.monotonic()
            self.set_mode('GERI')
        else:
            self.set_mode('ACIL_DUR')

    def control(self):
        now = time.monotonic()
        cmd = Twist()

        if self.inspection_only:
            self.pub.publish(cmd)
            return

        ready = (
            self.yaw is not None
            and self.position is not None
            and all(getattr(self, x) is not None
                    for x in ('front','left','right','rear'))
            and now-self.last_scan < 8
            and now-self.last_imu < 8
            and now-self.last_odom < 8
        )

        if self.mode == 'ACIL_DUR':
            self.pub.publish(cmd)
            return

        if not ready:
            self.set_mode('SENSOR_BEKLE')
            self.pub.publish(cmd)
            return

        if self.tipped:
            self.set_mode('ACIL_DUR')
            self.pub.publish(cmd)
            return

        if self.hit:
            self.hit = False
            self.get_logger().info('Tampon temasi algilandi.')
            self.start_recovery()

        if self.mode == 'GERI':
            if self.rear < 0.55:
                self.set_mode('ACIL_DUR')
            elif self.distance_from(self.back_position) >= 0.12:
                self.start_turn(100)
            elif now-self.back_started > 15:
                self.set_mode('ACIL_DUR')
            else:
                cmd.linear.x = -0.06

        elif self.mode == 'DONUS':
            delta = math.atan2(
                math.sin(self.yaw-self.start_yaw),
                math.cos(self.yaw-self.start_yaw)
            )
            turned = self.direction * delta

            if turned >= self.target_angle:
                self.set_mode('ILERI')
            elif now-self.turn_started > 35:
                self.set_mode('ACIL_DUR')
            else:
                cmd.angular.z = self.direction * 0.35

        elif self.front < 0.85:
            self.start_turn()
            cmd.angular.z = self.direction * 0.35

        else:
            if self.mode != 'ILERI':
                self.set_mode('ILERI')

            elapsed = now-self.forward_started
            moved = self.distance_from(self.forward_position)

            if elapsed > 15:
                if moved < 0.04:
                    self.get_logger().info(
                        'Ilerleme yok. Kurtarma deneniyor.'
                    )
                    self.start_recovery()
                else:
                    self.forward_position = self.position
                    self.forward_started = now

            if self.mode == 'ILERI':
                cmd.linear.x = (
                    0.04 if self.front < 1.2 else 0.08
                )
            elif self.mode == 'GERI':
                cmd.linear.x = -0.06

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ArchiveBotV3()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            try:
                node.pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
