#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts

# First test: travel only one meter
TEST_MODE = False

START = (-4.5, -3.3)

if TEST_MODE:
    ROUTE = [
        ("TEST_1M", -3.5, -3.3, None)
    ]
else:
    ROUTE = [
        ("GUNEY_1", 0.0, -3.3, None),
        ("DOGU_KORIDOR", 4.6, -3.3, None),
        ("KIRMIZI", 4.6, -2.5, "RED"),
        ("YESIL", 4.6, 2.5, "GREEN"),
        ("DONUS_SERIT_GIRISI", 4.95, 1.0, None),
        ("KIRMIZI_DONUS", 4.95, -2.5, None),
        ("DOGU_DONUS", 4.95, -3.3, None),
        ("GUNEY_DONUS", 0.0, -3.3, None),
        ("BASLANGIC", -4.5, -3.3, None)
    ]


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def angle_error(target, actual):
    return math.atan2(
        math.sin(target - actual),
        math.cos(target - actual)
    )


class ArchiveBotNavigation(Node):
    def __init__(self):
        super().__init__('archivebot_navigation_v11')

        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.create_subscription(
            LaserScan, '/scan', self.scan_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, '/archivebot/ground_truth/odom', self.odom_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, '/archivebot/imu', self.imu_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            String, '/archivebot/checkpoint_status',
            self.color_cb, 10
        )
        self.create_subscription(
            Contacts, '/archivebot/bumper/contacts',
            self.contact_cb, 10
        )

        self.position = None
        self.yaw = None
        self.origin = None
        self.front = None
        self.nearest_obstacle = None
        self.tipped = False
        self.hit = False

        self.last_odom = 0.0
        self.last_scan = 0.0
        self.last_imu = 0.0

        self.color = "NONE"
        self.color_count = 0
        self.last_color_time = 0.0

        self.index = 0
        self.reported_checkpoints = set()
        self.mode = "BEKLE"
        self.look_started = 0.0

        self.progress_position = None
        self.progress_time = 0.0

        self.create_timer(0.2, self.control)

        self.get_logger().info(
            "ArchiveBot V8 navigasyon hazir."
        )

    def scan_cb(self, msg):
        values = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment

            if abs(angle_error(angle, 0.0)) > math.radians(25):
                continue

            if math.isinf(distance) and distance > 0:
                distance = msg.range_max

            if (math.isfinite(distance)
                    and msg.range_min <= distance <= msg.range_max):
                values.append(distance)

        self.front = min(values) if values else None

        # Park icin 360 derece LiDAR mesafesi
        clearances = []
        for d in msg.ranges:
            if math.isinf(d) and d > 0:
                clearances.append(msg.range_max)
            elif math.isfinite(d) and msg.range_min <= d <= msg.range_max:
                clearances.append(d)

        self.nearest_obstacle = (
            min(clearances) if clearances else None
        )
        self.last_scan = time.monotonic()

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        if not all(map(math.isfinite, (p.x, p.y, yaw))):
            return

        self.position = (p.x, p.y)
        self.yaw = yaw
        self.last_odom = time.monotonic()

        if self.origin is None:
            self.origin = (p.x, p.y, yaw)

    def imu_cb(self, msg):
        q = msg.orientation
        roll = math.atan2(
            2*(q.w*q.x + q.y*q.z),
            1-2*(q.x*q.x + q.y*q.y)
        )
        pitch = math.asin(max(
            -1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))
        ))

        if abs(roll) > 0.65 or abs(pitch) > 0.65:
            self.tipped = True

        self.last_imu = time.monotonic()

    def color_cb(self, msg):
        color = msg.data.strip().upper()
        if color == self.color:
            self.color_count += 1
        else:
            self.color = color
            self.color_count = 1

        self.last_color_time = time.monotonic()

    def contact_cb(self, msg):
        for c in msg.contacts:
            names = (
                c.collision1.name.lower(),
                c.collision2.name.lower()
            )

            if any('floor' in name for name in names):
                continue

            self.hit = True
            break

    def record_inspection(self, checkpoint, wx, wy, method):
        import csv
        from datetime import datetime
        from pathlib import Path

        if checkpoint in self.reported_checkpoints:
            return

        report = Path.home() / "archivebot_ws/reports/archivebot_inspections.csv"

        try:
            report.parent.mkdir(parents=True, exist_ok=True)
            new_file = not report.exists() or report.stat().st_size == 0

            px, py = self.position if self.position else (None, None)

            with report.open("a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)

                if new_file:
                    writer.writerow([
                        "timestamp", "checkpoint", "result",
                        "method", "target_world_x", "target_world_y",
                        "odom_x", "odom_y", "detected_color"
                    ])

                writer.writerow([
                    datetime.now().astimezone().isoformat(timespec="seconds"),
                    checkpoint,
                    "BASARILI",
                    method,
                    wx, wy,
                    px, py,
                    self.color
                ])

            self.reported_checkpoints.add(checkpoint)
            self.get_logger().info("CSV kaydedildi: " + str(report))

        except OSError as e:
            self.get_logger().error("CSV yazma hatasi: " + str(e))

    def set_mode(self, mode):
        if mode != self.mode:
            self.mode = mode
            self.progress_position = None
            self.get_logger().info("Durum: " + mode)

    def stop(self, reason):
        self.get_logger().warning(reason)
        self.set_mode("GUVENLI_DUR")

    def target_in_odom(self, world_x, world_y):
        # Gazebo ground truth zaten dunya koordinatlarinda.
        return (world_x, world_y)

    def control(self):
        cmd = Twist()
        now = time.monotonic()

        if self.mode in ("GUVENLI_DUR", "TAMAMLANDI"):
            self.pub.publish(cmd)
            return

        ready = (
            self.position is not None
            and self.yaw is not None
            and self.front is not None
            and self.origin is not None
            and now - self.last_odom < 8
            and now - self.last_scan < 8
            and now - self.last_imu < 8
        )

        if not ready:
            self.pub.publish(cmd)
            return

        if self.tipped or self.hit:
            self.stop("Devrilme veya tampon temasi!")
            self.pub.publish(cmd)
            return

        if self.mode == "BEKLE":
            self.set_mode("NAVIGASYON")

        # Son hedefte baslangic yonune don
        if self.mode == "PARK_YONU":
            if (self.nearest_obstacle is None
                    or self.nearest_obstacle < 0.80):
                self.stop("Park donusu guvensiz: LiDAR engel algiladi.")
                self.pub.publish(cmd)
                return

            if now - self.parking_started > 90:
                self.stop("Park donusu zaman asimi.")
                self.pub.publish(cmd)
                return

            error = angle_error(0.0, self.yaw)

            if abs(error) <= 0.08:
                self.index += 1
                self.set_mode("TAMAMLANDI")
            else:
                speed = 0.10 if abs(error) < 0.30 else 0.17
                cmd.angular.z = math.copysign(speed, error)

            self.pub.publish(cmd)
            return

        if self.index >= len(ROUTE):
            self.set_mode("TAMAMLANDI")
            self.pub.publish(cmd)
            return

        name, wx, wy, checkpoint = ROUTE[self.index]
        tx, ty = self.target_in_odom(wx, wy)

        px, py = self.position
        distance = math.hypot(tx-px, ty-py)

        # Green checkpoint: confirm safely using camera.
        if (
            name == "YESIL"
            and distance <= 1.35
            and self.color == "GREEN"
            and self.color_count >= 5
            and now - self.last_color_time < 3.0
        ):
            self.get_logger().info(
                "DENETIM BASARILI: GREEN | "
                "Guvenli mesafeden kamera ile dogrulandi"
            )
            self.record_inspection(
                "GREEN", wx, wy, "GUVENLI_MESAFE_KAMERA"
            )
            self.index += 1
            self.set_mode("NAVIGASYON")
            self.pub.publish(Twist())
            return

        if self.mode == "PANEL_KONTROL":
            # Face +X toward the colored panel.
            desired_yaw = self.origin[2]
            error = angle_error(desired_yaw, self.yaw)

            if abs(error) > 0.12:
                cmd.angular.z = math.copysign(0.25, error)

            elif (
                self.color == checkpoint
                and self.color_count >= 5
                and self.last_color_time >= self.look_started
                and now-self.last_color_time < 5
            ):
                self.get_logger().info(
                    f"DENETIM BASARILI: {checkpoint} "
                    f"| Hedef: ({wx}, {wy})"
                )
                self.record_inspection(
                    checkpoint, wx, wy, "KAMERA"
                )
                self.index += 1
                self.set_mode("NAVIGASYON")

            elif now-self.look_started > 45:
                self.stop("Panel dogrulanamadi.")

            self.pub.publish(cmd)
            return

        if distance < 0.22:
            self.get_logger().info(
                f"Hedefe ulasildi: {name}"
            )

            if checkpoint is not None:
                self.look_started = now
                self.set_mode("PANEL_KONTROL")
            else:
                if name == "BASLANGIC":
                    self.parking_started = now
                    self.set_mode("PARK_YONU")
                else:
                    self.index += 1
                    self.set_mode("NAVIGASYON")

            self.pub.publish(cmd)
            return

        heading = math.atan2(ty-py, tx-px)
        error = angle_error(heading, self.yaw)

        if abs(error) > 0.16:
            cmd.angular.z = math.copysign(0.35, error)
            self.progress_position = None

        else:
            if self.front < 0.80:
                self.stop(
                    f"ENGEL ALGILANDI | "
                    f"On: {self.front:.2f} m | "
                    f"Konum: ({px:.2f}, {py:.2f}) | "
                    f"Hedef: {name}"
                )

            else:
                cmd.linear.x = 0.08 if distance < 0.8 else 0.18

                if self.progress_position is None:
                    self.progress_position = self.position
                    self.progress_time = now

                elif now-self.progress_time > 40:
                    moved = math.hypot(
                        px-self.progress_position[0],
                        py-self.progress_position[1]
                    )

                    if moved < 0.08:
                        self.stop("Robot ilerleyemiyor.")
                        cmd = Twist()
                    else:
                        self.progress_position = self.position
                        self.progress_time = now

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = ArchiveBotNavigation()

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
