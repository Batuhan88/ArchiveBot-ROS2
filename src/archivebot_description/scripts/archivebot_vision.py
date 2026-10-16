#!/usr/bin/env python3

import time
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class ArchiveBotVision(Node):

    def __init__(self):
        super().__init__('archivebot_vision')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/archivebot/camera/image',
            self.process_image,
            qos_profile_sensor_data
        )

        self.image_pub = self.create_publisher(
            Image,
            '/archivebot/camera/debug_image',
            qos_profile_sensor_data
        )

        self.status_pub = self.create_publisher(
            String,
            '/archivebot/checkpoint_status',
            10
        )

        self.last_status = None
        self.last_log = 0.0

        self.get_logger().info(
            'ArchiveBot renk algilama sistemi baslatildi.'
        )

    def detect_color(self, hsv, lower, upper):
        mask = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8)
        )

        kernel = np.ones((3, 3), np.uint8)

        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        return [
            c for c in contours
            if cv2.contourArea(c) >= 50
        ]

    def process_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(str(exc))
            return

        hsv = cv2.cvtColor(
            frame, cv2.COLOR_BGR2HSV
        )

        # Red has two hue ranges in HSV
        red1 = self.detect_color(
            hsv, [0, 110, 70], [10, 255, 255]
        )

        red2 = self.detect_color(
            hsv, [170, 110, 70], [179, 255, 255]
        )

        green = self.detect_color(
            hsv, [35, 100, 65], [85, 255, 255]
        )

        red = red1 + red2

        for contour in red:
            x, y, w, h = cv2.boundingRect(contour)

            cv2.rectangle(
                frame,
                (x, y),
                (x+w, y+h),
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame, 'RED CHECKPOINT',
                (x, max(15, y-8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1
            )

        for contour in green:
            x, y, w, h = cv2.boundingRect(contour)

            cv2.rectangle(
                frame,
                (x, y),
                (x+w, y+h),
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame, 'GREEN CHECKPOINT',
                (x, max(15, y-8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1
            )

        if red:
            status = 'RED'
        elif green:
            status = 'GREEN'
        else:
            status = 'NONE'

        self.status_pub.publish(
            String(data=status)
        )

        now = time.monotonic()

        if status != self.last_status or now-self.last_log > 5:
            self.get_logger().info(
                f'Kontrol noktasi: {status}'
            )
            self.last_status = status
            self.last_log = now

        output = self.bridge.cv2_to_imgmsg(
            frame, encoding='bgr8'
        )

        output.header = msg.header

        self.image_pub.publish(output)


def main():
    rclpy.init()
    node = ArchiveBotVision()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
