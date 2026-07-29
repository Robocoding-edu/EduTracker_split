#!/usr/bin/env python3
"""OpenCV node that consumes /camera/image_raw.

YOLO and other ROS vision nodes can subscribe to that same raw camera topic.
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker


class ColorDetector:
    """Detect red objects and estimate their position in the robot frame."""

    def __init__(self):
        self.lower_red1 = np.array([0, 120, 70])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])
        self.upper_red2 = np.array([180, 255, 255])
        self.min_contour_area = 200
        self.real_object_width = 0.15
        self.focal_length = 280.0
        self.fov_h = math.radians(62.2)

    def process_frame(self, frame):
        output_frame = frame.copy()
        hsv_image = cv2.cvtColor(output_frame, cv2.COLOR_BGR2HSV)
        center_x_frame = frame.shape[1] / 2.0

        mask = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
        mask += cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
        mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            if cv2.contourArea(contour) <= self.min_contour_area:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            distance = (self.real_object_width * self.focal_length) / width
            offset_x = x + width / 2.0 - center_x_frame
            angle = (offset_x / center_x_frame) * (self.fov_h / 2.0)
            robot_x = distance * math.cos(angle)
            robot_y = -distance * math.sin(angle)
            objects.append((robot_x, robot_y))
            cv2.rectangle(output_frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.putText(output_frame, f"RED OBJ: {distance:.2f}m", (x, max(y - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        return output_frame, objects


class OpenCVNode(Node):
    def __init__(self):
        super().__init__("opencv_processor")
        self.bridge = CvBridge()
        self.detector = ColorDetector()
        self.marker_id = 0
        self.processed_publisher = self.create_publisher(Image, "/camera/image_processed", 1)
        self.marker_publisher = self.create_publisher(Marker, "/robot/detected_objects", 10)
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, 1)
        self.get_logger().info("OpenCV processor subscribed to /camera/image_raw")

    def _on_image(self, image_message):
        try:
            frame = self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"Cannot convert camera frame: {error}")
            return

        processed_frame, objects = self.detector.process_frame(frame)
        processed_message = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
        processed_message.header = image_message.header
        self.processed_publisher.publish(processed_message)
        for robot_x, robot_y in objects:
            self._publish_marker(image_message.header.stamp, robot_x, robot_y)

    def _publish_marker(self, stamp, x, y):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "base_link"
        marker.ns = "red_objects"
        marker.id = self.marker_id
        self.marker_id = (self.marker_id + 1) % 20
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
