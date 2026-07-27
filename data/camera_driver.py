#!/usr/bin/env python3
"""ROS 2 driver for the Raspberry Pi camera UDP stream.

Publishes unprocessed BGR frames to /camera/image_raw. Vision nodes (OpenCV,
YOLO, etc.) subscribe to this topic instead of opening the camera.
"""

import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")
        self.declare_parameter("stream_url", "udp://127.0.0.1:8554")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("publish_rate", 10.0)

        self.stream_url = self.get_parameter("stream_url").value
        self.frame_id = self.get_parameter("frame_id").value
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0:
            raise ValueError("publish_rate must be greater than zero")

        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "fflags;nobuffer|max_delay;50000"
        self.capture = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 1)

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self._capture_thread.start()
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_frame)
        self.get_logger().info(f"Camera driver started: {self.stream_url}")

    def _capture_frames(self):
        while self._running and rclpy.ok():
            if not self.capture.isOpened():
                self.get_logger().warn("Camera stream is unavailable; reconnecting...")
                self.capture.open(self.stream_url, cv2.CAP_FFMPEG)
                time.sleep(0.5)
                continue

            ok, frame = self.capture.read()
            if ok:
                with self._frame_lock:
                    self._latest_frame = frame
            else:
                time.sleep(0.05)

    def _publish_frame(self):
        with self._frame_lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None
        if frame is None:
            return

        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        self._running = False
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
