#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from collections import OrderedDict


class CameraViewer(Node):
    # Label rendering constants
    _LABEL_CHAR_WIDTH = 14
    _LABEL_PADDING = 10
    _LABEL_HEIGHT = 36
    _LABEL_FONT_SCALE = 0.9
    _LABEL_THICKNESS = 2

    def __init__(self):
        super().__init__('camera_viewer')

        # Parameters
        self.declare_parameter('window_width', 1600)
        self.declare_parameter('window_height', 900)
        self.declare_parameter('camera_topics', [
            '/uav1/bluefox_down/image_raw',
            '/uav1/bluefox_reverse/image_raw',
            '/uav1/bluefox_front/image_raw',
        ])

        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        camera_topics = self.get_parameter('camera_topics').value

        self.bridge = CvBridge()
        self.images = OrderedDict()
        self.lock = threading.Lock()

        # Subscribe to each camera topic
        for topic in camera_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic: self._image_callback(msg, t),
                10,
            )
            self.get_logger().info(f'Subscribed to: {topic}')

        # Display loop runs in a separate thread so ROS spin is not blocked
        self._running = True
        self._display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self._display_thread.start()

    def _image_callback(self, msg: Image, topic: str) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.images[topic] = cv_image
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image from {topic}: {e}')

    def _display_loop(self) -> None:
        cv2.namedWindow('Drone Cameras', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone Cameras', self.window_width, self.window_height)

        while self._running and rclpy.ok():
            with self.lock:
                images_snapshot = dict(self.images)

            canvas = self._build_canvas(images_snapshot)
            cv2.imshow('Drone Cameras', canvas)

            key = cv2.waitKey(30) & 0xFF
            if key == 27:  # ESC
                self.get_logger().info('ESC pressed – shutting down camera_viewer')
                self._running = False
                rclpy.shutdown()
                break

        cv2.destroyAllWindows()

    def _build_canvas(self, images_dict: dict) -> np.ndarray:
        canvas = np.zeros((self.window_height, self.window_width, 3), dtype=np.uint8)

        if not images_dict:
            text = 'Waiting for camera streams...'
            (text_w, text_h), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 2
            )
            x = (self.window_width - text_w) // 2
            y = (self.window_height + text_h) // 2
            cv2.putText(
                canvas,
                text,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (100, 100, 100),
                2,
            )
            return canvas

        num = len(images_dict)

        # Compute grid dimensions
        if num <= 2:
            cols, rows = num, 1
        elif num <= 4:
            cols, rows = 2, 2
        else:
            cols = 3
            rows = (num + 2) // 3

        cell_w = self.window_width // cols
        cell_h = self.window_height // rows

        for idx, (topic, image) in enumerate(images_dict.items()):
            if image is None:
                continue

            row = idx // cols
            col = idx % cols

            resized = cv2.resize(image, (cell_w, cell_h))

            y0 = row * cell_h
            x0 = col * cell_w
            canvas[y0:y0 + cell_h, x0:x0 + cell_w] = resized

            # Label: use last segment of the topic path as the camera name
            label = topic.split('/')[-2] if topic.count('/') >= 2 else topic
            bg_w = len(label) * self._LABEL_CHAR_WIDTH + self._LABEL_PADDING
            cv2.rectangle(canvas, (x0, y0), (x0 + bg_w, y0 + self._LABEL_HEIGHT), (0, 0, 0), -1)
            cv2.putText(
                canvas,
                label,
                (x0 + 6, y0 + 26),
                cv2.FONT_HERSHEY_SIMPLEX,
                self._LABEL_FONT_SCALE,
                (0, 255, 0),
                self._LABEL_THICKNESS,
            )

        return canvas

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
