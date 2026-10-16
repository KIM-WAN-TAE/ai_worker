#!/usr/bin/env python3

"""Display the left and right wrist camera raw color streams side by side."""

import argparse
import threading
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_LEFT_TOPIC = "/camera_left/camera_left/color/image_rect_raw"
DEFAULT_RIGHT_TOPIC = "/camera_right/camera_right/color/image_rect_raw"
WINDOW_NAME = "F2 wrist cameras (raw)"


class WristCameraViewer(Node):
    def __init__(self, left_topic: str, right_topic: str, display_height: int) -> None:
        super().__init__("wrist_camera_raw_viewer")
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._left_frame: Optional[np.ndarray] = None
        self._right_frame: Optional[np.ndarray] = None
        self._left_topic = left_topic
        self._right_topic = right_topic
        self._display_height = display_height
        self._should_stop = False
        self._left_frame_received = False
        self._right_frame_received = False

        self.create_subscription(
            Image,
            left_topic,
            lambda msg: self._store_frame(msg, is_left=True),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            right_topic,
            lambda msg: self._store_frame(msg, is_left=False),
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / 30.0, self._render)

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, display_height * 8 // 3, display_height)
        cv2.moveWindow(WINDOW_NAME, 10, 45)
        try:
            cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_TOPMOST, 1)
        except cv2.error:
            self.get_logger().warning("This OpenCV backend cannot keep the window on top.")
        self.get_logger().info(f"Left raw topic:  {left_topic}")
        self.get_logger().info(f"Right raw topic: {right_topic}")
        self.get_logger().info("Press q or Esc in the image window to quit.")

    @property
    def should_stop(self) -> bool:
        return self._should_stop

    def _store_frame(self, msg: Image, *, is_left: bool) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().error(f"Could not decode image: {error}")
            return

        with self._lock:
            if is_left:
                self._left_frame = frame
                if not self._left_frame_received:
                    self._left_frame_received = True
                    self.get_logger().info(
                        f"Received first left frame: {frame.shape[1]}x{frame.shape[0]}"
                    )
            else:
                self._right_frame = frame
                if not self._right_frame_received:
                    self._right_frame_received = True
                    self.get_logger().info(
                        f"Received first right frame: {frame.shape[1]}x{frame.shape[0]}"
                    )

    def _render(self) -> None:
        with self._lock:
            left = None if self._left_frame is None else self._left_frame.copy()
            right = None if self._right_frame is None else self._right_frame.copy()

        left_panel = self._make_panel(left, "LEFT WRIST", self._left_topic)
        right_panel = self._make_panel(right, "RIGHT WRIST", self._right_topic)
        canvas = np.hstack((left_panel, right_panel))

        cv2.imshow(WINDOW_NAME, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self._should_stop = True

    def _make_panel(
        self,
        frame: Optional[np.ndarray],
        label: str,
        topic: str,
    ) -> np.ndarray:
        if frame is None:
            panel = np.zeros((self._display_height, self._display_height * 4 // 3, 3), dtype=np.uint8)
            cv2.putText(
                panel,
                "Waiting for image...",
                (30, self._display_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            height, width = frame.shape[:2]
            scaled_width = max(1, round(width * self._display_height / height))
            panel = cv2.resize(
                frame,
                (scaled_width, self._display_height),
                interpolation=cv2.INTER_AREA,
            )

        cv2.rectangle(panel, (0, 0), (panel.shape[1], 62), (0, 0, 0), -1)
        cv2.putText(panel, label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.putText(panel, topic, (12, 49), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)
        return panel

    def close(self) -> None:
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Show the left and right F2 wrist raw color images side by side."
    )
    parser.add_argument("--left-topic", default=DEFAULT_LEFT_TOPIC)
    parser.add_argument("--right-topic", default=DEFAULT_RIGHT_TOPIC)
    parser.add_argument(
        "--height",
        type=int,
        default=480,
        help="Displayed height of each camera panel in pixels (default: 480).",
    )
    args = parser.parse_args()
    if args.height < 120:
        parser.error("--height must be at least 120")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    viewer = WristCameraViewer(args.left_topic, args.right_topic, args.height)
    try:
        while rclpy.ok() and not viewer.should_stop:
            rclpy.spin_once(viewer, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        viewer.close()
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
