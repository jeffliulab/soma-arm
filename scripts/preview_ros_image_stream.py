#!/usr/bin/env python3
"""Preview a ROS 2 Image topic in a simple OpenCV window.

Why this exists:
- `rqt_image_view` is sometimes inconvenient for quick bridge debugging
- we want a direct, lightweight way to see `/camera/image_raw`
- it reuses the same "prefer system OpenCV over headless venv OpenCV" trick
  as the local camera preview helper
"""

from __future__ import annotations

import argparse
import site
import sys
import time
from pathlib import Path


def _strip_user_site_packages() -> None:
    user_site = site.getusersitepackages()
    if user_site in sys.path:
        sys.path.remove(user_site)


def _strip_virtualenv_site_packages() -> None:
    if sys.prefix == sys.base_prefix:
        return

    venv_prefix = Path(sys.prefix).resolve()
    filtered = []
    for path in sys.path:
        try:
            resolved = Path(path).resolve()
        except (OSError, RuntimeError):
            filtered.append(path)
            continue

        if (
            "site-packages" in path or "dist-packages" in path
        ) and (resolved == venv_prefix or venv_prefix in resolved.parents):
            continue
        filtered.append(path)
    sys.path[:] = filtered


_strip_user_site_packages()
_strip_virtualenv_site_packages()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSPresetProfiles  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


def _convert_to_bgr(msg: Image) -> np.ndarray:
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    encoding = msg.encoding.lower()
    raw = np.frombuffer(msg.data, dtype=np.uint8)

    if raw.size < height * step:
        raise ValueError(
            f"Image buffer is truncated: expected at least {height * step} bytes, got {raw.size}"
        )

    rows = raw[: height * step].reshape((height, step))

    if encoding == "bgr8":
        return rows[:, : width * 3].reshape((height, width, 3)).copy()

    if encoding == "rgb8":
        frame = rows[:, : width * 3].reshape((height, width, 3)).copy()
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if encoding == "mono8":
        frame = rows[:, :width].copy()
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    raise ValueError(
        f"Unsupported encoding '{msg.encoding}'. "
        "This preview currently supports bgr8, rgb8, and mono8."
    )


class RosImagePreview(Node):
    def __init__(self, topic: str, log_interval_sec: float) -> None:
        super().__init__("preview_ros_image_stream")
        self._latest_frame: np.ndarray | None = None
        self._received_frames = 0
        self._received_since_log = 0
        self._last_log_monotonic = time.monotonic()
        self._log_interval_sec = log_interval_sec
        self._error_logged = False
        self.create_subscription(
            Image,
            topic,
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    @property
    def latest_frame(self) -> np.ndarray | None:
        return self._latest_frame

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_frame = _convert_to_bgr(msg)
        except ValueError as exc:
            if not self._error_logged:
                self.get_logger().warning(str(exc))
                self._error_logged = True
            return

        self._received_frames += 1
        self._received_since_log += 1
        now = time.monotonic()
        if now - self._last_log_monotonic >= self._log_interval_sec:
            elapsed = max(now - self._last_log_monotonic, 1e-6)
            fps = self._received_since_log / elapsed
            self.get_logger().info(
                f"Receiving preview frames at {fps:.1f} FPS | total_frames={self._received_frames}"
            )
            self._last_log_monotonic = now
            self._received_since_log = 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--window-name", default="SOMA ROS Camera Preview")
    parser.add_argument("--log-interval-sec", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = RosImagePreview(topic=args.topic, log_interval_sec=args.log_interval_sec)
    print(f"Previewing ROS image topic {args.topic}. Press q to quit.")

    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                return 0
            frame = node.latest_frame
            if frame is not None:
                try:
                    cv2.imshow(args.window_name, frame)
                except cv2.error as exc:
                    print("OpenCV GUI preview is unavailable in this interpreter.")
                    print(f"Original error: {exc}")
                    return 1
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
