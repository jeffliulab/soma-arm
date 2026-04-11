#!/usr/bin/env python3
"""Publish Logitech C922 frames from native Windows into ROS 2.

Why this exists:
- C922 is clean in the native Windows Camera app.
- The `usbipd + WSL + UVC/MJPG` path causes horizontal tearing for this camera.
- We therefore capture on Windows, decode on Windows, and publish raw ROS
  image messages that WSL can subscribe to over DDS.
"""

from __future__ import annotations

import argparse
import os
import platform
import time
from dataclasses import dataclass

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def _decode_fourcc(value: float) -> str:
    code = int(value)
    if code <= 0:
        return "unknown"
    chars = [chr((code >> (8 * idx)) & 0xFF) for idx in range(4)]
    text = "".join(chars).strip("\x00").strip()
    return text or "unknown"


def _backend_name(code: int) -> str:
    known = {
        int(getattr(cv2, "CAP_ANY", 0)): "ANY",
        int(getattr(cv2, "CAP_DSHOW", -1)): "DSHOW",
        int(getattr(cv2, "CAP_MSMF", -1)): "MSMF",
        int(getattr(cv2, "CAP_V4L2", -1)): "V4L2",
        int(getattr(cv2, "CAP_GSTREAMER", -1)): "GSTREAMER",
    }
    return known.get(code, f"code={code}")


def _normalize_frame(frame) -> tuple[object, int, int]:
    if frame is None:
        raise ValueError("Camera returned an empty frame")

    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif len(frame.shape) == 3 and frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    elif len(frame.shape) != 3 or frame.shape[2] != 3:
        raise ValueError(f"Unsupported frame shape: {frame.shape}")

    height, width = frame.shape[:2]
    return frame, width, height


@dataclass(frozen=True)
class BackendCandidate:
    label: str
    code: int


class WindowsCameraRosPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("windows_camera_ros_publisher")
        self._args = args
        self._image_pub = self.create_publisher(
            Image,
            args.image_topic,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            args.camera_info_topic,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._capture = None
        self._capture_label = "unopened"
        self._last_open_attempt_monotonic = 0.0
        self._last_runtime_log_monotonic = 0.0
        self._published_frames = 0
        self._published_frames_since_log = 0
        self._reopen_cooldown_sec = 2.0

        timer_period = 1.0 / max(float(args.fps), 1.0)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            "Starting Windows camera ROS publisher | "
            f"device_index={args.device_index} requested={args.width}x{args.height}@{args.fps} "
            f"backend={args.backend} capture_format={args.capture_format} "
            f"image_topic={args.image_topic} camera_info_topic={args.camera_info_topic}"
        )
        self.get_logger().info(
            "ROS env | "
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')} "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')} "
            f"ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY', '(unset)')}"
        )

        self._try_open_capture(force=True)

    def destroy_node(self) -> bool:
        self._release_capture()
        return super().destroy_node()

    def _backend_candidates(self) -> list[BackendCandidate]:
        any_code = int(getattr(cv2, "CAP_ANY", 0))
        dshow_code = int(getattr(cv2, "CAP_DSHOW", any_code))
        msmf_code = int(getattr(cv2, "CAP_MSMF", any_code))

        requested = self._args.backend.lower()
        if requested == "msmf":
            labels = [BackendCandidate("MSMF", msmf_code)]
        elif requested == "dshow":
            labels = [BackendCandidate("DSHOW", dshow_code)]
        elif requested == "any":
            labels = [BackendCandidate("ANY", any_code)]
        else:
            if platform.system().lower() == "windows":
                labels = [
                    BackendCandidate("MSMF", msmf_code),
                    BackendCandidate("DSHOW", dshow_code),
                    BackendCandidate("ANY", any_code),
                ]
            else:
                labels = [BackendCandidate("ANY", any_code)]

        deduped: list[BackendCandidate] = []
        seen_codes: set[int] = set()
        for candidate in labels:
            if candidate.code in seen_codes:
                continue
            seen_codes.add(candidate.code)
            deduped.append(candidate)
        return deduped

    def _set_capture_properties(self, capture) -> None:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._args.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._args.height))
        capture.set(cv2.CAP_PROP_FPS, float(self._args.fps))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 1)

        requested_format = self._args.capture_format.lower()
        if requested_format == "mjpg":
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        elif requested_format == "yuy2":
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))

    def _try_open_capture(self, force: bool = False) -> bool:
        now = time.monotonic()
        if not force and now - self._last_open_attempt_monotonic < self._reopen_cooldown_sec:
            return False
        self._last_open_attempt_monotonic = now
        self._release_capture()

        for candidate in self._backend_candidates():
            self.get_logger().info(
                f"Trying backend {candidate.label} for camera index {self._args.device_index}"
            )
            capture = cv2.VideoCapture(self._args.device_index, candidate.code)
            if not capture or not capture.isOpened():
                if capture:
                    capture.release()
                continue

            self._set_capture_properties(capture)

            frame = None
            for _ in range(12):
                ok, frame = capture.read()
                if ok and frame is not None:
                    break
                time.sleep(0.03)
            else:
                capture.release()
                self.get_logger().warning(
                    f"Backend {candidate.label} opened the device but failed to deliver frames"
                )
                continue

            try:
                _, width, height = _normalize_frame(frame)
            except ValueError as exc:
                capture.release()
                self.get_logger().warning(
                    f"Backend {candidate.label} returned an unusable frame: {exc}"
                )
                continue

            actual_backend_code = int(capture.get(cv2.CAP_PROP_BACKEND))
            actual_backend = _backend_name(actual_backend_code)
            actual_fps = capture.get(cv2.CAP_PROP_FPS)
            actual_fourcc = _decode_fourcc(capture.get(cv2.CAP_PROP_FOURCC))

            self._capture = capture
            self._capture_label = candidate.label
            self.get_logger().info(
                "Camera opened successfully | "
                f"requested={self._args.width}x{self._args.height}@{self._args.fps} "
                f"backend={candidate.label} actual_backend={actual_backend} "
                f"actual={width}x{height}@{actual_fps:.2f} format={actual_fourcc}"
            )
            self._last_runtime_log_monotonic = time.monotonic()
            self._published_frames_since_log = 0
            return True

        self.get_logger().error(
            "Failed to open the camera on Windows. "
            "Check that the C922 is not attached to WSL and not occupied by another app."
        )
        return False

    def _release_capture(self) -> None:
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
            self._capture = None

    def _make_image_msg(self, frame, stamp) -> tuple[Image, int, int]:
        frame, width, height = _normalize_frame(frame)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._args.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = frame.tobytes()
        return msg, width, height

    def _make_camera_info_msg(self, width: int, height: int, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self._args.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = []
        msg.k = [0.0] * 9
        msg.r = [0.0] * 9
        msg.p = [0.0] * 12
        return msg

    def _log_runtime_stats(self, width: int, height: int) -> None:
        now = time.monotonic()
        if now - self._last_runtime_log_monotonic < self._args.runtime_log_interval_sec:
            return

        elapsed = max(now - self._last_runtime_log_monotonic, 1e-6)
        actual_fps = self._published_frames_since_log / elapsed
        self.get_logger().info(
            "Publishing | "
            f"backend={self._capture_label} size={width}x{height} "
            f"actual_publish_fps={actual_fps:.1f} total_frames={self._published_frames}"
        )
        self._last_runtime_log_monotonic = now
        self._published_frames_since_log = 0

    def _on_timer(self) -> None:
        if self._capture is None:
            self._try_open_capture()
            return

        ok, frame = self._capture.read()
        if not ok or frame is None:
            self.get_logger().warning(
                f"Camera read failed on backend {self._capture_label}; reopening after cooldown"
            )
            self._release_capture()
            return

        try:
            stamp = self.get_clock().now().to_msg()
            image_msg, width, height = self._make_image_msg(frame, stamp)
            camera_info_msg = self._make_camera_info_msg(width, height, stamp)
        except ValueError as exc:
            self.get_logger().warning(f"Dropping frame because it could not be converted: {exc}")
            return

        self._image_pub.publish(image_msg)
        self._camera_info_pub.publish(camera_info_msg)

        self._published_frames += 1
        self._published_frames_since_log += 1
        self._log_runtime_stats(width, height)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture C922 frames on Windows and publish them into ROS 2."
    )
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument(
        "--backend",
        choices=("auto", "msmf", "dshow", "any"),
        default="auto",
    )
    parser.add_argument(
        "--capture-format",
        choices=("auto", "mjpg", "yuy2"),
        default="mjpg",
        help="Requested local camera pixel format before Windows-side decoding",
    )
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument("--runtime-log-interval-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = WindowsCameraRosPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping camera publisher after Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
