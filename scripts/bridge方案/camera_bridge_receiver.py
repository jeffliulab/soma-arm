#!/usr/bin/env python3
"""Receive camera frames from Windows and republish them as ROS 2 topics in WSL."""

from __future__ import annotations

import argparse
import json
import site
import socket
import struct
import sys
import threading
import time


def _strip_user_site_packages() -> None:
    user_site = site.getusersitepackages()
    if user_site in sys.path:
        sys.path.remove(user_site)


_strip_user_site_packages()

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402
from rclpy.qos import QoSReliabilityPolicy  # noqa: E402
from rclpy.qos import QoSHistoryPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402


HEADER_STRUCT = struct.Struct("!II")


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Socket closed while receiving a frame")
        chunks.extend(chunk)
    return bytes(chunks)


class CameraBridgeReceiver(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("camera_bridge_receiver")
        self._args = args
        publisher_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._image_pub = self.create_publisher(
            Image,
            args.image_topic,
            publisher_qos,
        )
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            args.camera_info_topic,
            publisher_qos,
        )
        self._received_frames = 0
        self._received_frames_since_log = 0
        self._last_runtime_log_monotonic = time.monotonic()
        self._latest_packet_ready = threading.Condition()
        self._latest_packet_seq = 0
        self._latest_packet: tuple[bytes, dict] | None = None
        self._reader_error: Exception | None = None

    def log_startup(self) -> None:
        self.get_logger().info(
            "Starting WSL camera bridge receiver | "
            f"host={self._args.host} port={self._args.port} "
            f"image_topic={self._args.image_topic} camera_info_topic={self._args.camera_info_topic}"
        )

    def run(self) -> int:
        self.log_startup()
        while rclpy.ok():
            try:
                self.get_logger().info(
                    f"Connecting to Windows camera bridge at {self._args.host}:{self._args.port}"
                )
                with socket.create_connection((self._args.host, self._args.port), timeout=5.0) as sock:
                    # The bridge can legitimately stall for a few seconds when
                    # the Windows side switches camera backend, is warming up,
                    # or is under heavy JPEG load. Keep the connect timeout
                    # short, but once connected prefer a blocking socket so we
                    # don't mistake a slow frame burst for a dead bridge.
                    sock.settimeout(None)
                    self.get_logger().info("Connected to Windows camera bridge")
                    self._run_connection(sock)
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                self.get_logger().warning(f"Camera bridge disconnected: {exc}")
                time.sleep(self._args.reconnect_delay_sec)
        return 0

    def _run_connection(self, sock: socket.socket) -> None:
        with self._latest_packet_ready:
            self._latest_packet_seq = 0
            self._latest_packet = None
            self._reader_error = None

        reader = threading.Thread(
            target=self._reader_loop,
            args=(sock,),
            name="soma-camera-bridge-reader",
            daemon=True,
        )
        reader.start()

        last_published_seq = 0
        while rclpy.ok():
            packet = self._wait_for_latest_packet(last_published_seq)
            if packet is None:
                if self._reader_error is not None:
                    raise self._reader_error
                if not reader.is_alive():
                    raise ConnectionError("Reader thread exited before a frame was available")
                continue

            seq, jpeg_bytes, metadata = packet
            last_published_seq = seq
            self._publish_frame(jpeg_bytes, metadata)

    def _reader_loop(self, sock: socket.socket) -> None:
        try:
            while rclpy.ok():
                header = _recv_exact(sock, HEADER_STRUCT.size)
                metadata_len, jpeg_len = HEADER_STRUCT.unpack(header)
                metadata = json.loads(_recv_exact(sock, metadata_len).decode("utf-8"))
                jpeg_bytes = _recv_exact(sock, jpeg_len)
                with self._latest_packet_ready:
                    self._latest_packet_seq += 1
                    self._latest_packet = (jpeg_bytes, metadata)
                    self._latest_packet_ready.notify_all()
        except Exception as exc:
            with self._latest_packet_ready:
                self._reader_error = exc
                self._latest_packet_ready.notify_all()

    def _wait_for_latest_packet(
        self,
        last_published_seq: int,
    ) -> tuple[int, bytes, dict] | None:
        with self._latest_packet_ready:
            while rclpy.ok():
                if self._latest_packet is not None and self._latest_packet_seq > last_published_seq:
                    jpeg_bytes, metadata = self._latest_packet
                    return self._latest_packet_seq, jpeg_bytes, metadata

                if self._reader_error is not None:
                    return None

                self._latest_packet_ready.wait(timeout=0.2)

        return None

    def _publish_frame(self, jpeg_bytes: bytes, metadata: dict) -> None:
        array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("Failed to decode JPEG frame from Windows bridge")

        height, width = frame.shape[:2]
        stamp = self.get_clock().now().to_msg()

        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = metadata.get("frame_id", self._args.frame_id)
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = width * 3
        image_msg.data = frame.tobytes()

        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = stamp
        camera_info_msg.header.frame_id = image_msg.header.frame_id
        camera_info_msg.width = width
        camera_info_msg.height = height
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.d = []
        camera_info_msg.k = [0.0] * 9
        camera_info_msg.r = [0.0] * 9
        camera_info_msg.p = [0.0] * 12

        self._image_pub.publish(image_msg)
        self._camera_info_pub.publish(camera_info_msg)

        self._received_frames += 1
        self._received_frames_since_log += 1
        if self._received_frames == 1:
            self.get_logger().info(
                f"First bridged frame published | size={width}x{height} "
                f"jpeg_quality={metadata.get('jpeg_quality', 'unknown')} "
                f"frame_id={image_msg.header.frame_id}"
            )
        self._log_runtime_stats(width, height, metadata)

    def _log_runtime_stats(self, width: int, height: int, metadata: dict) -> None:
        now = time.monotonic()
        if now - self._last_runtime_log_monotonic < self._args.runtime_log_interval_sec:
            return

        elapsed = max(now - self._last_runtime_log_monotonic, 1e-6)
        actual_fps = self._received_frames_since_log / elapsed
        capture_time_ns = metadata.get("capture_time_ns")
        if isinstance(capture_time_ns, int):
            end_to_end_age_ms = max(0.0, (time.time_ns() - capture_time_ns) / 1_000_000.0)
            age_text = f" end_to_end_age_ms={end_to_end_age_ms:.0f}"
        else:
            age_text = ""
        self.get_logger().info(
            f"Publishing bridged frames | size={width}x{height} "
            f"actual_publish_fps={actual_fps:.1f}{age_text} total_frames={self._received_frames}"
        )
        self._last_runtime_log_monotonic = now
        self._received_frames_since_log = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive Windows camera bridge frames and republish ROS 2 topics in WSL."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=65433)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument("--reconnect-delay-sec", type=float, default=1.0)
    parser.add_argument("--runtime-log-interval-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = CameraBridgeReceiver(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Stopping camera bridge receiver after Ctrl+C")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
