#!/usr/bin/env python3
"""Save raw frames from a ROS 2 Image topic without depending on OpenCV.

This is a small diagnostic helper for the C922 bringup:
- if the saved PPM/PGM files already contain horizontal tearing, the corruption
  is in the published image data rather than only in the viewer
- if the saved files look clean, the problem is more likely in the display path
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from PIL import Image as PILImage


PROJECT_TMP_DIR = Path(__file__).resolve().parents[1] / "tmp" / "ros_camera_frames"


def _convert_row_bgr_to_rgb(row: bytes) -> bytes:
    output = bytearray(len(row))
    for idx in range(0, len(row), 3):
        output[idx] = row[idx + 2]
        output[idx + 1] = row[idx + 1]
        output[idx + 2] = row[idx]
    return bytes(output)


def _extract_image_bytes(msg: Image) -> tuple[str, int, int, bytes]:
    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    encoding = msg.encoding.lower()
    data = bytes(msg.data)

    if encoding in {"rgb8", "bgr8"}:
        channels = 3
        expected_step = width * channels
        rows = []
        for row_idx in range(height):
            start = row_idx * step
            end = start + expected_step
            row = data[start:end]
            if len(row) != expected_step:
                raise ValueError(
                    f"Row {row_idx} is truncated: expected {expected_step} bytes, got {len(row)}"
                )
            rows.append(_convert_row_bgr_to_rgb(row) if encoding == "bgr8" else row)

        return "RGB", width, height, b"".join(rows)

    if encoding == "mono8":
        expected_step = width
        rows = []
        for row_idx in range(height):
            start = row_idx * step
            end = start + expected_step
            row = data[start:end]
            if len(row) != expected_step:
                raise ValueError(
                    f"Row {row_idx} is truncated: expected {expected_step} bytes, got {len(row)}"
                )
            rows.append(row)

        return "L", width, height, b"".join(rows)

    raise ValueError(
        f"Unsupported encoding '{msg.encoding}'. "
        "This helper currently supports rgb8, bgr8, and mono8."
    )


def write_image(msg: Image, output_path: Path, output_format: str) -> None:
    mode, width, height, payload = _extract_image_bytes(msg)

    if output_format == "png":
        image = PILImage.frombytes(mode, (width, height), payload)
        image.save(output_path, format="PNG")
        return

    if output_format == "ppm":
        if mode == "RGB":
            header = f"P6\n{width} {height}\n255\n".encode("ascii")
        elif mode == "L":
            header = f"P5\n{width} {height}\n255\n".encode("ascii")
        else:
            raise ValueError(f"Unsupported mode for ppm export: {mode}")
        output_path.write_bytes(header + payload)
        return

    raise ValueError(f"Unsupported output format: {output_format}")


class FrameSaver(Node):
    def __init__(
        self,
        topic: str,
        output_dir: Path,
        count: int,
        skip: int,
        output_format: str,
    ) -> None:
        super().__init__("save_ros_image_frames")
        self._topic = topic
        self._output_dir = output_dir
        self._count = count
        self._skip = skip
        self._output_format = output_format
        self._seen = 0
        self._saved = 0
        self._done = False

        self._output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            Image,
            topic,
            self._on_image,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    @property
    def done(self) -> bool:
        return self._done

    def _on_image(self, msg: Image) -> None:
        self._seen += 1
        if self._seen <= self._skip:
            return
        if self._saved >= self._count:
            return

        suffix = ".png" if self._output_format == "png" else (
            ".pgm" if msg.encoding.lower() == "mono8" else ".ppm"
        )
        output_path = self._output_dir / f"frame_{self._saved:03d}{suffix}"
        write_image(msg, output_path, self._output_format)
        self._saved += 1

        self.get_logger().info(
            f"Saved frame {self._saved}/{self._count} to {output_path} | "
            f"encoding={msg.encoding} size={msg.width}x{msg.height} step={msg.step}"
        )

        if self._saved >= self._count:
            self._done = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--output-dir", default=str(PROJECT_TMP_DIR))
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--skip", type=int, default=10)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    parser.add_argument("--format", choices=("png", "ppm"), default="png")
    args = parser.parse_args()

    rclpy.init()
    node = FrameSaver(
        topic=args.topic,
        output_dir=Path(args.output_dir),
        count=args.count,
        skip=args.skip,
        output_format=args.format,
    )

    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() > deadline:
                node.get_logger().error(
                    f"Timed out waiting for {args.count} frames on {args.topic}. "
                    f"Saved {node._saved} frame(s)."
                )
                return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"Saved {args.count} frame(s) to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
