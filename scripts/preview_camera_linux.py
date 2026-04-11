#!/usr/bin/env python3
"""Quick local preview for a V4L2 camera inside WSL/Linux.

This is a minimal bringup helper:
- no ROS node required
- uses Ubuntu's system OpenCV
- strips the user site-packages path so a local NumPy/OpenCV mismatch does not
  break camera preview
"""

from __future__ import annotations

import argparse
from pathlib import Path
import site
import sys


def _strip_user_site_packages() -> None:
    user_site = site.getusersitepackages()
    if user_site in sys.path:
        sys.path.remove(user_site)


def _strip_virtualenv_site_packages() -> None:
    """Prefer Ubuntu's system OpenCV over the venv's headless wheel."""
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


def detect_default_device() -> str:
    candidates = sorted(Path("/dev").glob("video*"))
    if not candidates:
        return "/dev/video0"
    return str(candidates[0])


def build_gstreamer_pipeline(
    device: str,
    width: int,
    height: int,
    fps: int,
    pixel_format: str,
) -> str:
    appsink = "videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"

    if pixel_format == "mjpg":
        return (
            f"v4l2src device={device} ! "
            f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
            f"jpegdec ! {appsink}"
        )

    if pixel_format == "yuyv":
        return (
            f"v4l2src device={device} ! "
            f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
            f"{appsink}"
        )

    return f"v4l2src device={device} ! decodebin ! {appsink}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="", help="V4L2 device path, e.g. /dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--backend",
        choices=("v4l2", "gstreamer", "auto"),
        default="v4l2",
        help="Capture backend. Use gstreamer to test whether MJPG corruption is backend-specific.",
    )
    parser.add_argument(
        "--pixel-format",
        choices=("auto", "mjpg", "yuyv"),
        default="auto",
        help="Requested V4L2 pixel format. Default keeps the camera/driver default.",
    )
    parser.add_argument(
        "--mjpeg",
        action="store_true",
        help="Backward-compatible alias for --pixel-format mjpg.",
    )
    args = parser.parse_args()

    device = args.device or detect_default_device()
    if not Path(device).exists():
        print(f"Camera device not found: {device}")
        print("Hint: run `ls /dev/video*` first and verify the C922 is really attached to WSL.")
        return 1

    requested_pixel_format = args.pixel_format
    if args.mjpeg and requested_pixel_format == "auto":
        requested_pixel_format = "mjpg"

    backend_name = args.backend.upper()
    source_desc = device

    if args.backend == "gstreamer":
        source_desc = build_gstreamer_pipeline(
            device=device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            pixel_format=requested_pixel_format,
        )
        print(
            "Opening camera with GStreamer. "
            "If this hangs without opening a window, press Ctrl+C and treat the "
            "current GStreamer path as unavailable."
        )
        print(f"GStreamer pipeline: {source_desc}", flush=True)
        cap = cv2.VideoCapture(source_desc, cv2.CAP_GSTREAMER)
    elif args.backend == "auto":
        print(f"Opening camera with backend=AUTO device={device}", flush=True)
        cap = cv2.VideoCapture(device, cv2.CAP_ANY)
        if requested_pixel_format == "mjpg":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        elif requested_pixel_format == "yuyv":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    else:
        print(f"Opening camera with backend=V4L2 device={device}", flush=True)
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if requested_pixel_format == "mjpg":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        elif requested_pixel_format == "yuyv":
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)

    if not cap.isOpened():
        print(f"Failed to open camera device: {device}")
        if args.backend == "gstreamer":
            print(f"GStreamer pipeline: {source_desc}")
        return 1

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_text = "".join(chr((actual_fourcc >> (8 * i)) & 0xFF) for i in range(4)).strip()

    print(
        f"Previewing {device}. "
        f"backend={backend_name} "
        f"requested={args.width}x{args.height}@{args.fps} "
        f"requested_format={requested_pixel_format} "
        f"actual={actual_width}x{actual_height}@{actual_fps:.1f} "
        f"format={fourcc_text or 'unknown'}. Press q to quit."
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("Failed to read a frame from the camera.")
                return 1
            try:
                cv2.imshow("SOMA C922 Preview", frame)
            except cv2.error as exc:
                snapshot = Path("/tmp/soma_c922_preview.jpg")
                cv2.imwrite(str(snapshot), frame)
                print("OpenCV GUI preview is unavailable in this interpreter.")
                print(f"Saved one frame to: {snapshot}")
                print(f"Original error: {exc}")
                return 1
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                return 0
    finally:
        cap.release()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
