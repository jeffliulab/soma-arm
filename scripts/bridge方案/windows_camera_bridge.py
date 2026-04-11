#!/usr/bin/env python3
"""Windows-native camera bridge for WSL ROS 2.

Capture frames on native Windows, JPEG-encode them locally, and stream them
over TCP to a WSL-side ROS 2 republisher. This avoids the broken
`usbipd + WSL + UVC/MJPG` direct camera path on this machine.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass

try:
    import cv2
except ImportError as exc:  # pragma: no cover - environment-dependent
    sys.exit(
        "OpenCV for Windows is required.\n"
        "Install it with: py -m pip install opencv-python\n"
        f"Original import error: {exc}"
    )


HEADER_STRUCT = struct.Struct("!II")


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


class CameraBridgeServer:
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        self._capture = None
        self._capture_label = "unopened"
        self._capture_thread: threading.Thread | None = None
        self._capture_stop_event = threading.Event()
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._latest_frame_capture_time_ns = 0
        self._latest_frame_ready = threading.Condition()
        self._published_frames = 0
        self._published_frames_since_log = 0
        self._last_runtime_log_monotonic = time.monotonic()

    def run(self) -> int:
        self._open_capture()

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._args.host, self._args.port))
        server.listen(1)

        print(
            f"Camera bridge listening on {self._args.host}:{self._args.port} "
            "(waiting for WSL client...)",
            flush=True,
        )

        try:
            while True:
                conn, addr = server.accept()
                print(f"WSL client connected: {addr}", flush=True)
                try:
                    self._stream(conn)
                finally:
                    conn.close()
                    print("WSL client disconnected. Waiting for reconnect...", flush=True)
        except KeyboardInterrupt:
            print("Stopping camera bridge after Ctrl+C", flush=True)
            return 0
        finally:
            server.close()
            self._release_capture()

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
            labels = [
                BackendCandidate("MSMF", msmf_code),
                BackendCandidate("DSHOW", dshow_code),
                BackendCandidate("ANY", any_code),
            ]

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

    def _open_capture(self) -> None:
        self._release_capture()

        for candidate in self._backend_candidates():
            print(
                f"Trying backend {candidate.label} for camera index {self._args.device_index}",
                flush=True,
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
                continue

            try:
                _, width, height = _normalize_frame(frame)
            except ValueError:
                capture.release()
                continue

            actual_backend = _backend_name(int(capture.get(cv2.CAP_PROP_BACKEND)))
            actual_fps = capture.get(cv2.CAP_PROP_FPS)
            actual_fourcc = _decode_fourcc(capture.get(cv2.CAP_PROP_FOURCC))
            self._capture = capture
            self._capture_label = candidate.label
            self._start_capture_thread()
            print(
                "Camera opened successfully | "
                f"requested={self._args.width}x{self._args.height}@{self._args.fps} "
                f"backend={candidate.label} actual_backend={actual_backend} "
                f"actual={width}x{height}@{actual_fps:.2f} format={actual_fourcc}",
                flush=True,
            )
            return

        raise RuntimeError(
            "Failed to open the camera on Windows. Make sure the C922 is not "
            "attached to WSL and is not occupied by another app."
        )

    def _release_capture(self) -> None:
        self._stop_capture_thread()
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
            self._capture = None

    def _start_capture_thread(self) -> None:
        self._stop_capture_thread()
        self._capture_stop_event = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="soma-camera-bridge-capture",
            daemon=True,
        )
        self._capture_thread.start()

    def _stop_capture_thread(self) -> None:
        if self._capture_thread is None:
            return

        self._capture_stop_event.set()
        with self._latest_frame_ready:
            self._latest_frame_ready.notify_all()
        self._capture_thread.join(timeout=1.0)
        self._capture_thread = None
        with self._latest_frame_ready:
            self._latest_frame = None
            self._latest_frame_seq = 0
            self._latest_frame_capture_time_ns = 0

    def _capture_once(self):
        successful_grabs = 0
        for _ in range(int(self._args.drop_stale_grabs)):
            if not self._capture.grab():
                break
            successful_grabs += 1

        if successful_grabs > 0:
            ok, frame = self._capture.retrieve()
        else:
            ok, frame = self._capture.read()

        if not ok or frame is None:
            raise RuntimeError("Camera read failed during bridge capture")
        return frame

    def _capture_loop(self) -> None:
        while not self._capture_stop_event.is_set():
            try:
                frame = self._capture_once()
                frame, _, _ = _normalize_frame(frame)
                capture_time_ns = time.time_ns()
            except Exception:
                time.sleep(0.01)
                continue

            with self._latest_frame_ready:
                self._latest_frame = frame.copy()
                self._latest_frame_seq += 1
                self._latest_frame_capture_time_ns = capture_time_ns
                self._latest_frame_ready.notify_all()

    def _wait_for_latest_frame(self, last_seq: int) -> tuple[int, object, int]:
        deadline = time.monotonic() + 2.0
        with self._latest_frame_ready:
            while not self._capture_stop_event.is_set():
                if self._latest_frame is not None and self._latest_frame_seq > last_seq:
                    return (
                        self._latest_frame_seq,
                        self._latest_frame,
                        self._latest_frame_capture_time_ns,
                    )

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._latest_frame_ready.wait(timeout=remaining)

        raise RuntimeError("Timed out waiting for a fresh camera frame")

    def _encode_frame(self, frame, capture_time_ns: int) -> tuple[bytes, dict]:
        frame, width, height = _normalize_frame(frame)
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self._args.jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("Failed to JPEG-encode a frame on Windows")

        metadata = {
            "width": width,
            "height": height,
            "encoding": "bgr8",
            "frame_id": self._args.frame_id,
            "capture_time_ns": capture_time_ns,
            "jpeg_quality": int(self._args.jpeg_quality),
        }
        return bytes(encoded), metadata

    def _log_runtime_stats(self, width: int, height: int, capture_time_ns: int) -> None:
        now = time.monotonic()
        if now - self._last_runtime_log_monotonic < self._args.runtime_log_interval_sec:
            return

        elapsed = max(now - self._last_runtime_log_monotonic, 1e-6)
        actual_fps = self._published_frames_since_log / elapsed
        latest_age_ms = max(0.0, (time.time_ns() - capture_time_ns) / 1_000_000.0)
        print(
            f"Streaming | backend={self._capture_label} size={width}x{height} "
            f"actual_send_fps={actual_fps:.1f} latest_frame_age_ms={latest_age_ms:.0f} "
            f"total_frames={self._published_frames}",
            flush=True,
        )
        self._last_runtime_log_monotonic = now
        self._published_frames_since_log = 0

    def _stream(self, conn: socket.socket) -> None:
        interval = 1.0 / max(float(self._args.fps), 1.0)
        last_sent_seq = 0
        while True:
            t0 = time.perf_counter()
            seq, frame, capture_time_ns = self._wait_for_latest_frame(last_sent_seq)
            last_sent_seq = seq

            jpeg_bytes, metadata = self._encode_frame(frame, capture_time_ns)
            metadata_bytes = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
            conn.sendall(HEADER_STRUCT.pack(len(metadata_bytes), len(jpeg_bytes)))
            conn.sendall(metadata_bytes)
            conn.sendall(jpeg_bytes)

            self._published_frames += 1
            self._published_frames_since_log += 1
            self._log_runtime_stats(metadata["width"], metadata["height"], capture_time_ns)

            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, interval - elapsed))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture C922 frames on Windows and bridge them to WSL over TCP."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=65433)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument(
        "--drop-stale-grabs",
        type=int,
        default=2,
        help="How many buffered frames to discard before each encoded frame",
    )
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument(
        "--backend",
        choices=("auto", "msmf", "dshow", "any"),
        default="auto",
    )
    parser.add_argument(
        "--capture-format",
        choices=("auto", "mjpg", "yuy2"),
        default="mjpg",
        help="Requested local camera pixel format before Windows-side JPEG encoding",
    )
    parser.add_argument("--runtime-log-interval-sec", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    server = CameraBridgeServer(args)
    try:
        return server.run()
    except KeyboardInterrupt:
        print("Stopping camera bridge after Ctrl+C", flush=True)
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
