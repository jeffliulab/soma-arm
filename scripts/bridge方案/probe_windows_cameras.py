#!/usr/bin/env python3
"""Probe Windows camera indices for the SOMA camera bridge workflow."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import cv2
except ImportError as exc:  # pragma: no cover - environment-dependent
    sys.exit(
        "OpenCV for Windows is required.\n"
        "Install it with the managed bridge env bootstrap first.\n"
        f"Original import error: {exc}"
    )


def _decode_fourcc(value: float) -> str:
    code = int(value)
    if code <= 0:
        return "unknown"
    chars = [chr((code >> (8 * idx)) & 0xFF) for idx in range(4)]
    text = "".join(chars).strip("\x00").strip()
    return text or "unknown"


def _backend_candidates(name: str) -> list[tuple[str, int]]:
    any_code = int(getattr(cv2, "CAP_ANY", 0))
    dshow_code = int(getattr(cv2, "CAP_DSHOW", any_code))
    msmf_code = int(getattr(cv2, "CAP_MSMF", any_code))

    if name == "msmf":
        return [("MSMF", msmf_code)]
    if name == "dshow":
        return [("DSHOW", dshow_code)]
    return [("MSMF", msmf_code), ("DSHOW", dshow_code), ("ANY", any_code)]


def probe_index(index: int, backend_name: str, width: int, height: int, fps: float) -> bool:
    found = False
    for label, code in _backend_candidates(backend_name):
        cap = cv2.VideoCapture(index, code)
        if not cap or not cap.isOpened():
            if cap:
                cap.release()
            continue

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        cap.set(cv2.CAP_PROP_FPS, float(fps))
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

        frame = None
        for _ in range(10):
            ok, frame = cap.read()
            if ok and frame is not None:
                break
            time.sleep(0.03)

        if frame is None:
            cap.release()
            continue

        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_fourcc = _decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
        print(
            f"index={index} backend={label} "
            f"actual={actual_width}x{actual_height}@{actual_fps:.2f} "
            f"format={actual_fourcc}"
        )
        cap.release()
        found = True
        break
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe Windows camera indices.")
    parser.add_argument("--max-index", type=int, default=5)
    parser.add_argument("--backend", choices=("auto", "msmf", "dshow"), default="auto")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    any_found = False
    for index in range(args.max_index + 1):
        if probe_index(index, args.backend, args.width, args.height, args.fps):
            any_found = True

    if not any_found:
        print("No usable Windows cameras were detected with the requested settings.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
