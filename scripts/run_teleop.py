"""Run vision-based teleoperation.

Usage:
    python scripts/run_teleop.py [--no-hand] [--record] [--camera 0]
"""

import argparse

from src.teleoperation.vision_teleop import VisionTeleop, VisionTeleopConfig


def main() -> None:
    parser = argparse.ArgumentParser(description="Vision-based robot teleoperation")
    parser.add_argument("--camera", type=int, default=0, help="Camera device ID")
    parser.add_argument("--hand", action="store_true", help="Enable LEAP Hand (Phase 3+)")
    parser.add_argument("--record", action="store_true", help="Start recording immediately")
    parser.add_argument("--no-mirror", action="store_true", help="Disable camera mirroring")
    args = parser.parse_args()

    config = VisionTeleopConfig(
        camera_id=args.camera,
        enable_hand=args.hand,
        record=args.record,
        mirror_camera=not args.no_mirror,
    )

    teleop = VisionTeleop(config)
    teleop.run()


if __name__ == "__main__":
    main()
