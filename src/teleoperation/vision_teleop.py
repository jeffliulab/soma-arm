"""Vision-based teleoperation pipeline.

Camera -> MediaPipe -> Retargeting -> MuJoCo Simulation
Real-time loop with visualization.
"""

import time
from dataclasses import dataclass

import cv2
import numpy as np

from src.pose_estimation.holistic_tracker import HolisticTracker, HolisticTrackerConfig
from src.retargeting.arm_ik import ArmIKConfig
from src.retargeting.combined import CombinedAction, CombinedRetarget
from src.retargeting.hand_retarget import HandRetargetConfig
from src.simulation.arm_env import ArmEnvConfig, ArmSimEnv


@dataclass
class VisionTeleopConfig:
    """Configuration for vision teleoperation."""

    camera_id: int = 0
    camera_width: int = 640
    camera_height: int = 480
    control_freq: int = 20  # Hz
    show_camera: bool = True
    show_sim: bool = True
    enable_hand: bool = False  # Phase 3+: enable LEAP Hand
    mirror_camera: bool = True  # flip camera for mirror view
    record: bool = False  # enable data recording


class VisionTeleop:
    """Real-time vision teleoperation of simulated robot.

    Captures webcam frames, detects body/hand pose via MediaPipe,
    maps to robot joint angles via IK/retargeting, and controls
    MuJoCo simulation.
    """

    def __init__(self, config: VisionTeleopConfig | None = None) -> None:
        self.config = config or VisionTeleopConfig()
        self._running = False

        # Initialize components
        self.tracker = HolisticTracker(HolisticTrackerConfig())
        self.retarget = CombinedRetarget(
            arm_config=ArmIKConfig(),
            hand_config=HandRetargetConfig() if self.config.enable_hand else None,
            enable_hand=self.config.enable_hand,
        )
        self.env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))
        self.env.reset()

        # Recording buffer
        self._recorded_frames: list[dict] = []

    def run(self) -> None:
        """Main teleoperation loop."""
        cap = cv2.VideoCapture(self.config.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.camera_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.camera_height)

        if not cap.isOpened():
            print("ERROR: Cannot open camera")
            return

        print("Starting vision teleoperation...")
        print("Controls: 'q' = quit, 'r' = toggle recording, 's' = save recording")

        self._running = True
        dt = 1.0 / self.config.control_freq
        frame_count = 0
        fps_timer = time.time()

        try:
            while self._running:
                loop_start = time.time()

                # Capture frame
                ret, frame = cap.read()
                if not ret:
                    break

                if self.config.mirror_camera:
                    frame = cv2.flip(frame, 1)

                # Detect pose
                holistic_pose = self.tracker.detect(frame)
                timestamp = time.time()

                # Retarget to robot
                action = self.retarget.retarget(holistic_pose, timestamp)

                # Apply to simulation
                self.env.set_joint_positions(action.arm_joints)

                # Get sim image
                sim_image = self.env.render_image()

                # Record if enabled
                if self.config.record:
                    self._record_frame(frame, action, sim_image)

                # Visualization
                display = self._create_display(frame, sim_image, holistic_pose, action)
                frame_count += 1
                elapsed = time.time() - fps_timer
                if elapsed > 0:
                    fps = frame_count / elapsed
                    cv2.putText(
                        display,
                        f"FPS: {fps:.1f} | {'REC' if self.config.record else 'LIVE'}",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0) if not self.config.record else (0, 0, 255),
                        2,
                    )

                cv2.imshow("SmartRobotArm - Vision Teleop", display)

                # Handle keyboard
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    self._running = False
                elif key == ord("r"):
                    self.config.record = not self.config.record
                    print(f"Recording: {'ON' if self.config.record else 'OFF'}")
                elif key == ord("s"):
                    self._save_recording()

                # Rate limiting
                elapsed = time.time() - loop_start
                if elapsed < dt:
                    time.sleep(dt - elapsed)

        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.tracker.close()
            self.env.close()
            print("Teleoperation ended.")

    def _create_display(
        self,
        camera_frame: np.ndarray,
        sim_image: np.ndarray,
        holistic_pose,
        action: CombinedAction,
    ) -> np.ndarray:
        """Create side-by-side display: camera + simulation."""
        h, w = camera_frame.shape[:2]

        # Resize sim image to match camera
        sim_rgb = cv2.cvtColor(sim_image, cv2.COLOR_RGB2BGR)
        sim_resized = cv2.resize(sim_rgb, (w, h))

        # Draw pose info on camera frame
        annotated = camera_frame.copy()
        if holistic_pose.arm_pose is not None:
            ap = holistic_pose.arm_pose
            cv2.putText(
                annotated,
                f"Wrist: ({ap.wrist[0]:.2f}, {ap.wrist[1]:.2f}, {ap.wrist[2]:.2f})",
                (10, h - 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
                1,
            )

        # Draw joint angles on sim image
        joints_text = " ".join([f"{a:.1f}" for a in np.degrees(action.arm_joints[:5])])
        cv2.putText(
            sim_resized,
            f"Joints: {joints_text}",
            (10, h - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
            1,
        )

        # Side by side
        return np.hstack([annotated, sim_resized])

    def _record_frame(
        self,
        camera_frame: np.ndarray,
        action: CombinedAction,
        sim_image: np.ndarray,
    ) -> None:
        """Record a single frame for dataset collection."""
        self._recorded_frames.append({
            "timestamp": action.timestamp,
            "arm_joints": action.arm_joints.copy(),
            "hand_joints": action.hand_joints.copy() if action.hand_joints is not None else None,
            "camera_image": camera_frame.copy(),
            "sim_image": sim_image.copy(),
        })

    def _save_recording(self) -> None:
        """Save recorded frames to disk."""
        if not self._recorded_frames:
            print("No frames recorded.")
            return

        save_path = f"data/teleop_recording_{int(time.time())}.npz"
        print(f"Saving {len(self._recorded_frames)} frames to {save_path}...")

        timestamps = [f["timestamp"] for f in self._recorded_frames]
        arm_joints = np.array([f["arm_joints"] for f in self._recorded_frames])

        np.savez(
            save_path,
            timestamps=timestamps,
            arm_joints=arm_joints,
        )
        print(f"Saved to {save_path}")
        self._recorded_frames.clear()
