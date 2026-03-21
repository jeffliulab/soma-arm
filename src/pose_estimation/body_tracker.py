"""Body pose tracker using MediaPipe Pose.

Extracts upper body landmarks (shoulder, elbow, wrist) for arm control.
"""

from dataclasses import dataclass

import cv2
import mediapipe as mp
import numpy as np


@dataclass
class ArmPose:
    """3D positions of arm joints extracted from MediaPipe."""

    shoulder: np.ndarray  # (3,) xyz
    elbow: np.ndarray  # (3,) xyz
    wrist: np.ndarray  # (3,) xyz
    visibility: float = 0.0  # average visibility of the 3 landmarks


@dataclass
class BodyTrackerConfig:
    """Configuration for body pose tracker."""

    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    model_complexity: int = 1
    smooth_factor: float = 0.7  # EMA smoothing (0=no smooth, 1=max smooth)
    use_right_arm: bool = True  # track right arm by default


class BodyTracker:
    """Tracks upper body pose using MediaPipe Pose.

    Extracts shoulder, elbow, wrist positions for one arm,
    applies temporal smoothing via exponential moving average.
    """

    # MediaPipe Pose landmark indices
    RIGHT_SHOULDER = 12
    RIGHT_ELBOW = 14
    RIGHT_WRIST = 16
    LEFT_SHOULDER = 11
    LEFT_ELBOW = 13
    LEFT_WRIST = 15

    def __init__(self, config: BodyTrackerConfig | None = None) -> None:
        self.config = config or BodyTrackerConfig()
        self._pose = mp.solutions.pose.Pose(
            min_detection_confidence=self.config.min_detection_confidence,
            min_tracking_confidence=self.config.min_tracking_confidence,
            model_complexity=self.config.model_complexity,
        )
        self._prev_pose: ArmPose | None = None

        if self.config.use_right_arm:
            self._shoulder_idx = self.RIGHT_SHOULDER
            self._elbow_idx = self.RIGHT_ELBOW
            self._wrist_idx = self.RIGHT_WRIST
        else:
            self._shoulder_idx = self.LEFT_SHOULDER
            self._elbow_idx = self.LEFT_ELBOW
            self._wrist_idx = self.LEFT_WRIST

    def detect(self, frame_bgr: np.ndarray) -> ArmPose | None:
        """Detect arm pose from BGR frame.

        Args:
            frame_bgr: OpenCV BGR image.

        Returns:
            ArmPose with smoothed 3D positions, or None if not detected.
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._pose.process(rgb)

        if not results.pose_landmarks:
            return None

        lm = results.pose_landmarks.landmark
        shoulder = self._landmark_to_array(lm[self._shoulder_idx])
        elbow = self._landmark_to_array(lm[self._elbow_idx])
        wrist = self._landmark_to_array(lm[self._wrist_idx])

        visibility = (
            lm[self._shoulder_idx].visibility
            + lm[self._elbow_idx].visibility
            + lm[self._wrist_idx].visibility
        ) / 3.0

        pose = ArmPose(
            shoulder=shoulder,
            elbow=elbow,
            wrist=wrist,
            visibility=visibility,
        )

        # Apply EMA smoothing
        if self._prev_pose is not None and self.config.smooth_factor > 0:
            alpha = self.config.smooth_factor
            pose.shoulder = alpha * self._prev_pose.shoulder + (1 - alpha) * pose.shoulder
            pose.elbow = alpha * self._prev_pose.elbow + (1 - alpha) * pose.elbow
            pose.wrist = alpha * self._prev_pose.wrist + (1 - alpha) * pose.wrist

        self._prev_pose = pose
        return pose

    def close(self) -> None:
        """Release MediaPipe resources."""
        self._pose.close()

    @staticmethod
    def _landmark_to_array(landmark) -> np.ndarray:
        """Convert MediaPipe landmark to numpy array [x, y, z]."""
        return np.array([landmark.x, landmark.y, landmark.z], dtype=np.float32)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
