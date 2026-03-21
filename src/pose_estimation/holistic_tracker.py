"""Holistic tracker combining body and hand tracking.

Uses MediaPipe Holistic for efficient joint body+hand detection.
"""

from dataclasses import dataclass

import cv2
import mediapipe as mp
import numpy as np

from src.pose_estimation.body_tracker import ArmPose
from src.pose_estimation.hand_tracker import HandPose


@dataclass
class HolisticPose:
    """Combined body arm pose and hand poses."""

    arm_pose: ArmPose | None = None
    right_hand: HandPose | None = None
    left_hand: HandPose | None = None


@dataclass
class HolisticTrackerConfig:
    """Configuration for holistic tracker."""

    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    model_complexity: int = 1
    smooth_factor: float = 0.6
    use_right_arm: bool = True


class HolisticTracker:
    """Combined body + hand tracker using MediaPipe Holistic.

    More efficient than running separate body and hand models.
    """

    # Body landmark indices
    RIGHT_SHOULDER = 12
    RIGHT_ELBOW = 14
    RIGHT_WRIST = 16
    LEFT_SHOULDER = 11
    LEFT_ELBOW = 13
    LEFT_WRIST = 15

    def __init__(self, config: HolisticTrackerConfig | None = None) -> None:
        self.config = config or HolisticTrackerConfig()
        self._holistic = mp.solutions.holistic.Holistic(
            min_detection_confidence=self.config.min_detection_confidence,
            min_tracking_confidence=self.config.min_tracking_confidence,
            model_complexity=self.config.model_complexity,
        )
        self._prev_arm: ArmPose | None = None
        self._prev_hands: dict[str, np.ndarray] = {}

        if self.config.use_right_arm:
            self._shoulder_idx = self.RIGHT_SHOULDER
            self._elbow_idx = self.RIGHT_ELBOW
            self._wrist_idx = self.RIGHT_WRIST
        else:
            self._shoulder_idx = self.LEFT_SHOULDER
            self._elbow_idx = self.LEFT_ELBOW
            self._wrist_idx = self.LEFT_WRIST

    def detect(self, frame_bgr: np.ndarray) -> HolisticPose:
        """Detect body and hand poses from BGR frame.

        Args:
            frame_bgr: OpenCV BGR image.

        Returns:
            HolisticPose with arm and hand data.
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._holistic.process(rgb)

        pose = HolisticPose()

        # Extract arm pose from body landmarks
        if results.pose_landmarks:
            lm = results.pose_landmarks.landmark
            shoulder = self._lm_to_array(lm[self._shoulder_idx])
            elbow = self._lm_to_array(lm[self._elbow_idx])
            wrist = self._lm_to_array(lm[self._wrist_idx])
            visibility = (
                lm[self._shoulder_idx].visibility
                + lm[self._elbow_idx].visibility
                + lm[self._wrist_idx].visibility
            ) / 3.0

            arm = ArmPose(shoulder=shoulder, elbow=elbow, wrist=wrist, visibility=visibility)
            arm = self._smooth_arm(arm)
            self._prev_arm = arm
            pose.arm_pose = arm

        # Extract right hand
        if results.right_hand_landmarks:
            landmarks = self._hand_lm_to_array(results.right_hand_landmarks)
            landmarks = self._smooth_hand("Right", landmarks)
            pose.right_hand = HandPose(landmarks=landmarks, handedness="Right", score=1.0)

        # Extract left hand
        if results.left_hand_landmarks:
            landmarks = self._hand_lm_to_array(results.left_hand_landmarks)
            landmarks = self._smooth_hand("Left", landmarks)
            pose.left_hand = HandPose(landmarks=landmarks, handedness="Left", score=1.0)

        return pose

    def _smooth_arm(self, arm: ArmPose) -> ArmPose:
        if self._prev_arm is None or self.config.smooth_factor <= 0:
            return arm
        a = self.config.smooth_factor
        arm.shoulder = a * self._prev_arm.shoulder + (1 - a) * arm.shoulder
        arm.elbow = a * self._prev_arm.elbow + (1 - a) * arm.elbow
        arm.wrist = a * self._prev_arm.wrist + (1 - a) * arm.wrist
        return arm

    def _smooth_hand(self, key: str, landmarks: np.ndarray) -> np.ndarray:
        if key in self._prev_hands and self.config.smooth_factor > 0:
            a = self.config.smooth_factor
            landmarks = a * self._prev_hands[key] + (1 - a) * landmarks
        self._prev_hands[key] = landmarks
        return landmarks

    @staticmethod
    def _lm_to_array(landmark) -> np.ndarray:
        return np.array([landmark.x, landmark.y, landmark.z], dtype=np.float32)

    @staticmethod
    def _hand_lm_to_array(hand_landmarks) -> np.ndarray:
        return np.array(
            [[lm.x, lm.y, lm.z] for lm in hand_landmarks.landmark],
            dtype=np.float32,
        )

    def close(self) -> None:
        self._holistic.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
