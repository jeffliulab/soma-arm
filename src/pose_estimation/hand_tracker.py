"""Hand pose tracker using MediaPipe Hands.

Extracts 21 hand landmarks per hand for dexterous hand control.
"""

from dataclasses import dataclass

import cv2
import mediapipe as mp
import numpy as np


@dataclass
class HandPose:
    """21 hand landmarks extracted from MediaPipe."""

    landmarks: np.ndarray  # (21, 3) xyz for each landmark
    handedness: str  # "Left" or "Right"
    score: float  # detection confidence

    @property
    def wrist(self) -> np.ndarray:
        return self.landmarks[0]

    @property
    def thumb_tip(self) -> np.ndarray:
        return self.landmarks[4]

    @property
    def index_tip(self) -> np.ndarray:
        return self.landmarks[8]

    @property
    def middle_tip(self) -> np.ndarray:
        return self.landmarks[12]

    @property
    def ring_tip(self) -> np.ndarray:
        return self.landmarks[16]

    @property
    def pinky_tip(self) -> np.ndarray:
        return self.landmarks[20]

    @property
    def pinch_distance(self) -> float:
        """Distance between thumb tip and index tip (for gripper control)."""
        return float(np.linalg.norm(self.thumb_tip - self.index_tip))

    @property
    def fingertip_positions(self) -> np.ndarray:
        """Return all 5 fingertip positions as (5, 3) array."""
        return self.landmarks[[4, 8, 12, 16, 20]]


@dataclass
class HandTrackerConfig:
    """Configuration for hand tracker."""

    max_num_hands: int = 1
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    model_complexity: int = 1
    smooth_factor: float = 0.5


class HandTracker:
    """Tracks hand landmarks using MediaPipe Hands.

    Provides 21 3D landmarks per detected hand with temporal smoothing.
    """

    def __init__(self, config: HandTrackerConfig | None = None) -> None:
        self.config = config or HandTrackerConfig()
        self._hands = mp.solutions.hands.Hands(
            max_num_hands=self.config.max_num_hands,
            min_detection_confidence=self.config.min_detection_confidence,
            min_tracking_confidence=self.config.min_tracking_confidence,
            model_complexity=self.config.model_complexity,
        )
        self._prev_landmarks: dict[str, np.ndarray] = {}

    def detect(self, frame_bgr: np.ndarray) -> list[HandPose]:
        """Detect hand poses from BGR frame.

        Args:
            frame_bgr: OpenCV BGR image.

        Returns:
            List of HandPose, one per detected hand.
        """
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self._hands.process(rgb)

        if not results.multi_hand_landmarks:
            return []

        hands: list[HandPose] = []
        for hand_lm, hand_info in zip(
            results.multi_hand_landmarks, results.multi_handedness
        ):
            landmarks = np.array(
                [[lm.x, lm.y, lm.z] for lm in hand_lm.landmark],
                dtype=np.float32,
            )

            handedness = hand_info.classification[0].label
            score = hand_info.classification[0].score

            # Apply temporal smoothing
            if handedness in self._prev_landmarks and self.config.smooth_factor > 0:
                alpha = self.config.smooth_factor
                landmarks = alpha * self._prev_landmarks[handedness] + (1 - alpha) * landmarks

            self._prev_landmarks[handedness] = landmarks
            hands.append(HandPose(landmarks=landmarks, handedness=handedness, score=score))

        return hands

    def close(self) -> None:
        self._hands.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
