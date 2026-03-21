"""Hand retargeting: map MediaPipe hand landmarks to LEAP Hand joint angles.

LEAP Hand v2 has 16 DOF (4 fingers x 4 joints each), but typically controlled
as 8 independent DOF (coupled joints). This module maps from human hand
landmarks (21 points) to robot hand joint angles.
"""

from dataclasses import dataclass

import numpy as np

from src.pose_estimation.hand_tracker import HandPose


@dataclass
class HandRetargetConfig:
    """Configuration for hand retargeting."""

    # LEAP Hand joint limits (radians)
    joint_min: float = 0.0
    joint_max: float = np.pi / 2  # ~90 degrees flexion
    # Number of controlled DOFs
    num_dofs: int = 16
    # Smoothing
    smooth_factor: float = 0.4
    # Finger mapping weights
    curl_sensitivity: float = 2.0
    spread_sensitivity: float = 1.5


# MediaPipe hand landmark indices grouped by finger
FINGER_LANDMARKS = {
    "thumb": [1, 2, 3, 4],  # CMC, MCP, IP, TIP
    "index": [5, 6, 7, 8],  # MCP, PIP, DIP, TIP
    "middle": [9, 10, 11, 12],
    "ring": [13, 14, 15, 16],
}
# Note: LEAP Hand v2 has 4 fingers (no pinky), matching indices, ring, middle, index, thumb

WRIST_IDX = 0


class HandRetarget:
    """Maps human hand pose to LEAP Hand joint angles.

    Uses finger curl angles and spread angles computed from MediaPipe
    landmarks to drive the 16-DOF LEAP Hand.
    """

    def __init__(self, config: HandRetargetConfig | None = None) -> None:
        self.config = config or HandRetargetConfig()
        self._prev_angles: np.ndarray | None = None

    def retarget(self, hand_pose: HandPose) -> np.ndarray:
        """Convert hand landmarks to LEAP Hand joint angles.

        Args:
            hand_pose: MediaPipe hand landmarks (21 x 3).

        Returns:
            Array of 16 joint angles in radians for LEAP Hand.
            Ordered as [thumb(4), index(4), middle(4), ring(4)].
        """
        lm = hand_pose.landmarks
        wrist = lm[WRIST_IDX]
        angles = np.zeros(self.config.num_dofs)

        for i, (finger_name, indices) in enumerate(FINGER_LANDMARKS.items()):
            finger_angles = self._compute_finger_angles(lm, wrist, indices, finger_name)
            angles[i * 4 : (i + 1) * 4] = finger_angles

        # Clamp to joint limits
        angles = np.clip(angles, self.config.joint_min, self.config.joint_max)

        # Apply smoothing
        if self._prev_angles is not None and self.config.smooth_factor > 0:
            a = self.config.smooth_factor
            angles = a * self._prev_angles + (1 - a) * angles
        self._prev_angles = angles.copy()

        return angles

    def _compute_finger_angles(
        self,
        landmarks: np.ndarray,
        wrist: np.ndarray,
        indices: list[int],
        finger_name: str,
    ) -> np.ndarray:
        """Compute 4 joint angles for one finger.

        Uses the angle between consecutive finger segments relative
        to a straight finger configuration.
        """
        points = landmarks[indices]  # (4, 3)
        angles = np.zeros(4)

        if finger_name == "thumb":
            # Thumb has different kinematics - use simpler curl metric
            # Abduction (spread from palm)
            thumb_dir = points[-1] - wrist
            palm_dir = landmarks[9] - wrist  # middle MCP as palm reference
            angles[0] = self._angle_between(thumb_dir, palm_dir) * 0.5

            # Flexion of each thumb joint
            for j in range(1, 4):
                if j < len(points):
                    v1 = points[j] - points[j - 1]
                    v0 = points[j - 1] - (wrist if j == 1 else points[j - 2])
                    angles[j] = self._curl_angle(v0, v1)
        else:
            # MCP abduction (spread between fingers)
            finger_dir = points[0] - wrist
            middle_dir = landmarks[9] - wrist
            angles[0] = self._angle_between(finger_dir, middle_dir) * self.config.spread_sensitivity

            # MCP, PIP, DIP flexion
            prev_point = wrist
            for j in range(len(points) - 1):
                v0 = points[j] - prev_point
                v1 = points[j + 1] - points[j]
                angles[j + 1] = self._curl_angle(v0, v1) * self.config.curl_sensitivity
                prev_point = points[j]

        return np.clip(angles, self.config.joint_min, self.config.joint_max)

    @staticmethod
    def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
        """Compute angle between two vectors."""
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_angle = np.dot(v1, v2) / (n1 * n2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        return float(np.arccos(cos_angle))

    @staticmethod
    def _curl_angle(v_prev: np.ndarray, v_next: np.ndarray) -> float:
        """Compute curl (flexion) angle between consecutive segments.

        Returns positive angle for flexion (curling inward).
        """
        n1 = np.linalg.norm(v_prev)
        n2 = np.linalg.norm(v_next)
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_angle = np.dot(v_prev, v_next) / (n1 * n2)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        # 0 when straight, positive when curled
        return float(max(0, np.pi - np.arccos(cos_angle)))

    def compute_pinch_gripper(self, hand_pose: HandPose) -> float:
        """Compute gripper value from pinch gesture (for Phase 1-2).

        Returns:
            0.0 (fully closed) to 1.0 (fully open).
        """
        dist = hand_pose.pinch_distance
        # Normalize: typical pinch distance range is 0.02 (closed) to 0.15 (open)
        value = (dist - 0.02) / (0.15 - 0.02)
        return float(np.clip(value, 0.0, 1.0))
