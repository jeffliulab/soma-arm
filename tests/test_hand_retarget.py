"""Tests for hand retargeting."""

import numpy as np
import pytest

from src.pose_estimation.hand_tracker import HandPose
from src.retargeting.hand_retarget import HandRetarget, HandRetargetConfig


@pytest.fixture
def retarget():
    return HandRetarget(HandRetargetConfig())


def _make_hand_pose(spread: float = 0.0, curl: float = 0.0) -> HandPose:
    """Create a synthetic hand pose.

    Args:
        spread: How spread apart the fingers are (0-1).
        curl: How curled the fingers are (0-1).
    """
    # Start with a flat hand
    landmarks = np.zeros((21, 3), dtype=np.float32)
    landmarks[0] = [0.5, 0.5, 0]  # wrist

    # Finger base positions
    bases = {
        1: [0.45, 0.45, 0],  # thumb CMC
        5: [0.5, 0.4, 0],  # index MCP
        9: [0.5, 0.38, 0],  # middle MCP
        13: [0.5, 0.36, 0],  # ring MCP
        17: [0.5, 0.34, 0],  # pinky MCP
    }

    for base_idx, pos in bases.items():
        landmarks[base_idx] = pos
        for j in range(1, 4):
            idx = base_idx + j
            # Extend finger, optionally curl
            y_offset = -0.03 * j * (1 - curl)
            z_offset = -0.02 * j * curl  # curl inward
            landmarks[idx] = [
                pos[0] + spread * 0.01 * j,
                pos[1] + y_offset,
                pos[2] + z_offset,
            ]

    return HandPose(landmarks=landmarks, handedness="Right", score=1.0)


class TestHandRetarget:
    def test_output_shape(self, retarget):
        hand = _make_hand_pose()
        angles = retarget.retarget(hand)
        assert angles.shape == (16,), f"Expected 16 DOF, got {angles.shape}"

    def test_angles_within_limits(self, retarget):
        hand = _make_hand_pose(curl=0.8)
        angles = retarget.retarget(hand)
        assert np.all(angles >= retarget.config.joint_min)
        assert np.all(angles <= retarget.config.joint_max)

    def test_flat_hand_low_curl(self, retarget):
        """Flat hand should have low joint angles."""
        hand = _make_hand_pose(curl=0.0)
        angles = retarget.retarget(hand)
        mean_angle = np.mean(angles)
        assert mean_angle < 0.5, f"Flat hand should have low angles, got mean={mean_angle}"

    def test_pinch_distance(self):
        hand = _make_hand_pose(curl=0.0)
        dist = hand.pinch_distance
        assert dist > 0, "Pinch distance should be positive"

    def test_pinch_gripper_value(self, retarget):
        hand = _make_hand_pose(curl=0.0)
        value = retarget.compute_pinch_gripper(hand)
        assert 0 <= value <= 1, f"Gripper value should be [0, 1], got {value}"
