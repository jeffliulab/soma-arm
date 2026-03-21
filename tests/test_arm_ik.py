"""Tests for arm inverse kinematics."""

import numpy as np
import pytest

from src.pose_estimation.body_tracker import ArmPose
from src.retargeting.arm_ik import ArmIK, ArmIKConfig


@pytest.fixture
def ik_solver():
    return ArmIK(ArmIKConfig())


def _make_arm_pose(shoulder, elbow, wrist):
    return ArmPose(
        shoulder=np.array(shoulder, dtype=np.float32),
        elbow=np.array(elbow, dtype=np.float32),
        wrist=np.array(wrist, dtype=np.float32),
        visibility=1.0,
    )


class TestArmIK:
    def test_solve_returns_6_joints(self, ik_solver):
        pose = _make_arm_pose([0.5, 0.3, 0], [0.5, 0.5, 0], [0.5, 0.7, 0])
        angles = ik_solver.solve(pose)
        assert angles.shape == (6,), f"Expected 6 joints, got {angles.shape}"

    def test_angles_are_finite(self, ik_solver):
        pose = _make_arm_pose([0.5, 0.3, 0], [0.4, 0.5, 0.1], [0.3, 0.7, 0.2])
        angles = ik_solver.solve(pose)
        assert np.all(np.isfinite(angles)), "Joint angles contain non-finite values"

    def test_velocity_limiting(self, ik_solver):
        """Consecutive solves should not produce large jumps."""
        pose1 = _make_arm_pose([0.5, 0.3, 0], [0.5, 0.5, 0], [0.5, 0.7, 0])
        pose2 = _make_arm_pose([0.5, 0.3, 0], [0.3, 0.5, 0.3], [0.1, 0.7, 0.5])

        angles1 = ik_solver.solve(pose1)
        angles2 = ik_solver.solve(pose2)

        max_delta = np.max(np.abs(angles2 - angles1))
        assert max_delta <= ik_solver.config.velocity_limit + 0.01

    def test_gripper_set(self, ik_solver):
        pose = _make_arm_pose([0.5, 0.3, 0], [0.5, 0.5, 0], [0.5, 0.7, 0])
        angles = ik_solver.solve(pose)
        angles_open = ik_solver.set_gripper(angles, 1.0)
        assert angles_open[5] == pytest.approx(0.03, abs=0.001)

        angles_closed = ik_solver.set_gripper(angles, 0.0)
        assert angles_closed[5] == pytest.approx(0.0, abs=0.001)

    def test_workspace_clamping(self, ik_solver):
        """Very far wrist should be clamped to workspace."""
        pose = _make_arm_pose([0.5, 0.3, 0], [0.5, 0.5, 0], [0.0, 1.5, 1.0])
        angles = ik_solver.solve(pose)
        assert np.all(np.isfinite(angles))
