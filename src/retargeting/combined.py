"""Combined arm + hand retargeting.

Provides unified interface for 14-DOF (6 arm + 8 hand) control
from holistic pose detection.
"""

from dataclasses import dataclass

import numpy as np

from src.pose_estimation.holistic_tracker import HolisticPose
from src.retargeting.arm_ik import ArmIK, ArmIKConfig
from src.retargeting.hand_retarget import HandRetarget, HandRetargetConfig


@dataclass
class CombinedAction:
    """Combined arm and hand joint angles."""

    arm_joints: np.ndarray  # (6,) base, shoulder, elbow, wrist_pitch, wrist_roll, gripper
    hand_joints: np.ndarray | None  # (16,) LEAP Hand joints, or None if no hand
    timestamp: float = 0.0

    @property
    def full_joints(self) -> np.ndarray:
        """Concatenated arm + hand joints."""
        if self.hand_joints is not None:
            return np.concatenate([self.arm_joints, self.hand_joints])
        return self.arm_joints

    @property
    def num_dofs(self) -> int:
        if self.hand_joints is not None:
            return len(self.arm_joints) + len(self.hand_joints)
        return len(self.arm_joints)


class CombinedRetarget:
    """Unified arm + hand retargeting from holistic pose.

    Phase 1-2: Arm only (6 DOF) with optional pinch gripper.
    Phase 3+: Arm (6 DOF) + LEAP Hand (16 DOF) = 22 DOF.
    """

    def __init__(
        self,
        arm_config: ArmIKConfig | None = None,
        hand_config: HandRetargetConfig | None = None,
        enable_hand: bool = False,
    ) -> None:
        self.arm_ik = ArmIK(arm_config)
        self.hand_retarget = HandRetarget(hand_config) if enable_hand else None
        self.enable_hand = enable_hand

    def retarget(self, holistic_pose: HolisticPose, timestamp: float = 0.0) -> CombinedAction:
        """Convert holistic pose to robot joint angles.

        Args:
            holistic_pose: Combined body + hand pose from HolisticTracker.
            timestamp: Current timestamp for data recording.

        Returns:
            CombinedAction with arm (and optionally hand) joint angles.
        """
        # Arm retargeting
        arm_joints = np.zeros(6)
        if holistic_pose.arm_pose is not None:
            arm_joints = self.arm_ik.solve(holistic_pose.arm_pose)

        # Hand retargeting
        hand_joints = None
        if self.enable_hand and self.hand_retarget is not None:
            # Use the hand on the same side as the tracked arm
            hand = holistic_pose.right_hand
            if hand is not None:
                hand_joints = self.hand_retarget.retarget(hand)
            else:
                hand_joints = np.zeros(self.hand_retarget.config.num_dofs)
        elif not self.enable_hand:
            # Phase 1-2: Use pinch gesture for gripper control
            hand = holistic_pose.right_hand
            if hand is not None:
                gripper_value = HandRetarget().compute_pinch_gripper(hand)
                arm_joints = self.arm_ik.set_gripper(arm_joints, gripper_value)

        return CombinedAction(
            arm_joints=arm_joints,
            hand_joints=hand_joints,
            timestamp=timestamp,
        )
