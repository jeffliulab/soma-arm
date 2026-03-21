"""Arm inverse kinematics: map human arm pose to SO-ARM100 joint angles.

Uses ikpy for IK solving with workspace scaling and joint smoothing.
"""

from dataclasses import dataclass

import numpy as np

from src.pose_estimation.body_tracker import ArmPose

try:
    import ikpy.chain
    import ikpy.link

    HAS_IKPY = True
except ImportError:
    HAS_IKPY = False


@dataclass
class ArmIKConfig:
    """Configuration for arm IK solver."""

    # SO-ARM100 workspace bounds (meters)
    robot_reach: float = 0.3  # approximate max reach
    # Human arm workspace mapping
    human_arm_length: float = 0.6  # approximate human arm length (normalized coords)
    # Joint limits (degrees) for SO-ARM100
    joint_limits: list[tuple[float, float]] | None = None
    # Smoothing
    velocity_limit: float = 0.5  # max radians per step
    smooth_factor: float = 0.3  # output smoothing


# SO-ARM100 DH-like parameters for ikpy chain
# 6 DOF: base_rotation, shoulder, elbow, wrist_pitch, wrist_roll, gripper
SO_ARM100_CHAIN_PARAMS = [
    {"name": "base", "translation_vector": [0, 0, 0.04], "orientation": [0, 0, 0],
     "rotation": [0, 0, 1], "bounds": (-np.pi, np.pi)},
    {"name": "shoulder", "translation_vector": [0, 0, 0.06], "orientation": [0, 0, 0],
     "rotation": [0, 1, 0], "bounds": (-np.pi / 2, np.pi / 2)},
    {"name": "elbow", "translation_vector": [0, 0, 0.15], "orientation": [0, 0, 0],
     "rotation": [0, 1, 0], "bounds": (-3 * np.pi / 4, 3 * np.pi / 4)},
    {"name": "wrist_pitch", "translation_vector": [0, 0, 0.12], "orientation": [0, 0, 0],
     "rotation": [0, 1, 0], "bounds": (-np.pi / 2, np.pi / 2)},
    {"name": "wrist_roll", "translation_vector": [0, 0, 0.06], "orientation": [0, 0, 0],
     "rotation": [0, 0, 1], "bounds": (-np.pi, np.pi)},
]


class ArmIK:
    """Inverse kinematics solver for SO-ARM100.

    Maps human arm pose (shoulder, elbow, wrist) from MediaPipe
    to 6-DOF robot joint angles.
    """

    def __init__(self, config: ArmIKConfig | None = None) -> None:
        self.config = config or ArmIKConfig()
        self._prev_angles: np.ndarray | None = None

        if HAS_IKPY:
            self._chain = self._build_chain()
        else:
            self._chain = None

    def _build_chain(self) -> "ikpy.chain.Chain":
        """Build ikpy chain from SO-ARM100 parameters."""
        links = [ikpy.link.OriginLink()]
        for p in SO_ARM100_CHAIN_PARAMS:
            links.append(
                ikpy.link.URDFLink(
                    name=p["name"],
                    translation_vector=p["translation_vector"],
                    orientation=p["orientation"],
                    rotation=p["rotation"],
                    bounds=p["bounds"],
                )
            )
        # Add end-effector (no joint)
        links.append(
            ikpy.link.URDFLink(
                name="end_effector",
                translation_vector=[0, 0, 0.03],
                orientation=[0, 0, 0],
                rotation=[0, 0, 0],
            )
        )
        return ikpy.chain.Chain(name="so_arm100", links=links)

    def solve(self, arm_pose: ArmPose) -> np.ndarray:
        """Compute joint angles from human arm pose.

        Args:
            arm_pose: Detected arm pose with shoulder, elbow, wrist.

        Returns:
            Array of 6 joint angles in radians (base, shoulder, elbow,
            wrist_pitch, wrist_roll, gripper=0).
        """
        target_pos = self._map_human_to_robot(arm_pose)

        if self._chain is not None:
            # Use ikpy IK solver
            initial = np.zeros(len(self._chain.links))
            if self._prev_angles is not None:
                # Warm start from previous solution
                initial[1:6] = self._prev_angles[:5]
            full_angles = self._chain.inverse_kinematics(
                target_position=target_pos,
                initial_position=initial,
            )
            # Extract active joint angles (skip origin and end-effector)
            angles = full_angles[1:6]
        else:
            # Fallback: geometric IK
            angles = self._geometric_ik(arm_pose, target_pos)

        # Add gripper (0 = closed)
        angles = np.append(angles, 0.0)

        # Apply velocity limiting
        angles = self._limit_velocity(angles)

        # Apply output smoothing
        if self._prev_angles is not None and self.config.smooth_factor > 0:
            a = self.config.smooth_factor
            angles = a * self._prev_angles + (1 - a) * angles

        self._prev_angles = angles.copy()
        return angles

    def _map_human_to_robot(self, arm_pose: ArmPose) -> np.ndarray:
        """Map human arm wrist position to robot workspace coordinates.

        MediaPipe outputs normalized coordinates [0,1] for x,y and relative z.
        We map these to the robot's workspace.
        """
        # Center the coordinates around the shoulder
        wrist_rel = arm_pose.wrist - arm_pose.shoulder

        # Scale from MediaPipe normalized space to robot workspace
        scale = self.config.robot_reach / self.config.human_arm_length

        # Map: MediaPipe x (right) -> robot y, MediaPipe y (down) -> robot -z,
        # MediaPipe z (depth) -> robot x
        robot_x = -wrist_rel[2] * scale + 0.15  # offset forward
        robot_y = -wrist_rel[0] * scale
        robot_z = -wrist_rel[1] * scale + 0.2  # offset up from base

        # Clamp to workspace
        robot_pos = np.array([robot_x, robot_y, robot_z])
        dist = np.linalg.norm(robot_pos)
        if dist > self.config.robot_reach:
            robot_pos = robot_pos * (self.config.robot_reach / dist)

        return robot_pos

    def _geometric_ik(self, arm_pose: ArmPose, target: np.ndarray) -> np.ndarray:
        """Simple geometric IK fallback when ikpy is not available."""
        angles = np.zeros(5)

        # Base rotation from target xy
        angles[0] = np.arctan2(target[1], target[0])

        # Compute distance in xz plane for shoulder/elbow
        r = np.sqrt(target[0] ** 2 + target[1] ** 2)
        z = target[2]

        # Upper arm and forearm lengths from chain params
        l1 = 0.15  # upper arm
        l2 = 0.12  # forearm
        d = np.sqrt(r**2 + z**2)
        d = np.clip(d, abs(l1 - l2) + 0.01, l1 + l2 - 0.01)

        # Elbow angle (law of cosines)
        cos_elbow = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_elbow = np.clip(cos_elbow, -1, 1)
        angles[2] = np.pi - np.arccos(cos_elbow)

        # Shoulder angle
        cos_shoulder = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
        cos_shoulder = np.clip(cos_shoulder, -1, 1)
        angles[1] = np.arctan2(z, r) + np.arccos(cos_shoulder)

        # Wrist pitch: keep end-effector pointing down
        angles[3] = -(angles[1] + angles[2]) * 0.5

        # Wrist roll: from human arm rotation
        elbow_rel = arm_pose.elbow - arm_pose.shoulder
        angles[4] = np.arctan2(elbow_rel[0], -elbow_rel[1]) * 0.3

        return angles

    def _limit_velocity(self, angles: np.ndarray) -> np.ndarray:
        """Limit joint velocity to prevent jerky motion."""
        if self._prev_angles is None:
            return angles
        delta = angles - self._prev_angles
        limit = self.config.velocity_limit
        delta = np.clip(delta, -limit, limit)
        return self._prev_angles + delta

    def set_gripper(self, angles: np.ndarray, value: float) -> np.ndarray:
        """Set gripper value on existing joint angles.

        Args:
            angles: 6-element joint angle array.
            value: Gripper value 0.0 (closed) to 1.0 (open).
        """
        result = angles.copy()
        result[5] = np.clip(value, 0.0, 1.0) * 0.03  # map to gripper joint range
        return result
