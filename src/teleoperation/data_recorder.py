"""Data recorder in LeRobot-compatible format.

Records teleoperation episodes as datasets that can be used
for ACT training via the LeRobot framework.
"""

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np


@dataclass
class Episode:
    """Single demonstration episode."""

    timestamps: list[float] = field(default_factory=list)
    joint_positions: list[np.ndarray] = field(default_factory=list)
    joint_actions: list[np.ndarray] = field(default_factory=list)
    camera_images: list[np.ndarray] = field(default_factory=list)
    sim_images: list[np.ndarray] = field(default_factory=list)

    @property
    def length(self) -> int:
        return len(self.timestamps)


@dataclass
class DataRecorderConfig:
    """Configuration for data recorder."""

    save_dir: str = "data/demonstrations"
    dataset_name: str = "so100_pick_place"
    fps: int = 20
    save_camera_images: bool = True
    save_sim_images: bool = True
    image_format: str = "jpg"
    image_quality: int = 90


class DataRecorder:
    """Records teleoperation data in a format compatible with LeRobot.

    Data layout:
        save_dir/dataset_name/
        ├── metadata.json           # Dataset metadata
        ├── episodes.jsonl          # Episode boundaries
        ├── joint_positions.npy     # (N, num_joints) all frames
        ├── joint_actions.npy       # (N, num_joints) all frames
        ├── camera/                 # Camera images
        │   ├── frame_000000.jpg
        │   └── ...
        └── sim/                    # Sim rendered images
            ├── frame_000000.jpg
            └── ...
    """

    def __init__(self, config: DataRecorderConfig | None = None) -> None:
        self.config = config or DataRecorderConfig()
        self._episodes: list[Episode] = []
        self._current_episode: Episode | None = None
        self._total_frames = 0

    def start_episode(self) -> None:
        """Begin recording a new episode."""
        self._current_episode = Episode()
        print(f"Started episode {len(self._episodes)}")

    def record_frame(
        self,
        joint_positions: np.ndarray,
        joint_actions: np.ndarray,
        camera_image: np.ndarray | None = None,
        sim_image: np.ndarray | None = None,
    ) -> None:
        """Record a single frame."""
        if self._current_episode is None:
            self.start_episode()

        ep = self._current_episode
        ep.timestamps.append(time.time())
        ep.joint_positions.append(joint_positions.copy())
        ep.joint_actions.append(joint_actions.copy())

        if camera_image is not None:
            ep.camera_images.append(camera_image.copy())
        if sim_image is not None:
            ep.sim_images.append(sim_image.copy())

    def end_episode(self) -> int:
        """End current episode and return its index."""
        if self._current_episode is None or self._current_episode.length == 0:
            print("No frames in current episode, skipping.")
            return -1

        idx = len(self._episodes)
        self._total_frames += self._current_episode.length
        self._episodes.append(self._current_episode)
        self._current_episode = None
        print(f"Ended episode {idx} ({self._episodes[idx].length} frames)")
        return idx

    def save(self) -> Path:
        """Save all recorded episodes to disk.

        Returns:
            Path to saved dataset directory.
        """
        if self._current_episode is not None:
            self.end_episode()

        save_path = Path(self.config.save_dir) / self.config.dataset_name
        save_path.mkdir(parents=True, exist_ok=True)

        # Concatenate all episodes
        all_positions = []
        all_actions = []
        episode_boundaries = []
        frame_offset = 0

        for i, ep in enumerate(self._episodes):
            all_positions.extend(ep.joint_positions)
            all_actions.extend(ep.joint_actions)
            episode_boundaries.append({
                "episode_index": i,
                "start_frame": frame_offset,
                "end_frame": frame_offset + ep.length,
                "num_frames": ep.length,
            })
            frame_offset += ep.length

        # Save joint data
        np.save(save_path / "joint_positions.npy", np.array(all_positions))
        np.save(save_path / "joint_actions.npy", np.array(all_actions))

        # Save images
        if self.config.save_camera_images:
            cam_dir = save_path / "camera"
            cam_dir.mkdir(exist_ok=True)
            frame_idx = 0
            for ep in self._episodes:
                for img in ep.camera_images:
                    fname = cam_dir / f"frame_{frame_idx:06d}.{self.config.image_format}"
                    if self.config.image_format == "jpg":
                        cv2.imwrite(
                            str(fname), img,
                            [cv2.IMWRITE_JPEG_QUALITY, self.config.image_quality],
                        )
                    else:
                        cv2.imwrite(str(fname), img)
                    frame_idx += 1

        if self.config.save_sim_images:
            sim_dir = save_path / "sim"
            sim_dir.mkdir(exist_ok=True)
            frame_idx = 0
            for ep in self._episodes:
                for img in ep.sim_images:
                    fname = sim_dir / f"frame_{frame_idx:06d}.{self.config.image_format}"
                    cv2.imwrite(str(fname), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                    frame_idx += 1

        # Save episode boundaries
        with open(save_path / "episodes.jsonl", "w") as f:
            for ep_info in episode_boundaries:
                f.write(json.dumps(ep_info) + "\n")

        # Save metadata
        num_joints = all_positions[0].shape[0] if all_positions else 0
        metadata = {
            "dataset_name": self.config.dataset_name,
            "num_episodes": len(self._episodes),
            "total_frames": self._total_frames,
            "fps": self.config.fps,
            "num_joints": num_joints,
            "has_camera_images": self.config.save_camera_images,
            "has_sim_images": self.config.save_sim_images,
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(save_path / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

        print(f"Dataset saved to {save_path}")
        print(f"  Episodes: {len(self._episodes)}")
        print(f"  Total frames: {self._total_frames}")
        return save_path

    @property
    def num_episodes(self) -> int:
        return len(self._episodes)

    @property
    def total_frames(self) -> int:
        return self._total_frames
