"""Policy evaluation in MuJoCo simulation.

Runs trained policies and measures success rate.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import torch

from src.simulation.arm_env import ArmEnvConfig, ArmSimEnv
from src.training.act_train import load_policy


@dataclass
class EvalConfig:
    """Configuration for policy evaluation."""

    checkpoint_path: str = "models/trained/act_arm/policy_best.pt"
    num_episodes: int = 100
    max_steps: int = 500
    success_threshold: float = 0.03  # object within 3cm of target
    render: bool = True
    save_video: bool = False
    video_dir: str = "data/eval_videos"
    device: str = "cuda" if torch.cuda.is_available() else "cpu"


class PolicyEvaluator:
    """Evaluates trained policies in simulation."""

    def __init__(self, config: EvalConfig | None = None) -> None:
        self.config = config or EvalConfig()

    def evaluate(self) -> dict:
        """Run evaluation and return metrics.

        Returns:
            Dictionary with success_rate, avg_reward, episode_lengths, etc.
        """
        print(f"Loading policy from {self.config.checkpoint_path}...")
        policy = load_policy(self.config.checkpoint_path, self.config.device)

        env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))

        successes = 0
        total_rewards = []
        episode_lengths = []
        video_writer = None

        print(f"Evaluating {self.config.num_episodes} episodes...")

        for ep in range(self.config.num_episodes):
            obs, _ = env.reset()
            episode_reward = 0.0
            step = 0
            action_buffer = []
            buffer_idx = 0

            if self.config.save_video and ep < 5:
                video_writer = self._create_video_writer(ep)

            for step in range(self.config.max_steps):
                # Get action from policy
                if not action_buffer or buffer_idx >= len(action_buffer):
                    # Need new action chunk
                    state = torch.tensor(
                        obs[:6], dtype=torch.float32, device=self.config.device
                    ).unsqueeze(0)

                    with torch.no_grad():
                        action_chunk = policy(state)  # (1, chunk_size, num_joints)

                    action_buffer = action_chunk[0].cpu().numpy()
                    buffer_idx = 0

                # Execute next action from chunk
                action = action_buffer[buffer_idx]
                buffer_idx += 1

                # Normalize action to [-1, 1] for env
                action_normalized = np.clip(action / np.pi, -1, 1)

                obs, reward, terminated, truncated, _ = env.step(action_normalized)
                episode_reward += reward

                # Render and save video
                if video_writer is not None:
                    frame = env.render_image()
                    video_writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

                if terminated or truncated:
                    break

            # Check success: object near target
            object_pos = obs[7:10]  # object position from obs
            target_pos = obs[10:13]  # target position from obs
            dist = np.linalg.norm(object_pos - target_pos)
            success = dist < self.config.success_threshold

            if success:
                successes += 1

            total_rewards.append(episode_reward)
            episode_lengths.append(step + 1)

            if video_writer is not None:
                video_writer.release()
                video_writer = None

            if (ep + 1) % 10 == 0:
                print(
                    f"  Episode {ep+1}/{self.config.num_episodes} | "
                    f"Success rate: {successes/(ep+1):.1%} | "
                    f"Avg reward: {np.mean(total_rewards):.2f}"
                )

        env.close()

        results = {
            "success_rate": successes / self.config.num_episodes,
            "num_successes": successes,
            "num_episodes": self.config.num_episodes,
            "avg_reward": float(np.mean(total_rewards)),
            "std_reward": float(np.std(total_rewards)),
            "avg_episode_length": float(np.mean(episode_lengths)),
        }

        print("\n=== Evaluation Results ===")
        print(f"  Success rate: {results['success_rate']:.1%}")
        print(f"  Avg reward: {results['avg_reward']:.2f} +/- {results['std_reward']:.2f}")
        print(f"  Avg episode length: {results['avg_episode_length']:.0f}")

        return results

    def _create_video_writer(self, episode: int) -> cv2.VideoWriter:
        """Create video writer for recording evaluation."""
        video_dir = Path(self.config.video_dir)
        video_dir.mkdir(parents=True, exist_ok=True)
        video_path = str(video_dir / f"eval_ep{episode:03d}.mp4")
        fourcc = cv2.VideoWriter.fourcc(*"mp4v")
        return cv2.VideoWriter(video_path, fourcc, 20.0, (640, 480))


def replay_in_sim(
    checkpoint_path: str,
    num_episodes: int = 5,
    render_interactive: bool = True,
) -> None:
    """Replay a trained policy in MuJoCo interactive viewer.

    Args:
        checkpoint_path: Path to policy checkpoint.
        num_episodes: Number of episodes to replay.
        render_interactive: If True, show interactive MuJoCo viewer.
    """
    import mujoco.viewer

    from src.simulation.arm_env import SO_ARM100_XML

    policy = load_policy(checkpoint_path)
    model = mujoco.MjModel.from_xml_string(SO_ARM100_XML)
    data = mujoco.MjData(model)

    action_buffer = []
    buffer_idx = 0
    step = 0

    def controller(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        nonlocal action_buffer, buffer_idx, step

        # Get current joint positions
        state = np.array([d.sensordata[i] for i in range(6)])
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)

        if not action_buffer or buffer_idx >= len(action_buffer):
            with torch.no_grad():
                chunk = policy(state_tensor)
            action_buffer = chunk[0].numpy()
            buffer_idx = 0

        action = action_buffer[buffer_idx]
        buffer_idx += 1

        # Apply to actuators
        d.ctrl[:5] = action[:5]
        gripper_val = np.clip(action[5], 0, 0.025)
        d.ctrl[5] = gripper_val
        d.ctrl[6] = gripper_val
        step += 1

    print("Launching interactive viewer with trained policy...")
    mujoco.viewer.launch(model, data, controller=controller)
