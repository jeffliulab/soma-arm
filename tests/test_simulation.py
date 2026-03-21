"""Tests for MuJoCo simulation environments."""

import numpy as np

from src.simulation.arm_env import ArmEnvConfig, ArmSimEnv
from src.simulation.domain_randomization import DomainRandomizer, RandomizationConfig
from src.simulation.hand_env import HandEnvConfig, HandSimEnv


class TestArmSimEnv:
    def test_env_creation(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        assert env.observation_space.shape == (13,)
        assert env.action_space.shape == (6,)
        env.close()

    def test_reset(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        obs, info = env.reset(seed=42)
        assert obs.shape == (13,)
        assert np.all(np.isfinite(obs))
        env.close()

    def test_step(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        env.reset(seed=42)
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        assert obs.shape == (13,)
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        env.close()

    def test_set_joint_positions(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        env.reset(seed=42)
        positions = np.array([0.1, 0.2, 0.3, 0.1, 0.0, 0.01])
        env.set_joint_positions(positions)
        current = env.get_joint_positions()
        assert current.shape == (6,)
        env.close()

    def test_render_image(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode="rgb_array"))
        env.reset(seed=42)
        img = env.render_image()
        assert img.shape == (480, 640, 3)
        assert img.dtype == np.uint8
        env.close()

    def test_multiple_episodes(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        for _ in range(3):
            obs, _ = env.reset(seed=42)
            for _ in range(10):
                action = env.action_space.sample()
                obs, _, terminated, truncated, _ = env.step(action)
                if terminated or truncated:
                    break
        env.close()


class TestHandSimEnv:
    def test_env_creation(self):
        env = HandSimEnv(HandEnvConfig(render_mode=None))
        assert env.observation_space.shape == (23,)
        assert env.action_space.shape == (16,)
        env.close()

    def test_reset_and_step(self):
        env = HandSimEnv(HandEnvConfig(render_mode=None))
        obs, _ = env.reset(seed=42)
        assert obs.shape == (23,)
        action = env.action_space.sample()
        obs, reward, _, _, _ = env.step(action)
        assert np.all(np.isfinite(obs))
        env.close()


class TestDomainRandomization:
    def test_randomize_and_restore(self):
        env = ArmSimEnv(ArmEnvConfig(render_mode=None))
        env.reset(seed=42)
        randomizer = DomainRandomizer(RandomizationConfig(), seed=42)
        randomizer.save_defaults(env.model)

        # Save original values
        orig_damping = env.model.dof_damping.copy()

        # Randomize
        randomizer.randomize(env.model, env.data)

        # Restore
        randomizer.restore_defaults(env.model)
        np.testing.assert_array_equal(env.model.dof_damping, orig_damping)

        env.close()
