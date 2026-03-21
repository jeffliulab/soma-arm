"""LeRobot exploration script: understand data format and training pipeline.

Usage:
    python scripts/demo_lerobot.py

This script demonstrates:
1. LeRobot dataset format exploration
2. ACT policy configuration
3. Basic training pipeline structure
"""



def check_imports() -> bool:
    """Check if required packages are available."""
    available = {}
    for pkg in ["torch", "numpy", "lerobot"]:
        try:
            __import__(pkg)
            available[pkg] = True
        except ImportError:
            available[pkg] = False

    print("Package availability:")
    for pkg, avail in available.items():
        status = "OK" if avail else "NOT INSTALLED"
        print(f"  {pkg}: {status}")
    return all(available.values())


def explore_dataset_format() -> None:
    """Explore LeRobot dataset format and structure."""
    try:
        from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

        print("\n=== LeRobot Dataset Format ===")
        print("LeRobotDataset is available for loading and creating datasets.")

        # Try loading an example dataset from HuggingFace Hub
        print("\nAttempting to load example dataset from HuggingFace Hub...")
        try:
            dataset = LeRobotDataset("lerobot/pusht")
            print("Dataset loaded: lerobot/pusht")
            print(f"  Number of episodes: {dataset.num_episodes}")
            print(f"  Number of frames: {dataset.num_frames}")
            print(f"  FPS: {dataset.fps}")

            # Explore one sample
            sample = dataset[0]
            print(f"\nSample keys: {list(sample.keys())}")
            for key, value in sample.items():
                if hasattr(value, "shape"):
                    print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
                else:
                    print(f"  {key}: {type(value).__name__} = {value}")
        except Exception as e:
            print(f"  Could not load example dataset: {e}")
            print("  (This is expected if not connected to internet or dataset not cached)")

    except ImportError:
        print("\nLeRobot not installed. Install with: pip install lerobot")


def explore_act_config() -> None:
    """Explore ACT policy configuration."""
    try:
        print("\n=== ACT Policy Configuration ===")
        from lerobot.common.policies.act.configuration_act import ACTConfig

        config = ACTConfig()
        print("ACTConfig available with defaults:")
        print(f"  chunk_size: {config.chunk_size}")
        print(f"  n_action_steps: {config.n_action_steps}")
        print(f"  dim_model: {config.dim_model}")
        print(f"  n_heads: {config.n_heads}")
        print(f"  n_encoder_layers: {config.n_encoder_layers}")
        print(f"  n_decoder_layers: {config.n_decoder_layers}")
    except ImportError:
        print("\nACT policy config not available. This may be due to LeRobot version.")
    except Exception as e:
        print(f"\nCould not explore ACT config: {e}")


def explore_training_pipeline() -> None:
    """Show the typical LeRobot training pipeline structure."""
    print("\n=== LeRobot Training Pipeline ===")
    print(
        """
Typical workflow:
1. Data Collection (teleoperation):
   python -m lerobot.scripts.control_robot \\
     --robot.type=so100 \\
     --control.type=record \\
     --control.repo_id=<user>/<dataset_name> \\
     --control.num_episodes=50

2. Training (ACT policy):
   python -m lerobot.scripts.train \\
     --policy.type=act \\
     --dataset.repo_id=<user>/<dataset_name> \\
     --output_dir=models/trained/act_model

3. Evaluation:
   python -m lerobot.scripts.control_robot \\
     --robot.type=so100 \\
     --control.type=replay_policy \\
     --control.policy_path=models/trained/act_model

For simulation-only workflow:
   - Collect data via vision teleop in MuJoCo
   - Save in LeRobotDataset format (Parquet + images)
   - Train ACT with image observations + joint positions
   - Evaluate in MuJoCo simulation
"""
    )


def main() -> None:
    print("=" * 60)
    print("SmartRobotArm - LeRobot Exploration Script")
    print("=" * 60)

    all_available = check_imports()

    if not all_available:
        print("\nSome packages are missing. Install with:")
        print('  pip install -e ".[all]"')
        print("\nShowing pipeline structure anyway...\n")

    explore_dataset_format()
    explore_act_config()
    explore_training_pipeline()

    print("\n" + "=" * 60)
    print("Exploration complete!")
    if all_available:
        print("All core packages are available. Ready for Phase 1.")
    else:
        print("Install missing packages before proceeding to Phase 1.")


if __name__ == "__main__":
    main()
