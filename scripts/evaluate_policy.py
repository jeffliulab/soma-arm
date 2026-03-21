"""Evaluate trained policy in simulation.

Usage:
    python scripts/evaluate_policy.py --checkpoint models/trained/act_arm/policy_best.pt
    python scripts/evaluate_policy.py --checkpoint models/trained/act_arm/policy_best.pt --replay
"""

import argparse
import json

from src.training.evaluate import EvalConfig, PolicyEvaluator, replay_in_sim


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate trained policy")
    parser.add_argument(
        "--checkpoint", type=str, default="models/trained/act_arm/policy_best.pt"
    )
    parser.add_argument("--num-episodes", type=int, default=100)
    parser.add_argument("--save-video", action="store_true")
    parser.add_argument("--replay", action="store_true", help="Interactive replay in viewer")
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    if args.replay:
        replay_in_sim(args.checkpoint, num_episodes=5)
        return

    config = EvalConfig(
        checkpoint_path=args.checkpoint,
        num_episodes=args.num_episodes,
        save_video=args.save_video,
        device=args.device,
    )

    evaluator = PolicyEvaluator(config)
    results = evaluator.evaluate()

    # Save results
    results_path = args.checkpoint.replace(".pt", "_eval_results.json")
    with open(results_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to: {results_path}")


if __name__ == "__main__":
    main()
