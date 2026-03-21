"""Train ACT policy on collected demonstration data.

Usage:
    python scripts/train_act.py --dataset data/demonstrations/so100_pick_place
    python scripts/train_act.py --epochs 300 --batch-size 64
"""

import argparse

from src.training.act_train import ACTTrainConfig, ACTTrainer


def main() -> None:
    parser = argparse.ArgumentParser(description="Train ACT policy")
    parser.add_argument("--dataset", type=str, default="data/demonstrations/so100_pick_place")
    parser.add_argument("--output", type=str, default="models/trained/act_arm")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--chunk-size", type=int, default=20)
    parser.add_argument("--device", type=str, default="cuda")
    args = parser.parse_args()

    config = ACTTrainConfig(
        dataset_path=args.dataset,
        output_dir=args.output,
        num_epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        chunk_size=args.chunk_size,
        device=args.device,
    )

    trainer = ACTTrainer(config)
    save_path = trainer.train()
    print(f"\nModel saved to: {save_path}")


if __name__ == "__main__":
    main()
