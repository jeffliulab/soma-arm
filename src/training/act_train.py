"""ACT policy training wrapper.

Provides training pipeline for Action Chunking with Transformers
using data collected from vision teleoperation.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset


@dataclass
class ACTTrainConfig:
    """Configuration for ACT training."""

    # Data
    dataset_path: str = "data/demonstrations/so100_pick_place"
    # Model architecture
    dim_model: int = 256
    n_heads: int = 8
    n_encoder_layers: int = 4
    n_decoder_layers: int = 4
    chunk_size: int = 20  # predict this many future actions
    # Training
    batch_size: int = 32
    learning_rate: float = 1e-4
    weight_decay: float = 1e-5
    num_epochs: int = 200
    # Hardware
    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    # Output
    output_dir: str = "models/trained/act_arm"


class DemoDataset(Dataset):
    """Dataset loader for collected demonstrations."""

    def __init__(self, dataset_path: str, chunk_size: int = 20) -> None:
        self.path = Path(dataset_path)
        self.chunk_size = chunk_size

        # Load data
        self.joint_positions = np.load(self.path / "joint_positions.npy")
        self.joint_actions = np.load(self.path / "joint_actions.npy")

        # Load episode boundaries
        self.episodes = []
        with open(self.path / "episodes.jsonl") as f:
            for line in f:
                self.episodes.append(json.loads(line))

        # Load metadata
        with open(self.path / "metadata.json") as f:
            self.metadata = json.load(f)

        # Build valid indices (frames where we can extract a full chunk)
        self.valid_indices = []
        for ep in self.episodes:
            start = ep["start_frame"]
            end = ep["end_frame"]
            for i in range(start, end - chunk_size):
                self.valid_indices.append(i)

        self.num_joints = self.joint_positions.shape[1]

    def __len__(self) -> int:
        return len(self.valid_indices)

    def __getitem__(self, idx: int) -> dict:
        frame_idx = self.valid_indices[idx]

        # Current state
        state = torch.tensor(self.joint_positions[frame_idx], dtype=torch.float32)

        # Future action chunk
        action_chunk = torch.tensor(
            self.joint_actions[frame_idx : frame_idx + self.chunk_size],
            dtype=torch.float32,
        )

        return {"state": state, "action_chunk": action_chunk}


class SimpleACTPolicy(nn.Module):
    """Simplified ACT policy for joint-space control.

    Architecture: Transformer encoder-decoder that takes joint state
    and predicts a chunk of future actions.
    """

    def __init__(
        self,
        num_joints: int = 6,
        dim_model: int = 256,
        n_heads: int = 8,
        n_encoder_layers: int = 4,
        n_decoder_layers: int = 4,
        chunk_size: int = 20,
    ) -> None:
        super().__init__()
        self.num_joints = num_joints
        self.chunk_size = chunk_size
        self.dim_model = dim_model

        # State encoder
        self.state_encoder = nn.Sequential(
            nn.Linear(num_joints, dim_model),
            nn.ReLU(),
            nn.Linear(dim_model, dim_model),
        )

        # Transformer encoder
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=dim_model, nhead=n_heads, dim_feedforward=dim_model * 4, batch_first=True
        )
        self.transformer_encoder = nn.TransformerEncoder(
            encoder_layer, num_layers=n_encoder_layers
        )

        # Learned action queries (one per timestep in chunk)
        self.action_queries = nn.Parameter(torch.randn(chunk_size, dim_model))

        # Transformer decoder
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=dim_model, nhead=n_heads, dim_feedforward=dim_model * 4, batch_first=True
        )
        self.transformer_decoder = nn.TransformerDecoder(
            decoder_layer, num_layers=n_decoder_layers
        )

        # Action head
        self.action_head = nn.Linear(dim_model, num_joints)

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        """Predict action chunk from current state.

        Args:
            state: (batch, num_joints) current joint positions.

        Returns:
            (batch, chunk_size, num_joints) predicted action chunk.
        """
        batch_size = state.shape[0]

        # Encode state
        state_emb = self.state_encoder(state).unsqueeze(1)  # (B, 1, D)
        memory = self.transformer_encoder(state_emb)  # (B, 1, D)

        # Decode action chunk
        queries = self.action_queries.unsqueeze(0).expand(batch_size, -1, -1)  # (B, C, D)
        decoded = self.transformer_decoder(queries, memory)  # (B, C, D)

        # Project to joint space
        actions = self.action_head(decoded)  # (B, C, num_joints)
        return actions


class ACTTrainer:
    """Training pipeline for ACT policy."""

    def __init__(self, config: ACTTrainConfig | None = None) -> None:
        self.config = config or ACTTrainConfig()

    def train(self) -> Path:
        """Run training and return path to saved model.

        Returns:
            Path to the saved model checkpoint.
        """
        print(f"Loading dataset from {self.config.dataset_path}...")
        dataset = DemoDataset(self.config.dataset_path, self.config.chunk_size)
        print(f"  {len(dataset)} training samples, {dataset.num_joints} joints")

        dataloader = DataLoader(
            dataset, batch_size=self.config.batch_size, shuffle=True, num_workers=0
        )

        # Create model
        model = SimpleACTPolicy(
            num_joints=dataset.num_joints,
            dim_model=self.config.dim_model,
            n_heads=self.config.n_heads,
            n_encoder_layers=self.config.n_encoder_layers,
            n_decoder_layers=self.config.n_decoder_layers,
            chunk_size=self.config.chunk_size,
        ).to(self.config.device)

        param_count = sum(p.numel() for p in model.parameters())
        print(f"  Model parameters: {param_count:,}")

        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=self.config.learning_rate,
            weight_decay=self.config.weight_decay,
        )
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=self.config.num_epochs
        )
        loss_fn = nn.MSELoss()

        # Training loop
        print(f"Training for {self.config.num_epochs} epochs on {self.config.device}...")
        best_loss = float("inf")

        for epoch in range(self.config.num_epochs):
            model.train()
            epoch_loss = 0.0
            num_batches = 0

            for batch in dataloader:
                state = batch["state"].to(self.config.device)
                target_actions = batch["action_chunk"].to(self.config.device)

                predicted_actions = model(state)
                loss = loss_fn(predicted_actions, target_actions)

                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                optimizer.step()

                epoch_loss += loss.item()
                num_batches += 1

            scheduler.step()
            avg_loss = epoch_loss / max(num_batches, 1)

            if (epoch + 1) % 10 == 0 or epoch == 0:
                print(
                    f"  Epoch {epoch+1}/{self.config.num_epochs} | "
                    f"Loss: {avg_loss:.6f} | LR: {scheduler.get_last_lr()[0]:.2e}"
                )

            if avg_loss < best_loss:
                best_loss = avg_loss
                self._save_model(model, "best")

        # Save final model
        save_path = self._save_model(model, "final")
        print(f"Training complete. Best loss: {best_loss:.6f}")
        return save_path

    def _save_model(self, model: nn.Module, tag: str) -> Path:
        """Save model checkpoint."""
        output_dir = Path(self.config.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        save_path = output_dir / f"policy_{tag}.pt"
        torch.save({
            "model_state_dict": model.state_dict(),
            "config": {
                "num_joints": model.num_joints,
                "dim_model": model.dim_model,
                "chunk_size": model.chunk_size,
            },
        }, save_path)
        return save_path


def load_policy(checkpoint_path: str, device: str = "cpu") -> SimpleACTPolicy:
    """Load a trained ACT policy from checkpoint.

    Args:
        checkpoint_path: Path to .pt checkpoint file.
        device: Device to load model onto.

    Returns:
        Loaded SimpleACTPolicy model in eval mode.
    """
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=True)
    cfg = ckpt["config"]

    model = SimpleACTPolicy(
        num_joints=cfg["num_joints"],
        dim_model=cfg["dim_model"],
        chunk_size=cfg["chunk_size"],
    )
    model.load_state_dict(ckpt["model_state_dict"])
    model.to(device)
    model.eval()
    return model
