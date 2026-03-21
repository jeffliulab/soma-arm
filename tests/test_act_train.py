"""Tests for ACT training module."""

import torch

from src.training.act_train import SimpleACTPolicy


class TestSimpleACTPolicy:
    def test_forward_shape(self):
        model = SimpleACTPolicy(num_joints=6, chunk_size=20, dim_model=64)
        state = torch.randn(4, 6)  # batch of 4
        output = model(state)
        assert output.shape == (4, 20, 6)

    def test_forward_deterministic_in_eval(self):
        model = SimpleACTPolicy(num_joints=6, chunk_size=10, dim_model=64)
        model.eval()
        state = torch.randn(1, 6)
        with torch.no_grad():
            out1 = model(state)
            out2 = model(state)
        torch.testing.assert_close(out1, out2)

    def test_different_joint_counts(self):
        for n_joints in [6, 16, 22]:
            model = SimpleACTPolicy(num_joints=n_joints, chunk_size=10, dim_model=64)
            state = torch.randn(2, n_joints)
            output = model(state)
            assert output.shape == (2, 10, n_joints)

    def test_gradient_flow(self):
        model = SimpleACTPolicy(num_joints=6, chunk_size=10, dim_model=64)
        state = torch.randn(2, 6)
        target = torch.randn(2, 10, 6)
        output = model(state)
        loss = torch.nn.functional.mse_loss(output, target)
        loss.backward()
        # Check gradients exist
        for param in model.parameters():
            if param.requires_grad:
                assert param.grad is not None
