"""
friction_model.py

PyTorch dataset and model for predicting mu_d from
(position_um_aligned, velocity_mm_s_aligned, force_mN_aligned).

Used by train_model.py.  Can also be imported at inference time
in the real-time control loop.
"""

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset


# ── Dataset ──────────────────────────────────────────────────────────────────

class FrictionDataset(Dataset):
    """Loads a split (train/val/test) from the prepared .npz file.

    Inputs are z-score normalized using the provided mean/std stats.
    Target (mu_d) is NOT normalized — the network predicts raw mu_d.
    """

    INPUT_KEYS = ["position_um_aligned", "velocity_mm_s_aligned",
                   "force_mN_aligned"]

    def __init__(self, npz_path, split="train"):
        """
        Parameters
        ----------
        npz_path : str
            Path to friction_dataset.npz produced by prepare_data.py.
        split : str
            One of "train", "val", "test".
        """
        d = np.load(npz_path, allow_pickle=True)

        raw_inputs = []
        for key in self.INPUT_KEYS:
            arr = d[f"{split}_{key}"].astype(np.float32)
            mean = float(d[f"{key}_mean"])
            std = float(d[f"{key}_std"])
            if std < 1e-12:
                std = 1.0
            raw_inputs.append((arr - mean) / std)

        self.X = torch.from_numpy(np.column_stack(raw_inputs))
        self.y = torch.from_numpy(d[f"{split}_mu_d"].astype(np.float32))

        self.norm_stats = {}
        for key in self.INPUT_KEYS:
            self.norm_stats[key] = {
                "mean": float(d[f"{key}_mean"]),
                "std": float(d[f"{key}_std"]),
            }

    def __len__(self):
        return len(self.y)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]


# ── Model ────────────────────────────────────────────────────────────────────

ACTIVATIONS = {
    "relu": nn.ReLU,
    "tanh": nn.Tanh,
    "leaky_relu": nn.LeakyReLU,
}


class FrictionMLP(nn.Module):
    """Feedforward MLP: 3 inputs → hidden layers → 1 output (mu_d)."""

    def __init__(self, hidden_dims=(32, 32), activation="relu", dropout=0.0):
        """
        Parameters
        ----------
        hidden_dims : list of int
            Width of each hidden layer.
        activation : str
            Activation function name ("relu", "tanh", "leaky_relu").
        dropout : float
            Dropout probability applied after each hidden layer (0 = none).
        """
        super().__init__()

        act_cls = ACTIVATIONS.get(activation)
        if act_cls is None:
            raise ValueError(f"Unknown activation: {activation!r}. "
                             f"Choose from {list(ACTIVATIONS.keys())}")

        n_inputs = 3
        layers = []
        prev_dim = n_inputs
        for dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, dim))
            layers.append(act_cls())
            if dropout > 0:
                layers.append(nn.Dropout(dropout))
            prev_dim = dim
        layers.append(nn.Linear(prev_dim, 1))

        self.net = nn.Sequential(*layers)

    def forward(self, x):
        """x: (batch, 3) → (batch,)"""
        return self.net(x).squeeze(-1)
