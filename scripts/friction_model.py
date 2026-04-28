"""
friction_model.py

PyTorch dataset and model for predicting force_mN_sensed_aligned from
motor kinematic features. See MODEL_VARIANTS for what models are trained.

Used by train_model.py.  Can also be imported at inference time
in the real-time control loop.
"""

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset


# ── Model variants ──────────────────────────────────────────────────────────
# Each variant maps to an ordered list of input keys stored in friction_dataset.npz.
# All variants predict force_mN_sensed_aligned.

MODEL_VARIANTS = {
    1: [
        "position_um_aligned",
        "velocity_mm_s_aligned",
        "force_mN_sensed_aligned",
    ],
    2: [
        "position_um_aligned",
        "velocity_mm_s_aligned",
        "accel_mmpss_prev",
        "force_mN_sensed_aligned",
    ],
}

TARGET_KEY = "force_mN_realized_aligned"


# ── Dataset ──────────────────────────────────────────────────────────────────

class FrictionDataset(Dataset):
    """Loads a split (train/val/test) from the prepared .npz file.

    Inputs are z-score normalized using the provided mean/std stats.
    Target (force_mN_sensed_aligned) is NOT normalized.
    """

    def __init__(self, npz_path, split="train", model_variant=1):
        """
        Parameters
        ----------
        npz_path : str
            Path to friction_dataset.npz produced by prepare_data.py.
        split : str
            One of "train", "val", "test".
        model_variant : int
            Which input feature set to use (1–4). See MODEL_VARIANTS.
        """
        if model_variant not in MODEL_VARIANTS:
            raise ValueError(f"Unknown model_variant={model_variant}. "
                             f"Choose from {list(MODEL_VARIANTS.keys())}")

        self.input_keys = MODEL_VARIANTS[model_variant]
        d = np.load(npz_path, allow_pickle=True)

        raw_inputs = []
        self.norm_stats = {}
        for key in self.input_keys:
            arr = d[f"{split}_{key}"].astype(np.float32)
            mean = float(d[f"{key}_mean"])
            std = float(d[f"{key}_std"])
            if std < 1e-12:
                std = 1.0
            raw_inputs.append((arr - mean) / std)
            self.norm_stats[key] = {"mean": mean, "std": std}

        self.X = torch.from_numpy(np.column_stack(raw_inputs))

        target_arr = d[f"{split}_{TARGET_KEY}"].astype(np.float32)
        target_mean = float(d[f"{TARGET_KEY}_mean"])
        target_std = float(d[f"{TARGET_KEY}_std"])
        if target_std < 1e-12:
            target_std = 1.0
        self.y = torch.from_numpy((target_arr - target_mean) / target_std)

        self.norm_stats[TARGET_KEY] = {
            "mean": target_mean,
            "std": target_std,
        }
        self.n_inputs = len(self.input_keys)

    def __len__(self):
        return len(self.y)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]


# ── Model ────────────────────────────────────────────────────────────────────

ACTIVATIONS = {
    "relu": nn.ReLU,
    "tanh": nn.Tanh,
    "leaky_relu": nn.LeakyReLU,
    "gelu": nn.GELU,
    "elu": nn.ELU,
}


class FrictionMLP(nn.Module):
    """Feedforward MLP: n_inputs → hidden layers → 1 output (force_mN_sensed)."""

    def __init__(self, n_inputs=3, hidden_dims=(32, 32), activation="relu",
                 dropout=0.0):
        super().__init__()

        act_cls = ACTIVATIONS.get(activation)
        if act_cls is None:
            raise ValueError(f"Unknown activation: {activation!r}. "
                             f"Choose from {list(ACTIVATIONS.keys())}")

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
        return self.net(x).squeeze(-1)
