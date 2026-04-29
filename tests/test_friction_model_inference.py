#!/usr/bin/env python
"""
inference_example.py

Load a trained FrictionMLP checkpoint and run inference on raw
(un-normalized) inputs, returning predictions in original physical units.
"""

import numpy as np
import torch
import torch.nn as nn


# ── Reproduce the model class (or import from friction_model.py) ─────────

ACTIVATIONS = {
    "relu": nn.ReLU,
    "tanh": nn.Tanh,
    "leaky_relu": nn.LeakyReLU,
    "gelu": nn.GELU,
    "elu": nn.ELU,
}


class FrictionMLP(nn.Module):
    def __init__(self, n_inputs=3, hidden_dims=(32, 32), activation="relu",
                 dropout=0.0):
        super().__init__()
        act_cls = ACTIVATIONS[activation]
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


# ── Paths ────────────────────────────────────────────────────────────────

CHECKPOINT_PATH = "../models/friction/run_001/best_model.pt"       # trained checkpoint
DATASET_PATH = "../models/friction/friction_dataset.npz"   # only needed if checkpoint lacks norm_stats


def load_model(checkpoint_path):
    """
    Load the checkpoint and reconstruct the model.

    The checkpoint saved by train_model.py already contains:
      - model_state_dict
      - input_keys        (ordered list of feature names)
      - n_inputs
      - hidden_dims
      - activation
      - dropout
      - norm_stats        (dict: key -> {mean, std} for every feature + target)

    Returns
    -------
    model : FrictionMLP  (eval mode, on CPU)
    input_keys : list[str]
    norm_stats : dict
    """
    ckpt = torch.load(checkpoint_path, map_location="cpu")

    model = FrictionMLP(
        n_inputs=ckpt["n_inputs"],
        hidden_dims=ckpt["hidden_dims"],
        activation=ckpt["activation"],
        dropout=ckpt["dropout"],
    )
    model.load_state_dict(ckpt["model_state_dict"])
    model.eval()

    return model, ckpt["input_keys"], ckpt["norm_stats"]


def load_norm_stats_from_dataset(dataset_path):
    """
    Fallback: pull the mean/std values directly from friction_dataset.npz
    (in case you have an older checkpoint that didn't save norm_stats).
    """
    d = np.load(dataset_path, allow_pickle=True)

    TARGET_KEY = "force_mN_realized_aligned"
    ALL_KEYS = [
        "position_um_aligned",
        "velocity_mm_s_aligned",
        "force_mN_sensed_aligned",
        "accel_mmpss_prev",
        TARGET_KEY,
    ]

    norm_stats = {}
    for key in ALL_KEYS:
        norm_stats[key] = {
            "mean": float(d[f"{key}_mean"]),
            "std":  float(d[f"{key}_std"]),
        }
    return norm_stats


# ── Normalize / denormalize helpers ──────────────────────────────────────

def normalize_inputs(raw_values: dict, input_keys: list, norm_stats: dict):
    """
    Take a dict of raw physical values and return a (1, n_inputs) tensor
    ready for the model.

    Parameters
    ----------
    raw_values : dict
        e.g. {"position_um_aligned": 250000.0,
              "velocity_mm_s_aligned": 50.0,
              "accel_mmpss_prev": 1200.0,
              "force_mN_sensed_aligned": 300.0}
    input_keys : list[str]
        Ordered feature names (must match the order the model was trained with).
    norm_stats : dict
        key -> {"mean": float, "std": float}

    Returns
    -------
    x : torch.Tensor, shape (1, n_inputs)
    """
    normed = []
    for key in input_keys:
        mean = norm_stats[key]["mean"]
        std = norm_stats[key]["std"]
        if std < 1e-12:
            std = 1.0
        normed.append((raw_values[key] - mean) / std)

    return torch.tensor([normed], dtype=torch.float32)   # shape (1, n_inputs)


def denormalize_output(y_norm: float, norm_stats: dict,
                       target_key="force_mN_realized_aligned"):
    """
    Convert the model's normalized scalar output back to physical units (mN).

    During training the target was normalized as:
        y_norm = (y_raw - mean) / std

    So we invert:
        y_raw = y_norm * std + mean
    """
    mean = norm_stats[target_key]["mean"]
    std = norm_stats[target_key]["std"]
    return y_norm * std + mean


# ── Example usage ────────────────────────────────────────────────────────

def main():
    # 1. Load model + normalization stats from checkpoint
    model, input_keys, norm_stats = load_model(CHECKPOINT_PATH)

    print("Model architecture:")
    print(model)
    print(f"\nInput keys (in order): {input_keys}")
    print(f"\nNormalization statistics:")
    for key, stats in norm_stats.items():
        print(f"  {key:40s}  mean={stats['mean']:12.4f}  std={stats['std']:12.4f}")

    # ------------------------------------------------------------------
    # 2. Prepare a single raw (un-normalized) sample
    #
    #    For model_variant=2 the expected inputs are:
    #      position_um_aligned      (µm)
    #      velocity_mm_s_aligned    (mm/s)
    #      accel_mmpss_prev         (mm/s²)  – previous timestep
    #      force_mN_sensed_aligned  (mN)
    # ------------------------------------------------------------------
    raw_sample = {
        "position_um_aligned":     250000.0,   # µm
        "velocity_mm_s_aligned":       50.0,   # mm/s
        "accel_mmpss_prev":          1200.0,   # mm/s²
        "force_mN_sensed_aligned":    300.0,   # mN
    }

    # 3. Normalize inputs
    x_tensor = normalize_inputs(raw_sample, input_keys, norm_stats)
    print(f"\nRaw inputs:        {raw_sample}")
    print(f"Normalized tensor: {x_tensor}")

    # 4. Run inference
    with torch.no_grad():
        y_norm = model(x_tensor).item()        # single scalar

    # 5. Denormalize the output back to mN
    y_mN = denormalize_output(y_norm, norm_stats)

    print(f"\nModel output (normalized): {y_norm:.6f}")
    print(f"Model output (mN):         {y_mN:.4f}")

    # ------------------------------------------------------------------
    # 6. Batch inference example – process many samples at once
    # ------------------------------------------------------------------
    N = 5
    rng = np.random.default_rng(42)
    batch_raw = {
        "position_um_aligned":     rng.uniform(10000, 550000, size=N),
        "velocity_mm_s_aligned":   rng.uniform(-200, 200, size=N),
        "accel_mmpss_prev":        rng.uniform(-5000, 5000, size=N),
        "force_mN_sensed_aligned": rng.uniform(0, 600, size=N),
    }

    # Normalize each feature column
    cols = []
    for key in input_keys:
        mean = norm_stats[key]["mean"]
        std = norm_stats[key]["std"] if norm_stats[key]["std"] > 1e-12 else 1.0
        cols.append((batch_raw[key] - mean) / std)

    x_batch = torch.tensor(np.column_stack(cols), dtype=torch.float32)  # (N, n_inputs)

    with torch.no_grad():
        y_norm_batch = model(x_batch).numpy()    # (N,)

    target_mean = norm_stats["force_mN_realized_aligned"]["mean"]
    target_std = norm_stats["force_mN_realized_aligned"]["std"]
    y_mN_batch = y_norm_batch * target_std + target_mean

    print(f"\nBatch inference ({N} samples):")
    for i in range(N):
        print(f"  Sample {i}: predicted force_realized = {y_mN_batch[i]:.4f} mN")


if __name__ == "__main__":
    main()