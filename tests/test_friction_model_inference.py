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

CHECKPOINT_PATH = "../models/friction/results/run_001/best_model.pt"
TARGET_KEY = "force_mN_realized_aligned"


# ── FrictionPredictor (mirrors orca_interface.py) ────────────────────────

class FrictionPredictor:
    """Wraps a trained FrictionMLP checkpoint for inference.

    Handles normalization/denormalization internally so the caller can
    pass raw physical values and receive predictions in mN.
    Matches the implementation used in orca_interface.py.
    """

    def __init__(self, checkpoint_path):
        ckpt = torch.load(checkpoint_path, map_location="cpu")

        self.input_keys = ckpt["input_keys"]
        self.norm_stats = ckpt["norm_stats"]
        self.target_key = TARGET_KEY

        self._model = FrictionMLP(
            n_inputs=ckpt["n_inputs"],
            hidden_dims=ckpt["hidden_dims"],
            activation=ckpt["activation"],
            dropout=ckpt["dropout"],
        )
        self._model.load_state_dict(ckpt["model_state_dict"])
        self._model.eval()

        # Pre-compute normalization arrays for fast vectorized transform
        self._input_means = np.array(
            [self.norm_stats[k]["mean"] for k in self.input_keys],
            dtype=np.float32,
        )
        self._input_stds = np.array(
            [self.norm_stats[k]["std"] if self.norm_stats[k]["std"] > 1e-12 else 1.0
             for k in self.input_keys],
            dtype=np.float32,
        )
        self._target_mean = self.norm_stats[self.target_key]["mean"]
        self._target_std = (
            self.norm_stats[self.target_key]["std"]
            if self.norm_stats[self.target_key]["std"] > 1e-12 else 1.0
        )

        # Pre-allocate input tensor (1 sample) to avoid repeated allocation
        self._x_buf = torch.zeros(1, ckpt["n_inputs"], dtype=torch.float32)

    @torch.no_grad()
    def predict_mN(self, position_um, velocity_mm_s, accel_mmpss_prev,
                   force_mN_sensed):
        """Single-sample inference. Returns force_realized in mN."""
        raw = np.array([position_um, velocity_mm_s, accel_mmpss_prev,
                        force_mN_sensed], dtype=np.float32)
        normed = (raw - self._input_means) / self._input_stds
        self._x_buf[0] = torch.from_numpy(normed)
        y_norm = self._model(self._x_buf).item()
        return y_norm * self._target_std + self._target_mean

    @torch.no_grad()
    def predict_batch_mN(self, position_um, velocity_mm_s, accel_mmpss_prev,
                         force_mN_sensed):
        """Batch inference. All inputs are 1-D arrays of length N.
        Returns a 1-D numpy array of force_realized in mN."""
        raw = np.column_stack([
            position_um, velocity_mm_s, accel_mmpss_prev, force_mN_sensed,
        ]).astype(np.float32)

        normed = (raw - self._input_means) / self._input_stds
        x_batch = torch.from_numpy(normed)
        y_norm = self._model(x_batch).numpy()
        return y_norm * self._target_std + self._target_mean


# ── Example usage ────────────────────────────────────────────────────────

def main():
    # 1. Load predictor (model + norm stats from checkpoint)
    predictor = FrictionPredictor(CHECKPOINT_PATH)

    print("Model architecture:")
    print(predictor._model)
    print(f"\nInput keys (in order): {predictor.input_keys}")
    print(f"\nNormalization statistics (from checkpoint):")
    for key, stats in predictor.norm_stats.items():
        print(f"  {key:40s}  mean={stats['mean']:12.4f}  std={stats['std']:12.4f}")

    # ------------------------------------------------------------------
    # 2. Single-sample inference
    # ------------------------------------------------------------------
    print("\n--- Single-sample inference ---")
    point1=[250000, 50.0, 20, 5000]
    point1 = {"position_um":250000.0,
        "velocity_mm_s":50.0,
        "accel_mmpss_prev":20,
        "force_mN_sensed":10000
        }
    print(f"\nData point to predict:\n{point1}")
    y_mN = predictor.predict_mN(**point1)
    print(f"  Predicted force_realized = {y_mN:.4f} mN")

    # ------------------------------------------------------------------
    # 3. Batch inference
    # ------------------------------------------------------------------
    N = 5
    rng = np.random.default_rng(42)
    y_mN_batch = predictor.predict_batch_mN(
        position_um=rng.uniform(10000, 550000, size=N),
        velocity_mm_s=rng.uniform(-200, 200, size=N),
        accel_mmpss_prev=rng.uniform(-5000, 5000, size=N),
        force_mN_sensed=rng.uniform(0, 600, size=N),
    )

    print(f"\n--- Batch inference ({N} samples) ---")
    for i in range(N):
        print(f"  Sample {i}: predicted force_realized = {y_mN_batch[i]:.4f} mN")


if __name__ == "__main__":
    main()