#!/usr/bin/env python
"""
train_model.py

Train a FrictionMLP on the prepared friction dataset.

Saves:
  - best_model.pt          (checkpoint with lowest val loss)
  - training_curves.png    (train + val loss over epochs)
  - train_config.yaml      (copy of the config used)
  - metrics.yaml           (final train/val/test losses and metadata)

Usage:
    python train_model.py --config train_config.yaml
"""

import argparse
import copy
import os
import shutil
import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
import yaml

from friction_model import FrictionDataset, FrictionMLP, MODEL_VARIANTS


# ── CLI ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Train friction MLP")
    p.add_argument("--config", type=str, default="train_config.yaml",
                   help="Path to training config YAML")
    return p.parse_args()


# ── Training ─────────────────────────────────────────────────────────────────

def train_one_epoch(model, loader, optimizer, criterion, device):
    model.train()
    total_loss = 0.0
    n_samples = 0
    for X_batch, y_batch in loader:
        X_batch, y_batch = X_batch.to(device), y_batch.to(device)
        optimizer.zero_grad()
        pred = model(X_batch)
        loss = criterion(pred, y_batch)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * len(y_batch)
        n_samples += len(y_batch)
    return total_loss / n_samples


@torch.no_grad()
def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    n_samples = 0
    for X_batch, y_batch in loader:
        X_batch, y_batch = X_batch.to(device), y_batch.to(device)
        pred = model(X_batch)
        loss = criterion(pred, y_batch)
        total_loss += loss.item() * len(y_batch)
        n_samples += len(y_batch)
    return total_loss / n_samples


def plot_training_curves(train_losses, val_losses, output_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    epochs = range(1, len(train_losses) + 1)
    ax.plot(epochs, train_losses, label="Train")
    ax.plot(epochs, val_losses, label="Validation")
    ax.set_xlabel("Epoch")
    ax.set_ylabel("MSE Loss")
    ax.set_title("Training Curves")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_yscale("log")
    plt.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.normpath(os.path.join(script_dir, ".."))

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    # Resolve output_dir: absolute paths used as-is, relative joined with repo_root
    output_dir = cfg["output_dir"]
    if not os.path.isabs(output_dir):
        output_dir = os.path.join(repo_root, output_dir)

    # Dataset path: allow override via config, else default location
    dataset_path = cfg.get("dataset_path", None)
    if dataset_path is None:
        dataset_path = os.path.join(output_dir, "friction_dataset.npz")
    elif not os.path.isabs(dataset_path):
        dataset_path = os.path.join(repo_root, dataset_path)

    if not os.path.exists(dataset_path):
        print(f"ERROR: Dataset not found at {dataset_path}")
        print("Run prepare_data.py first.")
        return

    os.makedirs(output_dir, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    # Seed
    seed = cfg["seed"]
    torch.manual_seed(seed)
    np.random.seed(seed)

    # Model variant
    model_variant = cfg.get("model_variant", 1)
    print(f"Model variant: {model_variant} "
          f"(inputs: {MODEL_VARIANTS[model_variant]})")

    # Datasets and loaders
    train_ds = FrictionDataset(dataset_path, split="train",
                               model_variant=model_variant)
    val_ds = FrictionDataset(dataset_path, split="val",
                             model_variant=model_variant)
    test_ds = FrictionDataset(dataset_path, split="test",
                              model_variant=model_variant)

    batch_size = cfg["batch_size"]
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              drop_last=False)
    val_loader = DataLoader(val_ds, batch_size=batch_size, shuffle=False)
    test_loader = DataLoader(test_ds, batch_size=batch_size, shuffle=False)

    print(f"Train: {len(train_ds)} samples, Val: {len(val_ds)}, Test: {len(test_ds)}")

    # Model
    n_inputs = train_ds.n_inputs
    model = FrictionMLP(
        n_inputs=n_inputs,
        hidden_dims=cfg["hidden_dims"],
        activation=cfg["activation"],
        dropout=cfg["dropout"],
    ).to(device)
    print(f"Model: {model}")
    n_params = sum(p.numel() for p in model.parameters())
    print(f"Parameters: {n_params}")

    # Optimizer
    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=cfg["learning_rate"],
        weight_decay=cfg["weight_decay"],
    )

    # LR scheduler
    scheduler_name = cfg.get("lr_scheduler", "none")
    scheduler = None
    if scheduler_name == "plateau":
        scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            optimizer,
            mode="min",
            patience=cfg.get("plateau_patience", 10),
            factor=cfg.get("plateau_factor", 0.5),
        )
    elif scheduler_name == "cosine":
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
            optimizer, T_max=cfg["epochs"]
        )

    criterion = nn.MSELoss()

    # Training loop with early stopping
    epochs = cfg["epochs"]
    early_stopping_patience = cfg.get("early_stopping_patience", 0)
    train_losses = []
    val_losses = []
    best_val_loss = float("inf")
    best_state = None
    best_epoch = 0
    epochs_no_improve = 0

    t_start = time.time()
    print(f"\nTraining for up to {epochs} epochs "
          f"(early stopping patience: {early_stopping_patience or 'disabled'})...")

    for epoch in range(1, epochs + 1):
        train_loss = train_one_epoch(model, train_loader, optimizer,
                                     criterion, device)
        val_loss = evaluate(model, val_loader, criterion, device)
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        current_lr = optimizer.param_groups[0]["lr"]

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_state = copy.deepcopy(model.state_dict())
            best_epoch = epoch
            epochs_no_improve = 0
            marker = " *"
        else:
            epochs_no_improve += 1
            marker = ""

        if epoch % 10 == 0 or epoch == 1 or marker:
            print(f"  Epoch {epoch:4d} | "
                  f"Train: {train_loss:.6f} | Val: {val_loss:.6f} | "
                  f"LR: {current_lr:.2e}{marker}")

        if scheduler is not None:
            if scheduler_name == "plateau":
                scheduler.step(val_loss)
            else:
                scheduler.step()

        if early_stopping_patience > 0 and epochs_no_improve >= early_stopping_patience:
            print(f"  Early stopping at epoch {epoch} "
                  f"(no improvement for {early_stopping_patience} epochs)")
            break

    train_time_s = time.time() - t_start

    # Save best model
    checkpoint_path = os.path.join(output_dir, "best_model.pt")
    checkpoint = {
        "model_state_dict": best_state,
        "model_variant": model_variant,
        "input_keys": MODEL_VARIANTS[model_variant],
        "n_inputs": n_inputs,
        "hidden_dims": cfg["hidden_dims"],
        "activation": cfg["activation"],
        "dropout": cfg["dropout"],
        "best_epoch": best_epoch,
        "best_val_loss": best_val_loss,
        "norm_stats": train_ds.norm_stats,
    }
    torch.save(checkpoint, checkpoint_path)
    print(f"\nBest model (epoch {best_epoch}, val_loss={best_val_loss:.6f}) "
          f"saved to {checkpoint_path}")

    # Evaluate on test set
    model.load_state_dict(best_state)
    test_loss = evaluate(model, test_loader, criterion, device)
    print(f"Test MSE: {test_loss:.6f}  (RMSE: {test_loss**0.5:.6f})")

    # Training curves
    curves_path = os.path.join(output_dir, "training_curves.png")
    plot_training_curves(train_losses, val_losses, curves_path)
    print(f"Training curves saved to {curves_path}")

    # Copy config
    config_copy_path = os.path.join(output_dir, "train_config.yaml")
    shutil.copy2(args.config, config_copy_path)

    # Save metrics
    metrics = {
        "model_variant": model_variant,
        "n_inputs": n_inputs,
        "n_params": n_params,
        "hidden_dims": cfg["hidden_dims"],
        "activation": cfg["activation"],
        "dropout": cfg["dropout"],
        "learning_rate": cfg["learning_rate"],
        "weight_decay": cfg["weight_decay"],
        "batch_size": cfg["batch_size"],
        "best_epoch": best_epoch,
        "epochs_trained": len(train_losses),
        "train_mse": float(train_losses[best_epoch - 1]),
        "val_mse": float(best_val_loss),
        "test_mse": float(test_loss),
        "test_rmse_mN": float(test_loss ** 0.5),
        "train_time_s": round(train_time_s, 1),
    }
    metrics_path = os.path.join(output_dir, "metrics.yaml")
    with open(metrics_path, "w") as f:
        yaml.dump(metrics, f, default_flow_style=False, sort_keys=False)
    print(f"Metrics saved to {metrics_path}")


if __name__ == "__main__":
    main()
