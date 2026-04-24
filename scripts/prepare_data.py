#!/usr/bin/env python
"""
prepare_data.py

Load stribeck_data.npz files from multiple experiment dates, validate that
key calibration parameters are consistent across datasets, apply the
t_ignore filter, split by trial into train / val / test sets, compute
per-feature normalization statistics, and save a single ready-to-train .npz.

Usage:
    python prepare_data.py --config train_config.yaml
"""

import argparse
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import yaml


# ── CLI ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Prepare friction training data")
    p.add_argument("--config", type=str, default="train_config.yaml",
                   help="Path to training config YAML")
    return p.parse_args()


# ── Parameter validation ─────────────────────────────────────────────────────

PARAMS_MUST_MATCH = [
    "velocity_estimator",
    "rw_window",
    "savgol_window",
    "baud_rate",
    "interframe_delay_us",
    "usb_chip",
    "mass_shaft_kg",
    "shaft_absolute_length_inches",
]


def validate_params_across_dates(yaml_paths):
    """Load each calibration_params.yaml; error if critical params differ."""
    all_params = []
    for path in yaml_paths:
        with open(path, "r") as f:
            all_params.append(yaml.safe_load(f))

    reference = all_params[0]
    ref_path = yaml_paths[0]
    for i, (params, path) in enumerate(zip(all_params[1:], yaml_paths[1:]), start=1):
        for key in PARAMS_MUST_MATCH:
            ref_val = reference.get(key)
            cur_val = params.get(key)
            if ref_val != cur_val:
                print(f"ERROR: Parameter mismatch for '{key}':")
                print(f"  {ref_path}: {ref_val}")
                print(f"  {path}: {cur_val}")
                sys.exit(1)

    print(f"Validated {len(PARAMS_MUST_MATCH)} parameters across "
          f"{len(yaml_paths)} datasets — all consistent.")
    return reference


# ── Data loading ─────────────────────────────────────────────────────────────

def _get_npz_key(d, new_key, old_key):
    """Return data for new_key if present, else fall back to old_key."""
    if new_key in d:
        return d[new_key]
    return d[old_key]


def load_and_merge(data_dirs, t_ignore_s):
    """Load stribeck_data.npz from each date, apply t_ignore filter,
    and merge into flat arrays.

    Returns
    -------
    samples : dict of 1-D arrays, all length N_total
        Keys: position_um_aligned, velocity_mm_s_aligned,
              force_mN_realized_aligned, force_mN_sensed_aligned
    trial_ids : 1-D int array, length N_total
    n_trials_total : int
    """
    all_position = []
    all_velocity = []
    all_force_realized = []
    all_force_sensed = []
    all_trial_id = []
    global_trial = 0

    for data_dir in data_dirs:
        npz_path = os.path.join(data_dir, "stribeck_data.npz")
        d = np.load(npz_path, allow_pickle=True)

        n_trials = int(d["n_trials"])
        boundaries = d["trial_boundaries"]
        t_stream = d["t_stream_s"]
        position = d["position_um_aligned"]
        velocity = d["velocity_mm_s_aligned"]
        force_sensed = _get_npz_key(d, "force_mN_sensed_aligned",
                                    "force_mN_aligned")
        if "force_mN_realized_aligned" in d:
            force_realized = d["force_mN_realized_aligned"]
        else:
            mass = float(d["mass_shaft_kg"])
            force_realized = mass * d["accel_mmpss_aligned"]

        for i in range(n_trials):
            lo, hi = int(boundaries[i]), int(boundaries[i + 1])
            t_trial = t_stream[lo:hi]
            t0 = t_trial[0] if len(t_trial) > 0 else 0.0
            t_rel = t_trial - t0

            mask = t_rel >= t_ignore_s
            n_keep = mask.sum()
            if n_keep == 0:
                global_trial += 1
                continue

            all_position.append(position[lo:hi][mask])
            all_velocity.append(velocity[lo:hi][mask])
            all_force_realized.append(force_realized[lo:hi][mask])
            all_force_sensed.append(force_sensed[lo:hi][mask])
            all_trial_id.append(
                np.full(n_keep, global_trial, dtype=np.int32)
            )
            global_trial += 1

        print(f"  {npz_path}: {n_trials} trials loaded")

    samples = {
        "position_um_aligned": np.concatenate(all_position),
        "velocity_mm_s_aligned": np.concatenate(all_velocity),
        "force_mN_realized_aligned": np.concatenate(all_force_realized),
        "force_mN_sensed_aligned": np.concatenate(all_force_sensed),
    }
    trial_ids = np.concatenate(all_trial_id)

    return samples, trial_ids, global_trial


# ── Train / val / test split ────────────────────────────────────────────────

def split_by_trial(samples, trial_ids, n_trials, train_ratio, val_ratio,
                   test_ratio, seed):
    """Shuffle trial indices and split into train/val/test by trial."""
    rng = np.random.RandomState(seed)
    perm = rng.permutation(n_trials)

    n_train = int(round(n_trials * train_ratio))
    n_val = int(round(n_trials * val_ratio))

    train_trials = set(perm[:n_train].tolist())
    val_trials = set(perm[n_train:n_train + n_val].tolist())
    test_trials = set(perm[n_train + n_val:].tolist())

    train_mask = np.isin(trial_ids, list(train_trials))
    val_mask = np.isin(trial_ids, list(val_trials))
    test_mask = np.isin(trial_ids, list(test_trials))

    def select(mask):
        return {k: v[mask] for k, v in samples.items()}

    return select(train_mask), select(val_mask), select(test_mask)


# ── Normalization ────────────────────────────────────────────────────────────

INPUT_KEYS = ["position_um_aligned", "velocity_mm_s_aligned", "force_mN_realized_aligned"]
EXTRA_FEATURE_KEYS = []
TARGET_KEY = "force_mN_sensed_aligned"


def compute_norm_stats(train_samples):
    """Compute mean/std from training set for each input and extra feature."""
    stats = {}
    for key in INPUT_KEYS + EXTRA_FEATURE_KEYS:
        arr = train_samples[key]
        stats[key + "_mean"] = float(np.mean(arr))
        stats[key + "_std"] = float(np.std(arr))
    target = train_samples[TARGET_KEY]
    stats[TARGET_KEY + "_mean"] = float(np.mean(target))
    stats[TARGET_KEY + "_std"] = float(np.std(target))
    return stats


# ── Main ─────────────────────────────────────────────────────────────────────

def compute_coverage_report(train, val, test, n_bins=20):
    """Compute per-feature ranges and 3D grid coverage statistics.

    Divides the combined (position, velocity, force) space into an
    n_bins^3 grid and reports what fraction of cells contain data.
    """
    feature_keys = ["position_um_aligned", "velocity_mm_s_aligned",
                    "force_mN_realized_aligned"]
    feature_labels = {
        "position_um_aligned": {"unit": "µm", "short": "position"},
        "velocity_mm_s_aligned": {"unit": "mm/s", "short": "velocity"},
        "force_mN_realized_aligned": {"unit": "mN", "short": "force_realized"},
    }

    splits = {"train": train, "val": val, "test": test}
    all_combined = {k: np.concatenate([s[k] for s in [train, val, test]])
                    for k in feature_keys}

    report = {"feature_ranges": {}, "grid_coverage": {}}

    for key in feature_keys:
        info = feature_labels[key]
        entry = {"unit": info["unit"]}
        for sname, sdata in splits.items():
            arr = sdata[key]
            entry[sname] = {
                "min": float(np.min(arr)),
                "max": float(np.max(arr)),
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr)),
                "p5": float(np.percentile(arr, 5)),
                "p95": float(np.percentile(arr, 95)),
                "n_samples": int(len(arr)),
            }
        combined = all_combined[key]
        entry["all"] = {
            "min": float(np.min(combined)),
            "max": float(np.max(combined)),
        }
        report["feature_ranges"][info["short"]] = entry

    pos_all = all_combined["position_um_aligned"]
    vel_all = all_combined["velocity_mm_s_aligned"]
    frc_all = all_combined["force_mN_realized_aligned"]

    pos_edges = np.linspace(pos_all.min(), pos_all.max(), n_bins + 1)
    vel_edges = np.linspace(vel_all.min(), vel_all.max(), n_bins + 1)
    frc_edges = np.linspace(frc_all.min(), frc_all.max(), n_bins + 1)

    hist_all, _ = np.histogramdd(
        np.column_stack([pos_all, vel_all, frc_all]),
        bins=[pos_edges, vel_edges, frc_edges],
    )
    total_cells = n_bins ** 3
    occupied_cells = int(np.sum(hist_all > 0))

    report["grid_coverage"] = {
        "n_bins_per_axis": n_bins,
        "total_cells": total_cells,
        "occupied_cells": occupied_cells,
        "coverage_fraction": round(occupied_cells / total_cells, 4),
    }

    for sname, sdata in splits.items():
        hist_s, _ = np.histogramdd(
            np.column_stack([sdata["position_um_aligned"],
                             sdata["velocity_mm_s_aligned"],
                             sdata["force_mN_realized_aligned"]]),
            bins=[pos_edges, vel_edges, frc_edges],
        )
        occ = int(np.sum(hist_s > 0))
        report["grid_coverage"][sname] = {
            "occupied_cells": occ,
            "coverage_fraction": round(occ / total_cells, 4),
        }

    return report


def save_coverage_report(report, output_dir):
    """Write coverage report as YAML."""
    path = os.path.join(output_dir, "input_coverage.yaml")
    with open(path, "w") as f:
        yaml.dump(report, f, default_flow_style=False, sort_keys=False)
    print(f"Saved input coverage report: {path}")
    return path


def plot_coverage(train, val, test, output_dir):
    """Save 2D projections and a 3D scatter of the input space by split."""
    pairs = [
        ("position_um_aligned", "velocity_mm_s_aligned",
         "Position (µm)", "Velocity (mm/s)"),
        ("position_um_aligned", "force_mN_realized_aligned",
         "Position (µm)", "Force realized (mN)"),
        ("velocity_mm_s_aligned", "force_mN_realized_aligned",
         "Velocity (mm/s)", "Force realized (mN)"),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, (kx, ky, lx, ly) in zip(axes, pairs):
        step_tr = max(1, len(train[kx]) // 15000)
        step_vt = max(1, len(val[kx]) // 5000)
        ax.scatter(train[kx][::step_tr], train[ky][::step_tr],
                   s=2, alpha=0.15, c="tab:blue", label="train")
        ax.scatter(val[kx][::step_vt], val[ky][::step_vt],
                   s=2, alpha=0.25, c="tab:orange", label="val")
        ax.scatter(test[kx][::step_vt], test[ky][::step_vt],
                   s=2, alpha=0.25, c="tab:green", label="test")
        ax.set_xlabel(lx)
        ax.set_ylabel(ly)
        ax.legend(fontsize=8, markerscale=4)
        ax.grid(True, alpha=0.3)

    plt.suptitle("Input space coverage — 2D projections", fontsize=13)
    plt.tight_layout()
    proj_path = os.path.join(output_dir, "input_coverage_2d.png")
    fig.savefig(proj_path, dpi=150)
    plt.close(fig)
    print(f"Saved 2D coverage plot: {proj_path}")

    fig_3d = plt.figure(figsize=(10, 8))
    ax3 = fig_3d.add_subplot(111, projection="3d")
    step_tr = max(1, len(train["position_um_aligned"]) // 15000)
    step_vt = max(1, len(val["position_um_aligned"]) // 5000)
    ax3.scatter(train["position_um_aligned"][::step_tr],
                train["velocity_mm_s_aligned"][::step_tr],
                train["force_mN_realized_aligned"][::step_tr],
                s=2, alpha=0.1, c="tab:blue", label="train")
    ax3.scatter(val["position_um_aligned"][::step_vt],
                val["velocity_mm_s_aligned"][::step_vt],
                val["force_mN_realized_aligned"][::step_vt],
                s=2, alpha=0.2, c="tab:orange", label="val")
    ax3.scatter(test["position_um_aligned"][::step_vt],
                test["velocity_mm_s_aligned"][::step_vt],
                test["force_mN_realized_aligned"][::step_vt],
                s=2, alpha=0.2, c="tab:green", label="test")
    ax3.set_xlabel("Position (µm)")
    ax3.set_ylabel("Velocity (mm/s)")
    ax3.set_zlabel("Force realized (mN)")
    ax3.set_title("Input space coverage — 3D")
    ax3.legend(fontsize=8, markerscale=4)
    scatter_path = os.path.join(output_dir, "input_coverage_3d.png")
    fig_3d.savefig(scatter_path, dpi=150)
    plt.close(fig_3d)
    print(f"Saved 3D coverage plot: {scatter_path}")


def main():
    args = parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.normpath(os.path.join(script_dir, ".."))

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    data_dates = cfg["data_dates"]
    train_ratio = cfg["train_ratio"]
    val_ratio = cfg["val_ratio"]
    test_ratio = cfg["test_ratio"]
    seed = cfg["seed"]
    t_ignore_s = cfg.get("t_ignore_s_override", None)

    # Resolve data directories and YAML paths
    data_base = os.path.join(repo_root, "data")
    data_dirs = [os.path.join(data_base, d) for d in data_dates]
    yaml_paths = [os.path.join(d, "calibration_params.yaml") for d in data_dirs]

    for p in yaml_paths:
        if not os.path.exists(p):
            print(f"ERROR: Missing calibration_params.yaml: {p}")
            sys.exit(1)

    # Validate params
    print("Validating calibration parameters across dates...")
    ref_params = validate_params_across_dates(yaml_paths)

    # Use t_ignore from config override, else from calibration params
    if t_ignore_s is None:
        t_ignore_s = ref_params["t_ignore_s"]
    print(f"t_ignore_s = {t_ignore_s}")

    # Load and merge
    print(f"\nLoading data from {len(data_dates)} date(s)...")
    samples, trial_ids, n_trials = load_and_merge(data_dirs, t_ignore_s)
    n_total = len(samples["force_mN_sensed_aligned"])
    print(f"Total: {n_trials} trials, {n_total} samples "
          f"(after t_ignore={t_ignore_s}s filter)")

    # Split
    print(f"\nSplitting by trial: "
          f"train={train_ratio}, val={val_ratio}, test={test_ratio}, seed={seed}")
    train, val, test = split_by_trial(
        samples, trial_ids, n_trials, train_ratio, val_ratio, test_ratio, seed
    )
    print(f"  Train: {len(train['force_mN_sensed_aligned'])} samples")
    print(f"  Val:   {len(val['force_mN_sensed_aligned'])} samples")
    print(f"  Test:  {len(test['force_mN_sensed_aligned'])} samples")

    # Normalization stats (from training set only)
    norm_stats = compute_norm_stats(train)
    print("\nNormalization stats (from training set):")
    for k, v in norm_stats.items():
        print(f"  {k}: {v:.6f}")

    # Save
    output_dir = os.path.join(repo_root, cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, "friction_dataset.npz")

    np.savez(
        out_path,
        # Train
        train_position_um_aligned=train["position_um_aligned"],
        train_velocity_mm_s_aligned=train["velocity_mm_s_aligned"],
        train_force_mN_realized_aligned=train["force_mN_realized_aligned"],
        train_force_mN_sensed_aligned=train["force_mN_sensed_aligned"],
        # Validation
        val_position_um_aligned=val["position_um_aligned"],
        val_velocity_mm_s_aligned=val["velocity_mm_s_aligned"],
        val_force_mN_realized_aligned=val["force_mN_realized_aligned"],
        val_force_mN_sensed_aligned=val["force_mN_sensed_aligned"],
        # Test
        test_position_um_aligned=test["position_um_aligned"],
        test_velocity_mm_s_aligned=test["velocity_mm_s_aligned"],
        test_force_mN_realized_aligned=test["force_mN_realized_aligned"],
        test_force_mN_sensed_aligned=test["force_mN_sensed_aligned"],
        # Normalization
        **norm_stats,
        # Config snapshot
        data_dates=np.array(data_dates),
        seed=seed,
        t_ignore_s=t_ignore_s,
        train_ratio=train_ratio,
        val_ratio=val_ratio,
        test_ratio=test_ratio,
    )
    print(f"\nSaved dataset: {out_path}")

    # Coverage analysis
    print("\nComputing input space coverage...")
    report = compute_coverage_report(train, val, test)
    save_coverage_report(report, output_dir)
    plot_coverage(train, val, test, output_dir)

    gc = report["grid_coverage"]
    print(f"  3D grid ({gc['n_bins_per_axis']}³): "
          f"{gc['occupied_cells']}/{gc['total_cells']} cells occupied "
          f"({gc['coverage_fraction']*100:.1f}%)")


if __name__ == "__main__":
    main()
