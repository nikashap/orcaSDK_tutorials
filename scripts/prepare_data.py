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

    Handles both the current key names (e.g. force_mN_aligned) and
    legacy names (force_mN) for force data.

    Returns
    -------
    samples : dict of 1-D arrays, all length N_total
        Keys: position_um_aligned, velocity_mm_s_aligned,
              force_mN_aligned, mu_d
    trial_ids : 1-D int array, length N_total
    n_trials_total : int
    """
    all_position = []
    all_velocity = []
    all_force_aligned = []
    all_mu_d = []
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
        force_aligned = _get_npz_key(d, "force_mN_aligned", "force_mN")
        mu_d = d["mu_d"]

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
            all_force_aligned.append(force_aligned[lo:hi][mask])
            all_mu_d.append(mu_d[lo:hi][mask])
            all_trial_id.append(
                np.full(n_keep, global_trial, dtype=np.int32)
            )
            global_trial += 1

        print(f"  {npz_path}: {n_trials} trials loaded")

    samples = {
        "position_um_aligned": np.concatenate(all_position),
        "velocity_mm_s_aligned": np.concatenate(all_velocity),
        "force_mN_aligned": np.concatenate(all_force_aligned),
        "mu_d": np.concatenate(all_mu_d),
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

INPUT_KEYS = ["position_um_aligned", "velocity_mm_s_aligned", "force_mN_aligned"]
EXTRA_FEATURE_KEYS = []
TARGET_KEY = "mu_d"


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

def augment_velocity_sign_flip(samples):
    """Create paired data points with velocity sign flipped.

    For each original sample, a mirror sample is created where
    velocity_mm_s_aligned has its sign negated. All other features and the
    target (mu_d) are kept identical. Returns a new samples dict with double
    the original size.
    """
    flipped = {}
    for key, arr in samples.items():
        if key == "velocity_mm_s_aligned":
            flipped[key] = -arr
        else:
            flipped[key] = arr.copy()

    return {k: np.concatenate([samples[k], flipped[k]]) for k in samples}


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
    augment_velocity_flip = cfg.get("augment_velocity_flip", False)

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
    n_total = len(samples["mu_d"])
    print(f"Total: {n_trials} trials, {n_total} samples "
          f"(after t_ignore={t_ignore_s}s filter)")

    # Split
    print(f"\nSplitting by trial: "
          f"train={train_ratio}, val={val_ratio}, test={test_ratio}, seed={seed}")
    train, val, test = split_by_trial(
        samples, trial_ids, n_trials, train_ratio, val_ratio, test_ratio, seed
    )
    print(f"  Train: {len(train['mu_d'])} samples")
    print(f"  Val:   {len(val['mu_d'])} samples")
    print(f"  Test:  {len(test['mu_d'])} samples")

    # Velocity sign-flip augmentation (training set only)
    if augment_velocity_flip:
        n_before = len(train["mu_d"])
        train = augment_velocity_sign_flip(train)
        print(f"\nVelocity sign-flip augmentation: "
              f"{n_before} → {len(train['mu_d'])} training samples")

    # Normalization stats (from training set only, after augmentation)
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
        train_force_mN_aligned=train["force_mN_aligned"],
        train_mu_d=train["mu_d"],
        # Validation
        val_position_um_aligned=val["position_um_aligned"],
        val_velocity_mm_s_aligned=val["velocity_mm_s_aligned"],
        val_force_mN_aligned=val["force_mN_aligned"],
        val_mu_d=val["mu_d"],
        # Test
        test_position_um_aligned=test["position_um_aligned"],
        test_velocity_mm_s_aligned=test["velocity_mm_s_aligned"],
        test_force_mN_aligned=test["force_mN_aligned"],
        test_mu_d=test["mu_d"],
        # Normalization
        **norm_stats,
        # Config snapshot
        data_dates=np.array(data_dates),
        seed=seed,
        t_ignore_s=t_ignore_s,
        train_ratio=train_ratio,
        val_ratio=val_ratio,
        test_ratio=test_ratio,
        augment_velocity_flip=augment_velocity_flip,
    )
    print(f"\nSaved dataset: {out_path}")


if __name__ == "__main__":
    main()
