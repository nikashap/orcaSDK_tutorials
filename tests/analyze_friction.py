#!/usr/bin/env python
"""
analyze_friction.py

Analyze calibration data to produce:
  1) Velocity over time graph (per force level)
  2) Stribeck curve (μ_d vs shaft speed)
  3) Augmented data file with friction coefficient estimates appended

All parameters are read from calibration_params.yaml.

Usage:
  python analyze_friction.py
  python analyze_friction.py --data path/to/calibration_mass_estimation.npz
"""

import argparse
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import yaml


# ---------------------------------------------------------------------------
# Load parameters from YAML
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PARAMS_PATH = os.path.join(_SCRIPT_DIR, "calibration_params.yaml")

with open(_PARAMS_PATH, "r") as f:
    PARAMS = yaml.safe_load(f)

M_SHAFT_KG = PARAMS["mass_shaft_kg"]
G_MMPSS = 9810.0  # gravitational acceleration in mm/s²
WEIGHT_MN = M_SHAFT_KG * G_MMPSS  # normal force in mN

SAVGOL_WINDOW = PARAMS["savgol_window"]
SAVGOL_ORDER = PARAMS["savgol_order"]
T_IGNORE_S = PARAMS["t_ignore_s"]
F_STATIC_LOW_MN = PARAMS["static_friction_low_mN"]
F_STATIC_HIGH_MN = PARAMS["static_friction_high_mN"]

DATA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, PARAMS["data_dir"]))


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_master_data(filepath):
    """Load the master .npz file and split into per-trial DataFrames-like dicts."""
    data = np.load(filepath, allow_pickle=True)

    n_trials = int(data["n_trials"])
    boundaries = data["trial_boundaries"]
    force_commanded = data["trial_force_commanded_mN"]

    t_stream_all = data["t_stream"]
    position_um_all = data["position_um"]
    force_mN_all = data["force_mN"]
    t_accel_all = data["t_accel"]
    accel_mmpss_all = data["accel_mmpss"]

    trials = {}
    for i in range(n_trials):
        lo, hi = boundaries[i], boundaries[i + 1]
        t_stream = t_stream_all[lo:hi]
        t0 = t_stream[0] if len(t_stream) > 0 else 0.0

        trials[i + 1] = {
            "trial_num": i + 1,
            "force_commanded_mN": int(force_commanded[i]),
            "t_stream_s": t_stream - t0,
            "t_accel_s": t_accel_all[lo:hi] - t0,
            "position_um": position_um_all[lo:hi].astype(np.float64),
            "force_mN": force_mN_all[lo:hi].astype(np.float64),
            "accel_mmpss": accel_mmpss_all[lo:hi].astype(np.float64),
        }

    metadata = {
        "mass_shaft_kg": float(data["mass_shaft_kg"]),
        "motor_min_um": int(data["motor_min_um"]),
        "motor_max_um": int(data["motor_max_um"]),
        "force_levels_mN": data["force_levels_mN"].tolist(),
        "iterations_per_force": int(data["iterations_per_force"]),
    }

    return trials, metadata


# ---------------------------------------------------------------------------
# Friction computation
# ---------------------------------------------------------------------------

def compute_friction_for_trial(trial, savgol_window, savgol_order):
    """
    For one trial, compute:
      - velocity (Savitzky-Golay smoothed position, then finite differences)
      - F_friction = F_sensed - m * a
      - mu_d = F_friction / (m * g)

    Returns a dict with all original fields plus velocity, friction, mu_d.
    """
    t = trial["t_stream_s"]
    pos_um = trial["position_um"]
    sensed_force = trial["force_mN"]
    t_accel = trial["t_accel_s"]
    accel_raw = trial["accel_mmpss"]

    # Interpolate acceleration onto stream timestamps
    accel_interp_fn = interp1d(t_accel, accel_raw, kind="linear",
                               bounds_error=False, fill_value="extrapolate")
    accel_at_stream = accel_interp_fn(t)

    # Velocity via Savitzky-Golay analytical derivative (deriv=1).
    # Using delta=mean(dt) because savgol_filter assumes uniform spacing.
    # This computes velocity from the polynomial fit directly, avoiding the
    # systematic underestimate caused by smooth-then-diff on non-uniform data.
    mean_dt = np.mean(np.diff(t))
    velocity_um_s = savgol_filter(pos_um, window_length=savgol_window,
                                  polyorder=savgol_order, deriv=1, delta=mean_dt)
    velocity_mm_s = velocity_um_s / 1000.0

    # F_friction = F_sensed - m * a  (all in mN, since kg * mm/s² = mN)
    f_friction_mN = sensed_force - M_SHAFT_KG * accel_at_stream

    # Coefficient of dynamic friction
    mu_d = f_friction_mN / WEIGHT_MN

    return {
        **trial,
        "accel_interp_mmpss": accel_at_stream,
        "velocity_mm_s": velocity_mm_s,
        "f_friction_mN": f_friction_mN,
        "mu_d": mu_d,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_velocity_over_time(friction_trials, force_levels):
    """Plot velocity vs time for each force level (one subplot per level)."""
    fig, axes = plt.subplots(len(force_levels), 1,
                             figsize=(12, 3 * len(force_levels)), sharex=False)
    if len(force_levels) == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    for i, force_mN in enumerate(force_levels):
        ax = axes[i]
        recs = [r for r in friction_trials if r["force_commanded_mN"] == force_mN]
        for j, r in enumerate(recs):
            ax.plot(r["t_stream_s"], r["velocity_mm_s"],
                    linewidth=0.8, color=colors[j % len(colors)],
                    label=f"Trial {r['trial_num']}")
        ax.set_ylabel("Velocity (mm/s)")
        ax.set_title(f"F_commanded = {force_mN} mN")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.suptitle(f"Velocity estimates (Savitzky-Golay window={SAVGOL_WINDOW}, "
                 f"order={SAVGOL_ORDER}, m={M_SHAFT_KG} kg)", fontsize=12, y=1.01)
    plt.tight_layout()
    return fig


def plot_stribeck_curve(friction_trials, force_levels):
    """Plot Stribeck curve: μ_d vs shaft speed |v|."""
    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.tab10(np.linspace(0, 1, len(force_levels)))

    for i, force_mN in enumerate(force_levels):
        recs = [r for r in friction_trials if r["force_commanded_mN"] == force_mN]
        speeds_all = []
        mu_d_all = []
        for r in recs:
            mask = r["t_stream_s"] >= T_IGNORE_S
            speed = np.abs(r["velocity_mm_s"][mask])
            mu_d = r["mu_d"][mask]
            speeds_all.extend(speed.tolist())
            mu_d_all.extend(mu_d.tolist())

        ax.scatter(speeds_all, mu_d_all, s=8, alpha=0.4, color=colors[i],
                   label=f"F_cmd = {force_mN} mN")

    # Static friction reference lines
    ax.axhline(F_STATIC_LOW_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5,
               label=f"μ_s range: {F_STATIC_LOW_MN/WEIGHT_MN:.4f}–{F_STATIC_HIGH_MN/WEIGHT_MN:.4f}")
    ax.axhline(F_STATIC_HIGH_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5)

    ax.set_xlabel("Shaft speed |v| (mm/s)")
    ax.set_ylabel("μ_d = F_friction / (m·g)")
    ax.set_title(f"Stribeck curve (m={M_SHAFT_KG} kg, t ≥ {T_IGNORE_S} s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Save augmented data
# ---------------------------------------------------------------------------

def save_augmented_data(filepath_in, friction_trials):
    """
    Save a new .npz that contains all original data plus appended arrays:
      - velocity_mm_s, f_friction_mN, mu_d (concatenated across trials)
      - savgol_window, savgol_order (so you know what produced the estimates)
    """
    original = np.load(filepath_in, allow_pickle=True)

    # Concatenate the new per-sample arrays in trial order
    velocity_all = np.concatenate([r["velocity_mm_s"] for r in friction_trials])
    f_friction_all = np.concatenate([r["f_friction_mN"] for r in friction_trials])
    mu_d_all = np.concatenate([r["mu_d"] for r in friction_trials])

    # Build output dict with all original arrays + new ones
    out = {key: original[key] for key in original.files}
    out["velocity_mm_s"] = velocity_all
    out["f_friction_mN"] = f_friction_all
    out["mu_d"] = mu_d_all
    out["savgol_window"] = SAVGOL_WINDOW
    out["savgol_order"] = SAVGOL_ORDER

    # Save alongside the original
    dir_name = os.path.dirname(filepath_in)
    out_path = os.path.join(dir_name, "calibration_mass_estimation_with_friction.npz")
    np.savez(out_path, **out)
    print(f"Saved augmented data: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Analyze ORCA friction calibration data")
    parser.add_argument("--data", type=str, default=None,
                        help="Path to calibration_mass_estimation.npz "
                             "(default: <data_dir>/calibration_mass_estimation.npz)")
    args = parser.parse_args()

    data_path = args.data or os.path.join(DATA_DIR, "calibration_mass_estimation.npz")

    if not os.path.exists(data_path):
        print(f"ERROR: Data file not found: {data_path}")
        print("Run collect_calibration_data.py first (procedure 1 or 3).")
        return

    print(f"Loading data from: {data_path}")
    trials, metadata = load_master_data(data_path)
    force_levels = metadata["force_levels_mN"]
    print(f"Loaded {len(trials)} trials across force levels {force_levels}")

    print(f"\nSavitzky-Golay parameters: window={SAVGOL_WINDOW}, order={SAVGOL_ORDER}")
    print(f"Shaft mass: {M_SHAFT_KG} kg")

    # Compute friction for each trial (in trial order)
    friction_trials = []
    for tnum in sorted(trials.keys()):
        result = compute_friction_for_trial(trials[tnum], SAVGOL_WINDOW, SAVGOL_ORDER)
        friction_trials.append(result)

    # --- Plot 1: Velocity over time ---
    print("\nGenerating velocity over time plot...")
    fig_vel = plot_velocity_over_time(friction_trials, force_levels)
    vel_path = os.path.join(DATA_DIR, "velocity_over_time.png")
    fig_vel.savefig(vel_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {vel_path}")

    # --- Plot 2: Stribeck curve ---
    print("Generating Stribeck curve...")
    fig_stribeck = plot_stribeck_curve(friction_trials, force_levels)
    stribeck_path = os.path.join(DATA_DIR, "stribeck_curve.png")
    fig_stribeck.savefig(stribeck_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {stribeck_path}")

    # --- Save augmented data ---
    print("\nSaving augmented data with friction estimates...")
    save_augmented_data(data_path, friction_trials)

    # --- Summary statistics ---
    print(f"\n{'='*60}")
    print("FRICTION SUMMARY")
    print(f"{'='*60}")
    for force_mN in force_levels:
        recs = [r for r in friction_trials if r["force_commanded_mN"] == force_mN]
        all_mu = np.concatenate([r["mu_d"][r["t_stream_s"] >= T_IGNORE_S] for r in recs])
        all_fric = np.concatenate([r["f_friction_mN"][r["t_stream_s"] >= T_IGNORE_S] for r in recs])
        print(f"  F_cmd={force_mN:6d} mN | "
              f"F_friction={np.mean(all_fric):7.1f} ± {np.std(all_fric):6.1f} mN | "
              f"μ_d={np.mean(all_mu):.4f} ± {np.std(all_mu):.4f}")

    print(f"\n  Static friction reference: {F_STATIC_LOW_MN}–{F_STATIC_HIGH_MN} mN "
          f"(μ_s ≈ {F_STATIC_LOW_MN/WEIGHT_MN:.4f}–{F_STATIC_HIGH_MN/WEIGHT_MN:.4f})")

    plt.show()


if __name__ == "__main__":
    main()
