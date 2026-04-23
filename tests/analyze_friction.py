#!/usr/bin/env python
"""
analyze_friction.py

Analyze calibration data to produce:
  1) Several graphs that describe how position, velocity, and force command affect friction
  2) Augmented data file with friction coefficient estimates appended

All parameters are read from calibration_params.yaml.

Usage:
  python analyze_friction.py --date 26_03_20-14_30_00
  python analyze_friction.py --date 26_03_20-14_30_00 --data path/to/calibration_dynamic_friction.npz
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d
import yaml

# ---------------------------------------------------------------------------
# Parse CLI arguments
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

_parser = argparse.ArgumentParser(description="Analyze ORCA friction calibration data")
_parser.add_argument("--date", type=str, required=True,
                     help="Date folder name of the experiment (e.g. '26_03_20-14_30_00'). "
                          "The YAML and data files are loaded from <data_dir>/<date>/")
_parser.add_argument("--data", type=str, default=None,
                     help="Override path to calibration_dynamic_friction.npz")
_args = _parser.parse_args()

# ---------------------------------------------------------------------------
# Resolve experiment directory and load the saved YAML snapshot
# ---------------------------------------------------------------------------
# Base data dir is relative to script; the date subfolder identifies the experiment
_BASE_DATA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "../data"))
EXPERIMENT_DIR = os.path.join(_BASE_DATA_DIR, _args.date)

_PARAMS_PATH = os.path.join(EXPERIMENT_DIR, "calibration_params.yaml")
if not os.path.exists(_PARAMS_PATH):
    print(f"ERROR: No calibration_params.yaml found in {EXPERIMENT_DIR}")
    print("Make sure you collected data with collect_calibration_data.py first.")
    sys.exit(1)

with open(_PARAMS_PATH, "r") as f:
    PARAMS = yaml.safe_load(f)

M_SHAFT_KG = PARAMS["mass_shaft_kg"]
G_MMPSS = 9810.0  # gravitational acceleration in mm/s²
WEIGHT_MN = M_SHAFT_KG * G_MMPSS  # normal force in mN

VELOCITY_ESTIMATOR = PARAMS.get("velocity_estimator", "SG").upper()
SAVGOL_WINDOW = PARAMS["savgol_window"]
SAVGOL_ORDER = PARAMS["savgol_order"]
RW_WINDOW = PARAMS.get("rw_window", 5)
T_IGNORE_S = PARAMS["t_ignore_s"]
F_STATIC_LOW_MN = PARAMS["static_friction_low_mN"]
F_STATIC_HIGH_MN = PARAMS["static_friction_high_mN"]
STREAM_FORCE_SHIFT = PARAMS.get("stream_force_shift", 0)
USB_CHIP = PARAMS.get("usb_chip", "unknown")


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_master_data(filepath):
    """Load a master .npz file and split into per-trial dicts.

    Handles both constant-force data (calibration_dynamic_friction.npz) and
    ramp-down data (calibration_rampdown_friction.npz).  In both cases,
    force_commanded_mN is returned as a per-sample array.  A scalar
    force_label_mN is also included for grouping / plotting:
      - constant-force trials: the commanded force level
      - ramp-down trials: the target (post-switch) force
    """
    data = np.load(filepath, allow_pickle=True)

    n_trials = int(data["n_trials"])
    boundaries = data["trial_boundaries"]

    # Determine procedure type: explicit tag in new data, fallback for old data
    if "procedure_type" in data:
        procedure_type = str(data["procedure_type"])
    else:
        procedure_type = "constant"

    is_rampdown = (procedure_type == "rampdown")

    t_stream_all = data["t_stream"]
    position_um_all = data["position_um"]
    force_mN_all = data["force_mN"]
    t_accel_all = data["t_accel"]
    accel_mmpss_all = data["accel_mmpss"]

    if is_rampdown:
        force_cmd_all = data["force_commanded_mN"]
        trial_labels = data["trial_target_force_mN"]
        trial_switch_indices = data["trial_switch_index"]
    else:
        force_cmd_all = (data["force_commanded_mN"] if "force_commanded_mN" in data
                         else None)
        force_cmd_per_trial = data["trial_force_commanded_mN"]

    trials = {}
    for i in range(n_trials):
        lo, hi = boundaries[i], boundaries[i + 1]

        if is_rampdown:
            switch_idx = int(trial_switch_indices[i])
            if switch_idx < 0:
                continue
            lo = lo + switch_idx
            force_cmd = force_cmd_all[lo:hi].astype(np.float64)
            label = int(trial_labels[i])
        elif force_cmd_all is not None:
            force_cmd = force_cmd_all[lo:hi].astype(np.float64)
            label = int(force_cmd_per_trial[i])
        else:
            force_cmd = np.full(hi - lo, force_cmd_per_trial[i], dtype=np.float64)
            label = int(force_cmd_per_trial[i])

        t_stream = t_stream_all[lo:hi]
        if len(t_stream) == 0:
            continue
        t0 = t_stream[0]

        trials[i + 1] = {
            "trial_num": i + 1,
            "force_commanded_mN": force_cmd,
            "force_label_mN": label,
            "t_stream_s": t_stream - t0,
            "t_accel_s": t_accel_all[lo:hi] - t0,
            "position_um": position_um_all[lo:hi].astype(np.float64),
            "force_mN": force_mN_all[lo:hi].astype(np.float64),
            "accel_mmpss": accel_mmpss_all[lo:hi].astype(np.float64),
        }

    if is_rampdown:
        force_levels = sorted(set(int(trial_labels[i]) for i in range(n_trials)))
    else:
        force_levels = data["force_levels_mN"].tolist()

    metadata = {
        "mass_shaft_kg": float(data["mass_shaft_kg"]),
        "motor_min_um": int(data["motor_min_um"]),
        "motor_max_um": int(data["motor_max_um"]),
        "force_levels_mN": force_levels,
    }

    return trials, metadata


# ---------------------------------------------------------------------------
# Friction computation
# ---------------------------------------------------------------------------

def _velocity_savgol(pos_um, t, savgol_window, savgol_order):
    """Velocity via Savitzky-Golay analytical derivative (deriv=1).
    Using delta=mean(dt) because savgol_filter assumes uniform spacing.
    """
    mean_dt = np.mean(np.diff(t))
    velocity_um_s = savgol_filter(pos_um, window_length=savgol_window,
                                  polyorder=savgol_order, deriv=1, delta=mean_dt)
    return velocity_um_s / 1000.0


def _velocity_rolling_window(pos_um, t, rw_window):
    """Velocity via rolling mean of finite differences.

    v[i] is the mean of the last ``rw_window`` finite-difference
    velocity samples: (pos[i]-pos[i-1])/dt, ..., (pos[i-k+1]-pos[i-k])/dt.
    The first ``rw_window`` samples use a growing window (all differences
    available so far) so no samples are NaN.
    """
    dt = np.diff(t)                       # (n-1,)
    v_inst = np.diff(pos_um) / dt / 1000.0  # instantaneous velocity in mm/s, (n-1,)

    n = len(pos_um)
    velocity_mm_s = np.zeros(n)
    # v_inst[i] = velocity between sample i and i+1.
    # We assign v_inst[i] as the velocity "at" sample i+1 (backward difference).
    # Sample 0 gets the first available difference.
    cumsum = np.cumsum(np.concatenate([[0.0], v_inst]))
    for i in range(n):
        if i == 0:
            velocity_mm_s[0] = v_inst[0]  # only one difference available
        else:
            lo = max(0, i - rw_window)
            velocity_mm_s[i] = (cumsum[i] - cumsum[lo]) / (i - lo)

    return velocity_mm_s


def compute_friction_for_trial(trial, velocity_estimator="SG",
                               savgol_window=15, savgol_order=2,
                               rw_window=5, shift=0):
    """
    For one trial, compute:
      - velocity (via the chosen estimator: SG or RW)
      - F_friction = F_sensed_shifted - m * a
      - mu_d = F_friction / (m * g) (this is signed, NOT absolute value)

    Parameters
    ----------
    velocity_estimator : str
        "SG" for Savitzky-Golay (non-causal, offline), "RW" for rolling window (causal, online).
    savgol_window, savgol_order : int
        Parameters for the SG estimator (ignored when velocity_estimator=="RW").
    rw_window : int
        Number of finite-difference samples to average for the RW estimator
        (ignored when velocity_estimator=="SG").
    shift : int
        Frame shift to align force_sensed with acceleration. The motor stream
        (position, force_sensed) lags behind the real-time acceleration read
        by ``shift`` frames. So force_sensed[i+shift] corresponds to accel[i].
        We pair force_sensed[shift:n] with accel[0:n-shift].

    After shifting, only the overlapping region is kept — all arrays are
    trimmed to the same valid range.

    Returns a dict with all original fields (trimmed) plus velocity, friction,
    mu_d, force_commanded_mN (per-sample, trimmed), and
    force_sensed_unshifted_mN (sensed force at the same timepoint as
    position/velocity — NOT shift-aligned to accel).
    """
    t = trial["t_stream_s"]
    pos_um = trial["position_um"]
    sensed_force = trial["force_mN"]
    force_commanded = trial["force_commanded_mN"]
    t_accel = trial["t_accel_s"]
    accel_raw = trial["accel_mmpss"]

    # Interpolate acceleration onto stream timestamps
    accel_interp_fn = interp1d(t_accel, accel_raw, kind="linear",
                               bounds_error=False, fill_value="extrapolate")
    accel_at_stream = accel_interp_fn(t)

    # Velocity estimation
    if velocity_estimator == "SG":
        velocity_mm_s = _velocity_savgol(pos_um, t, savgol_window, savgol_order)
    elif velocity_estimator == "RW":
        velocity_mm_s = _velocity_rolling_window(pos_um, t, rw_window)
    else:
        raise ValueError(f"Unknown velocity_estimator: {velocity_estimator!r}. "
                         f"Expected 'SG' or 'RW'.")

    n = len(t)

    # Apply frame shift to align force_sensed with acceleration.
    # Position, velocity, and force_commanded are NOT shifted — they stay at
    # the stream timepoint.  force_sensed_unshifted is also at the stream
    # timepoint (same frame as position).  force_aligned (used for friction
    # calculation) is the shifted force_sensed that corresponds to accel.
    if shift > 0:
        force_aligned = sensed_force[shift:]
        accel_aligned = accel_at_stream[:n - shift]
        t_trimmed = t[:n - shift]
        pos_trimmed = pos_um[:n - shift]
        vel_trimmed = velocity_mm_s[:n - shift]
        pos_aligned = pos_um[shift:]
        vel_aligned = velocity_mm_s[shift:]
        force_cmd_trimmed = force_commanded[:n - shift]
        force_sensed_unshifted = sensed_force[:n - shift]
    elif shift < 0:
        abs_shift = abs(shift)
        force_aligned = sensed_force[:n - abs_shift]
        accel_aligned = accel_at_stream[abs_shift:]
        t_trimmed = t[abs_shift:]
        pos_trimmed = pos_um[abs_shift:]
        vel_trimmed = velocity_mm_s[abs_shift:]
        pos_aligned = pos_um[:n - abs_shift]
        vel_aligned = velocity_mm_s[:n - abs_shift]
        force_cmd_trimmed = force_commanded[abs_shift:]
        force_sensed_unshifted = sensed_force[abs_shift:]
    else:
        force_aligned = sensed_force
        accel_aligned = accel_at_stream
        t_trimmed = t
        pos_trimmed = pos_um
        vel_trimmed = velocity_mm_s
        pos_aligned = pos_um
        vel_aligned = velocity_mm_s
        force_cmd_trimmed = force_commanded
        force_sensed_unshifted = sensed_force

    # F_friction = F_sensed_aligned - m * a  (all in mN, since kg * mm/s² = mN)
    f_friction_mN = force_aligned - M_SHAFT_KG * accel_aligned

    # Coefficient of dynamic friction
    mu_d = f_friction_mN / WEIGHT_MN

    return {
        "trial_num": trial["trial_num"],
        "force_label_mN": trial["force_label_mN"],
        "force_commanded_mN": force_cmd_trimmed,
        "t_stream_s": t_trimmed,
        "position_um": pos_trimmed,
        "position_um_aligned": pos_aligned,
        "force_mN": force_aligned,
        "force_sensed_unshifted_mN": force_sensed_unshifted,
        "accel_mmpss": accel_aligned,
        "accel_interp_mmpss": accel_aligned,
        "velocity_mm_s": vel_trimmed,
        "velocity_mm_s_aligned": vel_aligned,
        "f_friction_mN": f_friction_mN,
        "mu_d": mu_d,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_velocity_over_time(friction_trials, force_levels):
    """Plot velocity vs time for each force level (one subplot per level)."""
    fig, axes = plt.subplots(len(force_levels), 1,
                             figsize=(12, 4 * len(force_levels)), sharex=False)
    if len(force_levels) == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    for i, force_mN in enumerate(force_levels):
        ax = axes[i]
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for j, r in enumerate(recs):
            ax.plot(r["t_stream_s"], r["velocity_mm_s"],
                    linewidth=0.8, color=colors[j % len(colors)],
                    label=f"Trial {r['trial_num']}")
        ax.set_ylabel("Velocity (mm/s)")
        ax.set_title(f"F_commanded = {force_mN} mN")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    if VELOCITY_ESTIMATOR == "SG":
        vel_label = f"Savitzky-Golay window={SAVGOL_WINDOW}, order={SAVGOL_ORDER}"
    else:
        vel_label = f"Rolling window K={RW_WINDOW}"
    plt.suptitle(f"Velocity estimates ({vel_label}, m={M_SHAFT_KG} kg)",
                 fontsize=12, y=0.98)
    plt.tight_layout()
    return fig

def plot_Fsensed_over_time(friction_trials, force_levels):
    """Plot sensed outputted force over time for ecah force level (one subplot per level)."""
    fig, axes = plt.subplots(len(force_levels), 1,
                             figsize=(12, 4 * len(force_levels)), sharex=False)
    if len(force_levels) == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0,1,10))

    for i, force_mN in enumerate(force_levels): 
        ax = axes[i]
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for j, r in enumerate(recs):
            ax.plot(r["t_stream_s"], r["force_mN"],
                    linewidth=0.8, color=colors[j % len(colors)],
                    label=f"Trial {r['trial_num']}")
        ax.set_ylabel("Sensed force (mN)")
        ax.set_title(f"F_commanded = {force_mN} mN")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.suptitle(f"Sensed force output; filter strength = ___ m={M_SHAFT_KG} kg)", fontsize=12, y=0.98)
    plt.tight_layout()
    return fig

def plot_accel_over_time(friction_trials, force_levels):
    """Plot measured acceleration over time for each force level (one subplot per level)."""
    fig, axes = plt.subplots(len(force_levels), 1,
                             figsize=(12, 4 * len(force_levels)), sharex=False)
    if len(force_levels) == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0,1,10))

    for i, force_mN in enumerate(force_levels): 
        ax = axes[i]
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for j, r in enumerate(recs):
            ax.plot(r["t_stream_s"], r["accel_mmpss"],
                    linewidth=0.8, color=colors[j % len(colors)],
                    label=f"Trial {r['trial_num']}")
        ax.set_ylabel("Acceleration (mm/s^2)")
        ax.set_title(f"F_commanded = {force_mN} mN")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.suptitle(f"Measured acceleration; filter strength = ___ m={M_SHAFT_KG} kg)", fontsize=12, y=0.98)
    plt.tight_layout()
    return fig

def plot_stribeck_curve(friction_trials, force_levels):
    """Plot Stribeck curve: μ_d vs shaft speed v."""
    fig, ax = plt.subplots(figsize=(10, 6))

    speeds_all = []
    mu_d_all = []
    force_all = []

    for force_mN in force_levels:
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for r in recs:
            mask = r["t_stream_s"] >= T_IGNORE_S
            speed = r["velocity_mm_s"][mask]
            mu_d = r["mu_d"][mask]
            force = r["force_mN"][mask]
            speeds_all.extend(speed.tolist())
            mu_d_all.extend(mu_d.tolist())
            force_all.extend(force.tolist())

    sc = ax.scatter(speeds_all, mu_d_all, s=8, alpha=0.4, c=force_all,
                    cmap="viridis")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Sensed force (mN)")

    # Static friction reference lines
    ax.axhline(F_STATIC_LOW_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5,
               label=f"μ_s range: {F_STATIC_LOW_MN/WEIGHT_MN:.4f}–{F_STATIC_HIGH_MN/WEIGHT_MN:.4f}")
    ax.axhline(F_STATIC_HIGH_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5)

    ax.set_xlabel("Velocity (mm/s)")
    ax.set_xlim(np.percentile(speeds_all, 2), np.percentile(speeds_all, 98))
    ax.set_ylabel("μ_d = F_friction / (m·g)")
    ax.set_title(f"Stribeck curve (m={M_SHAFT_KG} kg, t ≥ {T_IGNORE_S} s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig

def plot_position_vs_mud_curve(friction_trials, force_levels):
    """Plot position vs mu_d"""
    fig, ax = plt.subplots(figsize=(10, 6))

    speeds_all = []
    mu_d_all = []
    force_all = []

    for force_mN in force_levels:
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for r in recs:
            mask = r["t_stream_s"] >= T_IGNORE_S
            speed = r["position_um"][mask]
            mu_d = r["mu_d"][mask]
            force = r["force_mN"][mask]
            speeds_all.extend(speed.tolist())
            mu_d_all.extend(mu_d.tolist())
            force_all.extend(force.tolist())

    sc = ax.scatter(speeds_all, mu_d_all, s=8, alpha=0.4, c=force_all,
                    cmap="viridis")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Sensed force (mN)")

    # Static friction reference lines
    ax.axhline(F_STATIC_LOW_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5,
               label=f"μ_s range: {F_STATIC_LOW_MN/WEIGHT_MN:.4f}–{F_STATIC_HIGH_MN/WEIGHT_MN:.4f}")
    ax.axhline(F_STATIC_HIGH_MN / WEIGHT_MN, color="r", linestyle="--", alpha=0.5)

    ax.set_xlabel("Shaft position (um)")
    ax.set_ylabel("μ_d = F_friction / (m·g)")
    ax.set_title(f"Faux positional Stribeck curve (m={M_SHAFT_KG} kg, t ≥ {T_IGNORE_S} s)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig

def plot_stribeck_3d(friction_trials, force_levels):
    """Plot 3D Stribeck surface: position vs velocity vs μ_d, colored by sensed force."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    positions_all = []
    speeds_all = []
    mu_d_all = []
    force_all = []

    for force_mN in force_levels:
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        for r in recs:
            mask = r["t_stream_s"] >= T_IGNORE_S
            positions_all.extend(r["position_um"][mask].tolist())
            speeds_all.extend(r["velocity_mm_s"][mask].tolist())
            mu_d_all.extend(r["mu_d"][mask].tolist())
            force_all.extend(r["force_mN"][mask].tolist())

    sc = ax.scatter(positions_all, speeds_all, mu_d_all,
                    s=6, alpha=0.4, c=force_all, cmap="viridis")
    cbar = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label("Sensed force (mN)")

    ax.set_xlabel("Position (µm)")
    ax.set_ylabel("Velocity (mm/s)")
    ax.set_zlabel("μ_d = F_friction / (m·g)")
    ax.set_title(f"3D Stribeck (m={M_SHAFT_KG} kg, t ≥ {T_IGNORE_S} s, shift={STREAM_FORCE_SHIFT})")

    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Save augmented data
# ---------------------------------------------------------------------------

def save_augmented_data(experiment_dir, friction_trials, metadata):
    """
    Save a new .npz with all per-sample arrays aligned after shift trimming.

    The shift in compute_friction_for_trial trims each trial by `shift` samples.
    All per-sample arrays here come from the trimmed friction_trials dicts, so
    every array is guaranteed to be the same length and sample-aligned.

    Includes per-sample force_commanded_mN and force_sensed_unshifted_mN
    (sensed force at the same timepoint as position/velocity, before shift
    alignment — useful as a potential model input).
    """
    # Concatenate trimmed+aligned per-sample arrays from friction_trials
    t_stream_all = np.concatenate([r["t_stream_s"] for r in friction_trials])
    position_all = np.concatenate([r["position_um"] for r in friction_trials])
    position_aligned_all = np.concatenate(
        [r["position_um_aligned"] for r in friction_trials]
    )
    force_sensed_all = np.concatenate([r["force_mN"] for r in friction_trials])
    force_sensed_unshifted_all = np.concatenate(
        [r["force_sensed_unshifted_mN"] for r in friction_trials]
    )
    force_commanded_all = np.concatenate(
        [r["force_commanded_mN"] for r in friction_trials]
    )
    accel_all = np.concatenate([r["accel_mmpss"] for r in friction_trials])
    velocity_all = np.concatenate([r["velocity_mm_s"] for r in friction_trials])
    velocity_aligned_all = np.concatenate(
        [r["velocity_mm_s_aligned"] for r in friction_trials]
    )
    f_friction_all = np.concatenate([r["f_friction_mN"] for r in friction_trials])
    mu_d_all = np.concatenate([r["mu_d"] for r in friction_trials])

    # Recompute trial boundaries for trimmed data
    trial_n_samples = np.array([len(r["mu_d"]) for r in friction_trials],
                               dtype=np.int32)
    trial_boundaries = np.concatenate([[0], np.cumsum(trial_n_samples)])
    trial_force_labels = np.array([r["force_label_mN"] for r in friction_trials],
                                  dtype=np.int32)

    out = {}

    # Metadata
    out["mass_shaft_kg"] = metadata["mass_shaft_kg"]
    out["motor_min_um"] = metadata["motor_min_um"]
    out["motor_max_um"] = metadata["motor_max_um"]
    out["force_levels_mN"] = np.array(metadata["force_levels_mN"], dtype=np.int32)
    out["n_trials"] = len(friction_trials)

    out["trial_n_samples"] = trial_n_samples
    out["trial_boundaries"] = trial_boundaries
    out["trial_force_label_mN"] = trial_force_labels

    # Per-sample arrays — names indicate alignment relative to acceleration.
    # "aligned" = shift-corrected to match the acceleration reading.
    # "unshifted" = at the original stream timepoint (same frame as position).
    out["t_stream_s"] = t_stream_all
    out["position_um_unshifted"] = position_all
    out["position_um_aligned"] = position_aligned_all
    out["force_mN_aligned"] = force_sensed_all
    out["force_sensed_unshifted_mN"] = force_sensed_unshifted_all
    out["force_commanded_mN"] = force_commanded_all
    out["accel_mmpss_aligned"] = accel_all
    out["velocity_mm_s_unshifted"] = velocity_all
    out["velocity_mm_s_aligned"] = velocity_aligned_all
    out["f_friction_mN"] = f_friction_all
    out["mu_d"] = mu_d_all

    # Analysis parameters
    out["velocity_estimator"] = VELOCITY_ESTIMATOR
    out["savgol_window"] = SAVGOL_WINDOW
    out["savgol_order"] = SAVGOL_ORDER
    out["rw_window"] = RW_WINDOW
    out["stream_force_shift"] = STREAM_FORCE_SHIFT

    out_path = os.path.join(experiment_dir, "stribeck_data.npz")
    np.savez(out_path, **out)
    print(f"Saved augmented data: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # Locate data files — load constant-force and/or ramp-down
    const_path = os.path.join(EXPERIMENT_DIR, "calibration_dynamic_friction.npz")
    rampdown_path = os.path.join(EXPERIMENT_DIR, "calibration_rampdown_friction.npz")

    if _args.data:
        data_paths = [_args.data]
    else:
        data_paths = [p for p in [const_path, rampdown_path] if os.path.exists(p)]

    if not data_paths:
        print(f"ERROR: No data files found in {EXPERIMENT_DIR}")
        print("Run collect_calibration_data.py first.")
        return

    # Load and merge trials from all data files
    all_trials = {}
    all_force_levels = set()
    merged_metadata = None
    trial_offset = 0

    for data_path in data_paths:
        print(f"Loading data from: {data_path}")
        trials, metadata = load_master_data(data_path)

        if merged_metadata is None:
            merged_metadata = metadata
        else:
            all_force_levels.update(merged_metadata["force_levels_mN"])

        all_force_levels.update(metadata["force_levels_mN"])

        for tnum in sorted(trials.keys()):
            renumbered = trial_offset + tnum
            trial = trials[tnum]
            trial["trial_num"] = renumbered
            all_trials[renumbered] = trial

        trial_offset += len(trials)
        print(f"  {len(trials)} trials loaded")

    force_levels = sorted(all_force_levels)
    merged_metadata["force_levels_mN"] = force_levels
    print(f"\nTotal: {len(all_trials)} trials across force levels {force_levels}")

    print(f"\nVelocity estimator: {VELOCITY_ESTIMATOR}")
    if VELOCITY_ESTIMATOR == "SG":
        print(f"  Savitzky-Golay: window={SAVGOL_WINDOW}, order={SAVGOL_ORDER}")
    else:
        print(f"  Rolling window: window={RW_WINDOW}")
    print(f"Shaft mass: {M_SHAFT_KG} kg")
    print(f"Stream force shift: {STREAM_FORCE_SHIFT} frames (USB chip: {USB_CHIP})")

    # Compute friction for each trial (in trial order)
    friction_trials = []
    for tnum in sorted(all_trials.keys()):
        result = compute_friction_for_trial(
            all_trials[tnum],
            velocity_estimator=VELOCITY_ESTIMATOR,
            savgol_window=SAVGOL_WINDOW,
            savgol_order=SAVGOL_ORDER,
            rw_window=RW_WINDOW,
            shift=STREAM_FORCE_SHIFT,
        )
        friction_trials.append(result)

    # --- Plot 1: Velocity over time ---
    print("\nGenerating velocity over time plot...")
    fig_vel = plot_velocity_over_time(friction_trials, force_levels)
    vel_path = os.path.join(EXPERIMENT_DIR, "velocity_over_time.png")
    fig_vel.savefig(vel_path, dpi=150)
    print(f"  Saved: {vel_path}")

    # --- Plot 2: Sensed force over time ---
    print("\nGenerating sensed force over time plot...")
    fig_fsense = plot_Fsensed_over_time(friction_trials, force_levels)
    fsense_path = os.path.join(EXPERIMENT_DIR, "Fsensed_over_time.png")
    fig_fsense.savefig(fsense_path, dpi=150)
    print(f"  Saved: {fsense_path}")

    # --- Plot 3: Measured acceleration over time ---
    print("\nGenerating acceleration over time plot...")
    fig_accel = plot_accel_over_time(friction_trials, force_levels)
    accel_path = os.path.join(EXPERIMENT_DIR, "accel_over_time.png")
    fig_accel.savefig(accel_path, dpi=150)
    print(f"  Saved: {accel_path}")

    # --- Plot 4: Stribeck curve ---
    print("Generating Stribeck curve (velocity vs coefficient)...")
    fig_stribeck = plot_stribeck_curve(friction_trials, force_levels)
    stribeck_path = os.path.join(EXPERIMENT_DIR, "stribeck_curve.png")
    fig_stribeck.savefig(stribeck_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {stribeck_path}")

    # --- Plot 5: 3D Stribeck (position, velocity, μ_d) ---
    print("Generating 3D Stribeck plot (position, velocity, μ_d)...")
    print("  NOTE: Position and velocity values are NOT shifted by the "
          f"stream_force_shift={STREAM_FORCE_SHIFT} parameter. "
          "Only force_sensed is shifted to align with acceleration.")
    fig_3d = plot_stribeck_3d(friction_trials, force_levels)
    stribeck_3d_path = os.path.join(EXPERIMENT_DIR, "stribeck_3d.png")
    fig_3d.savefig(stribeck_3d_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {stribeck_3d_path}")

    # --- Plot 6: Faux Stribeck with position vs mu_d ---
    print("Generating Faux Stribeck plot (position, mu_d)")
    fig_faux = plot_position_vs_mud_curve(friction_trials, force_levels)
    faux_path = os.path.join(EXPERIMENT_DIR, "faux_stribeck.png")
    fig_faux.savefig(faux_path, dpi=150, bbox_inches="tight")
    print(f"  Saved: {faux_path}")

    # --- Save augmented data ---
    print("\nSaving augmented data with friction estimates...")
    save_augmented_data(EXPERIMENT_DIR, friction_trials, merged_metadata)

    # --- Summary statistics ---
    print(f"\n{'='*60}")
    print("FRICTION SUMMARY")
    print(f"{'='*60}")
    for force_mN in force_levels:
        recs = [r for r in friction_trials if r["force_label_mN"] == force_mN]
        if not recs:
            continue
        all_mu = np.concatenate([r["mu_d"][r["t_stream_s"] >= T_IGNORE_S] for r in recs])
        all_fric = np.concatenate([r["f_friction_mN"][r["t_stream_s"] >= T_IGNORE_S] for r in recs])
        if len(all_mu) == 0:
            continue
        print(f"  F_cmd={force_mN:6d} mN | "
              f"F_friction={np.mean(all_fric):7.1f} ± {np.std(all_fric):6.1f} mN | "
              f"μ_d={np.mean(all_mu):.4f} ± {np.std(all_mu):.4f}")

    print(f"\n  Static friction reference: {F_STATIC_LOW_MN}–{F_STATIC_HIGH_MN} mN "
          f"(μ_s ≈ {F_STATIC_LOW_MN/WEIGHT_MN:.4f}–{F_STATIC_HIGH_MN/WEIGHT_MN:.4f})")

    # plt.show()


if __name__ == "__main__":
    main()
