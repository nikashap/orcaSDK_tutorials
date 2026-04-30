#!/usr/bin/env python
"""
compare_simulation_to_shaft.py

Compare the cart-pendulum simulation's reference kinematics against what the
ORCA shaft actually did when commanded with the simulation's force trajectory
(produced by collect_simulation_data.py).

Loads:
  <data_dir>/<date>/calibration_simulation_friction.npz   (shaft data)
  <data_dir>/<date>/simulation_trajectories.npz           (reference trajectories)
  <data_dir>/<date>/calibration_params.yaml               (parameter snapshot)

For each trial, matches the shaft trial to its reference trajectory by
(theta_init, start_position_um) and produces:
  - Per-trial overlay figure: cart acceleration (sim) vs shaft acceleration,
    with position and commanded force as auxiliary subplots.
  - Summary figure: acceleration RMSE per trial across all trials.

Usage:
  python compare_simulation_to_shaft.py --date 26_04_30-12_00_00
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import yaml


# ---------------------------------------------------------------------------
# CLI + parameter loading (mirrors analyze_friction.py conventions)
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _parse_args():
    p = argparse.ArgumentParser(
        description="Compare simulation reference vs ORCA shaft kinematics."
    )
    p.add_argument(
        "--date", type=str, required=True,
        help="Date folder name of the experiment (e.g. '26_04_30-12_00_00'). "
             "Loaded from <data_dir>/<date>/."
    )
    p.add_argument(
        "--data-dir", type=str, default=None,
        help="Override base data dir. Default: ../data relative to this script."
    )
    p.add_argument(
        "--out-subdir", type=str, default="sim_vs_shaft",
        help="Subdirectory inside the experiment dir to save plots into."
    )
    p.add_argument(
        "--theta-tol-deg", type=float, default=0.5,
        help="Tolerance (degrees) when matching shaft trial to reference "
             "trajectory by theta_init."
    )
    p.add_argument(
        "--no-individual-plots", action="store_true",
        help="Skip per-trial plots and only produce the summary."
    )
    return p.parse_args()


def _load_params(experiment_dir):
    params_path = os.path.join(experiment_dir, "calibration_params.yaml")
    if not os.path.exists(params_path):
        print(f"ERROR: No calibration_params.yaml found in {experiment_dir}")
        sys.exit(1)
    with open(params_path, "r") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_shaft_data(filepath):
    """Load calibration_simulation_friction.npz produced by
    collect_simulation_data.py.

    Returns (list of per-trial dicts, metadata dict).
    """
    d = np.load(filepath, allow_pickle=True)
    n_trials = int(d["n_trials"])
    boundaries = d["trial_boundaries"]

    trials = []
    for i in range(n_trials):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        if hi - lo < 2:
            print(f"  Skipping shaft trial {i + 1}: only {hi - lo} samples")
            continue
        t_stream = d["t_stream"][lo:hi].astype(np.float64)
        t0 = t_stream[0]
        trials.append({
            "trial_num": i + 1,
            "theta_init": float(d["trial_theta_init"][i]),
            "start_position_um": int(d["trial_start_position_um"][i]),
            "n_samples": int(d["trial_n_samples"][i]),
            "t_stream_s": t_stream - t0,
            "t_accel_s": d["t_accel"][lo:hi].astype(np.float64) - t0,
            "position_um": d["position_um"][lo:hi].astype(np.float64),
            "force_mN_sensed": d["force_mN"][lo:hi].astype(np.float64),
            "accel_mmpss": d["accel_mmpss"][lo:hi].astype(np.float64),
            "force_commanded_mN": d["force_commanded_mN"][lo:hi].astype(np.float64),
        })

    metadata = {
        "mass_shaft_kg": float(d["mass_shaft_kg"]),
        "motor_min_um": int(d["motor_min_um"]),
        "motor_max_um": int(d["motor_max_um"]),
        "motor_center_um": int(d["motor_center_um"]),
    }
    return trials, metadata


def load_reference_trajectories(filepath):
    """Load simulation_trajectories.npz.

    Returns (list of per-trajectory dicts, metadata dict). Time axis for
    each trajectory is k * sim_timestep_s starting at 0.
    """
    d = np.load(filepath)
    n_traj = int(d["n_trajectories"])
    boundaries = d["traj_boundaries"]
    sim_dt_s = float(d["sim_timestep_s"])

    trajs = []
    for i in range(n_traj):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        n = hi - lo
        t_ref = np.arange(n, dtype=np.float64) * sim_dt_s
        trajs.append({
            "traj_num": i + 1,
            "theta_init": float(d["traj_theta_init"][i]),
            "start_position_um": int(d["traj_start_position_um"][i]),
            "n_steps": n,
            "t_ref_s": t_ref,
            "force_command_mN": d["force_commands_mN"][lo:hi].astype(np.float64),
            "sim_cart_x_m": d["sim_cart_x"][lo:hi].astype(np.float64),
            "sim_cart_xdot_mps": d["sim_cart_xdot"][lo:hi].astype(np.float64),
            "sim_cart_xddot_mps2": d["sim_cart_xddot"][lo:hi].astype(np.float64),
            "sim_pend_theta_rad": d["sim_pend_theta"][lo:hi].astype(np.float64),
            "sim_pend_thetadot_radps": d["sim_pend_thetadot"][lo:hi].astype(np.float64),
        })

    metadata = {
        "sim_timestep_s": sim_dt_s,
        "mass_shaft_kg": float(d["mass_shaft_kg"]),
        "motor_center_um": int(d["motor_center_um"]),
        "sim_mass_cart": float(d["sim_mass_cart"]),
        "sim_mass_bob": float(d["sim_mass_bob"]),
        "sim_pendulum_length": float(d["sim_pendulum_length"]),
    }
    return trajs, metadata


# ---------------------------------------------------------------------------
# Trial matching
# ---------------------------------------------------------------------------

def match_trials_to_references(shaft_trials, reference_trajs, theta_tol_rad):
    """Pair each shaft trial with the reference trajectory of matching
    (theta_init, start_position_um). A reference trajectory may match
    multiple shaft trials (if iterations_per_combo > 1).

    Returns list of (shaft_trial, ref_traj) tuples.
    """
    pairs = []
    for st in shaft_trials:
        match = None
        for rt in reference_trajs:
            if rt["start_position_um"] != st["start_position_um"]:
                continue
            if abs(rt["theta_init"] - st["theta_init"]) > theta_tol_rad:
                continue
            match = rt
            break
        if match is None:
            print(f"  WARNING: shaft trial {st['trial_num']} "
                  f"(theta={np.degrees(st['theta_init']):+.1f}°, "
                  f"start={st['start_position_um']} µm) "
                  f"has no matching reference trajectory; skipping.")
            continue
        pairs.append((st, match))
    return pairs


# ---------------------------------------------------------------------------
# Comparison metrics
# ---------------------------------------------------------------------------

def _interp_onto_grid(t_src, y_src, t_target):
    """Linear-interpolate (t_src, y_src) onto t_target. NaN outside support."""
    fn = interp1d(t_src, y_src, kind="linear",
                  bounds_error=False, fill_value=np.nan)
    return fn(t_target)


def compute_acceleration_comparison(shaft_trial, ref_traj):
    """Return (t_common_s, a_ref_mmpss, a_shaft_mmpss_on_ref_grid).

    Reference acceleration is converted m/s² → mm/s² to match shaft sensor
    units. The reference grid is trimmed to the shaft's recorded interval
    (the shaft may stop early due to safety limits).
    """
    a_ref_mmpss_full = ref_traj["sim_cart_xddot_mps2"] * 1e3
    t_ref_full = ref_traj["t_ref_s"]

    t_accel = shaft_trial["t_accel_s"]
    a_shaft_raw = shaft_trial["accel_mmpss"]

    t_max = min(float(t_accel[-1]), float(t_ref_full[-1]))
    mask = t_ref_full <= t_max
    t_common = t_ref_full[mask]
    a_ref = a_ref_mmpss_full[mask]
    a_shaft_on_ref = _interp_onto_grid(t_accel, a_shaft_raw, t_common)
    return t_common, a_ref, a_shaft_on_ref


def compute_position_comparison(shaft_trial, ref_traj, motor_center_um):
    """Return (t_common_s, x_ref_um, x_shaft_um_on_ref_grid). Reference
    sim position (m, cart-frame) is converted to motor-frame µm.
    """
    x_ref_um_full = ref_traj["sim_cart_x_m"] * 1e6 + motor_center_um
    t_ref_full = ref_traj["t_ref_s"]

    t_stream = shaft_trial["t_stream_s"]
    pos_shaft = shaft_trial["position_um"]

    t_max = min(float(t_stream[-1]), float(t_ref_full[-1]))
    mask = t_ref_full <= t_max
    t_common = t_ref_full[mask]
    x_ref = x_ref_um_full[mask]
    x_shaft_on_ref = _interp_onto_grid(t_stream, pos_shaft, t_common)
    return t_common, x_ref, x_shaft_on_ref


def acceleration_rmse(a_ref, a_shaft):
    """RMSE in mm/s² over points where both are finite."""
    valid = np.isfinite(a_ref) & np.isfinite(a_shaft)
    if not np.any(valid):
        return float("nan")
    return float(np.sqrt(np.mean((a_shaft[valid] - a_ref[valid]) ** 2)))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_trial_comparison(shaft_trial, ref_traj, motor_center_um):
    """Three stacked panels: acceleration, position, force command."""
    t_acc, a_ref, a_shaft_on_ref = compute_acceleration_comparison(
        shaft_trial, ref_traj
    )
    t_pos, x_ref, x_shaft_on_ref = compute_position_comparison(
        shaft_trial, ref_traj, motor_center_um
    )
    rmse = acceleration_rmse(a_ref, a_shaft_on_ref)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    # --- Panel 1: Acceleration overlay ---
    ax = axes[0]
    ax.plot(t_acc, a_ref, color="C0", linewidth=1.4,
            label="Cart accel (simulation reference)")
    ax.plot(shaft_trial["t_accel_s"], shaft_trial["accel_mmpss"],
            color="C1", linewidth=0.8, alpha=0.55,
            label="Shaft accel (raw, native timestamps)")
    ax.plot(t_acc, a_shaft_on_ref, color="C3", linewidth=1.0, linestyle="--",
            label="Shaft accel (interp to sim grid)")
    ax.set_ylabel("Acceleration (mm/s²)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_title(
        f"Trial {shaft_trial['trial_num']} | "
        f"theta_init = {np.degrees(shaft_trial['theta_init']):+.1f}° | "
        f"start = {shaft_trial['start_position_um']} µm | "
        f"accel RMSE = {rmse:.0f} mm/s²"
    )

    # --- Panel 2: Position overlay ---
    ax = axes[1]
    ax.plot(t_pos, x_ref, color="C0", linewidth=1.4,
            label="Cart position (simulation reference)")
    ax.plot(shaft_trial["t_stream_s"], shaft_trial["position_um"],
            color="C1", linewidth=0.9, alpha=0.7,
            label="Shaft position (raw)")
    ax.plot(t_pos, x_shaft_on_ref, color="C3", linewidth=1.0, linestyle="--",
            label="Shaft position (interp to sim grid)")
    ax.set_ylabel("Position (µm)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)

    # --- Panel 3: Force command sanity check ---
    ax = axes[2]
    ax.plot(ref_traj["t_ref_s"], ref_traj["force_command_mN"],
            color="C0", linewidth=1.0,
            label="F_command from sim (reference file)")
    ax.plot(shaft_trial["t_stream_s"], shaft_trial["force_commanded_mN"],
            color="C2", linewidth=0.9, linestyle=":",
            label="F_command actually sent to motor")
    ax.set_xlabel("Time since trial start (s)")
    ax.set_ylabel("Force command (mN)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig, rmse


def plot_summary(pairs, accel_rmse_list):
    """Bar chart of acceleration RMSE per trial."""
    fig, ax = plt.subplots(figsize=(max(8, 0.4 * len(pairs)), 5))
    labels = [
        f"#{st['trial_num']}\n{np.degrees(st['theta_init']):+.0f}°\n"
        f"{st['start_position_um'] // 1000}k"
        for st, _ in pairs
    ]
    xs = np.arange(len(pairs))
    ax.bar(xs, accel_rmse_list, color="C3", alpha=0.85)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, fontsize=7)
    ax.set_ylabel("Acceleration RMSE (mm/s²)")
    ax.set_xlabel("Trial (theta_init, start_position)")
    ax.set_title("Per-trial acceleration RMSE: shaft vs simulation reference")
    ax.grid(True, alpha=0.3, axis="y")
    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = _parse_args()

    base_data_dir = (
        args.data_dir if args.data_dir
        else os.path.normpath(os.path.join(_SCRIPT_DIR, "../data"))
    )
    experiment_dir = os.path.join(base_data_dir, args.date)
    if not os.path.isdir(experiment_dir):
        print(f"ERROR: experiment dir not found: {experiment_dir}")
        sys.exit(1)

    params = _load_params(experiment_dir)
    print(f"Loaded params from {experiment_dir}/calibration_params.yaml")
    print(f"  mass_shaft_kg = {params.get('mass_shaft_kg')}")

    shaft_path = os.path.join(experiment_dir, "calibration_simulation_friction.npz")
    ref_path = os.path.join(experiment_dir, "simulation_trajectories.npz")
    for p in (shaft_path, ref_path):
        if not os.path.exists(p):
            print(f"ERROR: required file not found: {p}")
            sys.exit(1)

    print(f"\nLoading shaft data: {shaft_path}")
    shaft_trials, shaft_meta = load_shaft_data(shaft_path)
    print(f"  {len(shaft_trials)} shaft trials")

    print(f"Loading reference trajectories: {ref_path}")
    ref_trajs, ref_meta = load_reference_trajectories(ref_path)
    print(f"  {len(ref_trajs)} reference trajectories "
          f"(sim_timestep = {ref_meta['sim_timestep_s'] * 1e3:.2f} ms)")

    # Sanity check: motor_center should agree between the two files
    if shaft_meta["motor_center_um"] != ref_meta["motor_center_um"]:
        print(f"  WARNING: motor_center_um disagrees: "
              f"shaft={shaft_meta['motor_center_um']}, "
              f"ref={ref_meta['motor_center_um']}. "
              f"Using shaft's value for position-frame conversion.")
    motor_center_um = shaft_meta["motor_center_um"]

    # --- Match trials to references ---
    theta_tol_rad = np.radians(args.theta_tol_deg)
    pairs = match_trials_to_references(shaft_trials, ref_trajs, theta_tol_rad)
    print(f"\nMatched {len(pairs)}/{len(shaft_trials)} shaft trials to references.")

    if not pairs:
        print("Nothing to plot.")
        return

    # --- Output directory ---
    out_dir = os.path.join(experiment_dir, args.out_subdir)
    os.makedirs(out_dir, exist_ok=True)

    # --- Per-trial plots + RMSE accumulation ---
    print(f"\nGenerating per-trial comparison plots in {out_dir}/...")
    accel_rmse_list = []
    for st, rt in pairs:
        fig, rmse = plot_trial_comparison(st, rt, motor_center_um)
        accel_rmse_list.append(rmse)
        if not args.no_individual_plots:
            fname = (f"trial_{st['trial_num']:03d}_"
                     f"theta_{int(np.degrees(st['theta_init'])):+04d}_"
                     f"start_{st['start_position_um']}.png")
            fpath = os.path.join(out_dir, fname)
            fig.savefig(fpath, dpi=130)
            print(f"  Saved: {fname}  (RMSE = {rmse:.0f} mm/s²)")
        plt.close(fig)

    # --- Summary plot ---
    print("\nGenerating summary plot...")
    fig_sum = plot_summary(pairs, accel_rmse_list)
    sum_path = os.path.join(out_dir, "summary_accel_rmse.png")
    fig_sum.savefig(sum_path, dpi=150)
    plt.close(fig_sum)
    print(f"  Saved: {sum_path}")

    # --- Console summary ---
    finite = [r for r in accel_rmse_list if np.isfinite(r)]
    if finite:
        print(f"\n{'=' * 60}")
        print("ACCELERATION RMSE SUMMARY")
        print(f"{'=' * 60}")
        print(f"  N trials       : {len(finite)}")
        print(f"  Mean RMSE      : {np.mean(finite):.0f} mm/s²")
        print(f"  Median RMSE    : {np.median(finite):.0f} mm/s²")
        print(f"  Min / Max RMSE : {np.min(finite):.0f} / {np.max(finite):.0f} mm/s²")

    print("\nDone.")


if __name__ == "__main__":
    main()