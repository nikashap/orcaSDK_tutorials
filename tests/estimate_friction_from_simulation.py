#!/usr/bin/env python
"""
estimate_friction_from_simulation.py

For each trial in a collect_simulation_data.py run, compute the empirical
friction estimate (the residual force-tracking error) at every step:

    F_net_realized[i]          = mass_shaft_kg * accel_mmpss[i]    # mN (kg * mm/s² = mN)
    force_friction_estimate[i] = force_commanded_mN[i] - F_net_realized[i]

The control loop sets the streamed force and calls motor.run() before
reading acceleration in the same iteration, so command and accel at the
same index correspond to the same timestep — no temporal shift is applied.

Iterative learning control
--------------------------
This script is meant to be run repeatedly as part of an ILC loop:

    iter 0:  collect_simulation_data option 2 (raw sim force)
             -> calibration_simulation_friction.npz
             -> estimate_friction_from_simulation.py (iteration auto-detected
                as 0)
             -> calibration_simulation_friction_with_estimate.npz

    iter 1:  collect_simulation_data option 4 (replays with the iter-0
             augmented file's compensation)
             -> calibration_simulation_friction_compensated.npz
             -> estimate_friction_from_simulation.py (iteration auto-detected
                as 1)
             -> calibration_simulation_friction_compensated_with_estimate.npz

    iter N:  collect_simulation_data option 4 (replays with iter-(N-1)
             augmented file)
             -> calibration_simulation_friction_compensated_iter{N}.npz
             -> estimate_friction_from_simulation.py
             -> calibration_simulation_friction_compensated_iter{N}_with_estimate.npz

The iteration number is auto-detected from the input filename, or can be
specified with --iteration. The augmented npz contains every original key
plus `force_friction_estimate` and `compensation_iteration`.

Outputs
-------
1. <data_dir>/<date>/friction_estimate[_iter{N}]/trial_*.png — per-trial plot:
   panel 1 overlays F_commanded with F_net_realized; panel 2 plots
   force_friction_estimate with the shaft velocity overlaid on a twin axis.
2. <data_dir>/<date>/<augmented_filename> — copy of the original shaft .npz
   with two new keys (force_friction_estimate, compensation_iteration).

Usage
-----
    # Iter 0 (raw run):
    python estimate_friction_from_simulation.py --date 26_04_30-12_00_00

    # Iter 1+ (auto-detected from the shaft filename):
    python estimate_friction_from_simulation.py --date 26_05_01-09_00_00 \\
        --iteration 1

    # Or override the input filename directly:
    python estimate_friction_from_simulation.py --date 26_05_01-09_00_00 \\
        --shaft-file calibration_simulation_friction_compensated.npz
"""

import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import yaml


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Iteration / filename conventions
# ---------------------------------------------------------------------------
#
# Iterative learning control over the simulation replay produces a chain of
# files. Each iteration consumes the previous iteration's shaft data, computes
# the residual tracking error as `force_friction_estimate`, and saves an
# augmented npz that the replay script (collect_simulation_data.py option 4)
# can load to produce the *next* iteration's command.
#
# Iter 0:  raw simulation force commands, no compensation.
#          shaft file:     calibration_simulation_friction.npz
#          augmented file: calibration_simulation_friction_with_estimate.npz
#
# Iter 1:  one round of compensation applied.
#          shaft file:     calibration_simulation_friction_compensated.npz
#          augmented file: calibration_simulation_friction_compensated_with_estimate.npz
#
# Iter N>=2: N rounds of compensation applied.
#          shaft file:     calibration_simulation_friction_compensated_iter{N}.npz
#          augmented file: calibration_simulation_friction_compensated_iter{N}_with_estimate.npz

import re

_SHAFT_RE_ITER = re.compile(
    r"^calibration_simulation_friction_compensated_iter(\d+)\.npz$"
)


def shaft_filename_for_iteration(iteration):
    """Filename of the shaft data npz for the given iteration."""
    if iteration == 0:
        return "calibration_simulation_friction.npz"
    if iteration == 1:
        return "calibration_simulation_friction_compensated.npz"
    return f"calibration_simulation_friction_compensated_iter{iteration}.npz"


def augmented_filename_for_iteration(iteration):
    """Filename of the augmented (with-estimate) npz for the given iteration."""
    if iteration == 0:
        return "calibration_simulation_friction_with_estimate.npz"
    if iteration == 1:
        return "calibration_simulation_friction_compensated_with_estimate.npz"
    return (
        f"calibration_simulation_friction_compensated_iter{iteration}"
        f"_with_estimate.npz"
    )


def detect_iteration_from_shaft_filename(filename):
    """Infer the iteration number from a shaft-data filename.

    Returns the iteration as an int, or None if the filename doesn't match
    a known pattern.
    """
    base = os.path.basename(filename)
    if base == "calibration_simulation_friction.npz":
        return 0
    if base == "calibration_simulation_friction_compensated.npz":
        return 1
    m = _SHAFT_RE_ITER.match(base)
    if m:
        return int(m.group(1))
    return None


def prior_augmented_filename_for_iteration(iteration):
    """Filename of the augmented npz from iteration-1, or None if iter == 0.

    The previous iteration's augmented file is needed to compute the
    per-iteration compensation contribution at the current iteration.
    """
    if iteration <= 0:
        return None
    return augmented_filename_for_iteration(iteration - 1)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args():
    p = argparse.ArgumentParser(
        description=(
            "Compute empirical friction estimates from a "
            "collect_simulation_data.py run."
        )
    )
    p.add_argument(
        "--date", type=str, required=True,
        help="Date folder name of the experiment (e.g. '26_04_30-12_00_00')."
    )
    p.add_argument(
        "--data-dir", type=str, default=None,
        help="Override base data dir. Default: ../data relative to this script."
    )
    p.add_argument(
        "--iteration", type=int, default=None,
        help="Iteration number being processed. If omitted, the script tries "
             "to auto-detect from the shaft filename. Iter 0 = raw "
             "(calibration_simulation_friction.npz); iter 1 = first "
             "compensation; iter N = Nth compensation. Use --iteration to "
             "override auto-detection or to disambiguate when --shaft-file "
             "is given."
    )
    p.add_argument(
        "--shaft-file", type=str, default=None,
        help="Override shaft input filename. By default the script picks "
             "the file matching --iteration (or auto-detects iter 0 if no "
             "iteration is specified)."
    )
    p.add_argument(
        "--out-subdir", type=str, default=None,
        help="Subdirectory inside the experiment dir to save plots into. "
             "Default: friction_estimate_iter{N} (or friction_estimate for "
             "iter 0)."
    )
    p.add_argument(
        "--no-plots", action="store_true",
        help="Skip per-trial plots; only save the augmented npz."
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_shaft_npz(filepath):
    """Load the npz as a dict of arrays plus per-trial index info.

    Returns (data_dict, trial_info), where data_dict has every key from the
    file (so we can re-save it cleanly), and trial_info is a list of dicts
    with the per-trial slice indices and metadata.
    """
    d = np.load(filepath, allow_pickle=True)
    data_dict = {k: d[k] for k in d.files}

    n_trials = int(data_dict["n_trials"])
    boundaries = data_dict["trial_boundaries"]

    has_type = "trial_trajectory_type" in data_dict
    has_force = "trial_force_type" in data_dict
    has_traj_id = "trial_traj_id" in data_dict

    trial_info = []
    for i in range(n_trials):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        trial_info.append({
            "trial_num": i + 1,
            "lo": lo,
            "hi": hi,
            "theta_init": float(data_dict["trial_theta_init"][i]),
            "start_position_um": int(data_dict["trial_start_position_um"][i]),
            "n_samples": int(data_dict["trial_n_samples"][i]),
            "trajectory_type": str(data_dict["trial_trajectory_type"][i]) if has_type else "pendulum_drop",
            "force_type": str(data_dict["trial_force_type"][i]) if has_force else "ctrl",
            "traj_id": int(data_dict["trial_traj_id"][i]) if has_traj_id else i,
        })
    return data_dict, trial_info


# ---------------------------------------------------------------------------
# Friction estimation
# ---------------------------------------------------------------------------

def compute_force_command_simulation(sim_accel_mmpss, mass_shaft_kg):
    """Return the simulation's intended net force on the cart, in mN.

    Units: kg * mm/s² == mN exactly. This is the invariant target across
    all ILC iterations -- it's what the cart simulation says the net force
    on the shaft should be at every step.
    """
    return mass_shaft_kg * sim_accel_mmpss.astype(np.float64)


def compute_force_friction_estimate(sim_accel_mmpss, accel_mmpss,
                                    mass_shaft_kg):
    """Compute force_friction_estimate against the simulation target.

    The control loop sets the streamed force and then calls motor.run()
    before reading acceleration in the same iteration, so the accel and
    sim_accel readings at index i correspond to the same timestep. No
    temporal shift is applied.

        F_command_simulation[i] = mass_shaft_kg * sim_accel_mmpss[i]   # invariant target
        F_net_realized[i]       = mass_shaft_kg * accel_mmpss[i]       # what the shaft did
        force_friction[i]       = F_command_simulation[i] - F_net_realized[i]

    This is the *additional* compensation that the next ILC iteration
    will need to add (after gain scaling), in order to push the realized
    net force toward the simulation target. It is measured against the
    simulation -- NOT against whatever was actually commanded -- so that
    successive iterations can keep refining the cumulative compensation
    rather than locking it at iter-1's value.
    """
    f_sim = mass_shaft_kg * sim_accel_mmpss.astype(np.float64)
    f_realized = mass_shaft_kg * accel_mmpss.astype(np.float64)
    return f_sim - f_realized


def compute_compensation_contribution_this_iteration(
    current_data, prior_data, mass_shaft_kg
):
    """Compute the per-sample compensation increment that this iteration's
    actual command added on top of the previous iteration's command.

    For sample i in the current iteration:

        contribution[i] = current_force_commanded[i] - prior_force_commanded[i]

    where prior_force_commanded is matched trial-by-trial via
    (theta_init, start_position_um). Where the prior trial was shorter
    than the current trial (or no prior match exists), the prior command
    is treated as `force_command_simulation` so the invariant

        sum_over_iters(contribution[i]) == current_force_commanded[i]
                                            - force_command_simulation[i]

    holds for every sample.

    Returns an array the same length as current_data['force_commanded_mN'].
    """
    cur_cmd = current_data["force_commanded_mN"].astype(np.float64)
    cur_sim_accel = current_data["sim_accel_mmpss"].astype(np.float64)
    sim_command = mass_shaft_kg * cur_sim_accel  # what an "uncompensated" iter would have sent

    # Default fallback: if a sample has no prior match, contribution is
    # cur_cmd - sim_command (i.e. all of the deviation gets attributed
    # to this iteration). This keeps the cumulative-history invariant
    # exact for samples with no prior coverage.
    contribution = cur_cmd - sim_command

    if prior_data is None:
        return contribution

    # Index prior trials by (theta, start) for fast matching.
    prior_boundaries = prior_data["trial_boundaries"]
    prior_thetas = prior_data["trial_theta_init"]
    prior_starts = prior_data["trial_start_position_um"]
    prior_cmd = prior_data["force_commanded_mN"].astype(np.float64)
    n_prior_trials = int(prior_data["n_trials"])

    cur_boundaries = current_data["trial_boundaries"]
    cur_thetas = current_data["trial_theta_init"]
    cur_starts = current_data["trial_start_position_um"]
    n_cur_trials = int(current_data["n_trials"])

    theta_tol_rad = np.radians(0.5)

    for i in range(n_cur_trials):
        clo, chi = int(cur_boundaries[i]), int(cur_boundaries[i + 1])
        n_cur = chi - clo
        theta = float(cur_thetas[i])
        start = int(cur_starts[i])

        # Find a matching prior trial.
        match_j = None
        for j in range(n_prior_trials):
            if int(prior_starts[j]) != start:
                continue
            if abs(float(prior_thetas[j]) - theta) > theta_tol_rad:
                continue
            match_j = j
            break
        if match_j is None:
            continue  # leave fallback (cur_cmd - sim_command) in place

        plo, phi = int(prior_boundaries[match_j]), int(prior_boundaries[match_j + 1])
        n_prior = phi - plo
        n_overlap = min(n_cur, n_prior)

        # Where the prior covers the current sample, contribution is the
        # actual incremental change. Where it doesn't, we keep the
        # cur_cmd - sim_command fallback already populated above.
        contribution[clo:clo + n_overlap] = (
            cur_cmd[clo:clo + n_overlap] - prior_cmd[plo:plo + n_overlap]
        )

    return contribution


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_trial(data, trial, mass_shaft_kg, friction_estimate, iteration=0):
    """Two-panel plot for one trial.

    Panel 1 plots the commanded force at its own native timestamps
    (t_stream) and the realized net force at its own native timestamps
    (t_accel). Each curve is shown unshifted at the time it was
    sent / recorded, so the natural ~1-sample spacing between command
    and effect is visible as horizontal offset rather than as an
    artificial index shift.

    Panel 2 plots the friction estimate computed as
        force_friction_estimate[i] = force_commanded_mN[i-1]
                                     − mass_shaft_kg · accel_mmpss[i]
    at the t_accel[i] timestamp, with shaft velocity on a twin axis.
    """
    lo, hi = trial["lo"], trial["hi"]
    if hi - lo < 2:
        return None

    t_stream = data["t_stream"][lo:hi].astype(np.float64)
    t_accel = data["t_accel"][lo:hi].astype(np.float64)
    t0 = float(t_stream[0])
    t_stream = t_stream - t0
    t_accel = t_accel - t0

    accel = data["accel_mmpss"][lo:hi].astype(np.float64)
    f_cmd = data["force_commanded_mN"][lo:hi].astype(np.float64)
    f_net_realized = mass_shaft_kg * accel  # mN

    f_fric = friction_estimate[lo:hi]

    # Velocity (if present in the file) for the twin axis on panel 2.
    have_vel = "vel_mm_s" in data and "t_vel" in data
    if have_vel:
        v = data["vel_mm_s"][lo:hi].astype(np.float64)
        t_vel = data["t_vel"][lo:hi].astype(np.float64) - t0

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    # --- Panel 1: F_commanded vs F_net_realized at native timestamps ---
    # Both quantities are paired index-wise at sample i (same control-loop
    # iteration), so we plot both against t_accel[i] for visual alignment.
    # The small wall-clock gap between motor.run() and the accel read within
    # one iteration is small enough that F_commanded doesn't meaningfully
    # change across it.
    ax = axes[0]
    ax.plot(t_accel, f_cmd, color="C0", linewidth=1.0,
            label="F_commanded[i] (plotted at t_accel[i] for alignment)")
    ax.plot(t_accel, f_net_realized, color="C3", linewidth=1.0,
            linestyle="--",
            label=f"F_net_realized = m_shaft · accel "
                  f"(measured at t_accel, m={mass_shaft_kg} kg)")
    ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
    ax.set_ylabel("Force (mN)")
    ax.legend(fontsize=8, loc="best")
    ax.grid(True, alpha=0.3)
    traj_type = trial.get("trajectory_type", "pendulum_drop")
    force_type = trial.get("force_type", "ctrl")
    ax.set_title(
        f"Trial {trial['trial_num']} [{traj_type}/{force_type}] | "
        f"iter {iteration} | "
        f"theta_init = {np.degrees(trial['theta_init']):+.1f}° | "
        f"start = {trial['start_position_um']} µm"
    )

    # --- Panel 2: force_friction_estimate, with shaft velocity twinned ---
    ax = axes[1]
    ax.plot(t_accel, f_fric, color="C2", linewidth=1.0,
            label="force_friction_estimate "
                  "= F_commanded[i] − m_shaft · accel[i]")
    ax.axhline(0.0, color="black", linewidth=0.6, alpha=0.5)
    ax.set_ylabel("Friction estimate (mN)")
    ax.set_xlabel("Time since trial start (s)")
    ax.grid(True, alpha=0.3)

    if have_vel:
        ax_v = ax.twinx()
        ax_v.plot(t_vel, v, color="C1", linewidth=0.8, alpha=0.6,
                  label="Shaft velocity (mm/s)")
        ax_v.axhline(0.0, color="C1", linewidth=0.4, alpha=0.3)
        ax_v.set_ylabel("Velocity (mm/s)", color="C1")
        ax_v.tick_params(axis="y", labelcolor="C1")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax_v.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="best")
    else:
        ax.legend(fontsize=8, loc="best")

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

    # --- Resolve iteration and input filename ---
    # Precedence: explicit --shaft-file > explicit --iteration > auto-detect.
    if args.shaft_file is not None:
        shaft_filename = args.shaft_file
        if args.iteration is not None:
            iteration = args.iteration
        else:
            inferred = detect_iteration_from_shaft_filename(shaft_filename)
            if inferred is None:
                print(f"ERROR: could not auto-detect iteration from "
                      f"--shaft-file '{shaft_filename}'. Pass --iteration "
                      f"explicitly.")
                sys.exit(1)
            iteration = inferred
    elif args.iteration is not None:
        iteration = args.iteration
        shaft_filename = shaft_filename_for_iteration(iteration)
    else:
        # No flags: default to iter 0.
        iteration = 0
        shaft_filename = shaft_filename_for_iteration(iteration)

    if iteration < 0:
        print(f"ERROR: iteration must be >= 0 (got {iteration}).")
        sys.exit(1)

    shaft_path = os.path.join(experiment_dir, shaft_filename)
    if not os.path.exists(shaft_path):
        print(f"ERROR: shaft data file not found: {shaft_path}")
        sys.exit(1)

    print(f"Processing iteration {iteration}")
    print(f"  Shaft file:  {shaft_filename}")

    data, trial_info = load_shaft_npz(shaft_path)
    mass_shaft_kg = float(data["mass_shaft_kg"])
    print(f"  mass_shaft_kg = {mass_shaft_kg}")
    print(f"  {len(trial_info)} trials")

    # --- Compute the friction estimate (residual tracking error) ---
    # sim_accel_mmpss is the reference: the acceleration the shaft should
    # achieve if the commanded force were applied with no friction.
    # For ctrl trials this equals cart x_ddot*1e3; for env trials it equals
    # sim_env_N*1e3 / mass_shaft_kg.
    friction_estimate = compute_force_friction_estimate(
        data["sim_accel_mmpss"],
        data["accel_mmpss"],
        mass_shaft_kg,
    )
    n_finite = int(np.sum(np.isfinite(friction_estimate)))
    n_total = len(friction_estimate)
    print(f"  Computed force_friction_estimate (residual at iter "
          f"{iteration}): {n_finite}/{n_total} finite samples")

    if n_finite > 0:
        finite_vals = friction_estimate[np.isfinite(friction_estimate)]
        print(f"    mean   : {finite_vals.mean():+8.0f} mN")
        print(f"    median : {np.median(finite_vals):+8.0f} mN")
        print(f"    std    : {finite_vals.std():8.0f} mN")
        print(f"    min/max: {finite_vals.min():+8.0f} / "
              f"{finite_vals.max():+8.0f} mN")

    # --- Save augmented npz ---
    augmented_filename = augmented_filename_for_iteration(iteration)
    augmented_path = os.path.join(experiment_dir, augmented_filename)
    out_dict = dict(data)  # copy
    out_dict["force_friction_estimate"] = friction_estimate
    # Also stamp the iteration into the file so downstream tools can verify.
    out_dict["compensation_iteration"] = np.int32(iteration)
    np.savez(augmented_path, **out_dict)
    print(f"\nSaved augmented npz: {augmented_filename}")

    # --- Per-trial plots ---
    if args.no_plots:
        print("\n--no-plots set; skipping per-trial plots.")
        return

    # Default subdir tracks the iteration so successive runs don't overwrite.
    if args.out_subdir is not None:
        subdir = args.out_subdir
    elif iteration == 0:
        subdir = "friction_estimate"
    else:
        subdir = f"friction_estimate_iter{iteration}"
    out_dir = os.path.join(experiment_dir, subdir)
    os.makedirs(out_dir, exist_ok=True)
    print(f"\nGenerating per-trial friction plots in {out_dir}/...")

    for trial in trial_info:
        fig = plot_trial(data, trial, mass_shaft_kg, friction_estimate,
                         iteration=iteration)
        if fig is None:
            print(f"  Skipping trial {trial['trial_num']}: too few samples.")
            continue
        force_type = trial.get("force_type", "ctrl")
        fname = (f"trial_{trial['trial_num']:03d}_{force_type}_"
                 f"theta_{int(np.degrees(trial['theta_init'])):+04d}_"
                 f"start_{trial['start_position_um']}.png")
        fig.savefig(os.path.join(out_dir, fname), dpi=130)
        plt.close(fig)
        print(f"  Saved: {fname}")

    print("\nDone.")


if __name__ == "__main__":
    main()