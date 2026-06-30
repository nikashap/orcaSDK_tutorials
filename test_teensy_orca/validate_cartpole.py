#!/usr/bin/env python3
"""validate_cartpole.py — Cart-pole physics validation: Teensy vs. scipy reference.

For each test case, sends SIM_CONFIG / SIM_INIT / SIM_RUN to the Teensy over
UDP, collects the streamed trajectory, and compares against a high-accuracy
solve_ivp reference.  Saves plots to results/ and exits nonzero on failure.
"""

import argparse
import math
import os
import socket
import sys
import time
from dataclasses import replace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from cartpole_reference import (make_fx_func, solve_reference,
                                compute_energy, compute_momentum)
from test_cases import (CASES, get_case, TestCase,
                        SWEEP_X0_VALUES, SWEEP_RESIDUAL_TOL,
                        CONVERGENCE_DTS, CONVERGENCE_SLOPE_RANGE,
                        ENERGY_MAX_DRIFT, ENERGY_MAX_MOMENTUM,
                        DAMPING_B_VALUES, DAMPING_TOL_ANGLE, DAMPING_TOL_ANGVEL)

DEFAULT_IP = "192.168.1.177"
DEFAULT_PORT = 8888
RECV_TIMEOUT_S = 0.2


# ── UDP helpers ──────────────────────────────────────────────────────

def _send(sock, addr, msg):
    sock.sendto((msg + "\n").encode(), addr)


def _wait_ack(sock, cmd_name, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        for line in data.decode(errors="replace").strip().split("\n"):
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "ACK" and len(parts) >= 2 and parts[1] == cmd_name:
                return True
            if parts[0] == "ERROR":
                raise RuntimeError(f"Teensy: {line}")
    return False


def _make_sock():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(RECV_TIMEOUT_S)
    sock.bind(("0.0.0.0", 0))
    return sock


# ── Run simulation on Teensy ────────────────────────────────────────

def run_on_teensy(case, teensy_ip, teensy_port):
    """Returns (times, states, fx_vals, total_steps, wall_us)."""
    sock = _make_sock()
    addr = (teensy_ip, teensy_port)
    try:
        _send(sock, addr,
              f"SIM_CONFIG {case.m_c} {case.m_p} {case.l} {case.g} {case.b}")
        if not _wait_ack(sock, "SIM_CONFIG"):
            raise RuntimeError("SIM_CONFIG not acknowledged")

        _send(sock, addr,
              f"SIM_INIT {case.x0} {case.theta0} {case.xdot0} {case.thetadot0}")
        if not _wait_ack(sock, "SIM_INIT"):
            raise RuntimeError("SIM_INIT not acknowledged")

        _send(sock, addr,
              f"SIM_RUN {case.duration} {case.dt} {case.fx_mode} "
              f"{case.fx_param1} {case.fx_param2} {case.sample_stride}")

        times, states, fx_vals = [], [], []
        max_wait = max(60.0, case.duration * 10)
        deadline = time.time() + max_wait

        while time.time() < deadline:
            try:
                data, _ = sock.recvfrom(65536)
            except socket.timeout:
                continue
            for line in data.decode(errors="replace").strip().split("\n"):
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == "SIM_SAMPLE" and len(parts) >= 7:
                    times.append(float(parts[2]))
                    states.append([float(parts[3]), float(parts[4]),
                                   float(parts[5]), float(parts[6])])
                    fx_vals.append(float(parts[7]) if len(parts) >= 8 else 0.0)
                elif parts[0] == "SIM_DONE" and len(parts) >= 3:
                    return (np.array(times), np.array(states),
                            np.array(fx_vals), int(parts[1]), int(parts[2]))
                elif parts[0] == "ERROR":
                    raise RuntimeError(f"Teensy: {line}")

        raise RuntimeError(
            f"Timed out waiting for SIM_DONE ({len(times)} samples received)")
    finally:
        sock.close()


# ── Reference solution ───────────────────────────────────────────────

def compute_ref(case, times):
    """Solve the reference ODE and evaluate at the Teensy sample times."""
    fx_func = make_fx_func(case.fx_mode, case.fx_param1, case.fx_param2)
    sol = solve_reference(
        [case.x0, case.theta0, case.xdot0, case.thetadot0],
        case.duration, case.m_c, case.m_p, case.l, case.g, fx_func, case.b)
    return sol.sol(times).T  # (N, 4)


# ── Plotting ─────────────────────────────────────────────────────────

def _plot_overlays_and_errors(fig, axes, times, states, ref, err):
    """Fill a (3, 2) axes array with state overlays (rows 0-1) + errors (row 2)."""
    labels = ["x (m)", "θ (rad)", "ẋ (m/s)", "θ̇ (rad/s)"]
    for i in range(4):
        ax = axes[i // 2, i % 2]
        ax.plot(times, ref[:, i], "b-", lw=0.8, label="reference")
        ax.plot(times, states[:, i], "r--", lw=0.8, label="Teensy")
        ax.set_ylabel(labels[i])
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    for col, (j0, j1) in enumerate([(0, 1), (2, 3)]):
        ax = axes[2, col]
        ax.plot(times, np.clip(np.abs(err[:, j0]), 1e-20, None),
                lw=0.8, label=labels[j0])
        ax.plot(times, np.clip(np.abs(err[:, j1]), 1e-20, None),
                lw=0.8, label=labels[j1])
        ax.set_ylabel("|error|")
        ax.set_xlabel("Time (s)")
        ax.set_yscale("log")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)


def plot_standard(case, times, states, ref, err, path, passed):
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f"{case.name} — {'PASS' if passed else 'FAIL'}",
                 fontsize=14, fontweight="bold")
    _plot_overlays_and_errors(fig, axes, times, states, ref, err)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


def plot_energy(case, times, states, ref, err, dE_rel, dp, path, passed):
    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    fig.suptitle(f"{case.name} — {'PASS' if passed else 'FAIL'}",
                 fontsize=14, fontweight="bold")
    _plot_overlays_and_errors(fig, axes[:3], times, states, ref, err)
    axes[3, 0].plot(times, dE_rel, "k-", lw=0.8)
    axes[3, 0].set_ylabel("ΔE / E₀")
    axes[3, 0].set_xlabel("Time (s)")
    axes[3, 0].axhline(0, color="gray", lw=0.5)
    axes[3, 0].grid(True, alpha=0.3)
    axes[3, 1].plot(times, dp, "k-", lw=0.8)
    axes[3, 1].set_ylabel("Δp (N·s)")
    axes[3, 1].set_xlabel("Time (s)")
    axes[3, 1].axhline(0, color="gray", lw=0.5)
    axes[3, 1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    plt.close()


# ── Tolerance check ──────────────────────────────────────────────────

def check_tol(case, err):
    """Returns (passed, dominant_message)."""
    max_errs = np.max(np.abs(err), axis=0)
    names = ["x", "θ", "ẋ", "θ̇"]
    tols = [case.tol_pos, case.tol_angle, case.tol_vel, case.tol_angvel]
    passed = True
    dominant = None
    for name, tol, val in zip(names, tols, max_errs):
        if val > tol:
            passed = False
            if dominant is None:
                dominant = f"{name} err {val:.3e} > {tol:.3e}"
    if dominant is None:
        dominant = f"max: x={max_errs[0]:.3e} θ={max_errs[1]:.3e}"
    return passed, dominant


# ── Test runners ─────────────────────────────────────────────────────

def _print_errors(max_errs):
    print(f"  max err: x={max_errs[0]:.3e}  θ={max_errs[1]:.3e}  "
          f"ẋ={max_errs[2]:.3e}  θ̇={max_errs[3]:.3e}")


def run_standard(case, teensy_ip, teensy_port, results_dir):
    """Run a standard test case. Returns (passed, summary_line)."""
    print(f"\n--- {case.name} ---")
    times, states, fx, nsteps, wall_us = run_on_teensy(
        case, teensy_ip, teensy_port)
    rate = nsteps / (wall_us / 1e6) if wall_us > 0 else 0
    print(f"  {len(times)} samples, {nsteps} steps, "
          f"wall={wall_us / 1e6:.3f}s ({rate:.0f} steps/s)")

    ref = compute_ref(case, times)
    err = states - ref
    max_errs = np.max(np.abs(err), axis=0)
    _print_errors(max_errs)

    passed, dominant = check_tol(case, err)
    plot_standard(case, times, states, ref, err,
                  os.path.join(results_dir, f"{case.name}.png"), passed)

    tag = "PASS" if passed else "FAIL"
    summary = f"  {tag}  {case.name}: {dominant}"
    print(summary)
    return passed, summary


def run_energy(case, teensy_ip, teensy_port, results_dir):
    """Run energy_conservation with extra energy / momentum analysis."""
    print(f"\n--- {case.name} ---")
    times, states, fx, nsteps, wall_us = run_on_teensy(
        case, teensy_ip, teensy_port)
    rate = nsteps / (wall_us / 1e6) if wall_us > 0 else 0
    print(f"  {len(times)} samples, {nsteps} steps, "
          f"wall={wall_us / 1e6:.3f}s ({rate:.0f} steps/s)")

    ref = compute_ref(case, times)
    err = states - ref
    max_errs = np.max(np.abs(err), axis=0)
    _print_errors(max_errs)

    passed, dominant = check_tol(case, err)

    E = compute_energy(states, case.m_c, case.m_p, case.l, case.g)
    p = compute_momentum(states, case.m_c, case.m_p, case.l, case.g)
    E0 = E[0]
    dE_rel = (E - E0) / abs(E0) if abs(E0) > 1e-30 else E - E0
    dp = p - p[0]

    max_dE = float(np.max(np.abs(dE_rel)))
    max_dp = float(np.max(np.abs(dp)))
    print(f"  max |ΔE/E₀| = {max_dE:.3e}  (tol {ENERGY_MAX_DRIFT:.0e})")
    print(f"  max |Δp|    = {max_dp:.3e}  (tol {ENERGY_MAX_MOMENTUM:.0e})")

    if max_dE > ENERGY_MAX_DRIFT:
        passed = False
        dominant = f"ΔE/E₀ = {max_dE:.3e} > {ENERGY_MAX_DRIFT:.0e}"
    if max_dp > ENERGY_MAX_MOMENTUM:
        passed = False
        dominant = f"Δp = {max_dp:.3e} > {ENERGY_MAX_MOMENTUM:.0e}"

    plot_energy(case, times, states, ref, err, dE_rel, dp,
                os.path.join(results_dir, f"{case.name}.png"), passed)

    tag = "PASS" if passed else "FAIL"
    summary = f"  {tag}  {case.name}: {dominant}"
    print(summary)
    return passed, summary


def run_sweep(teensy_ip, teensy_port, results_dir):
    """Run x_offset_sweep: translational invariance check."""
    print(f"\n--- x_offset_sweep ---")
    base = get_case("small_oscillation")
    all_states = {}
    all_times = None

    for x0 in SWEEP_X0_VALUES:
        case = replace(base, name=f"sweep_x0={x0:.1f}", x0=x0)
        times, states, _, _, _ = run_on_teensy(case, teensy_ip, teensy_port)
        print(f"  x0={x0:.1f}: {len(times)} samples")
        all_states[x0] = states
        if all_times is None:
            all_times = times

    ref_x0 = SWEEP_X0_VALUES[0]
    ref_s = all_states[ref_x0]
    max_residual = 0.0
    for x0 in SWEEP_X0_VALUES[1:]:
        s = all_states[x0]
        dx = s[:, 0] - (ref_s[:, 0] + (x0 - ref_x0))
        r = max(np.max(np.abs(dx)),
                np.max(np.abs(s[:, 1] - ref_s[:, 1])),
                np.max(np.abs(s[:, 2] - ref_s[:, 2])),
                np.max(np.abs(s[:, 3] - ref_s[:, 3])))
        max_residual = max(max_residual, r)

    passed = max_residual < SWEEP_RESIDUAL_TOL
    print(f"  max residual = {max_residual:.3e}  (tol {SWEEP_RESIDUAL_TOL:.0e})")

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f"x_offset_sweep — {'PASS' if passed else 'FAIL'}",
                 fontsize=14, fontweight="bold")
    labels = ["x (m)", "θ (rad)", "ẋ (m/s)", "θ̇ (rad/s)"]
    for i in range(4):
        ax = axes[i // 2, i % 2]
        for x0 in SWEEP_X0_VALUES:
            ax.plot(all_times, all_states[x0][:, i], lw=0.8,
                    label=f"x₀={x0:.1f}")
        ax.set_ylabel(labels[i])
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "x_offset_sweep.png"), dpi=150)
    plt.close()

    tag = "PASS" if passed else "FAIL"
    dominant = f"max residual = {max_residual:.3e}"
    summary = f"  {tag}  x_offset_sweep: {dominant}"
    print(summary)
    return passed, summary


def run_damping(teensy_ip, teensy_port, results_dir):
    """Validate pendulum-joint damping against the b-aware reference.

    Sweeps b over DAMPING_B_VALUES on a large free swing (theta0 = pi/3) so the
    damped decay is exercised, runs each on the Teensy, and compares to the scipy
    reference solved with the SAME b. Also emits an overlay of theta(t) showing the
    decay grow with b — a quick visual sanity check that damping removes energy.
    """
    print(f"\n--- damping_sweep ---")
    base = replace(get_case("large_swing"),
                   tol_angle=DAMPING_TOL_ANGLE, tol_angvel=DAMPING_TOL_ANGVEL)
    all_passed = True
    fails = []
    overlay = []  # (b, times, theta)

    for b in DAMPING_B_VALUES:
        case = replace(base, name=f"damp_b={b:g}", b=b)
        times, states, _, nsteps, wall_us = run_on_teensy(
            case, teensy_ip, teensy_port)
        ref = compute_ref(case, times)
        err = states - ref
        max_errs = np.max(np.abs(err), axis=0)
        passed, dominant = check_tol(case, err)
        overlay.append((b, times, states[:, 1]))
        tag = "PASS" if passed else "FAIL"
        print(f"  b={b:<8g} {len(times):>5} samples  "
              f"max|Δθ|={max_errs[1]:.2e}  max|Δθ̇|={max_errs[3]:.2e}  -> {tag}")
        plot_standard(case, times, states, ref, err,
                      os.path.join(results_dir, f"{case.name}.png"), passed)
        if not passed:
            all_passed = False
            fails.append(f"b={b:g} ({dominant})")

    fig, ax = plt.subplots(figsize=(11, 5))
    for b, t, th in overlay:
        ax.plot(t, th, lw=1.0, label=f"b={b:g}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("θ (rad)")
    ax.set_title("Pendulum-joint damping sweep — θ(t) (decay grows with b)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "damping_sweep_overlay.png"), dpi=150)
    plt.close()

    tag = "PASS" if all_passed else "FAIL"
    dominant = "all b within tol" if all_passed else "; ".join(fails)
    summary = f"  {tag}  damping_sweep: {dominant}"
    print(summary)
    return all_passed, summary


def run_convergence(teensy_ip, teensy_port, results_dir):
    """Run convergence test: error vs. dt on log-log, check first-order slope."""
    print(f"\n--- convergence ---")
    base = get_case("large_swing")
    errors_by_dt = {}

    for dt in CONVERGENCE_DTS:
        case = replace(base, name=f"conv_dt={dt * 1000:.3f}ms", dt=dt)
        times, states, _, nsteps, wall_us = run_on_teensy(
            case, teensy_ip, teensy_port)
        ref = compute_ref(case, times)
        max_err = np.max(np.abs(states - ref), axis=0)
        representative = max(max_err[0], max_err[1])
        errors_by_dt[dt] = representative
        print(f"  dt={dt * 1000:.3f}ms: max pos err={representative:.3e}  "
              f"({nsteps} steps, wall={wall_us / 1e6:.3f}s)")

    dts = np.array(sorted(errors_by_dt.keys()))
    errs = np.array([errors_by_dt[dt] for dt in dts])
    log_dt = np.log(dts)
    log_err = np.log(errs)
    slope, intercept = np.polyfit(log_dt, log_err, 1)

    lo, hi = CONVERGENCE_SLOPE_RANGE
    passed = lo <= slope <= hi
    print(f"  log-log slope = {slope:.3f}  (expected [{lo}, {hi}])")

    fig, ax = plt.subplots(figsize=(8, 6))
    fig.suptitle(
        f"convergence — slope={slope:.3f} — {'PASS' if passed else 'FAIL'}",
        fontsize=14, fontweight="bold")
    ax.loglog(dts * 1000, errs, "ko-", markersize=8, label="max position error")
    fit_errs = np.exp(intercept + slope * log_dt)
    ax.loglog(dts * 1000, fit_errs, "r--", lw=1.5,
              label=f"fit: slope={slope:.2f}")
    mid = len(dts) // 2
    ref_line = errs[mid] * (dts / dts[mid])
    ax.loglog(dts * 1000, ref_line, "b:", lw=1, alpha=0.5,
              label="O(dt) reference")
    ax.set_xlabel("dt (ms)")
    ax.set_ylabel("Max position error (m or rad)")
    ax.legend()
    ax.grid(True, alpha=0.3, which="both")
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "convergence.png"), dpi=150)
    plt.close()

    tag = "PASS" if passed else "FAIL"
    summary = f"  {tag}  convergence: slope={slope:.3f}"
    print(summary)
    return passed, summary


# ── Main ─────────────────────────────────────────────────────────────

ALL_CASE_NAMES = [c.name for c in CASES] + ["x_offset_sweep", "convergence",
                                            "damping_sweep"]


def main():
    parser = argparse.ArgumentParser(
        description="Cart-pole physics validation: Teensy vs. scipy reference")
    parser.add_argument("--ip", default=DEFAULT_IP,
                        help=f"Teensy IP address (default {DEFAULT_IP})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"Teensy UDP port (default {DEFAULT_PORT})")
    parser.add_argument("--cases", nargs="*", default=None,
                        help="Test cases to run (default: all). "
                             f"Available: {', '.join(ALL_CASE_NAMES)}")
    parser.add_argument("--results-dir", default="results",
                        help="Output directory for plots (default: results/)")
    args = parser.parse_args()

    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               args.results_dir)
    os.makedirs(results_dir, exist_ok=True)

    if args.cases is None:
        names = list(ALL_CASE_NAMES)
    else:
        for n in args.cases:
            if n not in ALL_CASE_NAMES:
                print(f"Unknown case: {n}")
                print(f"Available: {', '.join(ALL_CASE_NAMES)}")
                sys.exit(1)
        names = args.cases

    # Connectivity check
    print(f"Pinging Teensy at {args.ip}:{args.port} ...")
    sock = _make_sock()
    sock.settimeout(1.0)
    try:
        _send(sock, (args.ip, args.port), "PING")
        data, _ = sock.recvfrom(1024)
        if b"ACK" not in data:
            print(f"  Unexpected response: {data}")
            sys.exit(1)
        print("  Teensy is responding.\n")
    except socket.timeout:
        print("  No response — is the Teensy running?")
        sys.exit(1)
    finally:
        sock.close()

    summaries = []
    all_passed = True

    for name in names:
        if name == "x_offset_sweep":
            ok, s = run_sweep(args.ip, args.port, results_dir)
        elif name == "damping_sweep":
            ok, s = run_damping(args.ip, args.port, results_dir)
        elif name == "convergence":
            ok, s = run_convergence(args.ip, args.port, results_dir)
        elif name == "energy_conservation":
            ok, s = run_energy(get_case(name), args.ip, args.port, results_dir)
        else:
            ok, s = run_standard(get_case(name), args.ip, args.port, results_dir)
        summaries.append(s)
        if not ok:
            all_passed = False

    print(f"\n{'=' * 60}")
    print("SUMMARY")
    print(f"{'=' * 60}")
    for s in summaries:
        print(s)
    print(f"{'=' * 60}")
    print("ALL PASSED" if all_passed else "SOME FAILED")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
