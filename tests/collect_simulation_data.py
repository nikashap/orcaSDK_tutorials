#!/usr/bin/env python
"""
collect_simulation_data.py

Collect friction data from the ORCA motor by replaying force trajectories
pre-computed from the cart-pendulum simulation.

Workflow:
  1) Pre-compute force command trajectories by running the MuJoCo simulation
     with no control input (ctrl=0) from various initial pendulum angles and
     starting cart positions.  F_command = mass_shaft * cart_x_ddot * 1e3 mN.
  2) Replay each trajectory on the real motor: command the pre-computed force
     at each timestep while recording position, force_sensed, and acceleration.

The motor data captures friction effects that the frictionless simulation does
not model, providing training data in the force/velocity/position regime that
the actual experiment operates in.

Output format matches collect_calibration_data.py (per-sample
force_commanded_mN) so that analyze_friction.py can process the data directly.

Usage:
  python collect_simulation_data.py
"""

from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MotorMode
import pyorcasdk.orca_registers as orca_reg
from datetime import datetime
import time
import numpy as np
import os
import yaml
import gymnasium as gym
import mujoco
import cartpole_target as ct_gym  # registers HardwareEnv-v0

XML_FILEPATH = '~/Documents/nikashap/cartpole-target/orca/cartpendulum-orca.xml'
# Change filepath if on lab machine

# ---------------------------------------------------------------------------
# Load parameters from YAML
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PARAMS_PATH = os.path.join(_SCRIPT_DIR, "calibration_params.yaml")

FORCE_SAFETY_LIMIT_MN = 70000
FORCE_SAFETY_LIMIT_COMPENSATED_MN = 70000

# ---------------------------------------------------------------------------
# Iteration / filename conventions (must match estimate_friction_from_simulation.py)
# ---------------------------------------------------------------------------

import re

_AUG_RE_ITER = re.compile(
    r"^calibration_simulation_friction_compensated_iter(\d+)_with_estimate\.npz$"
)


def shaft_filename_for_iteration(iteration):
    """Filename of the shaft data npz produced at the given iteration."""
    if iteration == 0:
        return "calibration_simulation_friction.npz"
    if iteration == 1:
        return "calibration_simulation_friction_compensated.npz"
    return f"calibration_simulation_friction_compensated_iter{iteration}.npz"


def detect_iteration_from_augmented_filename(filename):
    """Infer the iteration of the augmented (with-estimate) npz at `filename`.

    The 'iteration' of an augmented file is the iteration that *produced* the
    underlying shaft data — so iter 0's augmented file came from iter 0's
    shaft data, etc. Replaying with that augmented file produces iter (N+1).

    Returns None if the filename doesn't match a known pattern.
    """
    base = os.path.basename(filename)
    if base == "calibration_simulation_friction_with_estimate.npz":
        return 0
    if base == "calibration_simulation_friction_compensated_with_estimate.npz":
        return 1
    m = _AUG_RE_ITER.match(base)
    if m:
        return int(m.group(1))
    return None

with open(_PARAMS_PATH, "r") as f:
    PARAMS = yaml.safe_load(f)

MOTOR_MIN_UM = PARAMS["motor_min_um"]
MOTOR_MAX_UM = PARAMS["motor_max_um"]
EXPERIMENT_MIN_POS_UM = PARAMS["experiment_min_pos_um"]
EXPERIMENT_MAX_POS_UM = PARAMS["experiment_max_pos_um"]

AUTO_ZERO_FORCE_N = PARAMS["auto_zero_force_n"]
AUTO_ZERO_SPEED_MMPS = PARAMS["auto_zero_speed_mmps"]

BAUD_RATE = PARAMS["baud_rate"]
INTERFRAME_DELAY = PARAMS["interframe_delay_us"]
DEFAULT_SERIAL_PORT = PARAMS["default_serial_port"]

DATE_STR = datetime.now().strftime('%y_%m_%d-%H_%M_%S')
DATA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, PARAMS["data_dir"], DATE_STR))

CTRL_LOOP_DURATION_S = 0.004 #dt=0.004 in the xml file that governs cart-pendulum dynamics

# ---------------------------------------------------------------------------
# Motor ↔ simulation coordinate mapping
# ---------------------------------------------------------------------------
# x_sim = (position_um - MOTOR_CENTER_UM) * 1e-6  metres
# position_um = x_sim * 1e6 + MOTOR_CENTER_UM
MOTOR_CENTER_UM = (MOTOR_MIN_UM + MOTOR_MAX_UM) // 2
SAFETY_MARGIN_UM = 5000

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
SIM_PARAMS = {
    "max_episode_steps": 2000,
    "mass_cart": 2.0,            # per cube; total cart = 2 * mass_cart
    "mass_bob": 3.0,
    "pendulum_length": -0.2,     # always negative (hangs down)
    "mass_rod": 0.0,
    "mass_shaft": 0.0,
    "pendulum_damping": 0.0,
    "cart_damping": 0.0,
}

# Initial pendulum angles to sweep (rad)
THETA_INITS = np.arange(-4 * np.pi / 6, 4 * np.pi/6, np.pi / 6)
# THETA_INITS = [0]

# Starting motor positions (um) — spread across range, avoiding limits
DEFAULT_START_POSITIONS_UM = [150000, 200000, 300000, 40000]

# Trajectory position bounds (metres, simulation frame)
X_MIN_M = (MOTOR_MIN_UM + SAFETY_MARGIN_UM - MOTOR_CENTER_UM) * 1e-6
X_MAX_M = (MOTOR_MAX_UM - SAFETY_MARGIN_UM - MOTOR_CENTER_UM) * 1e-6
T_TOTAL_S = CTRL_LOOP_DURATION_S * SIM_PARAMS["max_episode_steps"]


# ---------------------------------------------------------------------------
# Sinusoidal trajectory generation
# ---------------------------------------------------------------------------

def sinusoidal_trajectory(t, a1, a2, a3, b1, b2, b3, c1):
    """Compute position, velocity, acceleration for x(t) = c1 + a1*sin(a2*t+a3) + b1*cos(b2*t+b3)."""
    t = np.asarray(t, dtype=np.float64)
    sin_arg = a2 * t + a3
    cos_arg = b2 * t + b3
    x = c1 + a1 * np.sin(sin_arg) + b1 * np.cos(cos_arg)
    x_dot = a1 * a2 * np.cos(sin_arg) - b1 * b2 * np.sin(cos_arg)
    x_ddot = -a1 * a2**2 * np.sin(sin_arg) - b1 * b2**2 * np.cos(cos_arg)
    return x, x_dot, x_ddot


def sample_sinusoidal_params(
    n_trajectories,
    x_min_m=X_MIN_M,
    x_max_m=X_MAX_M,
    accel_limit_mps2=None,
    freq_range_hz=(0.3, 4.0),
    amplitude_range_m=(0.01, 0.2),
    c1=0.0,
    seed=42,
    max_attempts_per_traj=200,
):
    """Rejection-sample sinusoidal trajectory parameters within motor bounds.

    Returns list of dicts with keys: a1, a2, a3, b1, b2, b3, c1, start_position_um.
    """
    if accel_limit_mps2 is None:
        accel_limit_mps2 = (FORCE_SAFETY_LIMIT_MN * 1e-3) / PARAMS["mass_shaft_kg"]

    rng = np.random.default_rng(seed)
    half_range = (x_max_m - x_min_m) / 2.0
    omega_min = 2 * np.pi * freq_range_hz[0]
    omega_max = 2 * np.pi * freq_range_hz[1]

    params_list = []
    total_attempts = 0

    while len(params_list) < n_trajectories:
        total_attempts += 1
        if total_attempts > n_trajectories * max_attempts_per_traj:
            print(f"WARNING: hit max attempts ({total_attempts}), "
                  f"only generated {len(params_list)}/{n_trajectories}")
            break

        a1 = rng.uniform(*amplitude_range_m)
        b1 = rng.uniform(*amplitude_range_m)
        if a1 + b1 >= half_range * 0.95:
            continue

        a2 = rng.uniform(omega_min, omega_max)
        b2 = rng.uniform(omega_min, omega_max)
        if a1 * a2**2 + b1 * b2**2 > accel_limit_mps2 * 0.9:
            continue

        a3 = rng.uniform(0, 2 * np.pi)
        b3 = rng.uniform(0, 2 * np.pi)

        c1_min = x_min_m + a1 + b1
        c1_max = x_max_m - a1 - b1
        if c1_min >= c1_max:
            continue
        if c1 is not None:
            actual_c1 = c1
        else:
            actual_c1 = rng.uniform(c1_min, c1_max)
        if actual_c1 < c1_min or actual_c1 > c1_max:
            continue

        x0 = actual_c1 + a1 * np.sin(a3) + b1 * np.cos(b3)
        start_position_um = int(x0 * 1e6 + MOTOR_CENTER_UM)

        params_list.append({
            "a1": a1, "a2": a2, "a3": a3,
            "b1": b1, "b2": b2, "b3": b3,
            "c1": actual_c1,
            "start_position_um": start_position_um,
        })

    print(f"Generated {len(params_list)} sinusoidal parameter sets "
          f"(acceptance rate: {len(params_list)/max(total_attempts, 1):.1%})")
    return params_list


def run_imposed_trajectory(env, traj_dict, dt=None):
    """Step the simulation with the cart constrained to follow x(t).

    Returns dict with pend_theta, pend_thetadot, sim_ctrl_N, sim_env_N.
    sim_ctrl_N is the total control input (N).
    sim_env_N is the environment force on the cart (pendulum coupling + passive),
    i.e. ctrl = M[0,0]*x_ddot + sim_env_N.
    """
    if dt is None:
        dt = CTRL_LOOP_DURATION_S

    model = env.unwrapped.model
    data = env.unwrapped.data

    x_arr = traj_dict["x_m"] if "x_m" in traj_dict else traj_dict["sim_cart_x"]
    xdot_arr = traj_dict["x_dot_mps"] if "x_dot_mps" in traj_dict else traj_dict["sim_cart_xdot"]
    xddot_arr = traj_dict["x_ddot_mps2"] if "x_ddot_mps2" in traj_dict else traj_dict["sim_cart_xddot"]
    n_steps = len(x_arr)

    pend_theta = np.zeros(n_steps)
    pend_thetadot = np.zeros(n_steps)
    sim_ctrl = np.zeros(n_steps)
    sim_env = np.zeros(n_steps)

    env.reset()
    data.qpos[0] = x_arr[0]
    data.qpos[1] = 0.0
    data.qvel[0] = xdot_arr[0]
    data.qvel[1] = 0.0
    mujoco.mj_forward(model, data)

    nv = model.nv
    M = np.zeros((nv, nv))
    rhs = np.zeros(nv)

    for i in range(n_steps):
        pend_theta[i] = data.qpos[1]
        pend_thetadot[i] = data.qvel[1]

        data.qpos[0] = x_arr[i]
        data.qvel[0] = xdot_arr[i]

        data.ctrl[0] = 0.0
        mujoco.mj_forward(model, data)

        M[:] = 0.0
        mujoco.mj_fullM(model, M, data.qM)
        np.subtract(data.qfrc_passive, data.qfrc_bias, out=rhs)
        np.add(rhs, data.qfrc_applied, out=rhs)
        np.add(rhs, data.qfrc_constraint, out=rhs)

        a_cart = xddot_arr[i]
        a_theta = (rhs[1] - M[1, 0] * a_cart) / M[1, 1]

        ctrl = M[0, 0] * a_cart + M[0, 1] * a_theta - rhs[0]
        sim_ctrl[i] = ctrl
        sim_env[i] = M[0, 1] * a_theta - rhs[0]

        data.ctrl[0] = ctrl
        mujoco.mj_step(model, data)

        if i + 1 < n_steps:
            data.qpos[0] = x_arr[i + 1]
            data.qvel[0] = xdot_arr[i + 1]

    return {
        "pend_theta": pend_theta,
        "pend_thetadot": pend_thetadot,
        "sim_ctrl_N": sim_ctrl,
        "sim_env_N": sim_env,
    }


def generate_sinusoidal_trajectories(
    n_trajectories=20,
    seed=123,
    freq_range_hz=(0.3, 4.0),
    amplitude_range_m=(0.01, 0.2),
    c1=0.0,
    sim_params=None,
):
    """Generate sinusoidal cart trajectories with both ctrl and env force profiles.

    For each trajectory, produces TWO entries:
      - force_type="ctrl": F = mass_shaft * x_ddot * 1e3 mN (exact cart accel)
      - force_type="env":  F = sim_env_N * 1e3 mN (pendulum coupling force)

    Returns a list of trajectory dicts compatible with run_simulation_trial.
    """
    if sim_params is None:
        sim_params = SIM_PARAMS
    mass_shaft_kg = PARAMS["mass_shaft_kg"]

    params_list = sample_sinusoidal_params(
        n_trajectories, seed=seed,
        freq_range_hz=freq_range_hz,
        amplitude_range_m=amplitude_range_m,
        c1=c1,
    )

    env = gym.make(
        'HardwareEnv-v0',
        xml_file=XML_FILEPATH,
        max_episode_steps=sim_params["max_episode_steps"],
        frame_skip=1,
        render_mode="rgb_array",
        mass_cart=sim_params["mass_cart"],
        mass_bob=sim_params["mass_bob"],
        pendulum_length=sim_params["pendulum_length"],
        mass_rod=sim_params["mass_rod"],
        mass_shaft=sim_params["mass_shaft"],
        pendulum_damping=sim_params["pendulum_damping"],
        cart_damping=sim_params["cart_damping"],
    )

    t = np.arange(0, T_TOTAL_S, CTRL_LOOP_DURATION_S)
    trajectories = []
    total = len(params_list)

    print(f"\nGenerating {total} sinusoidal trajectories × 2 force types "
          f"({total * 2} total trials)...")

    for i, p in enumerate(params_list):
        x, x_dot, x_ddot = sinusoidal_trajectory(
            t, p['a1'], p['a2'], p['a3'],
            p['b1'], p['b2'], p['b3'], p['c1'],
        )

        traj_dict = {"x_m": x, "x_dot_mps": x_dot, "x_ddot_mps2": x_ddot}
        sim_result = run_imposed_trajectory(env, traj_dict)

        F_ctrl_mN = mass_shaft_kg * x_ddot * 1e3
        F_env_mN = sim_result["sim_env_N"] * 1e3

        common = {
            "theta_init": 0.0,
            "start_position_um": p["start_position_um"],
            "sim_cart_x": x,
            "sim_cart_xdot": x_dot,
            "sim_cart_xddot": x_ddot,
            "sim_pend_theta": sim_result["pend_theta"],
            "sim_pend_thetadot": sim_result["pend_thetadot"],
            "sim_env_N": sim_result["sim_env_N"],
            "sim_timestep_s": CTRL_LOOP_DURATION_S,
            "trajectory_type": "sinusoidal",
            "traj_id": i,
        }

        trajectories.append({
            **common,
            "force_commands_mN": F_ctrl_mN,
            "force_type": "ctrl",
            "n_steps": len(t),
        })
        trajectories.append({
            **common,
            "force_commands_mN": F_env_mN,
            "force_type": "env",
            "n_steps": len(t),
        })

        freq_sin = p['a2'] / (2 * np.pi)
        freq_cos = p['b2'] / (2 * np.pi)
        print(f"  [{i+1:3d}/{total}] a1={p['a1']:.3f}m @ {freq_sin:.1f}Hz, "
              f"b1={p['b1']:.3f}m @ {freq_cos:.1f}Hz, "
              f"start={p['start_position_um']}µm, "
              f"max|F_ctrl|={np.abs(F_ctrl_mN).max():.0f}mN, "
              f"max|F_env|={np.abs(F_env_mN).max():.0f}mN")

    env.close()
    return trajectories


# ---------------------------------------------------------------------------
# Motor utility functions (same as collect_calibration_data.py)
# ---------------------------------------------------------------------------

def auto_zero_motor(motor):
    """Run the ORCA auto-zero procedure to home the shaft."""
    motor.set_mode(MotorMode.SleepMode)
    time.sleep(0.1)

    motor.write_register_blocking(orca_reg.ZERO_MODE, orca_reg.ZERO_MODE_AUTO_ZERO_ENABLED)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_FORCE_N, AUTO_ZERO_FORCE_N)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_SPEED_MMPS, AUTO_ZERO_SPEED_MMPS)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_EXIT_MODE, MotorMode.SleepMode)

    motor.set_mode(MotorMode.AutoZeroMode)
    motor.clear_errors()

    time.sleep(6)

    while True:
        motor.run()
        time.sleep(0.005)
        error_check = motor.get_errors()

        if error_check.value & orca_reg.ERROR_0_AUTO_ZERO_FAILED_Mask:
            print("  Auto-zero FAILED.")
            break
        if error_check.value & 2048:
            print("  Communication timeout during auto-zero.")
            break
        if motor.get_mode().value != MotorMode.AutoZeroMode:
            print("  Auto-zero complete.")
            break

    return error_check


def read_raw_acceleration(motor):
    """Read shaft acceleration (blocking call). Returns value in mm/s^2."""
    accel_data = motor.read_wide_register_blocking(
        reg_address=orca_reg.SHAFT_ACCEL_MMPSS,
    )
    return accel_data.value

def read_raw_velocity(motor):
    """Read shaft velocity (blocking call). Returns value in mm/s"""
    speed_data = motor.read_wide_register_blocking(
        reg_address=orca_reg.SHAFT_SPEED_MMPS
    )
    return speed_data.value


def advance_motor_stream(motor, frames=4, sleeptime=0.002):
    """Run through the motor command stream to clear any potentially stale frames."""
    for _ in range(frames):
        motor.run()
        time.sleep(sleeptime)


def move_to_position(motor, target_pos_um, kick_force_mag_mN=10000,
                     duration_s=0.2, pos_tolerance_um=200):
    """
    Move the motor to ``target_pos_um``.

    The motor must already have streaming enabled before calling this function.
    On return the motor is placed back into SleepMode.
    """
    motor.set_mode(MotorMode.ForceMode)
    passed_target = False
    motor.run()
    stream_data = motor.get_stream_data()
    force_direction = +1
    if stream_data.position > target_pos_um:
        force_direction = -1

    while not passed_target:
        motor.run()
        stream_data = motor.get_stream_data()
        if stream_data.position <= target_pos_um and force_direction < 0:
            passed_target = True
        elif stream_data.position >= target_pos_um and force_direction > 0:
            passed_target = True
        motor.set_streamed_force_mN(kick_force_mag_mN * force_direction)

    motor.set_mode(MotorMode.PositionMode)
    advance_motor_stream(motor)
    stream_data = motor.get_stream_data()
    start_pos_um = stream_data.position

    motor.set_streamed_position_um(start_pos_um)
    motor.run()

    n_steps = max(int(duration_s / 0.005), 1)
    dt = duration_s / n_steps

    print(f"  Moving {start_pos_um} → {target_pos_um} µm over {duration_s:.1f} s "
          f"({n_steps} steps)...")

    for i in range(1, n_steps + 1):
        frac = i / n_steps
        interp_pos = int(start_pos_um + frac * (target_pos_um - start_pos_um))
        motor.set_streamed_position_um(interp_pos)
        motor.run()
        time.sleep(dt)

    while True:
        motor.run()
        stream_data = motor.get_stream_data()
        if abs(stream_data.position - target_pos_um) <= pos_tolerance_um:
            break
        time.sleep(0.005)

    motor.set_mode(MotorMode.SleepMode)
    print(f"  Reached position ({stream_data.position} µm).")


# ---------------------------------------------------------------------------
# Trajectory generation (offline — no motor needed)
# ---------------------------------------------------------------------------

def generate_trajectories(theta_inits=THETA_INITS,
                          start_positions_um=None,
                          sim_params=None):
    """Pre-compute force command trajectories from the cart-pendulum simulation.

    For each (theta_init, start_position_um) combo, resets the simulation,
    sets the initial pendulum angle and cart position, then steps with ctrl=0.
    At each step, records:
        F_command_mN = mass_shaft_kg * cart_x_ddot * 1e3

    Returns a list of trajectory dicts.
    """
    if start_positions_um is None:
        start_positions_um = DEFAULT_START_POSITIONS_UM
    if sim_params is None:
        sim_params = SIM_PARAMS

    env = gym.make(
        'HardwareEnv-v0',
        xml_file=XML_FILEPATH,
        max_episode_steps=sim_params["max_episode_steps"],
        frame_skip=1,
        render_mode="rgb_array",
        mass_cart=sim_params["mass_cart"],
        mass_bob=sim_params["mass_bob"],
        pendulum_length=sim_params["pendulum_length"],
        mass_rod=sim_params["mass_rod"],
        mass_shaft=sim_params["mass_shaft"],
        pendulum_damping=sim_params["pendulum_damping"],
        cart_damping=sim_params["cart_damping"],
    )

    model = env.unwrapped.model
    data = env.unwrapped.data
    mass_shaft_kg = PARAMS["mass_shaft_kg"]
    sim_timestep_s = model.opt.timestep

    trajectories = []
    total = len(theta_inits) * len(start_positions_um)
    idx = 0

    print(f"\nGenerating {total} trajectories "
          f"(sim timestep={sim_timestep_s * 1e3:.1f} ms, "
          f"max steps={sim_params['max_episode_steps']})...")

    for theta_init in theta_inits:
        for start_pos_um in start_positions_um:
            idx += 1
            x_start = (start_pos_um - MOTOR_CENTER_UM) * 1e-6

            env.reset()
            data.qpos[0] = x_start
            data.qpos[1] = theta_init
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)

            force_commands = []
            sim_cart_x = []
            sim_cart_xdot = []
            sim_cart_xddot = []
            sim_pend_theta = []
            sim_pend_thetadot = []

            for step in range(sim_params["max_episode_steps"]):
                sim_cart_x.append(data.qpos[0])
                sim_cart_xdot.append(data.qvel[0])
                sim_pend_theta.append(data.qpos[1])
                sim_pend_thetadot.append(data.qvel[1])

                obs, reward, terminated, truncated, info = env.step(np.array([0.0]))

                cart_x_ddot = data.qacc[0]
                sim_cart_xddot.append(cart_x_ddot)
                force_commands.append(mass_shaft_kg * cart_x_ddot * 1e3)

                if terminated:
                    break

            force_arr = np.array(force_commands, dtype=np.float64)
            max_force = np.max(np.abs(force_arr)) if len(force_arr) > 0 else 0

            print(f"  [{idx:3d}/{total}] theta={np.degrees(theta_init):+7.1f}°, "
                  f"start={start_pos_um:6d} um, "
                  f"{len(force_commands):4d} steps, "
                  f"max|F|={max_force:8.0f} mN")

            trajectories.append({
                "force_commands_mN": force_arr,
                "theta_init": theta_init,
                "start_position_um": start_pos_um,
                "sim_cart_x": np.array(sim_cart_x, dtype=np.float64),
                "sim_cart_xdot": np.array(sim_cart_xdot, dtype=np.float64),
                "sim_cart_xddot": np.array(sim_cart_xddot, dtype=np.float64),
                "sim_pend_theta": np.array(sim_pend_theta, dtype=np.float64),
                "sim_pend_thetadot": np.array(sim_pend_thetadot, dtype=np.float64),
                "sim_timestep_s": sim_timestep_s,
                "n_steps": len(force_commands),
                "trajectory_type": "pendulum_drop",
                "force_type": "ctrl",
                "traj_id": idx - 1,
            })

    env.close()
    return trajectories


def save_trajectories(trajectories, filepath):
    """Save pre-computed trajectories to .npz for later replay or inspection."""
    n_traj = len(trajectories)
    traj_n_steps = np.array([t["n_steps"] for t in trajectories], dtype=np.int32)
    traj_boundaries = np.concatenate([[0], np.cumsum(traj_n_steps)])

    save_dict = dict(
        n_trajectories=n_traj,
        sim_timestep_s=trajectories[0]["sim_timestep_s"],
        mass_shaft_kg=PARAMS["mass_shaft_kg"],
        motor_center_um=MOTOR_CENTER_UM,
        sim_mass_cart=SIM_PARAMS["mass_cart"],
        sim_mass_bob=SIM_PARAMS["mass_bob"],
        sim_pendulum_length=SIM_PARAMS["pendulum_length"],
        # Per-trajectory metadata
        traj_theta_init=np.array([t["theta_init"] for t in trajectories],
                                 dtype=np.float64),
        traj_start_position_um=np.array([t["start_position_um"] for t in trajectories],
                                        dtype=np.int32),
        traj_n_steps=traj_n_steps,
        traj_boundaries=traj_boundaries,
        traj_trajectory_type=np.array([t.get("trajectory_type", "pendulum_drop")
                                       for t in trajectories], dtype="U20"),
        traj_force_type=np.array([t.get("force_type", "ctrl")
                                  for t in trajectories], dtype="U10"),
        traj_traj_id=np.array([t.get("traj_id", i)
                               for i, t in enumerate(trajectories)], dtype=np.int32),
        # Concatenated per-step arrays
        force_commands_mN=np.concatenate([t["force_commands_mN"] for t in trajectories]),
        sim_cart_x=np.concatenate([t["sim_cart_x"] for t in trajectories]),
        sim_cart_xdot=np.concatenate([t["sim_cart_xdot"] for t in trajectories]),
        sim_cart_xddot=np.concatenate([t["sim_cart_xddot"] for t in trajectories]),
        sim_pend_theta=np.concatenate([t["sim_pend_theta"] for t in trajectories]),
        sim_pend_thetadot=np.concatenate([t["sim_pend_thetadot"] for t in trajectories]),
    )
    # Store environment force if available (sinusoidal trajectories)
    if any("sim_env_N" in t for t in trajectories):
        save_dict["sim_env_N"] = np.concatenate([
            t.get("sim_env_N", np.zeros(t["n_steps"])) for t in trajectories
        ])
    np.savez(filepath, **save_dict)

    global_max = max(np.max(np.abs(t["force_commands_mN"])) for t in trajectories)
    print(f"\nSaved {n_traj} trajectories to {filepath}")
    print(f"  Total steps: {traj_n_steps.sum()}")
    print(f"  Max |force_command|: {global_max:.0f} mN"
          f"{'  *** EXCEEDS SAFETY LIMIT ***' if global_max > FORCE_SAFETY_LIMIT_MN else ''}")


def load_trajectories(filepath):
    """Load pre-computed trajectories from .npz."""
    d = np.load(filepath, allow_pickle=True)
    n_traj = int(d["n_trajectories"])
    boundaries = d["traj_boundaries"]

    has_type = "traj_trajectory_type" in d
    has_force = "traj_force_type" in d
    has_traj_id = "traj_traj_id" in d
    has_env = "sim_env_N" in d

    trajectories = []
    for i in range(n_traj):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        traj = {
            "force_commands_mN": d["force_commands_mN"][lo:hi],
            "theta_init": float(d["traj_theta_init"][i]),
            "start_position_um": int(d["traj_start_position_um"][i]),
            "sim_cart_x": d["sim_cart_x"][lo:hi],
            "sim_cart_xdot": d["sim_cart_xdot"][lo:hi],
            "sim_cart_xddot": d["sim_cart_xddot"][lo:hi],
            "sim_pend_theta": d["sim_pend_theta"][lo:hi],
            "sim_pend_thetadot": d["sim_pend_thetadot"][lo:hi],
            "sim_timestep_s": float(d["sim_timestep_s"]),
            "n_steps": hi - lo,
            "trajectory_type": str(d["traj_trajectory_type"][i]) if has_type else "pendulum_drop",
            "force_type": str(d["traj_force_type"][i]) if has_force else "ctrl",
            "traj_id": int(d["traj_traj_id"][i]) if has_traj_id else i,
        }
        if has_env:
            traj["sim_env_N"] = d["sim_env_N"][lo:hi]
        trajectories.append(traj)

    n_types = {}
    for t in trajectories:
        key = f"{t['trajectory_type']}/{t['force_type']}"
        n_types[key] = n_types.get(key, 0) + 1
    type_summary = ", ".join(f"{k}: {v}" for k, v in n_types.items())
    print(f"Loaded {n_traj} trajectories from {filepath} ({type_summary})")
    return trajectories


def _smooth_residual_per_trial(residual_arr, trial_boundaries, window):
    """Apply a centered moving average of `window` samples to `residual_arr`,
    operating independently within each trial slice so the smoothing window
    never crosses a trial boundary.

    Edges of each trial use a smaller window (min_periods=1 semantics) so
    breakaway samples at trial start aren't diluted with zeros.

    `window=1` (or less) returns the input unchanged.
    """
    if window is None or window <= 1:
        return residual_arr.copy()
    out = np.empty_like(residual_arr)
    half = window // 2
    for k in range(len(trial_boundaries) - 1):
        lo, hi = int(trial_boundaries[k]), int(trial_boundaries[k + 1])
        if hi - lo == 0:
            continue
        seg = residual_arr[lo:hi]
        # Build per-sample mean over the window centered at each index,
        # clipped to the trial's own slice. This is equivalent to
        # pandas' rolling(window, center=True, min_periods=1).mean().
        n = len(seg)
        smoothed = np.empty(n, dtype=np.float64)
        for i in range(n):
            wlo = max(0, i - half)
            whi = min(n, i + half + 1)
            smoothed[i] = np.mean(seg[wlo:whi])
        out[lo:hi] = smoothed
    return out


def load_compensated_trajectories(augmented_npz_path,
                                   reference_trajectories_path,
                                   theta_tol_rad=np.radians(0.5),
                                   learning_gain=0.5,
                                   smoothing_window=5):
    """Build trajectory dicts for replay with friction-compensated commands.

    Loads two files:
      - augmented_npz_path: a calibration_simulation_friction_with_estimate.npz
        produced by estimate_friction_from_simulation.py. Provides per-trial
        force_commanded_mN, force_friction_estimate, and trial metadata.
      - reference_trajectories_path: the simulation_trajectories.npz produced
        alongside the original collection run. Provides per-sim-step
        sim_cart_x, sim_cart_xdot, sim_cart_xddot arrays needed by
        run_simulation_trial.

    For each trial in the augmented npz, the compensated force command is

        F_command_compensated[i] = force_commanded_mN[i]
                                   + learning_gain * smooth(force_friction_estimate)[i]

    where `smooth` is a centered moving average with `smoothing_window`
    samples (applied per-trial, no cross-boundary mixing).

    Parameters
    ----------
    learning_gain : float
        Scalar multiplier on the friction-estimate residual. Conservative
        ILC values are 0.3-0.7. Gain = 1.0 fully trusts the residual;
        gain < 1.0 hedges against trajectory-dependent friction drift
        between iterations.
    smoothing_window : int
        Window length (in samples) for the moving average applied to the
        residual before scaling. Window = 1 disables smoothing. The window
        is centered and clipped to each trial's slice.

    Each compensated trajectory is matched to a reference trajectory by
    (theta_init, start_position_um). The reference's per-sim-step arrays
    are sliced to the same n_steps as the recorded trial (which may be
    shorter than the original sim length if the trial hit a safety stop).

    Returns a list of trajectory dicts compatible with run_simulation_trial.
    """
    print(f"Loading augmented friction data: {augmented_npz_path}")
    aug = np.load(augmented_npz_path, allow_pickle=True)
    n_trials = int(aug["n_trials"])
    boundaries = aug["trial_boundaries"]

    print(f"Loading reference trajectories: {reference_trajectories_path}")
    ref = np.load(reference_trajectories_path, allow_pickle=True)
    n_ref = int(ref["n_trajectories"])
    ref_boundaries = ref["traj_boundaries"]
    sim_dt_s = float(ref["sim_timestep_s"])
    ref_thetas = ref["traj_theta_init"]
    ref_starts = ref["traj_start_position_um"]
    ref_has_type = "traj_trajectory_type" in ref
    ref_has_force = "traj_force_type" in ref
    ref_has_traj_id = "traj_traj_id" in ref

    # Pre-smooth the residual across the whole concatenated array but
    # respecting trial boundaries (smoothing within a trial only).
    raw_friction = aug["force_friction_estimate"].astype(np.float64)
    raw_friction = np.nan_to_num(raw_friction, nan=0.0)
    smoothed_friction = _smooth_residual_per_trial(
        raw_friction, boundaries, smoothing_window
    )
    print(f"  Learning gain: {learning_gain}")
    print(f"  Residual smoothing window: {smoothing_window} samples"
          f"{' (disabled)' if smoothing_window <= 1 else ''}")

    trajectories = []
    skipped = 0

    for i in range(n_trials):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        n_recorded = hi - lo
        if n_recorded < 2:
            print(f"  Trial {i + 1}: too few samples ({n_recorded}), skipping.")
            skipped += 1
            continue

        theta = float(aug["trial_theta_init"][i])
        start_pos = int(aug["trial_start_position_um"][i])
        aug_has_type = "trial_trajectory_type" in aug
        aug_has_force = "trial_force_type" in aug
        aug_has_traj_id = "trial_traj_id" in aug
        trial_traj_type = str(aug["trial_trajectory_type"][i]) if aug_has_type else "pendulum_drop"
        trial_force_type = str(aug["trial_force_type"][i]) if aug_has_force else "ctrl"
        trial_traj_id = int(aug["trial_traj_id"][i]) if aug_has_traj_id else i

        # Match to a reference trajectory.
        match_idx = None
        for j in range(n_ref):
            ref_force = str(ref["traj_force_type"][j]) if ref_has_force else "ctrl"
            ref_type = str(ref["traj_trajectory_type"][j]) if ref_has_type else "pendulum_drop"

            if trial_traj_type == "sinusoidal" and ref_type == "sinusoidal":
                ref_tid = int(ref["traj_traj_id"][j]) if ref_has_traj_id else j
                if ref_tid == trial_traj_id and ref_force == trial_force_type:
                    match_idx = j
                    break
            else:
                if int(ref_starts[j]) != start_pos:
                    continue
                if abs(float(ref_thetas[j]) - theta) > theta_tol_rad:
                    continue
                if ref_force != trial_force_type:
                    continue
                match_idx = j
                break
        if match_idx is None:
            print(f"  Trial {i + 1}: no reference trajectory match for "
                  f"({trial_traj_type}/{trial_force_type}, "
                  f"theta={np.degrees(theta):+.1f}°, start={start_pos} um); "
                  f"skipping.")
            skipped += 1
            continue

        # Build the compensated force trajectory:
        #   F_compensated = F_commanded + gain * smoothed_residual
        f_cmd = aug["force_commanded_mN"][lo:hi].astype(np.float64)
        f_residual = smoothed_friction[lo:hi]
        f_compensated = f_cmd + learning_gain * f_residual

        # Slice the matched reference trajectory to the recorded length so
        # run_simulation_trial sees the right number of sim steps.
        rlo, rhi = int(ref_boundaries[match_idx]), int(ref_boundaries[match_idx + 1])
        n_ref_steps = rhi - rlo
        n_use = min(n_recorded, n_ref_steps)
        if n_use < n_recorded:
            print(f"  Trial {i + 1}: recorded ({n_recorded}) longer than "
                  f"reference ({n_ref_steps}); truncating to {n_use}.")
        f_compensated = f_compensated[:n_use]

        trajectories.append({
            "force_commands_mN": f_compensated,
            "theta_init": theta,
            "start_position_um": start_pos,
            "sim_cart_x": ref["sim_cart_x"][rlo:rlo + n_use],
            "sim_cart_xdot": ref["sim_cart_xdot"][rlo:rlo + n_use],
            "sim_cart_xddot": ref["sim_cart_xddot"][rlo:rlo + n_use],
            "sim_pend_theta": ref["sim_pend_theta"][rlo:rlo + n_use],
            "sim_pend_thetadot": ref["sim_pend_thetadot"][rlo:rlo + n_use],
            "sim_timestep_s": sim_dt_s,
            "n_steps": n_use,
            "trajectory_type": trial_traj_type,
            "force_type": trial_force_type,
            "traj_id": trial_traj_id,
        })

    print(f"Built {len(trajectories)} compensated trajectories "
          f"(skipped {skipped}).")
    if trajectories:
        all_compensated = np.concatenate(
            [t["force_commands_mN"] for t in trajectories]
        )
        max_abs = float(np.max(np.abs(all_compensated)))
        print(f"  Max |F_command_compensated|: {max_abs:.0f} mN")
        if max_abs > FORCE_SAFETY_LIMIT_COMPENSATED_MN:
            print(f"  Note: some compensated commands exceed the tighter "
                  f"safety limit ({FORCE_SAFETY_LIMIT_COMPENSATED_MN} mN) "
                  f"and will be clamped during replay.")
    return trajectories


# ---------------------------------------------------------------------------
# Motor data collection
# ---------------------------------------------------------------------------

def run_simulation_trial(motor, trajectory,
                          force_safety_limit_mn=FORCE_SAFETY_LIMIT_MN):
    """Replay one pre-computed force trajectory on the motor.

    Commands each force value in sequence while recording the motor's
    position, force_sensed, and acceleration.  Also records the
    corresponding simulated cart state (position, velocity, acceleration)
    for later trajectory deviation analysis.

    Stops early if the shaft approaches motor limits.

    Returns dict with per-sample arrays matching collect_calibration_data.py
    output format, plus sim_* arrays for comparison.
    """
    start_pos_um = trajectory["start_position_um"]
    force_commands = trajectory["force_commands_mN"]
    sim_timestep_s = trajectory["sim_timestep_s"]

    # Pre-computed simulation trajectory arrays (one per sim step)
    sim_cart_x_arr = trajectory["sim_cart_x"]
    sim_cart_xdot_arr = trajectory["sim_cart_xdot"]
    sim_cart_xddot_arr = trajectory["sim_cart_xddot"]

    move_to_position(motor, target_pos_um=start_pos_um)

    t_stream_list = []
    position_list = []
    force_list = []
    t_accel_list = []
    accel_list = []
    t_vel_list = []
    vel_list = []
    force_cmd_list = []

    # Corresponding sim values at each motor step
    sim_position_um_list = []
    sim_velocity_mm_s_list = []
    sim_accel_mmpss_list = []

    motor.set_mode(MotorMode.ForceMode)

    safe_min = MOTOR_MIN_UM + SAFETY_MARGIN_UM
    safe_max = MOTOR_MAX_UM - SAFETY_MARGIN_UM

    for i, F_cmd in enumerate(force_commands):
        t_beg = time.perf_counter()
        F_cmd_clamped = int(max(-force_safety_limit_mn,
                                min(force_safety_limit_mn, F_cmd)))
        motor.set_streamed_force_mN(F_cmd_clamped)

        motor.run()
        t1 = time.perf_counter()

        stream_data = motor.get_stream_data()
        pos_um = stream_data.position
        force_mN = stream_data.force

        accel_mmpss = read_raw_acceleration(motor)
        t2 = time.perf_counter()

        vel_mm_s = read_raw_velocity(motor)
        t3 = time.perf_counter()

        t_stream_list.append(t1)
        position_list.append(pos_um)
        force_list.append(force_mN)
        t_accel_list.append(t2)
        accel_list.append(accel_mmpss)
        t_vel_list.append(t3)
        vel_list.append(vel_mm_s)
        force_cmd_list.append(F_cmd_clamped)

        # Store corresponding simulation cart state at this timestep
        sim_position_um_list.append(
            sim_cart_x_arr[i] * 1e6 + MOTOR_CENTER_UM
        )
        sim_velocity_mm_s_list.append(
            sim_cart_xdot_arr[i] * 1e3
        )
        # Reference acceleration: the acceleration the shaft SHOULD achieve
        # if the commanded force were applied with no friction.
        # F_cmd (unclamped, mN) / mass_shaft (kg) = mm/s² (since kg·mm/s²=mN).
        # This works for both ctrl (F=m*a_cart*1e3) and env (F=F_env*1e3) types.
        sim_accel_mmpss_list.append(
            F_cmd / PARAMS["mass_shaft_kg"]
        )

        if pos_um <= safe_min or pos_um >= safe_max:
            print(f"    Hit safety limit at position {pos_um} um "
                  f"(step {i}/{len(force_commands)})")
            break

        if stream_data.errors:
            print(f"    Motor error: {stream_data.errors}")
            break
        t_end = time.perf_counter()
        loop_dur = t_end - t_beg
        if (loop_dur) < CTRL_LOOP_DURATION_S:
            time.sleep(CTRL_LOOP_DURATION_S - loop_dur)

    motor.set_streamed_force_mN(0)
    motor.set_mode(MotorMode.SleepMode)

    return {
        # Motor measured data
        "t_stream": np.array(t_stream_list, dtype=np.float64),
        "position_um": np.array(position_list, dtype=np.int32),
        "force_mN": np.array(force_list, dtype=np.int32),
        "t_accel": np.array(t_accel_list, dtype=np.float64),
        "accel_mmpss": np.array(accel_list, dtype=np.int32),
        "t_vel": np.array(t_vel_list, dtype=np.float64),
        "vel_mm_s": np.array(vel_list, dtype=np.int32),
        "force_commanded_mN": np.array(force_cmd_list, dtype=np.int32),
        # Corresponding simulation cart state (same units as motor data)
        "sim_position_um": np.array(sim_position_um_list, dtype=np.float64),
        "sim_velocity_mm_s": np.array(sim_velocity_mm_s_list, dtype=np.float64),
        "sim_accel_mmpss": np.array(sim_accel_mmpss_list, dtype=np.float64),
    }


def collect_simulation_data(motor, trajectories,
                            output_filename="calibration_simulation_friction.npz",
                            force_safety_limit_mn=FORCE_SAFETY_LIMIT_MN):
    """Run all pre-computed trajectories on the motor and save data.

    Output npz has per-sample force_commanded_mN (same as rampdown/sinusoid
    format) so analyze_friction.py can process it directly.  Also includes
    sim_position_um, sim_velocity_mm_s, sim_accel_mmpss for trajectory
    deviation analysis.

    Parameters
    ----------
    output_filename : str
        Filename (saved into DATA_DIR) for the resulting npz.
    force_safety_limit_mn : int
        Per-step clamp magnitude on the commanded force.  Compensated
        replays should pass a tighter value than the standard
        FORCE_SAFETY_LIMIT_MN.
    """
    os.makedirs(DATA_DIR, exist_ok=True)

    all_t_stream = []
    all_position_um = []
    all_force_mN = []
    all_t_accel = []
    all_accel_mmpss = []
    all_t_vel = []
    all_vel_mm_s = []
    all_force_commanded_mN = []

    # Simulation reference trajectory arrays
    all_sim_position_um = []
    all_sim_velocity_mm_s = []
    all_sim_accel_mmpss = []

    trial_n_samples = []
    trial_theta_init = []
    trial_start_position_um = []
    trial_trajectory_type = []
    trial_force_type = []
    trial_traj_id = []

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    total_trials = len(trajectories)
    trial_num = 0

    try:
        for traj in trajectories:
            trial_num += 1
            theta = traj["theta_init"]
            start_pos = traj["start_position_um"]
            n_steps = traj["n_steps"]

            traj_type = traj.get("trajectory_type", "pendulum_drop")
            force_type = traj.get("force_type", "ctrl")
            traj_id = traj.get("traj_id", trial_num - 1)

            print(f"\n--- Trial {trial_num}/{total_trials} "
                  f"[{traj_type}/{force_type}]: "
                  f"theta_init={np.degrees(theta):+.1f}°, "
                  f"start_pos={start_pos} um, "
                  f"{n_steps} steps ---")

            print("  Auto-zeroing...")
            error = auto_zero_motor(motor)
            if error.value != 0:
                print(f"  WARNING: Auto-zero returned error {error.value}, "
                      f"skipping.")
                continue

            motor.enable_stream()
            time.sleep(0.3)

            print(f"  Replaying trajectory...")
            trial_data = run_simulation_trial(
                motor, traj, force_safety_limit_mn=force_safety_limit_mn,
            )

            n_samples = len(trial_data["t_stream"])
            if n_samples == 0:
                print(f"  WARNING: No samples recorded, skipping.")
                continue

            # Compute deviation statistics for this trial
            pos_dev = trial_data["position_um"].astype(np.float64) - trial_data["sim_position_um"]
            pos_rmse = np.sqrt(np.mean(pos_dev ** 2))

            print(f"  Recorded {n_samples}/{n_steps} samples, "
                  f"position range: {trial_data['position_um'][0]} - "
                  f"{trial_data['position_um'][-1]} um, "
                  f"pos RMSE vs sim: {pos_rmse:.0f} um")

            all_t_stream.append(trial_data["t_stream"])
            all_position_um.append(trial_data["position_um"])
            all_force_mN.append(trial_data["force_mN"])
            all_t_accel.append(trial_data["t_accel"])
            all_t_vel.append(trial_data["t_vel"])
            all_vel_mm_s.append(trial_data["vel_mm_s"])
            all_accel_mmpss.append(trial_data["accel_mmpss"])
            all_force_commanded_mN.append(trial_data["force_commanded_mN"])

            all_sim_position_um.append(trial_data["sim_position_um"])
            all_sim_velocity_mm_s.append(trial_data["sim_velocity_mm_s"])
            all_sim_accel_mmpss.append(trial_data["sim_accel_mmpss"])

            trial_n_samples.append(n_samples)
            trial_theta_init.append(theta)
            trial_start_position_um.append(start_pos)
            trial_trajectory_type.append(traj_type)
            trial_force_type.append(force_type)
            trial_traj_id.append(traj_id)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\nError during collection: {e}")
        raise
    finally:
        motor.set_mode(MotorMode.SleepMode)

    if not trial_n_samples:
        print("\nNo trials completed, nothing to save.")
        return []

    filepath = os.path.join(DATA_DIR, output_filename)

    trial_n_samples_arr = np.array(trial_n_samples, dtype=np.int32)
    trial_boundaries = np.concatenate([[0], np.cumsum(trial_n_samples_arr)])

    np.savez(
        filepath,
        # Metadata
        procedure_type="simulation",
        mass_shaft_kg=PARAMS["mass_shaft_kg"],
        motor_min_um=MOTOR_MIN_UM,
        motor_max_um=MOTOR_MAX_UM,
        experiment_min_pos_um=EXPERIMENT_MIN_POS_UM,
        experiment_max_pos_um=EXPERIMENT_MAX_POS_UM,
        motor_center_um=MOTOR_CENTER_UM,
        sim_mass_cart=SIM_PARAMS["mass_cart"],
        sim_mass_bob=SIM_PARAMS["mass_bob"],
        sim_pendulum_length=SIM_PARAMS["pendulum_length"],
        sim_timestep_s=trajectories[0]["sim_timestep_s"],
        n_trials=len(trial_n_samples),
        # Per-trial arrays
        trial_theta_init=np.array(trial_theta_init, dtype=np.float64),
        trial_start_position_um=np.array(trial_start_position_um, dtype=np.int32),
        trial_n_samples=trial_n_samples_arr,
        trial_boundaries=trial_boundaries,
        trial_trajectory_type=np.array(trial_trajectory_type, dtype="U20"),
        trial_force_type=np.array(trial_force_type, dtype="U10"),
        trial_traj_id=np.array(trial_traj_id, dtype=np.int32),
        # Concatenated per-sample arrays — motor measured
        t_stream=np.concatenate(all_t_stream),
        position_um=np.concatenate(all_position_um),
        force_mN=np.concatenate(all_force_mN),
        force_commanded_mN=np.concatenate(all_force_commanded_mN),
        t_accel=np.concatenate(all_t_accel),
        accel_mmpss=np.concatenate(all_accel_mmpss),
        t_vel=np.concatenate(all_t_vel),
        vel_mm_s=np.concatenate(all_vel_mm_s),
        # Concatenated per-sample arrays — simulation reference trajectory
        sim_position_um=np.concatenate(all_sim_position_um),
        sim_velocity_mm_s=np.concatenate(all_sim_velocity_mm_s),
        sim_accel_mmpss=np.concatenate(all_sim_accel_mmpss),
    )
    print(f"\nSaved simulation data file: {filepath}")
    print(f"Total trials: {len(trial_n_samples)}/{total_trials}")
    print(f"Total samples: {sum(trial_n_samples)}")

    # Print summary deviation statistics across all trials
    all_pos_motor = np.concatenate(all_position_um).astype(np.float64)
    all_pos_sim = np.concatenate(all_sim_position_um)
    pos_deviation = all_pos_motor - all_pos_sim
    print(f"\nTrajectory deviation summary (motor - sim):")
    print(f"  Position (µm):  mean={np.mean(pos_deviation):+.1f}, "
          f"std={np.std(pos_deviation):.1f}, "
          f"RMSE={np.sqrt(np.mean(pos_deviation**2)):.1f}, "
          f"max|dev|={np.max(np.abs(pos_deviation)):.0f}")

    return trial_n_samples


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 60)
    print("Simulation-based friction data collection")
    print("=" * 60)

    print(f"\nSimulation parameters:")
    print(f"  mass_cart (per cube): {SIM_PARAMS['mass_cart']} kg  "
          f"(total cart = {2 * SIM_PARAMS['mass_cart']} kg)")
    print(f"  mass_bob:            {SIM_PARAMS['mass_bob']} kg")
    print(f"  pendulum_length:     {SIM_PARAMS['pendulum_length']} m")
    print(f"  max_episode_steps:   {SIM_PARAMS['max_episode_steps']}")
    print(f"  theta_inits:         {len(THETA_INITS)} angles "
          f"({np.degrees(THETA_INITS[0]):.0f}° to "
          f"{np.degrees(THETA_INITS[-1]):.0f}°)")
    print(f"  start_positions:     {DEFAULT_START_POSITIONS_UM} um")

    print(f"\nMotor parameters (from calibration_params.yaml):")
    print(f"  motor range:         {MOTOR_MIN_UM} - {MOTOR_MAX_UM} um")
    print(f"  motor center:        {MOTOR_CENTER_UM} um")
    print(f"  mass_shaft:          {PARAMS['mass_shaft_kg']} kg")
    print(f"  output directory:    {DATA_DIR}/")

    total_pend = len(THETA_INITS) * len(DEFAULT_START_POSITIONS_UM)
    print(f"\nPendulum-drop trajectories: {total_pend}")

    print(f"\nOptions:")
    print(f"  1) Generate pendulum-drop trajectories only (no motor)")
    print(f"  2) Generate pendulum-drop trajectories and collect motor data")
    print(f"  3) Generate sinusoidal trajectories only (no motor)")
    print(f"  4) Generate sinusoidal trajectories and collect motor data")
    print(f"  5) Load existing trajectories and collect motor data")
    print(f"  6) Replay with friction compensation from a previous run")
    choice = input("Select [1-6]: ").strip() or "4"

    # Defaults for the standard collection path; overridden by option 6.
    output_filename = "calibration_simulation_friction.npz"
    force_safety_limit_mn = FORCE_SAFETY_LIMIT_MN

    if choice in ("1", "2"):
        trajectories = generate_trajectories()

        os.makedirs(DATA_DIR, exist_ok=True)
        traj_path = os.path.join(DATA_DIR, "simulation_trajectories.npz")
        save_trajectories(trajectories, traj_path)

        if choice == "1":
            print("\nDone (trajectories only, no motor data collected).")
            return

    elif choice in ("3", "4"):
        n_input = input("\nNumber of sinusoidal trajectories [20]: ").strip()
        n_sin = int(n_input) if n_input else 20
        seed_input = input("Random seed [123]: ").strip()
        seed = int(seed_input) if seed_input else 123

        trajectories = generate_sinusoidal_trajectories(
            n_trajectories=n_sin, seed=seed,
        )

        os.makedirs(DATA_DIR, exist_ok=True)
        traj_path = os.path.join(DATA_DIR, "simulation_trajectories.npz")
        save_trajectories(trajectories, traj_path)

        if choice == "3":
            print("\nDone (trajectories only, no motor data collected).")
            return

    elif choice == "5":
        traj_path = input("Path to trajectories .npz file: ").strip()
        if not os.path.isfile(traj_path):
            print(f"File not found: {traj_path}")
            return
        trajectories = load_trajectories(traj_path)

    elif choice == "6":
        aug_path = input(
            "Path to augmented friction-estimate npz "
            "(e.g. calibration_simulation_friction_with_estimate.npz "
            "for iter 0->1, or _compensated_with_estimate.npz for "
            "iter 1->2): "
        ).strip()
        if not os.path.isfile(aug_path):
            print(f"File not found: {aug_path}")
            return

        # Detect the source iteration from the filename, then prompt the
        # user to confirm or override the next-iteration number.
        source_iter = detect_iteration_from_augmented_filename(aug_path)
        if source_iter is None:
            print(
                f"Could not auto-detect iteration from filename. "
                f"Please enter the iteration of the source file (the one "
                f"you're loading): "
            )
            try:
                source_iter = int(input("  Source iteration: ").strip())
            except ValueError:
                print("Invalid iteration; aborting.")
                return
        next_iter = source_iter + 1
        prompt = (
            f"\nAuto-detected source iteration {source_iter}. "
            f"This run will produce iteration {next_iter}. "
            f"Press Enter to accept, or enter a different iteration number: "
        )
        user_iter = input(prompt).strip()
        if user_iter:
            try:
                next_iter = int(user_iter)
            except ValueError:
                print("Invalid iteration; aborting.")
                return
        if next_iter < 1:
            print(f"Iteration must be >= 1 (got {next_iter}); aborting.")
            return

        # ILC tuning: gain scales the residual before adding it to the
        # next iteration's command; smoothing window low-passes the
        # residual to suppress per-sample noise.
        gain_input = input(
            "\nLearning gain (default 0.5; 1.0 = full residual; "
            "lower = more conservative): "
        ).strip()
        try:
            learning_gain = float(gain_input) if gain_input else 0.5
        except ValueError:
            print("Invalid learning gain; aborting.")
            return
        if learning_gain <= 0.0:
            print(f"Learning gain must be > 0 (got {learning_gain}); "
                  f"aborting.")
            return

        smoothing_input = input(
            "Residual smoothing window in samples "
            "(default 5; 1 disables smoothing): "
        ).strip()
        try:
            smoothing_window = int(smoothing_input) if smoothing_input else 5
        except ValueError:
            print("Invalid smoothing window; aborting.")
            return
        if smoothing_window < 1:
            print(f"Smoothing window must be >= 1 (got {smoothing_window}); "
                  f"aborting.")
            return

        # Look for simulation_trajectories.npz alongside the augmented npz.
        source_dir = os.path.dirname(os.path.abspath(aug_path))
        ref_path = os.path.join(source_dir, "simulation_trajectories.npz")
        if not os.path.isfile(ref_path):
            ref_path = input(
                f"simulation_trajectories.npz not found in {source_dir}.\n"
                f"  Enter path to simulation_trajectories.npz: "
            ).strip()
            if not os.path.isfile(ref_path):
                print(f"File not found: {ref_path}")
                return

        trajectories = load_compensated_trajectories(
            aug_path, ref_path,
            learning_gain=learning_gain,
            smoothing_window=smoothing_window,
        )
        if not trajectories:
            print("No compensated trajectories built; aborting.")
            return

        # Output filename and tighter safety limit, both tagged with iter.
        output_filename = shaft_filename_for_iteration(next_iter)
        force_safety_limit_mn = FORCE_SAFETY_LIMIT_COMPENSATED_MN
        print(f"\nWriting iteration-{next_iter} shaft data to: "
              f"{output_filename}")

        # Copy simulation_trajectories.npz forward into the new run's DATA_DIR
        # so compare_simulation_to_shaft.py can find it without flags.
        os.makedirs(DATA_DIR, exist_ok=True)
        forwarded_ref = os.path.join(DATA_DIR, "simulation_trajectories.npz")
        if os.path.abspath(ref_path) != os.path.abspath(forwarded_ref):
            import shutil
            shutil.copy2(ref_path, forwarded_ref)
            print(f"Copied {ref_path} → {forwarded_ref}")

    else:
        print(f"Invalid choice: {choice}")
        return

    # --- Connect to motor ---
    serial_port = input(
        f"\nEnter serial port [{DEFAULT_SERIAL_PORT}]: "
    ).strip()
    if not serial_port:
        serial_port = DEFAULT_SERIAL_PORT

    print(f"Connecting to motor on {serial_port}...")
    serial = SerialFTDI(latency_ms=1)
    clock = ChronoClock()
    motor = Actuator(serial, clock, "OrcaMotor", 1)

    motor.open_serial_port(serial_port, BAUD_RATE, INTERFRAME_DELAY)
    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()

    print("Motor connected.")

    input("\nPress Enter to begin data collection...")

    try:
        collect_simulation_data(
            motor, trajectories,
            output_filename=output_filename,
            force_safety_limit_mn=force_safety_limit_mn,
        )
    finally:
        print("\nShutting down motor...")
        try:
            motor.set_mode(MotorMode.SleepMode)
            motor.disable_stream()
            motor.close_serial_port()
        except Exception as e:
            print(f"  Warning during cleanup: {e}")

        os.makedirs(DATA_DIR, exist_ok=True)
        params_copy_path = os.path.join(DATA_DIR, "calibration_params.yaml")
        with open(params_copy_path, "w") as f:
            yaml.dump(PARAMS, f, default_flow_style=False, sort_keys=False)
        print(f"Saved parameters snapshot: {params_copy_path}")
        print("Done.")


if __name__ == "__main__":
    main()
