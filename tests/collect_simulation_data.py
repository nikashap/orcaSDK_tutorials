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

FORCE_SAFETY_LIMIT_MN = 30000

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
THETA_INITS = np.arange(-5 * np.pi / 6, np.pi / 2 + np.pi / 6, np.pi / 6)

# Starting motor positions (um) — spread across range, avoiding limits
DEFAULT_START_POSITIONS_UM = [150000, 305000, 450000]


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
            })

    env.close()
    return trajectories


def save_trajectories(trajectories, filepath):
    """Save pre-computed trajectories to .npz for later replay or inspection."""
    n_traj = len(trajectories)
    traj_n_steps = np.array([t["n_steps"] for t in trajectories], dtype=np.int32)
    traj_boundaries = np.concatenate([[0], np.cumsum(traj_n_steps)])

    np.savez(
        filepath,
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
        # Concatenated per-step arrays
        force_commands_mN=np.concatenate([t["force_commands_mN"] for t in trajectories]),
        sim_cart_x=np.concatenate([t["sim_cart_x"] for t in trajectories]),
        sim_cart_xdot=np.concatenate([t["sim_cart_xdot"] for t in trajectories]),
        sim_cart_xddot=np.concatenate([t["sim_cart_xddot"] for t in trajectories]),
        sim_pend_theta=np.concatenate([t["sim_pend_theta"] for t in trajectories]),
        sim_pend_thetadot=np.concatenate([t["sim_pend_thetadot"] for t in trajectories]),
    )

    global_max = max(np.max(np.abs(t["force_commands_mN"])) for t in trajectories)
    print(f"\nSaved {n_traj} trajectories to {filepath}")
    print(f"  Total steps: {traj_n_steps.sum()}")
    print(f"  Max |force_command|: {global_max:.0f} mN"
          f"{'  *** EXCEEDS SAFETY LIMIT ***' if global_max > FORCE_SAFETY_LIMIT_MN else ''}")


def load_trajectories(filepath):
    """Load pre-computed trajectories from .npz."""
    d = np.load(filepath)
    n_traj = int(d["n_trajectories"])
    boundaries = d["traj_boundaries"]

    trajectories = []
    for i in range(n_traj):
        lo, hi = int(boundaries[i]), int(boundaries[i + 1])
        trajectories.append({
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
        })

    print(f"Loaded {n_traj} trajectories from {filepath}")
    return trajectories


# ---------------------------------------------------------------------------
# Motor data collection
# ---------------------------------------------------------------------------

def run_simulation_trial(motor, trajectory):
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
        F_cmd_clamped = int(max(-FORCE_SAFETY_LIMIT_MN,
                                min(FORCE_SAFETY_LIMIT_MN, F_cmd)))
        motor.set_streamed_force_mN(F_cmd_clamped)

        motor.run()
        t1 = time.perf_counter()

        stream_data = motor.get_stream_data()
        pos_um = stream_data.position
        force_mN = stream_data.force

        accel_mmpss = read_raw_acceleration(motor)
        t2 = time.perf_counter()

        t_stream_list.append(t1)
        position_list.append(pos_um)
        force_list.append(force_mN)
        t_accel_list.append(t2)
        accel_list.append(accel_mmpss)
        force_cmd_list.append(F_cmd_clamped)

        # Store corresponding simulation cart state at this timestep
        # sim_cart_x is in metres; convert to µm for direct comparison with motor
        sim_position_um_list.append(
            sim_cart_x_arr[i] * 1e6 + MOTOR_CENTER_UM
        )
        # sim_cart_xdot is in m/s; convert to mm/s for comparison
        sim_velocity_mm_s_list.append(
            sim_cart_xdot_arr[i] * 1e3
        )
        # sim_cart_xddot is in m/s²; convert to mm/s² for comparison
        sim_accel_mmpss_list.append(
            sim_cart_xddot_arr[i] * 1e3
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
        "force_commanded_mN": np.array(force_cmd_list, dtype=np.int32),
        # Corresponding simulation cart state (same units as motor data)
        "sim_position_um": np.array(sim_position_um_list, dtype=np.float64),
        "sim_velocity_mm_s": np.array(sim_velocity_mm_s_list, dtype=np.float64),
        "sim_accel_mmpss": np.array(sim_accel_mmpss_list, dtype=np.float64),
    }


def collect_simulation_data(motor, trajectories):
    """Run all pre-computed trajectories on the motor and save data.

    Output npz has per-sample force_commanded_mN (same as rampdown/sinusoid
    format) so analyze_friction.py can process it directly.  Also includes
    sim_position_um, sim_velocity_mm_s, sim_accel_mmpss for trajectory
    deviation analysis.
    """
    os.makedirs(DATA_DIR, exist_ok=True)

    all_t_stream = []
    all_position_um = []
    all_force_mN = []
    all_t_accel = []
    all_accel_mmpss = []
    all_force_commanded_mN = []

    # Simulation reference trajectory arrays
    all_sim_position_um = []
    all_sim_velocity_mm_s = []
    all_sim_accel_mmpss = []

    trial_n_samples = []
    trial_theta_init = []
    trial_start_position_um = []

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

            print(f"\n--- Simulation trial {trial_num}/{total_trials}: "
                  f"theta_init={np.degrees(theta):+.1f}°, "
                  f"start_pos={start_pos} um, "
                  f"{n_steps} force steps ---")

            print("  Auto-zeroing...")
            error = auto_zero_motor(motor)
            if error.value != 0:
                print(f"  WARNING: Auto-zero returned error {error.value}, "
                      f"skipping.")
                continue

            motor.enable_stream()
            time.sleep(0.3)

            print(f"  Replaying trajectory...")
            trial_data = run_simulation_trial(motor, traj)

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
            all_accel_mmpss.append(trial_data["accel_mmpss"])
            all_force_commanded_mN.append(trial_data["force_commanded_mN"])

            all_sim_position_um.append(trial_data["sim_position_um"])
            all_sim_velocity_mm_s.append(trial_data["sim_velocity_mm_s"])
            all_sim_accel_mmpss.append(trial_data["sim_accel_mmpss"])

            trial_n_samples.append(n_samples)
            trial_theta_init.append(theta)
            trial_start_position_um.append(start_pos)

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

    filepath = os.path.join(DATA_DIR, "calibration_simulation_friction.npz")

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
        # Concatenated per-sample arrays — motor measured
        t_stream=np.concatenate(all_t_stream),
        position_um=np.concatenate(all_position_um),
        force_mN=np.concatenate(all_force_mN),
        force_commanded_mN=np.concatenate(all_force_commanded_mN),
        t_accel=np.concatenate(all_t_accel),
        accel_mmpss=np.concatenate(all_accel_mmpss),
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

    total_traj = len(THETA_INITS) * len(DEFAULT_START_POSITIONS_UM)
    print(f"\nTotal trajectories: {total_traj}")

    print(f"\nOptions:")
    print(f"  1) Generate trajectories only (no motor needed)")
    print(f"  2) Generate trajectories and collect motor data")
    print(f"  3) Load existing trajectories and collect motor data")
    choice = input("Select [1/2/3]: ").strip() or "2"

    if choice in ("1", "2"):
        trajectories = generate_trajectories()

        os.makedirs(DATA_DIR, exist_ok=True)
        traj_path = os.path.join(DATA_DIR, "simulation_trajectories.npz")
        save_trajectories(trajectories, traj_path)

        if choice == "1":
            print("\nDone (trajectories only, no motor data collected).")
            return

    elif choice == "3":
        traj_path = input("Path to trajectories .npz file: ").strip()
        if not os.path.isfile(traj_path):
            print(f"File not found: {traj_path}")
            return
        trajectories = load_trajectories(traj_path)

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
        collect_simulation_data(motor, trajectories)
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
