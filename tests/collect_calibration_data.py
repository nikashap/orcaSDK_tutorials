#!/usr/bin/env python
"""
collect_calibration_data.py

Collect data from the ORCA motor for estimating shaft mass and friction.
All parameters are read from calibration_params.yaml.

Procedures:
  1) Dynamic friction estimation — constant force trials, saved to one master .npz
  2) Static friction estimation — force ramp, saved to a separate .npz
  3) Both (static first, then dynamic)
  4) Ramp-down — low-force / coasting friction from running start
  5) All (static, then dynamic, then ramp-down)

Master file layout (mass estimation):
  - metadata fields: mass_shaft_kg, motor_min_um, motor_max_um,
    experiment_min_pos_um, experiment_max_pos_um, force_levels_mN,
    iterations_per_force, n_trials, trial_boundaries
  - per-sample arrays (concatenated across all trials):
    t_stream, position_um, force_mN, t_accel, accel_mmpss
  - per-trial arrays:
    trial_force_commanded_mN, trial_n_samples

Usage:
  python collect_calibration_data.py
"""

from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MotorMode
import pyorcasdk.orca_registers as orca_reg
from datetime import datetime
import time
import numpy as np
import os
import yaml


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
EXPERIMENT_MID_POS_UM = (EXPERIMENT_MIN_POS_UM + EXPERIMENT_MAX_POS_UM) // 2
STATIC_NUM_TEST_POINTS = PARAMS.get("static_num_test_points", 5)

AUTO_ZERO_FORCE_N = PARAMS["auto_zero_force_n"]
AUTO_ZERO_SPEED_MMPS = PARAMS["auto_zero_speed_mmps"]

BAUD_RATE = PARAMS["baud_rate"]
INTERFRAME_DELAY = PARAMS["interframe_delay_us"]
DEFAULT_SERIAL_PORT = PARAMS["default_serial_port"]

DEFAULT_FORCES_MN = PARAMS["force_levels_mN"]
DEFAULT_ITERS_PER_FORCE = PARAMS["iterations_per_force"]

# Resolve data directory relative to the script location
# NOTE: I made a more specific directoy for the date at which data is collected
DATE_STR = datetime.now().strftime('%y_%m_%d-%H_%M_%S')
DATA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, PARAMS["data_dir"], DATE_STR))

# Static friction parameters
FRICTION_FORCE_START_MN = PARAMS["friction_force_start_mN"]
FRICTION_FORCE_STEP_MN = PARAMS["friction_force_step_mN"]
FRICTION_FORCE_MAX_MN = PARAMS["friction_force_max_mN"]
FRICTION_SAMPLES_PER_STEP = PARAMS["friction_samples_per_step"]
FRICTION_SETTLE_TIME_S = PARAMS["friction_settle_time_s"]
FRICTION_ACCEL_THRESHOLD_MMPSS = PARAMS["friction_accel_threshold_mmpss"]
FRICTION_POS_CHANGE_THRESHOLD_UM = PARAMS["friction_pos_change_threshold_um"]

# Ramp-down parameters
RAMPDOWN_STARTUP_FORCE_MN = PARAMS.get("rampdown_startup_force_mN", 12000)
RAMPDOWN_TARGET_FORCES_MN = PARAMS.get("rampdown_target_forces_mN", [0])
RAMPDOWN_SWITCH_POSITIONS_UM = PARAMS.get("rampdown_switch_positions_um", [370000])
RAMPDOWN_STARTING_POSITIONS_UM = PARAMS.get("rampdown_starting_positions_um",
                                            [EXPERIMENT_MIN_POS_UM])
RAMPDOWN_ITERATIONS_PER_COMBO = PARAMS.get("rampdown_iterations_per_combo", 3)


def auto_zero_motor(motor):
    """
    Run the ORCA auto-zero procedure to home the shaft.
    Returns the error status after completion.
    """
    motor.set_mode(MotorMode.SleepMode)
    time.sleep(0.1)

    motor.write_register_blocking(orca_reg.ZERO_MODE, orca_reg.ZERO_MODE_AUTO_ZERO_ENABLED)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_FORCE_N, AUTO_ZERO_FORCE_N)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_SPEED_MMPS, AUTO_ZERO_SPEED_MMPS)
    motor.write_register_blocking(orca_reg.AUTO_ZERO_EXIT_MODE, MotorMode.SleepMode)

    motor.set_mode(MotorMode.AutoZeroMode)
    motor.clear_errors()

    time.sleep(6) #Autozero needs to complete within 6 seconds

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
    """Run through the motor command stream to clear any potentially stale frames"""
    for _ in range(frames):
        motor.run()
        time.sleep(sleeptime)

def move_to_position(motor, target_pos_um, kick_force_mag_mN=10000, duration_s=0.2, pos_tolerance_um=200):
    """
    Move the motor to ``target_pos_um`` by streaming using ``kick_force_mag_mN`` to
    get the motor close to the target_pos_um, then command the shaft to move
    to target_pos_um and hold until the shaft is within ``pos_tolerance_um``
    of the target.

    The motor must already have streaming enabled before calling this function.
    On return the motor is placed back into SleepMode.
    """

    motor.set_mode(MotorMode.ForceMode)
    passed_target = False
    # Infer direction motor needs to travel
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
        motor.set_streamed_force_mN(kick_force_mag_mN*force_direction)
    
    motor.set_mode(MotorMode.PositionMode)
    advance_motor_stream(motor)
    stream_data = motor.get_stream_data()
    start_pos_um = stream_data.position

    # Command motor to move at consistent trajectory to target_pos
    motor.set_streamed_position_um(start_pos_um)
    motor.run()

    n_steps = max(int(duration_s / 0.005), 1)  # ~5 ms per step
    dt = duration_s / n_steps

    print(f"  Moving {start_pos_um} → {target_pos_um} µm over {duration_s:.1f} s "
          f"({n_steps} steps)...")

    for i in range(1, n_steps + 1):
        frac = i / n_steps
        interp_pos = int(start_pos_um + frac * (target_pos_um - start_pos_um))
        motor.set_streamed_position_um(interp_pos)
        motor.run()
        time.sleep(dt)

    # Hold at target until within tolerance
    while True:
        motor.run()
        stream_data = motor.get_stream_data()
        if abs(stream_data.position - target_pos_um) <= pos_tolerance_um:
            break
        time.sleep(0.005)

    motor.set_mode(MotorMode.SleepMode)
    print(f"  Reached position ({stream_data.position} µm).")


def run_single_trial(motor, force_command_mN):
    """
    Run one trial: command a constant force and record data until the shaft
    reaches the end of its travel.

    For positive forces the shaft travels toward EXPERIMENT_MAX_POS_UM.
    For negative forces the shaft travels toward EXPERIMENT_MIN_POS_UM.

    Returns dict with arrays:
        t_stream, position_um, force_mN, t_accel, accel_mmpss, force_commanded_mN
    """
    move_to_position(motor, target_pos_um=EXPERIMENT_MIN_POS_UM)
    t_stream_list = []
    position_list = []
    force_list = []
    t_accel_list = []
    accel_list = []

    motor.set_mode(MotorMode.ForceMode)
    motor.set_streamed_force_mN(force_command_mN)

    reached_end = False

    while not reached_end:
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

        if force_command_mN >= 0 and pos_um >= EXPERIMENT_MAX_POS_UM:
            reached_end = True
        elif force_command_mN < 0 and pos_um <= EXPERIMENT_MIN_POS_UM:
            reached_end = True

        if stream_data.errors:
            print(f"    Motor error: {stream_data.errors}")
            break

    motor.set_mode(MotorMode.SleepMode)

    return {
        "t_stream": np.array(t_stream_list, dtype=np.float64),
        "position_um": np.array(position_list, dtype=np.int32),
        "force_mN": np.array(force_list, dtype=np.int32),
        "t_accel": np.array(t_accel_list, dtype=np.float64),
        "accel_mmpss": np.array(accel_list, dtype=np.int32),
        "force_commanded_mN": force_command_mN,
    }


def collect_data(motor,
                 iters_per_force=DEFAULT_ITERS_PER_FORCE,
                 forces_mN_list=None):
    """
    Collect data across multiple force levels and iterations.
    Saves everything into one master .npz file with metadata prepended.
    """
    if forces_mN_list is None:
        forces_mN_list = DEFAULT_FORCES_MN

    os.makedirs(DATA_DIR, exist_ok=True)

    total_trials = iters_per_force * len(forces_mN_list)

    # Accumulators for concatenated per-sample arrays
    all_t_stream = []
    all_position_um = []
    all_force_mN = []
    all_t_accel = []
    all_accel_mmpss = []

    # Per-trial metadata
    trial_force_commanded = []
    trial_n_samples = []

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    trial_num = 0

    try:
        for force_mN in forces_mN_list:
            if force_mN > FORCE_SAFETY_LIMIT_MN:
                raise ValueError(f"Force command {force_mN} mN exceeds safety limit of {FORCE_SAFETY_LIMIT_MN} mN")
            
            for iteration in range(iters_per_force):
                trial_num += 1
                print(f"\n--- Trial {trial_num}/{total_trials}: "
                      f"force={force_mN} mN, iter={iteration+1}/{iters_per_force} ---")

                if force_mN >= 0:
                    # Positive force: auto-zero moves shaft to min position,
                    # then force drives it toward max
                    print("  Auto-zeroing...")
                    error = auto_zero_motor(motor)
                    if error.value != 0:
                        print(f"  WARNING: Auto-zero returned error {error.value}, skipping trial.")
                        continue

                    motor.enable_stream()
                    time.sleep(0.3)
                else:
                    # Negative force: smoothly move to experiment_max_pos_um,
                    # then force drives it toward min
                    move_to_position(motor, EXPERIMENT_MAX_POS_UM)
                    time.sleep(0.3)

                print(f"  Collecting data (force={force_mN} mN)...")
                trial_data = run_single_trial(motor, force_mN)

                n_samples = len(trial_data["t_stream"])
                print(f"  Recorded {n_samples} samples, "
                      f"position range: {trial_data['position_um'][0]} - "
                      f"{trial_data['position_um'][-1]} um")

                # Append to master arrays
                all_t_stream.append(trial_data["t_stream"])
                all_position_um.append(trial_data["position_um"])
                all_force_mN.append(trial_data["force_mN"])
                all_t_accel.append(trial_data["t_accel"])
                all_accel_mmpss.append(trial_data["accel_mmpss"])

                trial_force_commanded.append(force_mN)
                trial_n_samples.append(n_samples)

                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except ValueError as e:
        print(f"\nExcessive force command: {e}")
    except Exception as e:
        print(f"\nError during collection: {e}")
        raise
    finally:
        motor.set_mode(MotorMode.SleepMode)

    # --- Save everything to one master file ---
    filepath = os.path.join(DATA_DIR, "calibration_dynamic_friction.npz")

    # Compute trial boundaries (cumulative sum of n_samples)
    trial_n_samples_arr = np.array(trial_n_samples, dtype=np.int32)
    trial_boundaries = np.concatenate([[0], np.cumsum(trial_n_samples_arr)])

    np.savez(
        filepath,
        # Metadata
        procedure_type="constant",
        mass_shaft_kg=PARAMS["mass_shaft_kg"],
        motor_min_um=MOTOR_MIN_UM,
        motor_max_um=MOTOR_MAX_UM,
        experiment_min_pos_um=PARAMS["experiment_min_pos_um"],
        experiment_max_pos_um=EXPERIMENT_MAX_POS_UM,
        force_levels_mN=np.array(forces_mN_list, dtype=np.int32),
        iterations_per_force=iters_per_force,
        n_trials=len(trial_n_samples),
        # Per-trial arrays
        trial_force_commanded_mN=np.array(trial_force_commanded, dtype=np.int32),
        trial_n_samples=trial_n_samples_arr,
        trial_boundaries=trial_boundaries,
        # Concatenated per-sample arrays
        t_stream=np.concatenate(all_t_stream) if all_t_stream else np.array([], dtype=np.float64),
        position_um=np.concatenate(all_position_um) if all_position_um else np.array([], dtype=np.int32),
        force_mN=np.concatenate(all_force_mN) if all_force_mN else np.array([], dtype=np.int32),
        t_accel=np.concatenate(all_t_accel) if all_t_accel else np.array([], dtype=np.float64),
        accel_mmpss=np.concatenate(all_accel_mmpss) if all_accel_mmpss else np.array([], dtype=np.int32),
    )
    print(f"\nSaved master data file: {filepath}")
    print(f"Total trials: {len(trial_n_samples)}/{total_trials}")
    print(f"Total samples: {sum(trial_n_samples)}")

    return trial_n_samples


# ---------------------------------------------------------------------------
# Ramp-down (dynamic friction at low forces)
# ---------------------------------------------------------------------------

def run_rampdown_trial(motor, startup_force_mN, target_force_mN,
                       switch_pos_um, start_pos_um):
    """
    Run one ramp-down trial: move the shaft to start_pos_um, command a high
    startup force to get the shaft moving, then switch to a lower target force
    once the shaft passes switch_pos_um.  Record data for the entire trial.

    For positive startup force the shaft travels toward EXPERIMENT_MAX_POS_UM.
    For negative startup force the shaft travels toward EXPERIMENT_MIN_POS_UM.

    Returns dict with arrays:
        t_stream, position_um, force_mN, t_accel, accel_mmpss,
        force_commanded_mN (per-sample, changes at switch point),
        switch_index (sample index where force switched)
    """
    move_to_position(motor, target_pos_um=start_pos_um)

    t_stream_list = []
    position_list = []
    force_list = []
    t_accel_list = []
    accel_list = []
    force_cmd_list = []

    motor.set_mode(MotorMode.ForceMode)
    motor.set_streamed_force_mN(startup_force_mN)

    moving_positive = startup_force_mN >= 0

    switched = False
    switch_index = -1
    consecutive_slow = 0
    prev_pos = None

    while True:
        motor.run()
        t1 = time.perf_counter()

        stream_data = motor.get_stream_data()
        pos_um = stream_data.position
        force_mN = stream_data.force

        accel_mmpss = read_raw_acceleration(motor)
        t2 = time.perf_counter()

        if not switched:
            reached_switch = (pos_um >= switch_pos_um if moving_positive
                              else pos_um <= switch_pos_um)
            if reached_switch:
                motor.set_streamed_force_mN(target_force_mN)
                switched = True
                switch_index = len(t_stream_list)
                print(f"    Switched to {target_force_mN} mN at position {pos_um} um "
                      f"(sample {switch_index})")

        current_cmd = target_force_mN if switched else startup_force_mN

        t_stream_list.append(t1)
        position_list.append(pos_um)
        force_list.append(force_mN)
        t_accel_list.append(t2)
        accel_list.append(accel_mmpss)
        force_cmd_list.append(current_cmd)

        if moving_positive and pos_um >= EXPERIMENT_MAX_POS_UM:
            break
        if not moving_positive and pos_um <= EXPERIMENT_MIN_POS_UM:
            break

        if switched and prev_pos is not None:
            if abs(pos_um - prev_pos) < 5:
                consecutive_slow += 1
            else:
                consecutive_slow = 0
            if consecutive_slow > 50:
                print(f"    Shaft stopped after switch (position ~{pos_um} um)")
                break

        prev_pos = pos_um

        if stream_data.errors:
            print(f"    Motor error: {stream_data.errors}")
            break

    motor.set_mode(MotorMode.SleepMode)

    return {
        "t_stream": np.array(t_stream_list, dtype=np.float64),
        "position_um": np.array(position_list, dtype=np.int32),
        "force_mN": np.array(force_list, dtype=np.int32),
        "t_accel": np.array(t_accel_list, dtype=np.float64),
        "accel_mmpss": np.array(accel_list, dtype=np.int32),
        "force_commanded_mN": np.array(force_cmd_list, dtype=np.int32),
        "switch_index": switch_index,
    }


def collect_rampdown_data(motor,
                          startup_force_mN=RAMPDOWN_STARTUP_FORCE_MN,
                          target_forces_mN=None,
                          switch_positions_um=None,
                          starting_positions_um=None,
                          iters_per_combo=RAMPDOWN_ITERATIONS_PER_COMBO):
    """
    Collect ramp-down trials across all
    (target_force, switch_position, starting_position) combos.
    Saves everything into one master .npz file.

    Infeasible combos are skipped:
      - positive startup force + starting_pos > switch_pos
      - negative startup force + starting_pos < switch_pos
    """
    if target_forces_mN is None:
        target_forces_mN = RAMPDOWN_TARGET_FORCES_MN
    if switch_positions_um is None:
        switch_positions_um = RAMPDOWN_SWITCH_POSITIONS_UM
    if starting_positions_um is None:
        starting_positions_um = RAMPDOWN_STARTING_POSITIONS_UM

    # Enforce same-sign target forces
    signs = set(1 if f >= 0 else -1 for f in target_forces_mN)
    if len(signs) > 1:
        print("WARNING: rampdown_target_forces_mN contains both positive and "
              "negative values. All target forces should be the same sign.")

    os.makedirs(DATA_DIR, exist_ok=True)

    # Build combos, skipping infeasible ones
    combos = []
    for tf in target_forces_mN:
        for sp in switch_positions_um:
            for stp in starting_positions_um:
                if startup_force_mN >= 0 and stp > sp:
                    print(f"  Skipping infeasible combo: start={stp} > "
                          f"switch={sp} with positive startup force")
                    continue
                if startup_force_mN < 0 and stp < sp:
                    print(f"  Skipping infeasible combo: start={stp} < "
                          f"switch={sp} with negative startup force")
                    continue
                combos.append((tf, sp, stp))

    total_trials = len(combos) * iters_per_combo
    print(f"\nRamp-down: {len(combos)} feasible combos × "
          f"{iters_per_combo} iters = {total_trials} trials")

    all_t_stream = []
    all_position_um = []
    all_force_mN = []
    all_t_accel = []
    all_accel_mmpss = []
    all_force_commanded_mN = []

    trial_n_samples = []
    trial_target_force = []
    trial_switch_position = []
    trial_starting_position = []
    trial_switch_index = []

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    trial_num = 0

    try:
        for target_force, switch_pos, start_pos in combos:
            for iteration in range(iters_per_combo):
                trial_num += 1
                print(f"\n--- Ramp-down trial {trial_num}/{total_trials}: "
                      f"start={start_pos} um, "
                      f"startup={startup_force_mN} mN → target={target_force} mN "
                      f"at {switch_pos} um, iter={iteration+1}/{iters_per_combo} ---")

                print("  Auto-zeroing...")
                error = auto_zero_motor(motor)
                if error.value != 0:
                    print(f"  WARNING: Auto-zero returned error {error.value}, skipping.")
                    continue

                motor.enable_stream()
                time.sleep(0.3)

                trial_data = run_rampdown_trial(
                    motor, startup_force_mN, target_force, switch_pos, start_pos
                )

                n_samples = len(trial_data["t_stream"])
                print(f"  Recorded {n_samples} samples, "
                      f"switch at sample {trial_data['switch_index']}, "
                      f"position range: {trial_data['position_um'][0]} - "
                      f"{trial_data['position_um'][-1]} um")

                all_t_stream.append(trial_data["t_stream"])
                all_position_um.append(trial_data["position_um"])
                all_force_mN.append(trial_data["force_mN"])
                all_t_accel.append(trial_data["t_accel"])
                all_accel_mmpss.append(trial_data["accel_mmpss"])
                all_force_commanded_mN.append(trial_data["force_commanded_mN"])

                trial_n_samples.append(n_samples)
                trial_target_force.append(target_force)
                trial_switch_position.append(switch_pos)
                trial_starting_position.append(start_pos)
                trial_switch_index.append(trial_data["switch_index"])

                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\nError during collection: {e}")
        raise
    finally:
        motor.set_mode(MotorMode.SleepMode)

    filepath = os.path.join(DATA_DIR, "calibration_rampdown_friction.npz")

    trial_n_samples_arr = np.array(trial_n_samples, dtype=np.int32)
    trial_boundaries = np.concatenate([[0], np.cumsum(trial_n_samples_arr)])

    np.savez(
        filepath,
        # Metadata
        procedure_type="rampdown",
        mass_shaft_kg=PARAMS["mass_shaft_kg"],
        motor_min_um=MOTOR_MIN_UM,
        motor_max_um=MOTOR_MAX_UM,
        experiment_min_pos_um=PARAMS["experiment_min_pos_um"],
        experiment_max_pos_um=EXPERIMENT_MAX_POS_UM,
        startup_force_mN=startup_force_mN,
        target_forces_mN=np.array(target_forces_mN, dtype=np.int32),
        switch_positions_um=np.array(switch_positions_um, dtype=np.int32),
        starting_positions_um=np.array(starting_positions_um, dtype=np.int32),
        iterations_per_combo=iters_per_combo,
        n_trials=len(trial_n_samples),
        # Per-trial arrays
        trial_target_force_mN=np.array(trial_target_force, dtype=np.int32),
        trial_switch_position_um=np.array(trial_switch_position, dtype=np.int32),
        trial_starting_position_um=np.array(trial_starting_position, dtype=np.int32),
        trial_switch_index=np.array(trial_switch_index, dtype=np.int32),
        trial_n_samples=trial_n_samples_arr,
        trial_boundaries=trial_boundaries,
        # Concatenated per-sample arrays
        t_stream=np.concatenate(all_t_stream) if all_t_stream else np.array([], dtype=np.float64),
        position_um=np.concatenate(all_position_um) if all_position_um else np.array([], dtype=np.int32),
        force_mN=np.concatenate(all_force_mN) if all_force_mN else np.array([], dtype=np.int32),
        force_commanded_mN=np.concatenate(all_force_commanded_mN) if all_force_commanded_mN else np.array([], dtype=np.int32),
        t_accel=np.concatenate(all_t_accel) if all_t_accel else np.array([], dtype=np.float64),
        accel_mmpss=np.concatenate(all_accel_mmpss) if all_accel_mmpss else np.array([], dtype=np.int32),
    )
    print(f"\nSaved ramp-down data file: {filepath}")
    print(f"Total trials: {len(trial_n_samples)}/{total_trials}")
    print(f"Total samples: {sum(trial_n_samples)}")

    return trial_n_samples


# ---------------------------------------------------------------------------
# Static friction estimation
# ---------------------------------------------------------------------------

def run_static_friction_procedure(motor, force_sign=1):
    """
    Ramp commanded force upward in small steps from rest.
    At each step, take several acceleration and position readings.
    Detect the force at which the shaft begins to move (static friction breakaway).

    Parameters
    ----------
    motor : Actuator
    force_sign : int
        +1 for positive force ramp, -1 for negative force ramp.

    Detection criteria (either triggers a detection):
      1. Mean |acceleration| exceeds FRICTION_ACCEL_THRESHOLD_MMPSS
      2. Position changes by more than FRICTION_POS_CHANGE_THRESHOLD_UM

    Returns a dict with:
      - force_levels_mN, mean_accel_mmpss, std_accel_mmpss, pos_change_um
      - breakaway_force_mN
      - per-step raw samples (accels, positions)
    """
    motor.set_mode(MotorMode.ForceMode)

    force_levels = []
    mean_accels = []
    std_accels = []
    pos_changes = []
    all_step_accels = []
    all_step_positions = []
    all_step_forces_sensed = []
    all_step_times = []
    breakaway_force = None

    force_magnitude = FRICTION_FORCE_START_MN

    print(f"\n\n=== Command motor at incremental forces (sign={force_sign:+d}) ===\n\n")
    while force_magnitude <= FRICTION_FORCE_MAX_MN:
        force_mN = force_sign * force_magnitude
        motor.set_streamed_force_mN(force_mN)

        for _ in range(5):
            motor.run()
        time.sleep(FRICTION_SETTLE_TIME_S)

        accels = []
        positions = []
        forces_sensed = []
        t_samples = []
        for _ in range(FRICTION_SAMPLES_PER_STEP):
            motor.run()
            t1 = time.perf_counter()
            stream_data = motor.get_stream_data()
            accel = read_raw_acceleration(motor)
            accels.append(accel)
            positions.append(stream_data.position)
            forces_sensed.append(stream_data.force)
            t_samples.append(t1)

        accels = np.array(accels, dtype=np.float64)
        positions = np.array(positions, dtype=np.float64)
        forces_sensed = np.array(forces_sensed, dtype=np.float64)
        t_samples = np.array(t_samples, dtype=np.float64)

        mean_abs_accel = np.mean(np.abs(accels))
        std_abs_accel = np.std(np.abs(accels))
        pos_change = abs(positions[-1] - positions[0])

        force_levels.append(force_mN)
        mean_accels.append(mean_abs_accel)
        std_accels.append(std_abs_accel)
        pos_changes.append(pos_change)
        all_step_accels.append(accels)
        all_step_positions.append(positions)
        all_step_forces_sensed.append(forces_sensed)
        all_step_times.append(t_samples)

        moving_by_accel = mean_abs_accel > FRICTION_ACCEL_THRESHOLD_MMPSS
        moving_by_pos = pos_change > FRICTION_POS_CHANGE_THRESHOLD_UM

        mean_force_sensed = np.mean(forces_sensed)
        print(f"  Force={force_mN:6d} mN | "
              f"F_sensed={mean_force_sensed:8.1f} mN | "
              f"mean|a|={mean_abs_accel:8.1f} mm/s² | "
              f"std|a|={std_abs_accel:8.1f} mm/s² | "
              f"Δpos={pos_change:8.1f} µm | "
              f"{'*** MOVING ***' if (moving_by_accel or moving_by_pos) else ''}")

        if breakaway_force is None and (moving_by_accel or moving_by_pos):
            breakaway_force = force_mN

        if breakaway_force is not None and force_magnitude >= abs(breakaway_force) + FRICTION_FORCE_STEP_MN * 3:
            break

        force_magnitude += FRICTION_FORCE_STEP_MN

    motor.set_mode(MotorMode.SleepMode)

    return {
        "force_levels_mN": np.array(force_levels, dtype=np.int32),
        "mean_accel_mmpss": np.array(mean_accels, dtype=np.float64),
        "std_accel_mmpss": np.array(std_accels, dtype=np.float64),
        "pos_change_um": np.array(pos_changes, dtype=np.float64),
        "breakaway_force_mN": breakaway_force,
        "all_step_accels": all_step_accels,
        "all_step_positions": all_step_positions,
        "all_step_forces_sensed": all_step_forces_sensed,
        "all_step_times": all_step_times,
    }


def _save_static_friction_result(result, start_pos_um, label, filepath):
    """Pack one static friction result into a .npz file.

    Saves per-step raw samples as 2D arrays (n_steps x max_step_len), padded
    with NaN where steps have fewer samples. This includes accelerations,
    positions, sensed forces, and timestamps — consistent with how dynamic
    friction data is saved, so the same shift-based analysis can be applied.
    """
    max_step_len = max(len(a) for a in result["all_step_accels"])
    n_steps = len(result["all_step_accels"])
    step_accels_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    step_positions_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    step_forces_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    step_times_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    for i in range(n_steps):
        n = len(result["all_step_accels"][i])
        step_accels_2d[i, :n] = result["all_step_accels"][i]
        step_positions_2d[i, :n] = result["all_step_positions"][i]
        step_forces_2d[i, :n] = result["all_step_forces_sensed"][i]
        step_times_2d[i, :n] = result["all_step_times"][i]

    np.savez(
        filepath,
        start_pos_um=start_pos_um,
        force_levels_mN=result["force_levels_mN"],
        mean_accel_mmpss=result["mean_accel_mmpss"],
        std_accel_mmpss=result["std_accel_mmpss"],
        pos_change_um=result["pos_change_um"],
        breakaway_force_mN=result["breakaway_force_mN"] or 0,
        step_accels=step_accels_2d,
        step_positions=step_positions_2d,
        step_forces_sensed=step_forces_2d,
        step_times=step_times_2d,
        samples_per_step=FRICTION_SAMPLES_PER_STEP,
    )
    print(f"\n  Saved friction data ({label}): {filepath}")

    if result["breakaway_force_mN"] is not None:
        print(f"  >>> Breakaway force ({label}): "
              f"{result['breakaway_force_mN']} mN <<<")
    else:
        print(f"  WARNING: No breakaway detected ({label}) up to "
              f"±{FRICTION_FORCE_MAX_MN} mN")


def collect_static_friction_data(motor):
    """
    Estimate static friction at 7 shaft positions in [experiment_min_pos_um, experiment_max_pos_um]
    """
    os.makedirs(DATA_DIR, exist_ok=True)

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    print("\n=== Static Friction Estimation ===")
    print("  Auto-zeroing first...")
    error = auto_zero_motor(motor)
    if error.value != 0:
        print(f"  Auto-zero failed with error {error.value}")
        return None

    motor.enable_stream()
    time.sleep(0.1)

    # Define the three test points: (position, force_sign, label)
    test_points = np.linspace(EXPERIMENT_MIN_POS_UM,
                              EXPERIMENT_MAX_POS_UM,
                              STATIC_NUM_TEST_POINTS)
    
    test_points_params = []
    for i, tp in enumerate(test_points):
        # force_sign = -1 if tp > EXPERIMENT_MID_POS_UM else +1
        test_points_params.append((int(tp), +1, str(i)))
        test_points_params.append((int(tp), -1, str(i)))

    results = {}

    for target_pos_um, force_sign, label in test_points_params:
        print(f"\n--- Static friction at point {label} ; ({target_pos_um} µm, "
              f"force_sign={force_sign:+d}) ---")

        move_to_position(motor, target_pos_um)
        motor.enable_stream()
        print("Clear motor stream for a few frames...")
        time.sleep(3.0)
        advance_motor_stream(motor)

        print("  Ramping force to find breakaway point...")
        result = run_static_friction_procedure(motor, force_sign=force_sign)

        filepath = os.path.join(DATA_DIR, f"calibration_static_friction_{label}.npz")
        _save_static_friction_result(result, target_pos_um, label, filepath)

        results[label] = result

        # Re-enable stream for next move
        motor.enable_stream()
        time.sleep(0.3)

    return results


def main():
    serial_port = input(
        f"Enter serial port [{DEFAULT_SERIAL_PORT}]: "
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
    print(f"\nData collection parameters (from calibration_params.yaml):")
    print(f"  Force levels: {DEFAULT_FORCES_MN} mN")
    print(f"  Iterations per force: {DEFAULT_ITERS_PER_FORCE}")
    print(f"  Total trials: {DEFAULT_ITERS_PER_FORCE * len(DEFAULT_FORCES_MN)}")
    print(f"  Position window: {EXPERIMENT_MIN_POS_UM} - {EXPERIMENT_MAX_POS_UM} um")
    print(f"  Output directory: {DATA_DIR}/")

    print(f"\nProcedures:")
    print(f"  1) Dynamic friction estimation (constant force trials)")
    print(f"  2) Static friction estimation (force ramp)")
    print(f"  3) Both (static first, then dynamic)")
    print(f"  4) Ramp-down (low-force / coasting friction from running start)")
    print(f"  5) All (static, then dynamic, then ramp-down)")
    procedure = input("Select procedure [1/2/3/4/5]: ").strip() or "1"

    input("\nPress Enter to begin data collection...")

    try:
        if procedure in ("2", "3", "5"):
            collect_static_friction_data(motor)
        if procedure in ("1", "3", "5"):
            collect_data(motor)
        if procedure in ("4", "5"):
            collect_rampdown_data(motor)
    finally:
        print("\nShutting down motor...")
        try:
            motor.set_mode(MotorMode.SleepMode)
            motor.disable_stream()
            motor.close_serial_port()
        except Exception as e:
            print(f"  Warning during cleanup: {e}")

        # Save a copy of the parameters used for this experiment
        os.makedirs(DATA_DIR, exist_ok=True)
        params_copy_path = os.path.join(DATA_DIR, "calibration_params.yaml")
        with open(params_copy_path, "w") as f:
            yaml.dump(PARAMS, f, default_flow_style=False, sort_keys=False)
        print(f"Saved parameters snapshot: {params_copy_path}")
        print("Done.")

if __name__ == "__main__":
    main()
