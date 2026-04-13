#!/usr/bin/env python
"""
collect_calibration_data.py

Collect data from the ORCA motor for estimating shaft mass and friction.
All parameters are read from calibration_params.yaml.

Procedures:
  1) Mass estimation — constant force trials, all saved to one master .npz
  2) Static friction estimation — force ramp, saved to a separate .npz
  3) Both (friction first, then mass estimation)

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


def run_single_trial(motor, force_command_mN):
    """
    Run one trial: command a constant force and record data until the shaft
    reaches the end of its travel.

    For positive forces the shaft travels toward EXPERIMENT_MAX_POS_UM.
    For negative forces the shaft travels toward EXPERIMENT_MIN_POS_UM.

    Returns dict with arrays:
        t_stream, position_um, force_mN, t_accel, accel_mmpss, force_commanded_mN
    """
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
                    # Negative force: move to experiment_max_pos_um via
                    # PositionMode, then force drives it toward min
                    print(f"  Moving to start position ({EXPERIMENT_MAX_POS_UM} µm)...")
                    motor.set_mode(MotorMode.PositionMode)
                    motor.set_streamed_position_um(EXPERIMENT_MAX_POS_UM)
                    pos_tolerance_um = 200
                    while True:
                        motor.run()
                        stream_data = motor.get_stream_data()
                        if abs(stream_data.position - EXPERIMENT_MAX_POS_UM) <= pos_tolerance_um:
                            break
                        time.sleep(0.005)
                    motor.set_mode(MotorMode.SleepMode)
                    print(f"  Reached start position ({stream_data.position} µm).")
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
# Static friction estimation
# ---------------------------------------------------------------------------

def run_static_friction_procedure(motor):
    """
    Ramp commanded force upward in small steps from rest, with motor
    starting at MIN_POS_UM.
    At each step, take several acceleration and position readings.
    Detect the force at which the shaft begins to move (static friction breakaway).

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
    breakaway_force = None

    force_mN = FRICTION_FORCE_START_MN

    print("\n\n=== 2) Command motor at incremental forces ===\n\n")
    while force_mN <= FRICTION_FORCE_MAX_MN:
        motor.set_streamed_force_mN(force_mN)

        for _ in range(5):
            motor.run()
        time.sleep(FRICTION_SETTLE_TIME_S)

        accels = []
        positions = []
        for _ in range(FRICTION_SAMPLES_PER_STEP):
            motor.run()
            stream_data = motor.get_stream_data()
            accel = read_raw_acceleration(motor)
            accels.append(accel)
            positions.append(stream_data.position)

        accels = np.array(accels, dtype=np.float64)
        positions = np.array(positions, dtype=np.float64)

        mean_abs_accel = np.mean(np.abs(accels))
        std_abs_accel = np.std(np.abs(accels))
        pos_change = abs(positions[-1] - positions[0])

        force_levels.append(force_mN)
        mean_accels.append(mean_abs_accel)
        std_accels.append(std_abs_accel)
        pos_changes.append(pos_change)
        all_step_accels.append(accels)
        all_step_positions.append(positions)

        moving_by_accel = mean_abs_accel > FRICTION_ACCEL_THRESHOLD_MMPSS
        moving_by_pos = pos_change > FRICTION_POS_CHANGE_THRESHOLD_UM

        print(f"  Force={force_mN:6d} mN | "
              f"mean|a|={mean_abs_accel:8.1f} mm/s² | "
              f"std|a|={std_abs_accel:8.1f} mm/s² | "
              f"Δpos={pos_change:8.1f} µm | "
              f"{'*** MOVING ***' if (moving_by_accel or moving_by_pos) else ''}")

        if breakaway_force is None and (moving_by_accel or moving_by_pos):
            breakaway_force = force_mN

        if breakaway_force is not None and force_mN >= breakaway_force + FRICTION_FORCE_STEP_MN * 3:
            break

        force_mN += FRICTION_FORCE_STEP_MN

    motor.set_mode(MotorMode.SleepMode)

    return {
        "force_levels_mN": np.array(force_levels, dtype=np.int32),
        "mean_accel_mmpss": np.array(mean_accels, dtype=np.float64),
        "std_accel_mmpss": np.array(std_accels, dtype=np.float64),
        "pos_change_um": np.array(pos_changes, dtype=np.float64),
        "breakaway_force_mN": breakaway_force,
        "all_step_accels": all_step_accels,
        "all_step_positions": all_step_positions,
    }


def collect_static_friction_data(motor):
    """Run the static friction estimation procedure with auto-zero, save results."""
    os.makedirs(DATA_DIR, exist_ok=True)

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    print("\n=== Static Friction Estimation ===")
    print("  Auto-zeroing...")
    error = auto_zero_motor(motor)
    if error.value != 0:
        print(f"  Auto-zero failed with error {error.value}")
        return None

    motor.enable_stream()
    time.sleep(0.1)

    # Move to experiment_min_pos_um before starting the friction ramp
    target_pos_um = PARAMS["experiment_min_pos_um"]
    pos_tolerance_um = 200
    print(f"  Moving to experiment start position ({target_pos_um} µm)...")
    motor.set_mode(MotorMode.PositionMode)
    motor.set_streamed_position_um(target_pos_um)

    while True:
        motor.run()
        stream_data = motor.get_stream_data()
        current_pos_um = stream_data.position
        if abs(current_pos_um - target_pos_um) <= pos_tolerance_um:
            break
        time.sleep(0.005)

    motor.set_mode(MotorMode.SleepMode)
    print(f"  Reached start position ({current_pos_um} µm).")
    time.sleep(2.0)

    print("  Ramping force to find breakaway point...")
    result = run_static_friction_procedure(motor)

    # Save to separate file
    filepath = os.path.join(DATA_DIR, "calibration_static_friction.npz")

    # Pack per-step raw samples as 2D arrays (padded to max step length)
    max_step_len = max(len(a) for a in result["all_step_accels"])
    n_steps = len(result["all_step_accels"])
    step_accels_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    step_positions_2d = np.full((n_steps, max_step_len), np.nan, dtype=np.float64)
    for i in range(n_steps):
        n = len(result["all_step_accels"][i])
        step_accels_2d[i, :n] = result["all_step_accels"][i]
        step_positions_2d[i, :n] = result["all_step_positions"][i]

    np.savez(
        filepath,
        force_levels_mN=result["force_levels_mN"],
        mean_accel_mmpss=result["mean_accel_mmpss"],
        std_accel_mmpss=result["std_accel_mmpss"],
        pos_change_um=result["pos_change_um"],
        breakaway_force_mN=result["breakaway_force_mN"] or 0,
        step_accels=step_accels_2d,
        step_positions=step_positions_2d,
        samples_per_step=FRICTION_SAMPLES_PER_STEP,
    )
    print(f"\n  Saved friction data: {filepath}")

    if result["breakaway_force_mN"] is not None:
        print(f"\n  >>> Estimated static friction breakaway force: "
              f"{result['breakaway_force_mN']} mN <<<")
    else:
        print(f"\n  WARNING: No breakaway detected up to {FRICTION_FORCE_MAX_MN} mN")

    return result


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
    print(f"  Position window: {MOTOR_MIN_UM} - {EXPERIMENT_MAX_POS_UM} um")
    print(f"  Output directory: {DATA_DIR}/")

    print(f"\nProcedures:")
    print(f"  1) Mass estimation (constant force trials)")
    print(f"  2) Static friction estimation (force ramp)")
    print(f"  3) Both (friction first, then mass estimation)")
    procedure = input("Select procedure [1/2/3]: ").strip() or "1"

    input("\nPress Enter to begin data collection...")

    try:
        if procedure in ("2", "3"):
            collect_static_friction_data(motor)
        if procedure in ("1", "3"):
            collect_data(motor)
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
