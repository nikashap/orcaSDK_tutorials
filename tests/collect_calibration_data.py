#!/usr/bin/env python
"""
collect_calibration_data.py

Collect data from the ORCA motor for estimating the effective moving mass of the shaft.
The motor is commanded with a constant force and the shaft accelerates from the zero
position toward the other end. Position, force, and acceleration data are recorded
with timestamps and saved to .npz files for offline analysis.

Procedure (per trial):
  1. Auto-zero the motor (moves shaft to zero/home position)
  2. Switch to ForceMode with a constant commanded force
  3. Record streamed position + force (from motor.run()) and
     blocking acceleration reads (from read_wide_register_blocking)
  4. Stop when shaft reaches MAX_POS_UM
  5. Repeat for each force level and iteration count

Data saved per trial:
  - t_stream[i]:    timestamp of motor.run() response (perf_counter seconds)
  - position[i]:    shaft position from stream (um)
  - force[i]:       sensed force from stream (mN) -- note: this is commanded force
  - t_accel[i]:     timestamp of acceleration read (perf_counter seconds)
  - accel[i]:       shaft acceleration from register (mm/s^2)
  - force_commanded: the constant force commanded for this trial (mN)

Usage:
  python estimate_mass.py
"""

from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MotorMode
import pyorcasdk.orca_registers as orca_reg
import time
import numpy as np
import os

# ---------------------------------------------------------------------------
# ORCA Motor Parameters
# ---------------------------------------------------------------------------
MOTOR_MIN_UM = 106 #Change these based on AutoZero results from IrisGui
MOTOR_MAX_UM = 223615
MOTOR_CENTER_UM = int((MOTOR_MAX_UM - MOTOR_MIN_UM) / 2 + MOTOR_MIN_UM)

# Auto-zero parameters
AUTO_ZERO_FORCE_N = 30        # max force during auto-zero (Newtons)
AUTO_ZERO_SPEED_MMPS = 50     # max speed during auto-zero (mm/s)

# Communication
BAUD_RATE = 1_000_000
INTERFRAME_DELAY = 0        # microseconds
DEFAULT_SERIAL_PORT = "/dev/cu.usbserial-ABA76SF6"

# Data collection range — only record when shaft is in this window
# (avoids startup transients near zero and deceleration near end-stop)
MIN_POS_UM = 40_000
MAX_POS_UM = 160_000

# ---------------------------------------------------------------------------
# Force levels and repetitions
# ---------------------------------------------------------------------------
DEFAULT_FORCES_MN = [7500, 8750, 10000, 11250, 12500, 13750, 15000]
DEFAULT_ITERS_PER_FORCE = 5

# Output directory
OUTPUT_DIR = "mass_estimation_data_2026_03_05"


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


def collect_single_trial(motor, force_command_mN):
    """
    Run one trial: command a constant force and record data until the shaft
    reaches MAX_POS_UM.

    Returns dict with arrays:
        t_stream, position, force, t_accel, accel, force_commanded
    """
    # Pre-allocate lists (faster than appending to numpy arrays)
    t_stream_list = []
    position_list = []
    force_list = []
    t_accel_list = []
    accel_list = []

    # Start force mode
    motor.set_mode(MotorMode.ForceMode)
    motor.set_streamed_force_mN(force_command_mN)

    reached_max = False

    while not reached_max:
        # --- Transaction 1: Stream command (sends force, receives sensor data) ---
        motor.run()
        t1 = time.perf_counter()

        stream_data = motor.get_stream_data()
        pos_um = stream_data.position
        force_mN = stream_data.force

        # --- Transaction 2: Blocking acceleration read ---
        accel_mmpss = read_raw_acceleration(motor)
        t2 = time.perf_counter()

        # Record ALL data points (we'll filter by position range offline)
        t_stream_list.append(t1)
        position_list.append(pos_um)
        force_list.append(force_mN)
        t_accel_list.append(t2)
        accel_list.append(accel_mmpss)

        # Check termination
        if pos_um >= MAX_POS_UM:
            reached_max = True

        # Check for motor errors
        if stream_data.errors:
            print(f"    Motor error: {stream_data.errors}")
            break

    # Stop the motor
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
    Saves each trial as a separate .npz file, plus a summary index.
    """
    if forces_mN_list is None:
        forces_mN_list = DEFAULT_FORCES_MN

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    total_trials = iters_per_force * len(forces_mN_list)
    trial_index = []  # metadata for each trial

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

                # Auto-zero to start from a known position
                print("  Auto-zeroing...")
                error = auto_zero_motor(motor)
                if error.value != 0:
                    print(f"  WARNING: Auto-zero returned error {error.value}, skipping trial.")
                    continue

                # Re-enable stream after auto-zero (auto-zero exits to SleepMode)
                motor.enable_stream()
                time.sleep(0.1)  # let stream stabilize

                # Collect data for this trial
                print(f"  Collecting data (force={force_mN} mN)...")
                trial_data = collect_single_trial(motor, force_mN)

                n_samples = len(trial_data["t_stream"])
                print(f"  Recorded {n_samples} samples, "
                      f"position range: {trial_data['position_um'][0]} - "
                      f"{trial_data['position_um'][-1]} um")

                # Save trial data
                filename = f"trial_{trial_num:03d}_force_{force_mN}mN_iter_{iteration}.npz"
                filepath = os.path.join(OUTPUT_DIR, filename)
                np.savez(filepath, **trial_data)
                print(f"  Saved: {filepath}")

                trial_index.append({
                    "trial_num": trial_num,
                    "force_commanded_mN": force_mN,
                    "iteration": iteration,
                    "n_samples": n_samples,
                    "filename": filename,
                })

                # Brief pause between trials
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    except Exception as e:
        print(f"\nError during collection: {e}")
        raise
    finally:
        motor.set_mode(MotorMode.SleepMode)

    # Save trial index as a separate file
    index_path = os.path.join(OUTPUT_DIR, "trial_index.npz")
    np.savez(index_path,
             trial_nums=[t["trial_num"] for t in trial_index],
             forces_mN=[t["force_commanded_mN"] for t in trial_index],
             iterations=[t["iteration"] for t in trial_index],
             n_samples=[t["n_samples"] for t in trial_index],
             filenames=[t["filename"] for t in trial_index])
    print(f"\nSaved trial index: {index_path}")
    print(f"Total trials completed: {len(trial_index)}/{total_trials}")

    return trial_index


def main():
    serial_port = input(
        f"Enter serial port [{DEFAULT_SERIAL_PORT}]: "
    ).strip()
    if not serial_port:
        serial_port = DEFAULT_SERIAL_PORT

    # Initialize motor
    print(f"Connecting to motor on {serial_port}...")
    serial = SerialFTDI(latency_ms=1)
    clock = ChronoClock()
    motor = Actuator(serial, clock, "OrcaMotor", 1)

    motor.open_serial_port(serial_port, BAUD_RATE, INTERFRAME_DELAY)
    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()

    print("Motor connected.")
    print(f"\nData collection parameters:")
    print(f"  Force levels: {DEFAULT_FORCES_MN} mN")
    print(f"  Iterations per force: {DEFAULT_ITERS_PER_FORCE}")
    print(f"  Total trials: {DEFAULT_ITERS_PER_FORCE * len(DEFAULT_FORCES_MN)}")
    print(f"  Position window: {MIN_POS_UM} - {MAX_POS_UM} um")
    print(f"  Output directory: {OUTPUT_DIR}/")

    input("\nPress Enter to begin data collection...")

    try:
        trial_index = collect_data(motor)
    finally:
        print("\nShutting down motor...")
        try:
            motor.set_mode(MotorMode.SleepMode)
            motor.disable_stream()
            motor.close_serial_port()
        except Exception as e:
            print(f"  Warning during cleanup: {e}")
        print("Done.")


if __name__ == "__main__":
    main()