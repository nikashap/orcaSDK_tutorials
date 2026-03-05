#!/usr/bin/env python
"""
collect_calibration_data_v2.py

Collect data from the ORCA motor for estimating the effective moving mass of the shaft.
The motor is commanded with an oscillating force and a spring effect centered at the midpoint
of the shaft. Position, force, and acceleration data are recorded
with timestamps and saved to .npz files for offline analysis.

Procedure:
  1. Auto-zero the motor (moves shaft to zero/home position)
  2. Switch to HapticMode with an Oscillator and Spring effect
  3. Record streamed position + force (from motor.run()) and
     blocking acceleration reads (from read_wide_register_blocking)
  4. Stop after T seconds (T set as a parameter)

Data saved during the procedure:
  - t_stream[i]:    timestamp of motor.run() response (perf_counter seconds)
  - position[i]:    shaft position from stream (um)
  - force[i]:       sensed force from stream (mN) -- note: this is commanded force
  - t_accel[i]:     timestamp of acceleration read (perf_counter seconds)
  - accel[i]:       shaft acceleration from register (mm/s^2)
  - force_commanded: the constant force commanded for this trial (mN)

Usage:
  python collect_calibration_data_v2
"""

from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MotorMode
from pyorcasdk import HapticEffect, OscillatorType
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

# Duration of procedure
T = 30 #seconds

# ---------------------------------------------------------------------------
# Haptic effect settings
# ---------------------------------------------------------------------------

# Spring effect
springID = 0
springStrength = 150
centerPosition = MOTOR_CENTER_UM

# Oscillator effect
oscillatorID = 0
oscillatorMaxForce = 15
oscillatorFrequency = 10
oscillatorDutyCycle = 0

# Output directory
OUTPUT_DIR = "mass_estimation_data_v2_2026_03_05"


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


def collect_single_trial(motor):
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

    # Start Haptic Mode and update effects
    motor.set_mode(MotorMode.HapticMode)
    motor.update_haptic_stream_effects(HapticEffect.Osc0 | HapticEffect.Spring0)

    motor.set_spring_effect(
        springID,
        springStrength,
        centerPosition
    )

    motor.set_osc_effect(
        oscillatorID,
        oscillatorMaxForce,
        oscillatorFrequency,
        oscillatorDutyCycle,
        OscillatorType.Sine
    )

    t_elapsed = 0
    t_start = time.perf_counter()

    while t_elapsed < T:
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

        # Check for motor errors
        if stream_data.errors:
            print(f"    Motor error: {stream_data.errors}")
            break

        # Update elapsed time
        t_elapsed = time.perf_counter() - t_start

    # Stop the motor
    motor.set_mode(MotorMode.SleepMode)

    return {
        "t_stream": np.array(t_stream_list, dtype=np.float64),
        "position_um": np.array(position_list, dtype=np.int32),
        "force_mN": np.array(force_list, dtype=np.int32),
        "t_accel": np.array(t_accel_list, dtype=np.float64),
        "accel_mmpss": np.array(accel_list, dtype=np.int32),
        "time_elapsed": t_elapsed,
        "spring_strength": springStrength,
        "spring_center_um": centerPosition,
        "osc_max_force_mN": oscillatorMaxForce,
        "osc_frequency_hz": oscillatorFrequency,
        "osc_duty_cycle": oscillatorDutyCycle,
    }


def collect_data(motor):
    """
    Collect data
    Saves each trial as a separate .npz file, plus a summary index.
    """

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    motor.set_mode(MotorMode.SleepMode)
    motor.clear_errors()
    motor.enable_stream()

    trial_index = []  # metadata for the singular trial
    trial_num = 0

    try:
        # Auto-zero to start from a known position
        print("  Auto-zeroing...")
        error = auto_zero_motor(motor)
        if error.value != 0:
            raise ValueError(f"  WARNING: Auto-zero returned error {error.value}, skipping procedure.")

        # Re-enable stream after auto-zero (auto-zero exits to SleepMode)
        motor.enable_stream()
        time.sleep(0.1)  # let stream stabilize

        # Collect data for this trial
        print(f"  Collecting data for sinusoidal prodecure")
        trial_data = collect_single_trial(motor)

        n_samples = len(trial_data["t_stream"])
        print(f"  Recorded {n_samples} samples, "
                f"  Time elapsed: {trial_data['time_elapsed']} seconds")

        # Save trial data
        filename = f"trial_{trial_num:03d}_sinusoidal.npz"
        filepath = os.path.join(OUTPUT_DIR, filename)
        np.savez(filepath, **trial_data)
        print(f"  Saved: {filepath}")

        trial_index.append({
            "trial_num": trial_num,
            "n_samples": n_samples,
            "filename": filename,
            "spring_strength": springStrength,
            "spring_center_um": centerPosition,
            "osc_max_force_mN": oscillatorMaxForce,
            "osc_frequency_hz": oscillatorFrequency,
            "osc_duty_cycle": oscillatorDutyCycle,
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
             n_samples=[t["n_samples"] for t in trial_index],
             filenames=[t["filename"] for t in trial_index],
             spring_strengths=[t["spring_strength"] for t in trial_index],
             spring_centers_um=[t["spring_center_um"] for t in trial_index],
             osc_max_forces_mN=[t["osc_max_force_mN"] for t in trial_index],
             osc_frequencies_hz=[t["osc_frequency_hz"] for t in trial_index],
             osc_duty_cycles=[t["osc_duty_cycle"] for t in trial_index])
    print(f"\nSaved trial index: {index_path}")

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
    print(f"  Spring: strength={springStrength}, center={centerPosition} µm")
    print(f"  Oscillator: max_force={oscillatorMaxForce} mN, freq={oscillatorFrequency} Hz, duty_cycle={oscillatorDutyCycle}")
    print(f"  Duration: {T} seconds")
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