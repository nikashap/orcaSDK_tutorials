"""
Configure ORCA Motor Baud Rate (Without IrisControls GUI)

This script configures the ORCA motor's default Modbus baud rate and interframe
delay settings via register writes. This is an alternative to using the
IrisControls GUI, which is not available on Mac.

Key insight: Writing to the baud rate registers (164/165) takes effect IMMEDIATELY
on the motor's active session (per Modbus manual: "These configurations will take
effect immediately when adjusted"). This means:
  - The write goes through on the motor side
  - The motor's response comes back at the NEW baud rate
  - Our client (still at the old baud rate) can't read the response → timeout
  - This timeout is EXPECTED and does not mean the write failed

Strategy:
  1. Connect at current baud rate (default 19200)
  2. Write baud rate registers → expect timeout (motor switches immediately)
  3. Close connection
  4. Reconnect at the NEW baud rate (with old interframe delay)
  5. Write interframe delay, save to flash, verify

Registers used:
    - USR_MB_BAUD_LO (164) + USR_MB_BAUD_HIGH (165): Default baud rate (32-bit)
    - USR_MB_DELAY (168): Default interframe delay in microseconds (16-bit)
    - CTRL_REG_2 (2): Write 64 to save user options to flash

Reference: ORCA Series Reference Manual, ORCA Motors Modbus RTU User Guide
"""

from pyorcasdk import Actuator, MessagePriority
import time

# Register addresses
USR_MB_BAUD_LO = 164       # Low 16 bits of default baud rate (start address for wide write)
USR_MB_BAUD_HIGH = 165     # High 16 bits of default baud rate
USR_MB_DELAY = 168          # Interframe delay in microseconds
CTRL_REG_2 = 2
USER_OPT_SAVE_FLAG = 64    # Bit 6: save user options to flash

# Current baud rate registers (read-only, reflect active session)
MB_BAUD = 482
MB_BAUD_H = 483

# Default serial port for Mac
DEFAULT_PORT = "/dev/cu.usbserial-ABA76SF6"


def read_current_settings(motor: Actuator) -> dict:
    """Read the current baud rate and interframe delay settings."""
    # Read default baud rate as a 32-bit wide register
    baud_result = motor.read_wide_register_blocking(USR_MB_BAUD_LO, MessagePriority.important)
    delay_result = motor.read_register_blocking(USR_MB_DELAY, MessagePriority.important)

    # Read active session baud rate as a 32-bit wide register
    active_baud_result = motor.read_wide_register_blocking(MB_BAUD, MessagePriority.important)

    default_baud = baud_result.value
    active_baud = active_baud_result.value

    return {
        "default_baud": default_baud,
        "default_baud_display": default_baud if default_baud != 0 else 19200,
        "active_baud": active_baud,
        "active_baud_display": active_baud if active_baud != 0 else 19200,
        "interframe_delay_us": delay_result.value if delay_result.value != 0 else 2000,
        "raw_delay": delay_result.value,
    }


def print_settings(settings: dict, label: str = "Settings"):
    """Print settings in a consistent format."""
    print(f"  Default baud rate (raw register): {settings['default_baud']}")
    print(f"  Default baud rate (effective):    {settings['default_baud_display']} bps")
    print(f"  Active baud rate:                 {settings['active_baud_display']} bps")
    print(f"  Interframe delay (raw register):  {settings['raw_delay']}")
    print(f"  Interframe delay (effective):     {settings['interframe_delay_us']} us")


def main():
    print("=" * 60)
    print("ORCA Motor Baud Rate Configuration")
    print("=" * 60)

    # Get serial port
    port = input(f"\nEnter serial port [{DEFAULT_PORT}]: ").strip()
    if not port:
        port = DEFAULT_PORT

    # Get current baud rate (what to connect with initially)
    current_baud_input = input("Enter CURRENT motor baud rate [19200]: ").strip()
    current_baud = int(current_baud_input) if current_baud_input else 19200

    # Get target baud rate
    baud_input = input("Enter TARGET baud rate [1000000]: ").strip()
    target_baud = int(baud_input) if baud_input else 1000000

    # Get interframe delay
    delay_input = input("Enter interframe delay in microseconds [80]: ").strip()
    interframe_delay = int(delay_input) if delay_input else 80

    motor = Actuator("ConfigMotor")

    # ── PHASE 1: Connect and read current settings ──────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 1: Read current settings")
    print(f"{'─' * 60}")
    print(f"Connecting to {port} at {current_baud} baud...")

    try:
        if current_baud != 19200:
            motor.open_serial_port(port, current_baud)
        else:
            motor.open_serial_port(port)
        print("Connected!")
    except Exception as e:
        print(f"CONNECTION FAILED: {e}")
        return

    current = read_current_settings(motor)
    print_settings(current)

    if current_baud == target_baud:
        print(f"\nMotor is already at {target_baud} bps.")
        # Still might want to change interframe delay
        if current['interframe_delay_us'] == interframe_delay:
            print(f"Interframe delay is already {interframe_delay} us. Nothing to do.")
            motor.close_serial_port()
            return

    # Confirm
    print(f"\n{'─' * 60}")
    print("Planned changes:")
    print(f"{'─' * 60}")
    print(f"  Baud rate:       {current['active_baud_display']} → {target_baud} bps")
    print(f"  Interframe delay: {current['interframe_delay_us']} → {interframe_delay} us")
    print()
    print("NOTE: Writing the baud rate will immediately change the motor's")
    print("active baud rate. The script will reconnect at the new rate.")

    confirm = input("\nProceed with configuration? [y/N]: ").strip().lower()
    if confirm != 'y':
        print("Configuration cancelled.")
        motor.close_serial_port()
        return

    # ── PHASE 2: Write baud rate (expect timeout) ───────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 2: Write baud rate")
    print(f"{'─' * 60}")

    baud_lo = target_baud & 0xFFFF
    baud_hi = (target_baud >> 16) & 0xFFFF
    print(f"Writing {target_baud} bps to registers 164-165 (FC16)")
    print(f"  USR_MB_BAUD_LO (reg 164) = {baud_lo} (0x{baud_lo:04X})")
    print(f"  USR_MB_BAUD_HIGH (reg 165) = {baud_hi} (0x{baud_hi:04X})")

    err = motor.write_wide_register_blocking(USR_MB_BAUD_LO, target_baud, MessagePriority.important)
    if err:
        # Timeout is EXPECTED because the motor switches baud rate immediately
        # and sends the response at the new baud rate
        print(f"  Write returned: {err}")
        print(f"  (This timeout is expected — the motor has already switched to {target_baud} bps)")
    else:
        print(f"  Write acknowledged (no baud rate change, or motor echoed at old rate)")

    # Close connection regardless — motor is now at the new baud rate
    motor.close_serial_port()
    print("  Connection closed.")

    # Brief pause to let the motor settle
    time.sleep(0.5)

    # ── PHASE 3: Reconnect at new baud rate ─────────────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 3: Reconnect at {target_baud} bps")
    print(f"{'─' * 60}")

    # Reconnect with the new baud rate but the motor's CURRENT interframe delay
    # (which hasn't been changed yet)
    current_delay = current['interframe_delay_us']
    print(f"Connecting at {target_baud} baud, {current_delay} us interframe delay...")

    motor2 = Actuator("ConfigMotor2")
    try:
        motor2.open_serial_port(port, target_baud, current_delay)
        print("Connected!")
    except Exception as e:
        print(f"CONNECTION FAILED: {e}")
        print("\nThe motor may not have accepted the baud rate change.")
        print("Try power cycling the motor to reset to defaults.")
        return

    # Verify the baud rate took effect
    print("\nVerifying baud rate...")
    try:
        verified = read_current_settings(motor2)
        print_settings(verified)

        if verified['active_baud'] != target_baud:
            print(f"\n  WARNING: Active baud rate is {verified['active_baud']}, expected {target_baud}")
            print("  The baud rate change may not have taken effect.")
            print("  Power cycle the motor and try again.")
            motor2.close_serial_port()
            return
        print(f"  Baud rate verified: {target_baud} bps")
    except Exception as e:
        print(f"  ERROR reading settings: {e}")
        print("  Cannot verify. Power cycle the motor and try again.")
        motor2.close_serial_port()
        return

    # ── PHASE 4: Write interframe delay ─────────────────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 4: Write interframe delay")
    print(f"{'─' * 60}")

    print(f"Writing {interframe_delay} us to register 168 (FC6)")
    err2 = motor2.write_register_blocking(USR_MB_DELAY, interframe_delay, MessagePriority.important)
    if err2:
        print(f"  ERROR writing interframe delay: {err2}")
        print("  Continuing to save baud rate at least...")
    else:
        print(f"  Interframe delay written successfully!")

    # ── PHASE 5: Save to flash ──────────────────────────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 5: Save to flash")
    print(f"{'─' * 60}")

    print(f"Writing {USER_OPT_SAVE_FLAG} to CTRL_REG_2 (save user options)")
    err3 = motor2.write_register_blocking(CTRL_REG_2, USER_OPT_SAVE_FLAG, MessagePriority.important)
    if err3:
        print(f"  ERROR saving to flash: {err3}")
        print("  Settings are active but will NOT persist through power cycle.")
    else:
        time.sleep(0.5)
        print("  Saved to flash!")

    # ── PHASE 6: Final verification ─────────────────────────────
    print(f"\n{'─' * 60}")
    print(f"PHASE 6: Final verification")
    print(f"{'─' * 60}")

    try:
        final = read_current_settings(motor2)
        print_settings(final)
    except Exception as e:
        print(f"  ERROR reading final settings: {e}")

    motor2.close_serial_port()

    # ── Summary ─────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("CONFIGURATION COMPLETE")
    print(f"{'=' * 60}")
    print(f"\n  Baud rate:       {target_baud} bps")
    print(f"  Interframe delay: {interframe_delay} us")
    print(f"\nTo connect at the new settings:")
    print(f"  motor.open_serial_port(\"{port}\", {target_baud}, {interframe_delay})")
    print(f"\nTo verify, run the test script:")
    print(f"  python3 ../../tests/tutorial10_test_serial_connection.py")
    print(f"  (enter baud rate: {target_baud}, interframe delay: {interframe_delay})")


if __name__ == "__main__":
    main()
