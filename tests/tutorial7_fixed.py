"""
Tutorial 7 - Command Stream (Fixed Version)

This version properly waits for the stream to initialize before
reading position data, preventing the flickering-to-zero issue.
"""
from pyorcasdk import Actuator, MessagePriority
import time

def set_target_baudrate(motor, target_baud, interframe_delay):
    """
    Notes:
    - Assumes motor is already open at default settings
    - will cause serial port to disconnect
    """
    #Register addresses
    USR_MB_BAUD_LO = 164
    USR_MB_DELAY = 168

    #Format target_baud into two 16-bit chunks of data
    baud_low = target_baud & 0xFFFF
    baud_high = target_baud >> 16

    #Set values (delay first)
    err_delay = motor.write_register_blocking(USR_MB_DELAY, interframe_delay, MessagePriority.important)
    err_baudrate = motor.write_multiple_registers_blocking(USR_MB_BAUD_LO, [baud_low, baud_high], MessagePriority.important)

    motor.close_serial_port()
    return
    
motor = Actuator("MyMotorName")

# Parameters
port = "/dev/cu.usbserial-ABA76SF6"
target_baud = 230400
interframe_delay = 200

# Set the target baudrate after opening port
motor.open_serial_port(port)
set_target_baudrate(motor, target_baud, interframe_delay)

# Reopen the port
motor.open_serial_port(port, target_baud, interframe_delay)

motor.enable_stream()

# Wait for stream to initialize and receive first valid response
print("Initializing stream...")
initialized = False
for _ in range(100):  # Try for up to ~1 second
    motor.run()
    # Check if we've received a response (gap will be small after first response)
    if motor.time_since_last_response_microseconds() < 100000:  # < 100ms
        initialized = True
        break
    time.sleep(0.01)

if not initialized:
    print("Warning: Stream may not be fully initialized")

print("Streaming position data (Ctrl+C to stop):\n")

try:
    while True:
        motor.run()
        stream_data = motor.get_stream_data()
        print(f"Current Position: {stream_data.position} um", end="        \r")
        time.sleep(0.002)  # Small delay to prevent CPU spinning
except KeyboardInterrupt:
    print("\n\nStopping...")

motor.disable_stream()
motor.close_serial_port()
print("Done.")
