from pyorcasdk import Actuator, MotorMode, MessagePriority
from time import time, sleep

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

# ---------------------------

motor.clear_errors()

motor.enable_stream()

# Wait for stream to initialize
print("Initializing stream...")
for _ in range(100):
    motor.run()
    if motor.time_since_last_response_microseconds() < 100000:
        break
    sleep(0.01)

motor.set_mode(MotorMode.ForceMode)

force_switch_time = 1.000
force_to_command_mN = 5000

last_force_switch_time = time()

print("Running force control (Ctrl+C to stop):\n")

try:
    while True:
        motor.run()

        if (time() - last_force_switch_time) > force_switch_time:
            motor.set_streamed_force_mN(force_to_command_mN)
            last_force_switch_time = time()
            force_to_command_mN *= -1

        print("Current Position: " + str(motor.get_stream_data().position), end="        \r")
        sleep(0.002)
except KeyboardInterrupt:
    print("\n\nStopping...")

motor.set_mode(MotorMode.SleepMode)
motor.disable_stream()
motor.close_serial_port()
print("Done.")
