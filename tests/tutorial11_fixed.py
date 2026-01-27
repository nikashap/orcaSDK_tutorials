from pyorcasdk import Actuator, MotorMode, MessagePriority
from time import time, sleep
from math import sin, pi

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

SINE_OFFSET = 50000
AMPLITUDE = 10000
FREQUENCY = 2
FREQ_MIN = 2
FREQ_MAX = 5
DUR = 10
#time is set in seconds
scale = (FREQ_MAX - FREQ_MIN) / DUR

motor.set_mode(MotorMode.SleepMode)

motor.enable_stream()

motor.set_mode(MotorMode.PositionMode)

def get_sine_target_original() -> int:
    curr_time = time()
    two_pi_ft = 2 * pi * FREQUENCY * curr_time
    position_target = (AMPLITUDE * sin(two_pi_ft)) + SINE_OFFSET
    return int(position_target)

init_time = time()
def get_sine_target_changing() -> int:
    curr_time = time()
    time_diff = curr_time - init_time
    factor = time_diff * scale
    if factor > 10:
        command_freq = FREQ_MAX * curr_time
    else:
        command_freq = FREQ_MIN + factor
    two_pi_ft = 2 * pi * command_freq
    position_target = (AMPLITUDE * sin(two_pi_ft)) + SINE_OFFSET
    return int(position_target)

def arb_traj() -> int:
    curr_time = time()
    
while True:
    motor.run()

    motor.set_streamed_position_um(get_sine_target_original())
    # motor.set_streamed_position_um(80000)
    
    print("Current Position: " + str(motor.get_stream_data().position), end="        \r")