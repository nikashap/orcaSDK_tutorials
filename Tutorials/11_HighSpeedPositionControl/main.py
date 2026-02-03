from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MessagePriority, MotorMode
from time import time
from math import sin, pi

# Create SerialFTDI with 1ms latency
serial = SerialFTDI(latency_ms=1)
clock = ChronoClock()

# Instantiate motor
motor = Actuator(serial, clock, "MyMotorName", 1)

# Parameters
baud_rate = 1000000
interframe_delay = 80

SINE_OFFSET = 100200
AMPLITUDE = 50000
FREQUENCY = 0.5

serial_port = str(input("Please input the serial port of your connected motor. "))

motor.open_serial_port(serial_port, baud_rate, interframe_delay)

motor.set_mode(MotorMode.SleepMode)

motor.clear_errors()

motor.enable_stream()

motor.set_mode(MotorMode.PositionMode)

def get_sine_target() -> int:
    curr_time = time()
    two_pi_ft = 2 * pi * FREQUENCY * curr_time
    position_target = (AMPLITUDE * sin(two_pi_ft)) + SINE_OFFSET
    return int(position_target)

try:
    while True:
        motor.run()

        motor.set_streamed_position_um(get_sine_target())
        # motor.set_streamed_position_um(80000)
        
        print("Current Position: " + str(motor.get_stream_data().position), end="        \r")
except KeyboardInterrupt:
    print("\n\nStopping...")

motor.set_mode(MotorMode.SleepMode)
motor.disable_stream()
motor.close_serial_port()
print("Done.")