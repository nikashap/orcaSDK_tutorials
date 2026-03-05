from pyorcasdk import Actuator, MotorMode, MessagePriority, HapticEffect, OscillatorType, SerialFTDI, ChronoClock
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


# Parameters
port = "/dev/cu.usbserial-ABA76SF6"
target_baud = 1000000
interframe_delay = 0

# # Set the target baudrate after opening port
# motor.open_serial_port(port)
# set_target_baudrate(motor, target_baud, interframe_delay)

serial = SerialFTDI(latency_ms=1)
clock = ChronoClock()
motor = Actuator(serial, clock, "OrcaMotor", 1)

# Reopen the port
motor.open_serial_port(port, target_baud, interframe_delay)

motor.clear_errors() # Orca motors raise an error when communication stops during haptics mode

motor.enable_stream()

motor.set_mode(MotorMode.HapticMode)

motor.update_haptic_stream_effects(HapticEffect.Osc0 | HapticEffect.Spring0)

motor.set_spring_effect( #Update spring parameters
    0,      #Spring ID (0, 1, or 2)
    150,    #Spring strength
    110000   #Center position, set based off of motor min and motor max
)

motor.set_osc_effect(
    0,      #Oscillator ID (0 or 1)
    15,     #Oscillator max force (newtons)
    10,     #Frequency (decihertz)
    0,      #Duty cycle (not used in sine wave)
    OscillatorType.Sine
)

while True:
    motor.run()

    print("Current Sensed Force: " + str(motor.get_stream_data().force), end="        \r")