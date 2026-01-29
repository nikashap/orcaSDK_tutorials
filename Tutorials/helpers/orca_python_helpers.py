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
    return motor, err_delay, err_baudrate

def startup_from_default(motor_name,
                         port="/dev/cu.usbserial-ABA76SF6",
                         target_baud=1000000,
                         interframe_delay=80):
    """
    Startup the motor from default configuration settings (19200 baudrate, 2000 delay)
    Run this function after power-cycling the motor, if custom settings weren't saved,
    or after hard-resetting to defaults

    Args:
    - motor_name (str): arbitrary name to give the motor
    - port (str): address for the motor (defaults to "/dev/cu.usbserial-ABA76SF6")
    - target_baud (int): baudrate to set the motor (defaults to high speed communication at 1M baudrate)
    - interframe_delay (int): delay (in microseconds) between binary packets (aka, "effective bytes") being streamed to the motor

    Returns:
    - motor (pyorcasdk.Actuator): instance of the main motor interface
    - err_delay (pyorcasdk.OrcaError): error object from setting the delay
    - err_baudrate (pyorcasdk.OrcaError): error object from setting the baudrate
    """
    motor = Actuator(motor_name)
    motor.open_serial_port(port) #assumes from default config

    return set_target_baudrate(motor, target_baud, interframe_delay)