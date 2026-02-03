from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MessagePriority, MotorMode
import pyorcasdk.orca_registers as orca_reg

FORCE_NEWTONS = 30
SPEED_MM_PER_SEC = 50

baud_rate = 1000000
interframe_delay = 80

def auto_zero_motor(motor):
    # zero mode - 2: auto zero enabled
    motor.write_register_blocking(orca_reg.ZERO_MODE, orca_reg.ZERO_MODE_AUTO_ZERO_ENABLED)
    # at most 30 N of force is applied to move the motor
    motor.write_register_blocking(orca_reg.AUTO_ZERO_FORCE_N, FORCE_NEWTONS)
    # shaft speed moves up to 50 mmps while completing auto zeroing
    motor.write_register_blocking(orca_reg.AUTO_ZERO_SPEED_MMPS, SPEED_MM_PER_SEC)
    # motor will sleep after completing auto zeroing
    motor.write_register_blocking(orca_reg.AUTO_ZERO_EXIT_MODE, MotorMode.SleepMode)

    motor.set_mode(MotorMode.AutoZeroMode)

# Create SerialFTDI with 1ms latency
serial = SerialFTDI(latency_ms=1)
clock = ChronoClock()

motor = Actuator(serial, clock, "TestAutoZero", 1)

serial_port = str(input("Please input the serial port of your connected motor. "))
serial_port_error = motor.open_serial_port(serial_port, baud_rate, interframe_delay)

motor.set_mode(MotorMode.SleepMode)

auto_zero_motor(motor)
motor.clear_errors()

while True:
    error_check = motor.get_errors()
    # check if auto zero error is present, as part of greater check, in case other errors are encountered
    if error_check.value & orca_reg.ERROR_0_AUTO_ZERO_FAILED_Mask:
        print("Auto Zeroing Failed.")
        break
    if error_check.value & 2048:
        print("Communication with the motor timed out")
        break
    elif motor.get_mode().value != MotorMode.AutoZeroMode:
        print("Auto Zeroing Complete!")
        break
        
motor.close_serial_port()