from pyorcasdk import Actuator, SerialFTDI, ChronoClock

# Setup
baud_rate = 1000000
interframe_delay = 80

## Create SerialFTDI with 1ms latency
serial = SerialFTDI(latency_ms=1)
clock = ChronoClock()

motor = Actuator(serial, clock, "TestHighSpeed", 1)

serial_port = str(input("Please input the serial port of your connected motor. "))

motor.open_serial_port(serial_port, baud_rate, interframe_delay)

while True:
    print("Current Position: " + str(motor.get_position_um().value), end="        \r")