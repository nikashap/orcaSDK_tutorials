#!/usr/bin/env python
"""
Test SerialFTDI with pyorcasdk at high baud rates.
"""

from pyorcasdk import Actuator, SerialFTDI, ChronoClock, MessagePriority, MotorMode, HapticEffect
import pyorcasdk.orca_registers as orca_reg
import time
import numpy as np

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
    print(f"Opening motor called '{motor_name}...'")
    motor.open_serial_port(port) #assumes from default config
    print("Opened.")

    #Register addresses
    USR_MB_BAUD_LO = 164
    USR_MB_DELAY = 168

    #Format target_baud into two 16-bit chunks of data
    baud_low = target_baud & 0xFFFF
    baud_high = target_baud >> 16

    #Set values (delay first)
    print(f"Setting registers to target baudrate and interframe delay...")
    err_delay = motor.write_register_blocking(USR_MB_DELAY, interframe_delay, MessagePriority.important)
    err_baudrate = motor.write_multiple_registers_blocking(USR_MB_BAUD_LO, [baud_low, baud_high], MessagePriority.important)
    print("Set. Now closing serial port.\n\n")
    motor.close_serial_port()
    return motor, err_delay, err_baudrate

def test_serial_ftdi(port, baud_rate, interframe_delay, test_stream=True):
    print("=" * 60)
    print("Testing SerialFTDI with pyorcasdk")
    print("=" * 60)
    
    # Create SerialFTDI with 1ms latency
    serial = SerialFTDI(latency_ms=1)
    clock = ChronoClock()
    
    # Create Actuator with custom serial interface
    print("\nCreating motor instance using SerialInterface argument")
    motor = Actuator(serial, clock, "TestMotor", 1)
    
    # Test at high baud rate!
    
    print(f"\nConnecting at {baud_rate:,} baud...")
    
    try:
        error = motor.open_serial_port(port, baud_rate, interframe_delay)
        if len(error.what()) > 0:
            print(f"Failed to open: {error.what()}")
            return
        
        print("Connected!")
        
        # Read baud rate registers to verify
        print("\nReading baud rate settings...")
        result = motor.read_wide_register_blocking(164, MessagePriority.important)
        print(f"  Default baud (reg 164-165): {result.value}")
        
        result = motor.read_wide_register_blocking(482, MessagePriority.important)
        print(f"  Active baud (reg 482-483): {result.value}")
        
        # Latency test
        num_samples = 100000
        print(f"\nMeasuring latency ({num_samples} samples)...")
        latencies = []
        
        if test_stream:
            print("Testing speed of Motor stream with one additional read/write,")
            print("in FORCE mode\n")

            motor.set_mode(MotorMode.ForceMode)
            motor.clear_errors()

            #Enable stream and update haptic effects
            motor.enable_stream()
            force_effect = 10000 #millinewtons
            
            ## The block below is for Haptic mode
            # motor.update_haptic_stream_effects(HapticEffect.ConstF)
            # force_effect_low = force_effect & 0xFFFF
            # force_effect_high = force_effect >> 16

            # motor.set_constant_force(force_effect)
            ##

            ## Wait for stream to initialize and receive first valid response
            print("Initializing stream...")
            initialized = False
            for _ in range(1000):  # Try for up to 1 second
                motor.run()
                # Check if we've received a response (gap will be small after first response)
                if motor.time_since_last_response_microseconds() < 100000:  # < 100ms
                    initialized = True
                    print("Stream initialization confirmed.")
                    break

            if not initialized:
                print("Warning: Stream may not be fully initialized")

            ## More accurate latency monitoring
            prev_pos = motor.get_stream_data().position
            t_start = time.perf_counter()
            ##
            print("Begin timer test...")
            start = time.perf_counter()
            for i in range(num_samples):

                motor.run()

                #Force control
                motor.set_streamed_force_mN(force_effect)
                print("Current Sensed Force: " + str(motor.get_stream_data().force), end="        \r")
                
                current_pos = motor.get_stream_data().position
                if current_pos != prev_pos:  # we know a response actually arrived
                    t_now = time.perf_counter()
                    latencies.append((t_now - t_start) * 1_000_000)
                    t_start = t_now
                    prev_pos = current_pos
                
                ## For testing latencies with single read/writes to registers (not streamed)
                # read_data = motor.read_write_multiple_registers_blocking(
                #     read_starting_address = orca_reg.SHAFT_ACCEL_MMPSS,
                #     read_num_registers = 2,
                #     write_starting_address = orca_reg.CONSTANT_FORCE_MN,
                #     write_data = [force_effect_low, force_effect_high]
                # )
                # measured_accel = read_data.value[1] << 16 | read_data.value[0]
                # print()
                # print("\nAcceleration: " + str(measured_accel), end="        \r")
                ## 
            
            motor.disable_stream()
            print("="*60)
            print("This may be more accurate")
            print(f"Actual stream update rate: {1_000_000 / np.mean(latencies):.0f} Hz")
            print(f"Avg round-trip: {np.mean(latencies):.0f} µs")
            print(f"Stdev round-trip: {np.std(latencies):.0f} µs")
            print("="*60)

        else:
            for i in range(num_samples):
                start = time.perf_counter()
                motor.read_register_blocking(342, MessagePriority.important)  # Shaft position
                end = time.perf_counter()
                latencies.append((end - start) * 1_000_000)
        
        avg = float(np.mean(latencies))
        min_lat = float(np.min(latencies))
        max_lat = float(np.max(latencies))
        std_dev = float(np.std(latencies))
        
        print(f"\n  Min:  {min_lat:,.0f} µs")
        print(f"  Avg:  {avg:,.0f} µs")
        print(f"  Max:  {max_lat:,.0f} µs")
        print(f"  Std dev:  {std_dev:,.0f} µs")
        print(f"  Rate: {1_000_000 / avg:.0f} Hz")
        
        motor.set_mode(MotorMode.SleepMode)
        motor.close_serial_port()
        print("\nTest complete!")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    port="/dev/cu.usbserial-ABA76SF6"
    target_baud = 1000000
    interframe_delay = 80

    # startup_from_default("Startup Motor", port, target_baud, interframe_delay)
    test_serial_ftdi(port, target_baud, interframe_delay)