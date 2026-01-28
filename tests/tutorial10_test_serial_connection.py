"""
Test Serial Connection and Measure Communication Performance

This script tests the serial connection to an ORCA motor and measures:
- Current baud rate settings (from registers)
- Round-trip latency for register reads
- Estimated message throughput (messages per second)
- Effective data rate

Usage:
    python tutorial10_test_serial_connection.py

The script will prompt for connection parameters and run diagnostic tests.
"""

from pyorcasdk import Actuator, MessagePriority
import time
import statistics

# Register addresses for baud rate configuration
USR_MB_BAUD_LO = 164
USR_MB_BAUD_HIGH = 165
USR_MB_DELAY = 168
MB_BAUD = 482
MB_BAUD_H = 483

# A register we can safely read repeatedly for latency testing
SHAFT_POS_UM = 342  # Shaft position in micrometers

# Default serial port for Mac
DEFAULT_PORT = "/dev/cu.usbserial-ABA76SF6"


def read_baud_settings(motor: Actuator) -> dict:
    """
    Read the current baud rate and interframe delay settings from registers.

    Uses read_wide_register_blocking for 32-bit baud rate register pairs.

    Returns:
        Dictionary with baud rate and delay settings
    """
    # Read default baud rate as 32-bit wide register (regs 164+165)
    default_baud_result = motor.read_wide_register_blocking(USR_MB_BAUD_LO, MessagePriority.important)
    delay = motor.read_register_blocking(USR_MB_DELAY, MessagePriority.important)

    # Read active baud rate as 32-bit wide register (regs 482+483)
    active_baud_result = motor.read_wide_register_blocking(MB_BAUD, MessagePriority.important)

    default_baud_raw = default_baud_result.value
    active_baud = active_baud_result.value

    # Handle default values (0 means system default)
    default_baud = default_baud_raw if default_baud_raw != 0 else 19200
    delay_us = delay.value if delay.value != 0 else 2000

    return {
        "default_baud": default_baud,
        "default_baud_raw": default_baud_raw,
        "active_baud": active_baud,
        "interframe_delay_us": delay_us,
        "interframe_delay_raw": delay.value,
    }


def measure_register_read_latency(motor: Actuator, num_samples: int = 100) -> dict:
    """
    Measure round-trip latency for register read operations.

    Args:
        motor: Connected Actuator instance
        num_samples: Number of read operations to perform

    Returns:
        Dictionary with latency statistics in microseconds
    """
    latencies = []

    for _ in range(num_samples):
        start = time.perf_counter()
        motor.read_register_blocking(SHAFT_POS_UM, MessagePriority.important)
        end = time.perf_counter()

        latency_us = (end - start) * 1_000_000
        latencies.append(latency_us)

    return {
        "min_us": min(latencies),
        "max_us": max(latencies),
        "mean_us": statistics.mean(latencies),
        "median_us": statistics.median(latencies),
        "stdev_us": statistics.stdev(latencies) if len(latencies) > 1 else 0,
        "samples": num_samples,
        "raw_latencies": latencies,
    }


def measure_stream_throughput(motor: Actuator, duration_seconds: float = 3.0) -> dict:
    """
    Measure stream message throughput.

    Args:
        motor: Connected Actuator instance
        duration_seconds: How long to measure

    Returns:
        Dictionary with throughput statistics
    """
    motor.enable_stream()

    # Wait for stream to initialize
    for _ in range(20):
        motor.run()
        time.sleep(0.005)

    message_count = 0
    gaps = []
    last_time = None

    start = time.perf_counter()
    end_time = start + duration_seconds

    while time.perf_counter() < end_time:
        motor.run()

        gap_us = motor.time_since_last_response_microseconds()

        # Only count valid gaps (not uninitialized values)
        if gap_us < 1_000_000:  # Less than 1 second
            gaps.append(gap_us)

        message_count += 1
        time.sleep(0.001)  # 1ms delay to prevent overwhelming

    actual_duration = time.perf_counter() - start
    motor.disable_stream()

    # Calculate statistics
    if gaps:
        return {
            "duration_s": actual_duration,
            "message_count": message_count,
            "messages_per_second": message_count / actual_duration,
            "avg_gap_us": statistics.mean(gaps),
            "min_gap_us": min(gaps),
            "max_gap_us": max(gaps),
            "median_gap_us": statistics.median(gaps),
        }
    else:
        return {
            "duration_s": actual_duration,
            "message_count": message_count,
            "messages_per_second": message_count / actual_duration,
            "avg_gap_us": None,
            "min_gap_us": None,
            "max_gap_us": None,
            "median_gap_us": None,
        }


def estimate_effective_data_rate(baud_rate: int, latency_us: float) -> dict:
    """
    Estimate effective data rates based on baud rate and measured latency.

    Args:
        baud_rate: Configured baud rate in bps
        latency_us: Measured round-trip latency in microseconds

    Returns:
        Dictionary with data rate estimates
    """
    # Modbus RTU frame sizes for single register read (FC03):
    #   Request:  8 bytes (addr + func + start_reg + num_regs + CRC)
    #   Response: 7 bytes (addr + func + byte_count + data + CRC)
    #   Total:   15 bytes per transaction
    #
    # Bits per byte (per UG210912 page 5 serial configuration):
    #   1 start bit + 8 data bits + 1 parity bit (even) + 1 stop bit = 11 bits
    #   The start bit signals the beginning of a byte, the parity bit provides
    #   basic error detection, and the stop bit ensures a minimum idle period
    #   before the next byte, allowing the receiver to resynchronize.
    bytes_per_transaction = 15
    bits_per_byte = 11
    bits_per_transaction = bytes_per_transaction * bits_per_byte

    # Theoretical time at wire speed
    theoretical_time_us = (bits_per_transaction / baud_rate) * 1_000_000

    # Messages per second based on latency
    if latency_us > 0:
        max_messages_per_second = 1_000_000 / latency_us
    else:
        max_messages_per_second = 0

    # Effective data rate (bytes per second, excluding overhead)
    # Payload per read: 2 bytes
    effective_data_rate = max_messages_per_second * 2

    return {
        "baud_rate_bps": baud_rate,
        "theoretical_transaction_time_us": theoretical_time_us,
        "measured_latency_us": latency_us,
        "overhead_us": latency_us - theoretical_time_us,
        "max_messages_per_second": max_messages_per_second,
        "effective_payload_bytes_per_second": effective_data_rate,
        "efficiency_percent": (theoretical_time_us / latency_us * 100) if latency_us > 0 else 0,
    }


def run_tests(port: str, baud_rate: int = None, interframe_delay: int = None):
    """
    Run all connection tests.

    Args:
        port: Serial port path
        baud_rate: Baud rate to connect with (None for default 19200)
        interframe_delay: Interframe delay in us (None for default)
    """
    motor = Actuator("TestMotor")

    print("\n" + "=" * 60)
    print("ORCA Motor Serial Connection Test")
    print("=" * 60)

    # Connect
    connect_baud = baud_rate if baud_rate else 19200
    connect_delay = interframe_delay if interframe_delay else 2000

    print(f"\nConnecting to: {port}")
    print(f"  Baud rate: {connect_baud} bps")
    print(f"  Interframe delay: {connect_delay} us")

    try:
        if baud_rate and interframe_delay:
            motor.open_serial_port(port, baud_rate, interframe_delay)
        elif baud_rate:
            motor.open_serial_port(port, baud_rate)
        else:
            motor.open_serial_port(port)

        print("  Connected successfully!")

    except Exception as e:
        print(f"\n  CONNECTION FAILED: {e}")
        print("\n  Troubleshooting:")
        print("    - Check that motor is powered on")
        print("    - Verify serial port path")
        print("    - Try different baud rate if motor was reconfigured")
        return

    # Test 1: Read baud rate settings
    print("\n" + "-" * 60)
    print("TEST 1: Read Baud Rate Settings from Registers")
    print("-" * 60)

    try:
        settings = read_baud_settings(motor)
        print(f"  Default baud rate (raw register): {settings['default_baud_raw']}")
        print(f"  Default baud rate (effective):    {settings['default_baud']:,} bps")
        print(f"  Active baud rate (session):       {settings['active_baud']:,} bps")
        print(f"  Interframe delay (raw register):  {settings['interframe_delay_raw']}")
        print(f"  Interframe delay (effective):     {settings['interframe_delay_us']} us")
    except Exception as e:
        print(f"  ERROR reading settings: {e}")

    # Test 2: Measure register read latency
    print("\n" + "-" * 60)
    print("TEST 2: Register Read Latency (100 samples)")
    print("-" * 60)

    try:
        latency = measure_register_read_latency(motor, num_samples=100)
        print(f"  Min latency:    {latency['min_us']:,.0f} us ({latency['min_us']/1000:.2f} ms)")
        print(f"  Max latency:    {latency['max_us']:,.0f} us ({latency['max_us']/1000:.2f} ms)")
        print(f"  Mean latency:   {latency['mean_us']:,.0f} us ({latency['mean_us']/1000:.2f} ms)")
        print(f"  Median latency: {latency['median_us']:,.0f} us ({latency['median_us']/1000:.2f} ms)")
        print(f"  Std deviation:  {latency['stdev_us']:,.0f} us")
    except Exception as e:
        print(f"  ERROR measuring latency: {e}")
        latency = {"mean_us": 0}

    # Test 3: Stream throughput
    print("\n" + "-" * 60)
    print("TEST 3: Stream Throughput (3 second test)")
    print("-" * 60)

    try:
        throughput = measure_stream_throughput(motor, duration_seconds=3.0)
        print(f"  Test duration:      {throughput['duration_s']:.2f} seconds")
        print(f"  Messages processed: {throughput['message_count']}")
        print(f"  Throughput:         {throughput['messages_per_second']:.1f} messages/second")
        if throughput['avg_gap_us']:
            print(f"  Avg gap between responses: {throughput['avg_gap_us']:,.0f} us")
            print(f"  Min gap: {throughput['min_gap_us']:,.0f} us, Max gap: {throughput['max_gap_us']:,.0f} us")
    except Exception as e:
        print(f"  ERROR measuring throughput: {e}")

    # Test 4: Estimate effective data rate
    print("\n" + "-" * 60)
    print("TEST 4: Effective Data Rate Estimates")
    print("-" * 60)

    try:
        if latency.get("mean_us", 0) > 0:
            estimates = estimate_effective_data_rate(connect_baud, latency["mean_us"])
            print(f"  Configured baud rate:      {estimates['baud_rate_bps']:,} bps")
            print(f"  Theoretical tx time:       {estimates['theoretical_transaction_time_us']:.0f} us")
            print(f"  Measured round-trip:       {estimates['measured_latency_us']:.0f} us")
            print(f"  Overhead (processing/OS):  {estimates['overhead_us']:.0f} us")
            print(f"  Max messages/second:       {estimates['max_messages_per_second']:.0f}")
            print(f"  Effective payload rate:    {estimates['effective_payload_bytes_per_second']:.0f} bytes/sec")
            print(f"  Wire efficiency:           {estimates['efficiency_percent']:.1f}%")
        else:
            print("  Could not calculate (no valid latency data)")
    except Exception as e:
        print(f"  ERROR calculating estimates: {e}")

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    if latency.get("mean_us", 0) > 0:
        expected_hz = 1_000_000 / latency["mean_us"]
        print(f"  Connection: OK at {connect_baud:,} bps")
        print(f"  Expected message rate: ~{expected_hz:.0f} Hz")

        if expected_hz > 500:
            print("  Status: HIGH-SPEED communication achieved!")
        elif expected_hz > 100:
            print("  Status: MEDIUM-SPEED communication")
        else:
            print("  Status: LOW-SPEED communication (consider increasing baud rate)")
    else:
        print("  Could not determine performance metrics")

    motor.close_serial_port()
    print("\nTest complete. Connection closed.")


def main():
    print("\nORCA Motor Serial Connection Tester")
    print("-" * 40)

    # Get connection parameters
    port = input(f"Enter serial port [{DEFAULT_PORT}]: ").strip()
    if not port:
        port = DEFAULT_PORT

    baud_input = input("Enter baud rate (press Enter for default 19200, or enter value): ").strip()
    baud_rate = int(baud_input) if baud_input else None

    delay_input = input("Enter interframe delay in us (press Enter for default, or enter value): ").strip()
    interframe_delay = int(delay_input) if delay_input else None

    run_tests(port, baud_rate, interframe_delay)


if __name__ == "__main__":
    main()
