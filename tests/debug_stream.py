from pyorcasdk import Actuator
import time

motor = Actuator("MyMotorName")

# Use your specific port path
motor.open_serial_port("/dev/cu.usbserial-ABA76SF6")

motor.enable_stream()

# Give the stream time to initialize
print("Waiting for stream to initialize...")
for _ in range(10):
    motor.run()
    time.sleep(0.01)

print("\nMonitoring communication timing for 5 seconds...\n")
print("Position(um) | Time since last response(us) | Errors")
print("-" * 60)

start_time = time.time()
max_gap = 0
zero_count = 0
total_reads = 0

while time.time() - start_time < 5:
    motor.run()

    stream_data = motor.get_stream_data()
    gap_us = motor.time_since_last_response_microseconds()

    if gap_us > max_gap:
        max_gap = gap_us

    if stream_data.position == 0:
        zero_count += 1

    total_reads += 1

    # Print every 100ms to reduce print overhead
    if total_reads % 50 == 0:
        print(f"{stream_data.position:12} | {gap_us:28} | {stream_data.errors}")

    time.sleep(0.002)  # 2ms delay to prevent overwhelming

print("\n" + "=" * 60)
print(f"Summary:")
print(f"  Max gap between responses: {max_gap} us ({max_gap/1000:.1f} ms)")
print(f"  Zero position readings: {zero_count} / {total_reads}")
print(f"  Communication timeout threshold: 500,000 us (500 ms)")

if max_gap > 500000:
    print("\n  WARNING: Max gap exceeds timeout! This explains the flickering.")
else:
    print(f"\n  Gap is within timeout. Issue may be stream initialization.")

motor.disable_stream()
motor.close_serial_port()
