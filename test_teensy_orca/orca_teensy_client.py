#!/usr/bin/env python3
"""
orca_teensy_client.py — Mac-side controller for the Teensy ORCA bridge.

Sends high-level commands to the Teensy over UDP and processes
telemetry / errors streamed back.

Usage example:
    python3 orca_teensy_client.py

By default this runs an interactive demo:
    1. CONNECT at 1M baud, 100 us interframe delay
    2. ENABLE_STREAM force=0 mN at 1 kHz
    3. Hold force=0 for 2 s while printing telemetry
    4. Ramp force to 500 mN over 2 s
    5. DISABLE_STREAM
    6. DISCONNECT

Use as a library by importing OrcaBridge.
"""

import argparse
import csv
import os
import queue
import socket
import statistics
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Callable

TEENSY_IP   = "192.168.1.177"
TEENSY_PORT = 8888
LOCAL_PORT  = 8889


@dataclass
class Telemetry:
    phase:         str   = ""
    position_um:   int   = 0
    force_mn:      int   = 0
    power_w:       int   = 0
    temperature_c: int   = 0
    voltage_mv:    int   = 0
    errors:        int   = 0
    timestamp:     float = field(default_factory=time.time)


@dataclass
class ExtTelemetry:
    """One cycle of extended telemetry: 0x64 run + read speed + read accel,
    with per-transaction Teensy-measured timing."""
    seq:               int   = 0
    t_run_us:          int   = 0
    t_speed_us:        int   = 0
    t_accel_us:        int   = 0
    t_total_us:        int   = 0
    position_um:       int   = 0
    force_mn:          int   = 0
    power_w:           int   = 0
    temperature_c:     int   = 0
    voltage_mv:        int   = 0
    errors:            int   = 0
    shaft_speed_mmps:  int   = 0
    shaft_accel_mmpss: int   = 0
    arrival_time:      float = 0.0   # Mac-side time.perf_counter() when received


class OrcaBridge:
    """Client for the Teensy ORCA bridge. Thread-safe, reads telemetry
    in a background thread."""

    def __init__(self,
                 teensy_ip: str = TEENSY_IP,
                 teensy_port: int = TEENSY_PORT,
                 local_port: int = LOCAL_PORT,
                 on_error: Optional[Callable[[str, str], None]] = None):
        self.teensy_addr = (teensy_ip, teensy_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.settimeout(0.5)

        self._latest_tele = Telemetry()
        self._tele_lock   = threading.Lock()
        self._errors_seen: list[tuple[float, str, str]] = []
        self._on_error    = on_error or self._default_on_error
        self._ack_events  = {}  # cmd_name -> threading.Event
        self._ack_lock    = threading.Lock()

        # Unbounded queue of EXT_TELEMETRY samples. Tests pull from this
        # while the receiver thread fills it. Unbounded is fine — at 1 kHz
        # for 10 seconds that's only ~10k small dataclass instances.
        self._ext_queue: queue.Queue[ExtTelemetry] = queue.Queue()
        self._ext_collecting = False
        self._ext_collect_lock = threading.Lock()

        self._stop  = threading.Event()
        self._rx_thread = threading.Thread(target=self._receiver, daemon=True)
        self._rx_thread.start()

    @staticmethod
    def _default_on_error(phase: str, msg: str):
        print(f"[ERROR @ {phase}] {msg}")

    # --------------------------------------------------------------
    # Internal: receive loop
    # --------------------------------------------------------------
    def _receiver(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                line = data.decode(errors="replace").strip()
            except Exception:
                continue
            self._handle_line(line)

    def _handle_line(self, line: str):
        parts = line.split()
        if not parts:
            return
        tag = parts[0]

        if tag == "TELEMETRY" and len(parts) >= 8:
            try:
                with self._tele_lock:
                    self._latest_tele = Telemetry(
                        phase         = parts[1],
                        position_um   = int(parts[2]),
                        force_mn      = int(parts[3]),
                        power_w       = int(parts[4]),
                        temperature_c = int(parts[5]),
                        voltage_mv    = int(parts[6]),
                        errors        = int(parts[7]),
                    )
            except ValueError:
                pass

        elif tag == "EXT_TELEMETRY" and len(parts) >= 14:
            # Format: EXT_TELEMETRY seq t_run t_speed t_accel t_total
            #         pos force power temp voltage errors speed accel
            try:
                ext = ExtTelemetry(
                    seq               = int(parts[1]),
                    t_run_us          = int(parts[2]),
                    t_speed_us        = int(parts[3]),
                    t_accel_us        = int(parts[4]),
                    t_total_us        = int(parts[5]),
                    position_um       = int(parts[6]),
                    force_mn          = int(parts[7]),
                    power_w           = int(parts[8]),
                    temperature_c     = int(parts[9]),
                    voltage_mv        = int(parts[10]),
                    errors            = int(parts[11]),
                    shaft_speed_mmps  = int(parts[12]),
                    shaft_accel_mmpss = int(parts[13]),
                    arrival_time      = time.perf_counter(),
                )
                with self._ext_collect_lock:
                    if self._ext_collecting:
                        self._ext_queue.put(ext)
            except ValueError:
                pass

        elif tag == "ERROR" and len(parts) >= 3:
            phase = parts[1]
            msg   = " ".join(parts[2:])
            self._errors_seen.append((time.time(), phase, msg))
            try:
                self._on_error(phase, msg)
            except Exception:
                pass

        elif tag == "ACK" and len(parts) >= 2:
            cmd = parts[1]
            with self._ack_lock:
                ev = self._ack_events.get(cmd)
            if ev is not None:
                ev.set()

        elif tag == "INFO":
            # Diagnostic messages from the Teensy — print so they're visible.
            print(f"[INFO] {' '.join(parts[1:])}")

    # --------------------------------------------------------------
    # Internal: send + wait for ACK
    # --------------------------------------------------------------
    def _send(self, line: str):
        self.sock.sendto(line.encode(), self.teensy_addr)

    def _send_and_ack(self, line: str, ack_name: str, timeout: float = 2.0) -> bool:
        ev = threading.Event()
        with self._ack_lock:
            self._ack_events[ack_name] = ev
        try:
            self._send(line)
            return ev.wait(timeout)
        finally:
            with self._ack_lock:
                self._ack_events.pop(ack_name, None)

    # --------------------------------------------------------------
    # Public API
    # --------------------------------------------------------------
    def ping(self, timeout: float = 1.0) -> bool:
        return self._send_and_ack("PING", "PING", timeout)

    def connect(self, baud: int = 1000000, interframe_us: int = 100,
                timeout: float = 3.0) -> bool:
        return self._send_and_ack(f"CONNECT {baud} {interframe_us}",
                                  "CONNECT", timeout)

    def enable_force_stream(self, force_mn: int = 0,
                            period_us: int = 1000,
                            timeout: float = 2.0) -> bool:
        # Sub-code 0x1C = Force Control Stream
        return self._send_and_ack(
            f"ENABLE_STREAM 1C {force_mn} {period_us}",
            "ENABLE_STREAM", timeout)

    def enable_extended_force_stream(self, force_mn: int = 0,
                                     period_us: int = 0,
                                     timeout: float = 2.0) -> bool:
        """Enable extended mode: each cycle does 0x64 + read speed + read accel.
        Per-cycle timing is reported in EXT_TELEMETRY messages."""
        return self._send_and_ack(
            f"ENABLE_EXT_STREAM 1C {force_mn} {period_us}",
            "ENABLE_EXT_STREAM", timeout)

    def start_ext_collection(self):
        """Start queueing incoming EXT_TELEMETRY samples for retrieval."""
        with self._ext_collect_lock:
            # Drain anything stale
            while not self._ext_queue.empty():
                try: self._ext_queue.get_nowait()
                except queue.Empty: break
            self._ext_collecting = True

    def stop_ext_collection(self) -> list[ExtTelemetry]:
        """Stop collection and return all queued samples in arrival order."""
        with self._ext_collect_lock:
            self._ext_collecting = False
        out = []
        while not self._ext_queue.empty():
            try:
                out.append(self._ext_queue.get_nowait())
            except queue.Empty:
                break
        return out

    def enable_sleep_stream(self, period_us: int = 1000,
                            timeout: float = 2.0) -> bool:
        # Sub-code 0x00 = Sleep Data Stream
        return self._send_and_ack(
            f"ENABLE_STREAM 00 0 {period_us}",
            "ENABLE_STREAM", timeout)

    def set_force(self, force_mn: int):
        # Hot path: fire-and-forget for low latency.
        self._send(f"SET {force_mn}")

    def sleep_motor(self, timeout: float = 2.0) -> bool:
        """Switch the active stream to Sleep sub-code (0x00). Motor stops
        generating forces but the stream stays alive, so the motor doesn't
        time out and assert error 2048. Use before disable_stream()."""
        return self._send_and_ack("SLEEP", "SLEEP", timeout)

    def disable_stream(self, timeout: float = 2.0) -> bool:
        return self._send_and_ack("DISABLE_STREAM", "DISABLE_STREAM", timeout)

    def disconnect(self, timeout: float = 3.0) -> bool:
        return self._send_and_ack("DISCONNECT", "DISCONNECT", timeout)

    def telemetry(self) -> Telemetry:
        with self._tele_lock:
            return self._latest_tele

    def errors(self) -> list[tuple[float, str, str]]:
        return list(self._errors_seen)

    def close(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass


# ==================================================================
# Demo runner
# ==================================================================
def demo():
    print("Starting ORCA Teensy bridge demo")
    bridge = OrcaBridge()
    phase_log = []

    def log_phase(name: str, ok: bool, note: str = ""):
        msg = f"  [{'OK' if ok else 'FAIL'}] {name}"
        if note: msg += f" — {note}"
        print(msg)
        phase_log.append((name, ok, note))

    try:
        # ---- Sanity: confirm Teensy is reachable
        if not bridge.ping():
            log_phase("PING", False, "no response — is Teensy running?")
            return
        log_phase("PING", True)

        # ---- Connect
        if not bridge.connect(baud=1000000, interframe_us=100):
            log_phase("CONNECT", False, "did not ACK")
            return
        log_phase("CONNECT", True)

        # ---- Enable force stream at 0 mN, 1 kHz
        if not bridge.enable_force_stream(force_mn=0, period_us=1000):
            log_phase("ENABLE_STREAM", False)
            return
        log_phase("ENABLE_STREAM", True)

        # ---- Hold for 2s, print telemetry occasionally
        t0 = time.time()
        while time.time() - t0 < 2.0:
            t = bridge.telemetry()
            print(f"    pos={t.position_um}µm  force={t.force_mn}mN  "
                  f"temp={t.temperature_c}°C  V={t.voltage_mv}mV  "
                  f"err=0x{t.errors:04X}")
            time.sleep(0.25)

        # ---- Ramp force 0 → 500 mN over 2s
        print("  Ramping force 0 → 500 mN over 2 s …")
        ramp_start = time.time()
        ramp_duration = 2.0
        while True:
            elapsed = time.time() - ramp_start
            if elapsed >= ramp_duration: break
            force = int(500 * elapsed / ramp_duration)
            bridge.set_force(force)
            time.sleep(0.01)  # 100 Hz update rate from Python
        bridge.set_force(0)  # back to zero
        log_phase("RAMP", True)

        # ---- Put motor to sleep before disabling stream so it exits Force
        # Mode cleanly. Otherwise the comms-timeout timer (page 14 of the
        # user guide) will fire and raise error 2048 once we stop streaming.
        if not bridge.sleep_motor():
            log_phase("SLEEP", False)
        else:
            log_phase("SLEEP", True)
        # Give a few stream frames time to land at the motor.
        time.sleep(0.05)

        # ---- Disable stream
        if not bridge.disable_stream():
            log_phase("DISABLE_STREAM", False)
        else:
            log_phase("DISABLE_STREAM", True)

        # ---- Disconnect
        if not bridge.disconnect():
            log_phase("DISCONNECT", False)
        else:
            log_phase("DISCONNECT", True)

    finally:
        # ---- Error summary
        errs = bridge.errors()
        if errs:
            print("\n=== Errors observed during run ===")
            for ts, phase, msg in errs:
                print(f"  t={ts:.3f}  phase={phase}  {msg}")
        else:
            print("\nNo errors observed.")
        bridge.close()


def run_timing_test(duration_s: float = 10.0,
                    force_mn: int = 0,
                    output_prefix: str = "timing_test"):
    """Run the extended-stream timing test for `duration_s` seconds.
    At the end, print summary stats, save CSV of raw samples, and save
    a PNG of histograms."""
    print(f"=== Timing test: {duration_s}s, force={force_mn} mN, "
          f"period=as-fast-as-possible ===\n")
    bridge = OrcaBridge()

    try:
        if not bridge.ping():
            print("  [FAIL] PING — is Teensy running?")
            return
        print("  [OK] PING")

        if not bridge.connect(baud=1000000, interframe_us=100):
            print("  [FAIL] CONNECT")
            return
        print("  [OK] CONNECT")

        # Start collecting BEFORE enabling the stream so we don't miss the
        # very first samples.
        bridge.start_ext_collection()

        if not bridge.enable_extended_force_stream(force_mn=force_mn, period_us=0):
            print("  [FAIL] ENABLE_EXT_STREAM")
            bridge.stop_ext_collection()
            return
        print(f"  [OK] ENABLE_EXT_STREAM — collecting for {duration_s}s …")

        # Let the Teensy push samples; just sleep on the Mac side.
        time.sleep(duration_s)

        samples = bridge.stop_ext_collection()
        print(f"  Collected {len(samples)} samples.\n")

        # Clean shutdown: sleep motor, stop stream, disconnect.
        bridge.sleep_motor()
        time.sleep(0.05)
        bridge.disable_stream()
        bridge.disconnect()

    finally:
        errs = bridge.errors()
        if errs:
            print("=== Errors observed during run ===")
            for ts, phase, msg in errs:
                print(f"  t={ts:.3f}  phase={phase}  {msg}")
        bridge.close()

    if not samples:
        print("No samples collected — nothing to analyze.")
        return

    # ---- Drop-detection via sequence numbers ----
    seqs = [s.seq for s in samples]
    expected_count = seqs[-1] - seqs[0] + 1
    dropped = expected_count - len(seqs)
    print(f"Sequence span: {seqs[0]}..{seqs[-1]} "
          f"({expected_count} expected, {len(seqs)} received, "
          f"{dropped} dropped on UDP)")

    # ---- Stats ----
    def summarize(label: str, values: list[int], unit: str = "µs"):
        if not values:
            print(f"  {label}: (no data)")
            return
        mean = statistics.mean(values)
        median = statistics.median(values)
        stdev = statistics.stdev(values) if len(values) > 1 else 0.0
        print(f"  {label}:")
        print(f"    n={len(values)}  mean={mean:.2f}{unit}  "
              f"median={median:.2f}{unit}  stdev={stdev:.2f}{unit}")
        print(f"    min={min(values)}{unit}  max={max(values)}{unit}")
        if len(values) >= 20:
            q = statistics.quantiles(values, n=100)
            print(f"    p95={q[94]:.2f}{unit}  p99={q[98]:.2f}{unit}")

    print("\n=== Per-transaction timing (Teensy-measured) ===")
    summarize("motor.run() (0x64)",       [s.t_run_us   for s in samples])
    summarize("read shaft speed (0x03)",  [s.t_speed_us for s in samples])
    summarize("read shaft accel (0x03)",  [s.t_accel_us for s in samples])
    summarize("TOTAL cycle",              [s.t_total_us for s in samples])
    duration_actual = (samples[-1].arrival_time - samples[0].arrival_time)
    if duration_actual > 0:
        achieved_rate = len(samples) / duration_actual
        print(f"\n  Achieved cycle rate: {achieved_rate:.1f} Hz "
              f"({1e6/achieved_rate:.1f} µs/cycle)")

    # ---- Save CSV ----
    csv_path = f"{output_prefix}.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "arrival_time_s",
                    "t_run_us", "t_speed_us", "t_accel_us", "t_total_us",
                    "position_um", "force_mn", "power_w", "temperature_c",
                    "voltage_mv", "errors", "shaft_speed_mmps", "shaft_accel_mmpss"])
        for s in samples:
            w.writerow([s.seq, f"{s.arrival_time:.6f}",
                        s.t_run_us, s.t_speed_us, s.t_accel_us, s.t_total_us,
                        s.position_um, s.force_mn, s.power_w, s.temperature_c,
                        s.voltage_mv, s.errors, s.shaft_speed_mmps, s.shaft_accel_mmpss])
    print(f"\nSaved raw samples → {csv_path}")

    # ---- Plot histograms ----
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — skipping plots. (pip3 install matplotlib)")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()

    series = [
        ("motor.run() (0x64)",      [s.t_run_us   for s in samples]),
        ("read shaft speed (0x03)", [s.t_speed_us for s in samples]),
        ("read shaft accel (0x03)", [s.t_accel_us for s in samples]),
        ("TOTAL cycle",             [s.t_total_us for s in samples]),
    ]
    for ax, (label, values) in zip(axes, series):
        ax.hist(values, bins=60, edgecolor="black")
        ax.set_xlabel("Time (µs)")
        ax.set_ylabel("Count")
        ax.set_title(f"{label}  (n={len(values)})")
        mean = statistics.mean(values)
        ax.axvline(mean, color="red", linestyle="--",
                   label=f"mean={mean:.1f} µs")
        ax.legend()

    plt.tight_layout()
    png_path = f"{output_prefix}.png"
    plt.savefig(png_path, dpi=100)
    print(f"Saved plot     → {png_path}")
    plt.show()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true",
                   help="Run the canned force-ramp demo")
    p.add_argument("--timing-test", action="store_true",
                   help="Run the extended-stream timing test")
    p.add_argument("--duration", type=float, default=10.0,
                   help="Duration in seconds for --timing-test (default 10)")
    p.add_argument("--force", type=int, default=0,
                   help="Force in mN during timing test (default 0)")
    p.add_argument("--output", type=str, default="timing_test",
                   help="Output filename prefix (default 'timing_test')")
    args = p.parse_args()

    if args.timing_test:
        run_timing_test(duration_s=args.duration,
                        force_mn=args.force,
                        output_prefix=args.output)
    else:
        # Default: run the demo (preserves old behavior)
        demo()


if __name__ == "__main__":
    main()