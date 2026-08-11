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
    rtt_seq:           int   = 0   # 0 = untagged; nonzero = Python's seq for this cycle
    calc_accel_window: int   = 0   # 0 = accel from register 346; >=2 = slope of N speed samples
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
            #         [rtt_seq] [calc_accel_window]
            # Trailing fields are optional (older Teensy firmware omits them).
            try:
                rtt = int(parts[14]) if len(parts) >= 15 else 0
                caw = int(parts[15]) if len(parts) >= 16 else 0
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
                    rtt_seq           = rtt,
                    calc_accel_window = caw,
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
                                     calc_accel_window: int = 0,
                                     timeout: float = 2.0) -> bool:
        """Enable extended mode: each cycle does 0x64 + read speed + (accel).

        calc_accel_window selects the acceleration source:
          0    -> read the motor's accel register 346 (t_accel_us = Modbus read)
          >=2  -> compute accel from a causal least-squares slope of the last N
                  speed samples (t_accel_us = computation; no register read)
        Per-cycle timing is reported in EXT_TELEMETRY messages."""
        return self._send_and_ack(
            f"ENABLE_EXT_STREAM 1C {force_mn} {period_us} {calc_accel_window}",
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

    def set_force_rtt(self, rtt_seq: int, force_mn: int):
        """Tagged variant of set_force — the next cycle's EXT_TELEMETRY
        will echo rtt_seq, letting Python measure end-to-end RTT.
        Fire-and-forget. rtt_seq must be > 0 (0 means 'no tag')."""
        self._send(f"SET_RTT {rtt_seq} {force_mn}")

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
        if not bridge.connect(baud=1000000, interframe_us=0):
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
                    calc_accel_window: int = 0,
                    output_prefix: str = "timing_test"):
    """Run the extended-stream timing test for `duration_s` seconds.
    At the end, print summary stats, save CSV of raw samples, and save
    a PNG of histograms.

    calc_accel_window == 0 reads the accel register; >=2 computes accel from a
    trailing-window slope of speed. The t_accel_us column times whichever ran."""
    accel_src = (f"rolling-window slope (W={calc_accel_window})"
                 if calc_accel_window >= 2 else "register 346 read")
    print(f"=== Timing test: {duration_s}s, force={force_mn} mN, "
          f"period=as-fast-as-possible, accel={accel_src} ===\n")
    bridge = OrcaBridge()

    try:
        if not bridge.ping():
            print("  [FAIL] PING — is Teensy running?")
            return
        print("  [OK] PING")

        if not bridge.connect(baud=1000000, interframe_us=0):
            print("  [FAIL] CONNECT")
            return
        print("  [OK] CONNECT")

        # Start collecting BEFORE enabling the stream so we don't miss the
        # very first samples.
        bridge.start_ext_collection()

        if not bridge.enable_extended_force_stream(
                force_mn=force_mn, period_us=0,
                calc_accel_window=calc_accel_window):
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

    # Confirm which accel path the Teensy ACTUALLY ran (from the data, not what we
    # asked for). If this disagrees with --calc-accel-window the firmware is stale
    # or the arg didn't arrive, and t_accel_us reflects the register read.
    reported = {s.calc_accel_window for s in samples}
    print(f"Requested calc_accel_window={calc_accel_window}; "
          f"Teensy reported {sorted(reported)}")
    if calc_accel_window >= 2 and reported == {0}:
        print("  !! Teensy ran REGISTER mode despite the request — reflash the "
              ".ino and confirm the 'INFO accel=rolling_window' line on enable.")

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

    accel_label = (f"accel: rolling slope W={calc_accel_window}"
                   if calc_accel_window >= 2 else "accel: register 346 read")
    print("\n=== Per-transaction timing (Teensy-measured) ===")
    summarize("motor.run() (0x64)",       [s.t_run_us   for s in samples])
    summarize("read shaft speed (0x03)",  [s.t_speed_us for s in samples])
    summarize(accel_label,                [s.t_accel_us for s in samples])
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
                    "voltage_mv", "errors", "shaft_speed_mmps", "shaft_accel_mmpss",
                    "calc_accel_window"])
        for s in samples:
            w.writerow([s.seq, f"{s.arrival_time:.6f}",
                        s.t_run_us, s.t_speed_us, s.t_accel_us, s.t_total_us,
                        s.position_um, s.force_mn, s.power_w, s.temperature_c,
                        s.voltage_mv, s.errors, s.shaft_speed_mmps, s.shaft_accel_mmpss,
                        s.calc_accel_window])
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
        (accel_label,               [s.t_accel_us for s in samples]),
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


def run_rtt_test(duration_s: float = 10.0,
                 send_period_ms: float = 5.0,
                 force_amplitude_mn: int = 100,
                 output_prefix: str = "rtt_test"):
    """Measure end-to-end RTT: Python sends a tagged force update, the
    Teensy applies it on its next cycle, EXT_TELEMETRY comes back tagged.
    RTT = (telemetry_arrival - tagged_send_time).

    The force value alternates between +force_amplitude_mn and
    -force_amplitude_mn each send so that every SET_RTT actually changes
    something — keeps the test honest. Use force_amplitude_mn = 0 to send
    zero forces (still measures RTT, motor just stays idle).

    send_period_ms controls how often Python fires a tagged update. Should
    be larger than the Teensy's cycle floor (~1.5 ms for ext mode at 1 Mbps)
    so each send is consumed by a fresh cycle, not collapsed with prior ones.
    """
    print(f"=== RTT test: duration={duration_s}s, "
          f"send_period={send_period_ms}ms, "
          f"force_amplitude=±{force_amplitude_mn} mN ===\n")
    bridge = OrcaBridge()

    # Map from rtt_seq → send timestamp (perf_counter)
    sent_at: dict[int, float] = {}
    sent_lock = threading.Lock()

    try:
        if not bridge.ping():
            print("  [FAIL] PING — is Teensy running?")
            return
        print("  [OK] PING")

        if not bridge.connect(baud=1000000, interframe_us=0):
            print("  [FAIL] CONNECT")
            return
        print("  [OK] CONNECT")

        bridge.start_ext_collection()

        if not bridge.enable_extended_force_stream(force_mn=0, period_us=0):
            print("  [FAIL] ENABLE_EXT_STREAM")
            bridge.stop_ext_collection()
            return
        print(f"  [OK] ENABLE_EXT_STREAM — measuring RTT for {duration_s}s …\n")

        # ---- Send tagged updates at the configured rate ----
        rtt_seq = 0
        send_interval_s = send_period_ms / 1000.0
        next_send = time.perf_counter()
        end_time  = next_send + duration_s

        # Sender runs in main thread; collector in bridge's RX thread fills
        # the queue. We pull from queue here periodically to keep memory
        # bounded — but for 10s × 200 Hz that's only 2000 samples, fine.
        while True:
            now = time.perf_counter()
            if now >= end_time:
                break
            if now >= next_send:
                rtt_seq += 1
                # Alternate the sign so the force value actually changes
                force = force_amplitude_mn if (rtt_seq % 2) else -force_amplitude_mn
                with sent_lock:
                    sent_at[rtt_seq] = time.perf_counter()
                bridge.set_force_rtt(rtt_seq, force)
                next_send += send_interval_s
                # If we've fallen behind, don't try to catch up — just
                # skip to the next slot.
                if next_send < now:
                    next_send = now + send_interval_s
            time.sleep(0.0002)  # 200 µs poll granularity

        # Give the last few in-flight responses time to arrive
        time.sleep(0.05)
        samples = bridge.stop_ext_collection()
        print(f"  Total telemetry samples received: {len(samples)}")
        print(f"  Tagged sends:                     {rtt_seq}")

        # Sleep motor & clean disconnect
        bridge.sleep_motor()
        time.sleep(0.05)
        bridge.disable_stream()
        bridge.disconnect()

    finally:
        errs = bridge.errors()
        if errs:
            print("\n=== Errors during run ===")
            for ts, phase, msg in errs:
                print(f"  t={ts:.3f}  phase={phase}  {msg}")
        bridge.close()

    if not samples:
        print("No samples — nothing to analyze.")
        return

    # ---- Match tagged sends to their responses ----
    rtts_us: list[float] = []
    matched_samples: list[tuple[ExtTelemetry, float]] = []
    untagged_count = 0
    for s in samples:
        if s.rtt_seq == 0:
            untagged_count += 1
            continue
        with sent_lock:
            send_time = sent_at.get(s.rtt_seq)
        if send_time is None:
            # Tag we didn't send (shouldn't happen unless Teensy state
            # carried over from a prior run); skip.
            continue
        rtt = (s.arrival_time - send_time) * 1_000_000  # → µs
        rtts_us.append(rtt)
        matched_samples.append((s, rtt))

    print(f"  Tagged responses matched:         {len(rtts_us)}")
    print(f"  Untagged (intervening) cycles:    {untagged_count}")
    if rtt_seq > len(rtts_us):
        print(f"  Lost/unmatched tagged sends:      {rtt_seq - len(rtts_us)}")

    if not rtts_us:
        print("\nNo RTT samples to plot — was anything actually tagged?")
        return

    # ---- RTT stats ----
    def summarize(label, values, unit="µs"):
        if not values:
            print(f"  {label}: (no data)")
            return
        mean = statistics.mean(values)
        median = statistics.median(values)
        stdev = statistics.stdev(values) if len(values) > 1 else 0.0
        print(f"  {label}:")
        print(f"    n={len(values)}  mean={mean:.1f}{unit}  "
              f"median={median:.1f}{unit}  stdev={stdev:.1f}{unit}")
        print(f"    min={min(values):.1f}{unit}  max={max(values):.1f}{unit}")
        if len(values) >= 20:
            q = statistics.quantiles(values, n=100)
            print(f"    p95={q[94]:.1f}{unit}  p99={q[98]:.1f}{unit}")

    print("\n=== End-to-end RTT (Python send → Python receive) ===")
    summarize("RTT", rtts_us)

    # Also report Teensy-measured cycle times for context
    print("\n=== Teensy cycle times (for reference) ===")
    summarize("TOTAL cycle (Teensy)", [s.t_total_us for s in samples])

    # ---- Save CSV ----
    csv_path = f"{output_prefix}.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rtt_seq", "rtt_us", "teensy_seq", "t_total_us",
                    "t_run_us", "t_speed_us", "t_accel_us",
                    "position_um", "force_mn", "shaft_speed_mmps",
                    "shaft_accel_mmpss", "errors"])
        for s, rtt in matched_samples:
            w.writerow([s.rtt_seq, f"{rtt:.1f}",
                        s.seq, s.t_total_us, s.t_run_us, s.t_speed_us, s.t_accel_us,
                        s.position_um, s.force_mn,
                        s.shaft_speed_mmps, s.shaft_accel_mmpss, s.errors])
    print(f"\nSaved raw → {csv_path}")

    # ---- Plot ----
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — skipping plot. (pip3 install matplotlib)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    # Panel 1: RTT histogram
    axes[0].hist(rtts_us, bins=60, edgecolor="black")
    axes[0].set_xlabel("Round-trip time (µs)")
    axes[0].set_ylabel("Count")
    axes[0].set_title(f"End-to-end RTT  (n={len(rtts_us)})")
    mean = statistics.mean(rtts_us)
    axes[0].axvline(mean, color="red", linestyle="--",
                    label=f"mean={mean:.0f} µs")
    axes[0].legend()

    # Panel 2: Teensy cycle time histogram (for comparison)
    cycle_us = [s.t_total_us for s in samples]
    axes[1].hist(cycle_us, bins=60, edgecolor="black")
    axes[1].set_xlabel("Teensy cycle time (µs)")
    axes[1].set_ylabel("Count")
    axes[1].set_title(f"Teensy cycle  (n={len(cycle_us)})")
    mean_c = statistics.mean(cycle_us)
    axes[1].axvline(mean_c, color="red", linestyle="--",
                    label=f"mean={mean_c:.0f} µs")
    axes[1].legend()

    plt.tight_layout()
    png_path = f"{output_prefix}.png"
    plt.savefig(png_path, dpi=100)
    print(f"Saved plot → {png_path}")
    plt.show()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true",
                   help="Run the canned force-ramp demo")
    p.add_argument("--timing-test", action="store_true",
                   help="Run the extended-stream timing test "
                        "(Teensy-side per-transaction timing)")
    p.add_argument("--rtt-test", action="store_true",
                   help="Run the end-to-end RTT test "
                        "(Python send → Python receive)")
    p.add_argument("--duration", type=float, default=10.0,
                   help="Duration in seconds (default 10)")
    p.add_argument("--force", type=int, default=0,
                   help="Force in mN for --timing-test (default 0), "
                        "or amplitude for --rtt-test (default 100 if 0)")
    p.add_argument("--send-period-ms", type=float, default=5.0,
                   help="Tagged-send interval for --rtt-test (default 5.0 ms)")
    p.add_argument("--calc-accel-window", type=int, default=0,
                   help="For --timing-test: 0 reads the accel register (default); "
                        ">=2 computes accel from a trailing-window slope of that "
                        "many speed samples")
    p.add_argument("--output", type=str, default=None,
                   help="Output filename prefix "
                        "(default: 'timing_test' or 'rtt_test' depending on mode)")
    args = p.parse_args()

    if args.rtt_test:
        prefix = args.output or "rtt_test"
        # For RTT test, default a nonzero force amplitude so each tagged
        # send actually does something
        amplitude = args.force if args.force != 0 else 100
        run_rtt_test(duration_s=args.duration,
                     send_period_ms=args.send_period_ms,
                     force_amplitude_mn=amplitude,
                     output_prefix=prefix)
    elif args.timing_test:
        prefix = args.output or "timing_test"
        run_timing_test(duration_s=args.duration,
                        force_mn=args.force,
                        calc_accel_window=args.calc_accel_window,
                        output_prefix=prefix)
    else:
        # Default: run the demo (preserves old behavior)
        demo()


if __name__ == "__main__":
    main()