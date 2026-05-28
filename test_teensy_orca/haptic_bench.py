#!/usr/bin/env python3
"""
haptic_bench.py — Mac-side bench client for teensy_cartpole_haptic.

Binary UDP protocol.  Sends CONFIG/INIT/BEGIN, collects packed telemetry,
saves to HDF5, prints summary stats.

Usage:
    python3 haptic_bench.py                    # default 10s run
    python3 haptic_bench.py --duration 30
    python3 haptic_bench.py --ping-only
    python3 haptic_bench.py --autozero
"""

import argparse
import struct
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# =====================================================================
# Network
# =====================================================================
TEENSY_IP   = "192.168.1.177"
TEENSY_PORT = 8888
LOCAL_PORT  = 8889

# =====================================================================
# Binary protocol constants (must match teensy_cartpole_haptic.ino)
# =====================================================================
CMD_PING     = 0x01
CMD_AUTOZERO = 0x02
CMD_CONFIG   = 0x03
CMD_INIT     = 0x04
CMD_BEGIN    = 0x05
CMD_END      = 0x06
CMD_SLEEP    = 0x07

REPLY_ACK   = 0xA1
REPLY_ERROR = 0xA2
TELE_BATCH  = 0xB0
TELE_FINAL  = 0xB1
DEBUG_INFO  = 0xD0

ERR_NAMES = {
    0x01: "ERR_BADOP",
    0x02: "ERR_BADLEN",
    0x03: "ERR_MOTOR_FAULT",
    0x04: "ERR_SIGN_CHECK",
    0x05: "ERR_NAN_STATE",
    0x06: "ERR_MODBUS_TIMEOUT",
    0x07: "ERR_NOT_CONFIGURED",
}

# Sample struct layout (must match teensy_cartpole_haptic.ino):
# cycle(u32) t_meas_us(u32) x(f) xdot(f) xddot(f) theta(f) thetadot(f)
# fx(f) fpc(f) f_command_mN(f) force_sensed_mN(f) loop_us(u32)
SAMPLE_FMT  = "<II9fI"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)
assert SAMPLE_SIZE == 52, f"Expected 52, got {SAMPLE_SIZE}"

SAMPLE_FIELDS = [
    "cycle", "t_meas_us",
    "x", "xdot", "xddot",
    "theta", "thetadot",
    "fx", "fpc", "f_command_mN", "force_sensed_mN",
    "loop_us",
]


@dataclass
class Sample:
    cycle:           int   = 0
    t_meas_us:       int   = 0
    x:               float = 0.0
    xdot:            float = 0.0
    xddot:           float = 0.0
    theta:           float = 0.0
    thetadot:        float = 0.0
    fx:              float = 0.0
    fpc:             float = 0.0
    f_command_mN:    float = 0.0
    force_sensed_mN: float = 0.0
    loop_us:         int   = 0
    arrival_time:    float = 0.0


def unpack_samples(data: bytes, count: int) -> list[Sample]:
    out = []
    t = time.perf_counter()
    for i in range(count):
        off = i * SAMPLE_SIZE
        if off + SAMPLE_SIZE > len(data):
            break
        vals = struct.unpack_from(SAMPLE_FMT, data, off)
        s = Sample(*vals)
        s.arrival_time = t
        out.append(s)
    return out


# =====================================================================
# Client
# =====================================================================
class HapticBench:
    def __init__(self,
                 teensy_ip: str = TEENSY_IP,
                 teensy_port: int = TEENSY_PORT,
                 local_port: int = LOCAL_PORT):
        self.teensy_addr = (teensy_ip, teensy_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(("0.0.0.0", local_port))
        self.sock.settimeout(0.5)

        self._samples: list[Sample] = []
        self._samples_lock = threading.Lock()
        self._collecting = False

        self._final_sample: Sample | None = None
        self._final_total_cycles: int = 0

        self._ack_events: dict[int, threading.Event] = {}
        self._ack_data: dict[int, bytes] = {}  # extra bytes from ACK replies
        self._ack_lock = threading.Lock()
        self._errors: list[tuple[float, int, bytes]] = []
        self._infos: list[str] = []

        # Clock sync: offset in microseconds such that
        #   mac_perf_counter_us = teensy_micros + clock_offset_us
        # None until clock_sync() is called.
        self.clock_offset_us: float | None = None

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._receiver, daemon=True)
        self._rx_thread.start()

    def _receiver(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                continue
            self._handle_packet(data)

    def _handle_packet(self, data: bytes):
        tag = data[0]

        if tag == REPLY_ACK and len(data) >= 2:
            opcode = data[1]
            with self._ack_lock:
                ev = self._ack_events.get(opcode)
                self._ack_data[opcode] = data[2:]  # extra bytes (e.g. PING timestamp)
            if ev is not None:
                ev.set()

        elif tag == REPLY_ERROR and len(data) >= 2:
            err_code = data[1]
            name = ERR_NAMES.get(err_code, f"0x{err_code:02X}")
            extra = data[2:] if len(data) > 2 else b""
            self._errors.append((time.time(), err_code, extra))
            print(f"[ERROR] {name}" +
                  (f" extra={extra.hex()}" if extra else ""))

        elif tag == TELE_BATCH and len(data) >= 2:
            count = data[1]
            samples = unpack_samples(data[2:], count)
            with self._samples_lock:
                if self._collecting:
                    self._samples.extend(samples)

        elif tag == TELE_FINAL and len(data) >= 1 + SAMPLE_SIZE + 4:
            vals = struct.unpack_from(SAMPLE_FMT, data, 1)
            self._final_sample = Sample(*vals)
            self._final_total_cycles = struct.unpack_from(
                "<I", data, 1 + SAMPLE_SIZE)[0]

        elif tag == DEBUG_INFO:
            msg = data[1:].decode(errors="replace").strip()
            self._infos.append(msg)
            print(f"[INFO] {msg}")

    # ------------------------------------------------------------------
    # Send helpers
    # ------------------------------------------------------------------
    def _send(self, payload: bytes):
        self.sock.sendto(payload, self.teensy_addr)

    def _send_and_ack(self, opcode: int, payload: bytes = b"",
                      timeout: float = 2.0) -> bool:
        ev = threading.Event()
        with self._ack_lock:
            self._ack_events[opcode] = ev
            self._ack_data.pop(opcode, None)
        try:
            self._send(bytes([opcode]) + payload)
            return ev.wait(timeout)
        finally:
            with self._ack_lock:
                self._ack_events.pop(opcode, None)

    def _get_ack_data(self, opcode: int) -> bytes:
        with self._ack_lock:
            return self._ack_data.get(opcode, b"")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def ping(self, timeout: float = 1.0) -> bool:
        return self._send_and_ack(CMD_PING, timeout=timeout)

    def clock_sync(self, rounds: int = 11) -> float:
        """NTP-style clock sync over PING.

        Each round: Mac records t1 (perf_counter), sends PING, Teensy
        replies with its micros(), Mac records t2.  One-way latency is
        estimated as RTT/2.  The round with the lowest RTT is the most
        symmetric, so we use it to compute the offset.

        Returns the offset in microseconds:
            mac_perf_counter_us = teensy_micros + offset
        so to convert a teensy timestamp to Mac time:
            mac_time = (teensy_us + offset) * 1e-6  (seconds, perf_counter epoch)
        """
        best_rtt = float("inf")
        best_offset = 0.0

        for _ in range(rounds):
            t1 = time.perf_counter()
            ok = self._send_and_ack(CMD_PING, timeout=1.0)
            t2 = time.perf_counter()
            if not ok:
                continue

            extra = self._get_ack_data(CMD_PING)
            if len(extra) < 4:
                continue

            teensy_us = struct.unpack_from("<I", extra, 0)[0]
            rtt_s = t2 - t1
            rtt_us = rtt_s * 1e6
            mac_mid_us = ((t1 + t2) / 2.0) * 1e6
            offset = mac_mid_us - teensy_us

            if rtt_us < best_rtt:
                best_rtt = rtt_us
                best_offset = offset

            time.sleep(0.01)

        self.clock_offset_us = best_offset
        print(f"[CLOCK] synced: offset={best_offset:.0f} us, "
              f"best RTT={best_rtt:.0f} us")
        return best_offset

    def autozero(self, timeout: float = 8.0) -> bool:
        return self._send_and_ack(CMD_AUTOZERO, timeout=timeout)

    def config(self, m_c: float, m_p: float, m_s: float,
               l: float, g: float = 9.81,
               max_force_mN: float = 5000.0,
               loop_period_us: float = 1800.0,
               timeout: float = 3.0) -> bool:
        payload = struct.pack("<7f", m_c, m_p, m_s, l, g,
                              max_force_mN, loop_period_us)
        return self._send_and_ack(CMD_CONFIG, payload, timeout)

    def init_state(self, x0: float = 0.0, theta0: float = 0.0,
                   xdot0: float = 0.0, thetadot0: float = 0.0,
                   timeout: float = 2.0) -> bool:
        payload = struct.pack("<4f", x0, theta0, xdot0, thetadot0)
        return self._send_and_ack(CMD_INIT, payload, timeout)

    def begin(self, timeout: float = 3.0) -> bool:
        with self._samples_lock:
            self._samples.clear()
            self._collecting = True
        self._final_sample = None
        self._final_total_cycles = 0
        return self._send_and_ack(CMD_BEGIN, timeout=timeout)

    def end(self, timeout: float = 2.0) -> bool:
        ok = self._send_and_ack(CMD_END, timeout=timeout)
        with self._samples_lock:
            self._collecting = False
        return ok

    def sleep_motor(self, timeout: float = 2.0) -> bool:
        return self._send_and_ack(CMD_SLEEP, timeout=timeout)

    def get_samples(self) -> list[Sample]:
        with self._samples_lock:
            return list(self._samples)

    @property
    def errors(self):
        return list(self._errors)

    def close(self):
        self._stop.set()
        try:
            self.sock.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # HDF5 save
    # ------------------------------------------------------------------
    def save_h5(self, path: str, samples: list[Sample] | None = None,
                attrs: dict | None = None):
        import h5py
        if samples is None:
            samples = self.get_samples()
        if not samples:
            print("No samples to save.")
            return

        n = len(samples)
        dt = np.dtype([
            ("cycle",           np.uint32),
            ("t_meas_us",       np.uint32),
            ("x",               np.float32),
            ("xdot",            np.float32),
            ("xddot",           np.float32),
            ("theta",           np.float32),
            ("thetadot",        np.float32),
            ("fx",              np.float32),
            ("fpc",             np.float32),
            ("f_command_mN",    np.float32),
            ("force_sensed_mN", np.float32),
            ("loop_us",         np.uint32),
            ("arrival_time",    np.float64),
        ])

        arr = np.empty(n, dtype=dt)
        for i, s in enumerate(samples):
            arr[i] = (s.cycle, s.t_meas_us,
                      s.x, s.xdot, s.xddot,
                      s.theta, s.thetadot,
                      s.fx, s.fpc, s.f_command_mN,
                      s.force_sensed_mN,
                      s.loop_us, s.arrival_time)

        with h5py.File(path, "w") as f:
            ds = f.create_dataset("samples", data=arr)
            if attrs:
                for k, v in attrs.items():
                    ds.attrs[k] = v
            if self._final_sample is not None:
                ds.attrs["final_total_cycles"] = self._final_total_cycles

        print(f"Saved {n} samples to {path}")


# =====================================================================
# Summary stats
# =====================================================================
def print_summary(samples: list[Sample], clock_offset_us: float | None = None):
    if not samples:
        print("No samples.")
        return

    cycles = [s.cycle for s in samples]
    expected = cycles[-1] - cycles[0] + 1
    received = len(cycles)
    dropped = expected - received
    print(f"\nCycle span: {cycles[0]}..{cycles[-1]} "
          f"({expected} expected, {received} received, {dropped} dropped)")

    loop_us = np.array([s.loop_us for s in samples])
    print(f"\nLoop timing (us):")
    print(f"  mean={loop_us.mean():.1f}  median={np.median(loop_us):.1f}  "
          f"std={loop_us.std():.1f}")
    print(f"  min={loop_us.min()}  max={loop_us.max()}")
    if len(loop_us) >= 20:
        print(f"  p95={np.percentile(loop_us, 95):.1f}  "
              f"p99={np.percentile(loop_us, 99):.1f}")

    overruns = np.sum(loop_us > 1800)
    print(f"  overruns (>1800us): {overruns} ({100*overruns/len(loop_us):.2f}%)")

    duration_s = (samples[-1].arrival_time - samples[0].arrival_time)
    if duration_s > 0:
        rate = received / duration_s
        print(f"\n  Achieved rate: {rate:.1f} Hz")

    # Teensy→Mac latency (requires clock sync)
    if clock_offset_us is not None:
        latency_us = np.array([
            s.arrival_time * 1e6 - (s.t_meas_us + clock_offset_us)
            for s in samples
        ])
        print(f"\nTeensy→Mac latency (us) [meas → Mac receipt, includes batching]:")
        print(f"  mean={latency_us.mean():.0f}  median={np.median(latency_us):.0f}  "
              f"std={latency_us.std():.0f}")
        print(f"  min={latency_us.min():.0f}  max={latency_us.max():.0f}")
        if len(latency_us) >= 20:
            print(f"  p95={np.percentile(latency_us, 95):.0f}  "
                  f"p99={np.percentile(latency_us, 99):.0f}")
        over_16ms = np.sum(latency_us > 16000)
        print(f"  samples >16ms (missed 60Hz frame): "
              f"{over_16ms} ({100*over_16ms/len(latency_us):.2f}%)")

    theta = np.array([s.theta for s in samples])
    f_cmd = np.array([s.f_command_mN for s in samples])
    print(f"\nPendulum angle (rad):")
    print(f"  mean={theta.mean():.4f}  std={theta.std():.4f}  "
          f"min={theta.min():.4f}  max={theta.max():.4f}")
    print(f"\nCommand force (mN):")
    print(f"  mean={f_cmd.mean():.1f}  std={f_cmd.std():.1f}  "
          f"min={f_cmd.min():.1f}  max={f_cmd.max():.1f}")


# =====================================================================
# Demo runner
# =====================================================================
def run(duration_s: float = 10.0,
        m_c: float = 1.0,
        m_p: float = 0.1,
        m_s: float = 0.5,
        l: float = 0.5,
        theta0: float = 0.1,
        max_force_mN: float = 5000.0,
        output: str | None = None):

    bench = HapticBench()

    try:
        # -- Ping + clock sync
        if not bench.ping():
            print("[FAIL] PING")
            return
        print("[OK] PING")
        bench.clock_sync()

        # -- Config
        if not bench.config(m_c=m_c, m_p=m_p, m_s=m_s, l=l,
                            max_force_mN=max_force_mN):
            print("[FAIL] CONFIG")
            return
        print("[OK] CONFIG")

        # -- Init
        if not bench.init_state(theta0=theta0):
            print("[FAIL] INIT")
            return
        print(f"[OK] INIT (theta0={theta0})")

        # -- Begin
        if not bench.begin(timeout=5.0):
            print("[FAIL] BEGIN — check Teensy serial output")
            return
        print(f"[OK] BEGIN — running for {duration_s}s ...")

        time.sleep(duration_s)

        # -- End
        if not bench.end():
            print("[WARN] END did not ACK")
        else:
            print("[OK] END")

        # Small delay to receive final packets
        time.sleep(0.1)

        samples = bench.get_samples()
        print(f"\nCollected {len(samples)} samples.")
        print_summary(samples, bench.clock_offset_us)

        # -- Save
        if output is None:
            output = f"haptic_bench_{time.strftime('%Y%m%d_%H%M%S')}.h5"
        save_attrs = {
            "m_c": m_c, "m_p": m_p, "m_s": m_s, "l": l,
            "g": 9.81, "max_force_mN": max_force_mN,
            "theta0": theta0, "duration_s": duration_s,
        }
        if bench.clock_offset_us is not None:
            save_attrs["clock_offset_us"] = bench.clock_offset_us
        bench.save_h5(output, samples, attrs=save_attrs)

    finally:
        errs = bench.errors
        if errs:
            print("\n=== Errors ===")
            for ts, code, extra in errs:
                name = ERR_NAMES.get(code, f"0x{code:02X}")
                print(f"  t={ts:.3f}  {name}"
                      + (f" extra={extra.hex()}" if extra else ""))
        bench.close()


def main():
    p = argparse.ArgumentParser(description="Haptic cart-pole bench client")
    p.add_argument("--ping-only", action="store_true")
    p.add_argument("--autozero", action="store_true")
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--m-c", type=float, default=1.0)
    p.add_argument("--m-p", type=float, default=0.1)
    p.add_argument("--m-s", type=float, default=0.5)
    p.add_argument("--l", type=float, default=0.5)
    p.add_argument("--theta0", type=float, default=0.1,
                   help="Initial pendulum angle in radians (default 0.1)")
    p.add_argument("--max-force", type=float, default=5000.0,
                   help="Force clip in mN (default 5000)")
    p.add_argument("--output", type=str, default=None,
                   help="Output HDF5 path (default: auto-timestamped)")
    args = p.parse_args()

    if args.ping_only:
        bench = HapticBench()
        ok = bench.ping()
        print(f"PING: {'OK' if ok else 'FAIL'}")
        bench.close()
        return

    if args.autozero:
        bench = HapticBench()
        try:
            if not bench.ping():
                print("[FAIL] PING")
                return
            print("[OK] PING")
            if bench.autozero():
                print("[OK] AUTOZERO")
            else:
                print("[FAIL] AUTOZERO")
        finally:
            bench.close()
        return

    run(duration_s=args.duration,
        m_c=args.m_c, m_p=args.m_p, m_s=args.m_s, l=args.l,
        theta0=args.theta0,
        max_force_mN=args.max_force,
        output=args.output)


if __name__ == "__main__":
    main()
