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
import socket
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

    def enable_sleep_stream(self, period_us: int = 1000,
                            timeout: float = 2.0) -> bool:
        # Sub-code 0x00 = Sleep Data Stream
        return self._send_and_ack(
            f"ENABLE_STREAM 00 0 {period_us}",
            "ENABLE_STREAM", timeout)

    def set_force(self, force_mn: int):
        # Hot path: fire-and-forget for low latency.
        self._send(f"SET {force_mn}")

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


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true", default=True,
                   help="Run the canned demo sequence (default)")
    args = p.parse_args()
    if args.demo:
        demo()


if __name__ == "__main__":
    main()