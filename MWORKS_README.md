# Step 6 — MWorks Integration

> **Status: integrated and running — bring-up completed 2026-06-02.**
> The experiment runs end-to-end (autozero → config → init → begin → 60 s trial →
> end clock sync → clean shutdown and replay) without visual lag or premature
> termination. The bring-up surfaced several issues whose fixes **supersede parts
> of the spec below** (notably the per-sample `sample_*` setvar scheme in §"Telemetry
> batch processing", which was removed). See **Debugging & Changes (bring-up)** at
> the end of this file for the authoritative list of what was changed and why.

## Purpose

Replace the Mac-side MuJoCo control loop with a thin UDP client that talks to the Teensy haptic sketch from Step 4 (`teensy_cartpole_haptic.ino`). MWorks continues to handle the experiment trial logic, visual rendering, and logging; it no longer participates in physics or motor control. The Teensy is the sole owner of the haptic loop.

This step ports the existing MWorks experiment from `cart_pendulum_task/cart_pendulum_chris/` (which uses FTDI serial + MuJoCo on the Mac) into a new sibling directory at `cart_pendulum_task/cart_pendulum_teensy/` that uses UDP/Ethernet + Teensy. Behaviorally, pilot data should reproduce the legacy setup.

## Reference Material

- **Old (working) MWorks experiment, FTDI + MuJoCo:** `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_chris/`
  - `cart_pendulum.mwel`, `cart_pendulum.py`, `orca_interface.py`, `data_logger.py` — copy the structure and trial flow; replace only the parts described below.
- **Teensy sketch and protocol spec:** `/Users/Nikasha/Documents/GitHub/orcaSDK_tutorials/test_teensy_orca/teensy_cartpole_haptic` and `HAPTIC_LOOP_README.md`.
  - The opcode table, `Sample` struct layout, and error codes defined there are authoritative — do not redefine them here.
  - Read this file as a reference `/Users/Nikasha/Documents/GitHub/orcaSDK_tutorials/test_teensy_orca/haptic_bench.py`
  - The `haptic_bench.py` file is a working example script for interfacing python with the teensy
- **MWorks 0.13 docs:** https://mworks.github.io/documentation/0.13/index.html

## Architecture

Old (reference):
```
MWorks ──Python ctrl_loop (thread)── pyorcasdk ── FTDI ── Orca motor
              │
              └── MuJoCo step()  (physics on the Mac)
```

New:
```
MWorks ──Python UDP client (RX thread)── UDP/Ethernet ── Teensy ── Modbus ── Orca motor
                                                          │
                                                          └── physics + control loop on-MCU
```

The Python side no longer runs MuJoCo, no longer commands forces, and no longer paces a control loop. Its only jobs are:

1. Configure the Teensy at trial start (`CMD_CONFIG`, `CMD_INIT_STATE`, `CMD_AUTOZERO`).
2. Send `CMD_ENTER_HAPTIC` / `CMD_EXIT_MODE` at the appropriate trial boundaries.
3. Receive telemetry UDP batches in a background thread, parse them, set MWorks variables for rendering, and log every sample with enough metadata that post-hoc analysis can reconstruct the per-sample timeline.

## Files to Create

All under `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/`:

| File | Role | Closest reference |
|---|---|---|
| `cart_pendulum.mwel` | MWorks experiment script. Visuals, trial flow, variable declarations. | `cart_pendulum_chris/cart_pendulum.mwel` — copy and adapt. |
| `cart_pendulum.py` | Python entry point loaded by the `.mwel` via `python_file`. | `cart_pendulum_chris/cart_pendulum.py` — strip MuJoCo, swap Orca for Teensy. |
| `teensy_interface.py` | UDP client + telemetry receiver + MWorks variable bridge. | `cart_pendulum_chris/orca_interface.py` — keep the class shape and lifecycle, replace the control loop with UDP RX. |
| `data_logger.py` | Background-thread CSV writer for high-rate per-sample logging. | `cart_pendulum_chris/data_logger.py` — verbatim copy, then replace the `FIELDS` list (see below). |

Do not modify files in `cart_pendulum_chris/`. The new directory must be self-contained.

## `teensy_interface.py` — Detailed Spec

### Imports and constants

Pull from `mworkscore` (`setvar`, `error`, `message`, `warning`) as in the reference. Standard library only (`socket`, `struct`, `threading`, `time`, `collections`). No `pyorcasdk`, no `mujoco`, no `torch`. No Kalman filter boosting for now (may be changed later).

Define module-level constants for the protocol (mirror `HAPTIC_LOOP_README.md`):

```python
# Opcodes (Mac → Teensy)
CMD_PING         = 0x01
CMD_AUTOZERO     = 0x02
CMD_CONFIG       = 0x03
CMD_INIT_STATE   = 0x04
CMD_ENTER_HAPTIC = 0x05
# 0x06 reserved (was CMD_END — folded into CMD_EXIT_MODE)
CMD_SLEEP        = 0x07

# Reply leading bytes (Teensy → Mac)
REPLY_ACK       = 0xA1
REPLY_ERROR     = 0xA2
REPLY_TELEMETRY = 0xB0
REPLY_SUMMARY   = 0xB1

# Error codes (follow REPLY_ERROR)
ERR_BADOP, ERR_BADLEN, ERR_MOTOR_FAULT, ERR_SIGN_CHECK, \
ERR_NAN_STATE, ERR_MODBUS_TIMEOUT, ERR_NOT_CONFIGURED = range(1, 8)
```

Define a `Sample` `struct` format string matching the on-wire layout from `HAPTIC_LOOP_README.md`. Verify with `struct.calcsize()` and assert at import time. Document the field order in a comment.

### Class `TeensyInterface`

Mirror the lifecycle of `OrcaInterface` so `cart_pendulum.py` changes minimally:

```python
class TeensyInterface:
    def __init__(self, teensy_ip, teensy_port, local_port, params):
        ...
    def reset_experiment(self):  # configure + begin
        ...
    def attach_logger(self, logger):
        ...
    def start(self):  # start RX thread
        ...
    def stop(self):
        ...
    def cleanup(self):  # END + sleep motor + stop thread
        ...
```

`params` is a dict carrying `m_c, m_p, m_s, l, g, max_force_mN, loop_period_us` for `CMD_CONFIG` and `x0, theta0, xdot0, thetadot0` for `CMD_INIT_STATE`. Default values live in `cart_pendulum.py`, not here.

### UDP socket setup

One non-blocking `SOCK_DGRAM` socket bound to `('', local_port)`. Outbound packets go to `(teensy_ip, teensy_port)`. Use a short `settimeout()` (e.g. 50 ms) so the RX loop can poll a stop flag.

Reuse any relevant functions from `haptic_bench.py`

### `reset_experiment` flow

1. Send `CMD_AUTOZERO`, wait for `ACK 0x02`. (Optional: skip if a flag says autozero is already done.).
2. Send `CMD_CONFIG` with the parameters from `params`, wait for `ACK 0x03`.
3. Send `CMD_INIT_STATE` with initial state, wait for `ACK 0x04`.
4. Send `CMD_ENTER_HAPTIC`, wait for `ACK 0x05`. Any `ERROR` reply must propagate to MWorks via `error(...)` and abort the experiment cleanly.

After `ACK 0x05`, the RX thread should already be running (started in `start()`); telemetry will begin streaming in.

### RX thread (`_run`)
Use `haptic_bench.py` as a reference and reuse any relevant code from that file

A daemon thread that calls `recvfrom()` in a loop and dispatches based on the first byte:

- `0xA1` (ACK) and `0xA2` (ERROR) → put on an internal `queue.Queue` for the command-side `_send_and_wait_ack` to consume.
- `0xB0` (TELEMETRY) → parse and process (next section).
- `0xB1` (SUMMARY) → parse, log final summary, signal end.
- Unknown leading byte → `warning(...)` and discard.

The thread exits when a `threading.Event` is set by `stop()`.

### Telemetry batch processing — the core of this step
Use `haptic_bench.py` as a reference

A telemetry packet is:

```
byte 0:     0xB0
byte 1:     uint8 count N  (1 ≤ N ≤ ~12, but do not assume any specific value)
bytes 2..:  N × Sample  (each Sample has the layout from HAPTIC_LOOP_README.md)
```

For each batch:

1. **Unpack all `N` samples.** Use `struct.iter_unpack` over `packet[2:]`. Confirm `len(packet) == 2 + N * SAMPLE_SIZE`; on mismatch, `warning(...)` and skip the batch.

2. **Determine the most recent sample** — index `N-1` (the last one in the batch). This is the sample whose `(x, theta)` drives the MWorks render.

3. **Set the "rendered" variables** with values from sample `N-1`:
   ```python
   setvar('rendered_cart_x', samples[-1].x * length_scale)
   setvar('rendered_pendulum_angle', samples[-1].theta)
   setvar('rendered_t_meas_us', samples[-1].t_meas_us)
   setvar('rendered_cycle', samples[-1].cycle)
   ```
   These four variables are the ground truth for what the subject saw on screen at that MWorks event timestamp. Update the legacy MuJoCo display vars (`mj_cart_position_x`, `mj_pendulum_angle`) from the same most-recent sample so the visual scene renders correctly.

4. **Log every sample in the batch.** For each sample at position `i` in `[0, N)`:
   - Compute `sample_offset = i - (N - 1)`. The most recent sample has offset `0`; the second most recent has `-1`; and so on. This indexing is stable regardless of batch size.
   - Set a fixed set of per-sample variables (listed below) so each `setvar` produces a timestamped MWorks event. The `sample_t_meas_us` value is the authoritative per-sample timing; the MWorks event timestamp records when Python processed the batch, which is what differs.
   - Order matters: set `sample_offset` and `sample_t_meas_us` **first** for each sample, then the other fields. Post-hoc analysis can group consecutive setvar events between successive `sample_offset` writes to recover one sample's worth of fields.

   Per-sample variable names (all `var`s in the `.mwel`):
   ```
   sample_offset           (int)
   sample_cycle            (int)
   sample_t_meas_us        (int)
   sample_x                (float, meters)
   sample_xdot             (float, m/s)
   sample_xddot            (float, m/s^2)
   sample_theta            (float, rad)
   sample_thetadot         (float, rad/s)
   sample_fx               (float, N, inferred human force)
   sample_fpc              (float, N, pendulum reaction)
   sample_f_command_mN     (float)
   sample_force_sensed_mN  (float, motor-reported force, lags ~3 frames, log only)
   sample_loop_us          (int)
   ```

5. **Optionally also forward to a `DataLogger`** (the same `data_logger.py` used by the reference, attached via `attach_logger`). The DataLogger receives one row per *sample*, not per *batch* — this is the high-rate ground-truth log, separate from the MWorks event log.

### Render-vs-log distinction (important)

The `rendered_*` variables answer the question "what did the subject see?" — there is exactly one set per UDP batch. The `sample_*` variables answer "what was the full state history within this batch?" — there are `N` sets per batch. Both are needed because MWorks's own event timestamps are coarsened to batch arrival time, but the per-sample `t_meas_us` carries microsecond accuracy from the Teensy. Confirm in post-hoc analysis that `rendered_*` for batch `k` matches the `sample_*` row in batch `k` whose `sample_offset == 0`.

### Error handling

- An `ERROR` reply at any time → call `error(...)` to surface to MWorks, attempt a `CMD_SLEEP` (best-effort), set an internal `_shutdown` flag that makes subsequent operations no-ops, and stop the RX thread.
- Socket exceptions during `recvfrom` other than timeout → same path.
- Sample with non-finite values in the batch → log a warning but do not crash; downstream analysis can filter.

### `cleanup`

1. Send `CMD_EXIT_MODE`, wait briefly for `ACK 0x0A` and the `0xB1` summary.
2. Set the stop flag, join the RX thread.
3. Close the socket.
4. If a logger is attached, call `logger.stop()`.

Mirror the structure of `OrcaInterface.cleanup()` so MWorks lifecycle hooks unchanged.

## `cart_pendulum.py` — Detailed Spec

Start from `cart_pendulum_chris/cart_pendulum.py` and make the following modifications:

- **Remove** the `gymnasium`/`mujoco`/`pyorcasdk` imports and the `Scene` class's MuJoCo setup. The visual scene no longer derives geometry from a MuJoCo XML.
- **Replace** the `Scene` class with a small struct that holds only what MWorks needs for rendering — the rail length scale and any geometry parameters previously read from the MuJoCo model. Hard-code those geometry values (or load from a small YAML/JSON) instead of from `model.geom(...)`.
- **Replace** `OrcaInterface` instantiation with `TeensyInterface`. Pass in the Teensy's IP/port, the local UDP port, and a `params` dict assembled from MWorks variables (`mass_cart`, `mass_bob`, `mass_shaft`, `pendulum_length`, etc.). Document these MWorks variables in the `.mwel` so the experimenter can edit them at the top.
- **Keep** the `DataLogger` setup and the `reset_all()` function shape. `reset_all()` now calls `teensy.reset_experiment()` instead of `orca.reset_experiment()`.

`Scene.update()` remains a no-op (the RX thread does the work).

Top-of-file path constants and the `setvar`/`getvar` calls retain the same names as the reference where possible, to minimize churn in the `.mwel`.

## `cart_pendulum.mwel` — Detailed Spec

Start from `cart_pendulum_chris/cart_pendulum.mwel`. Changes:

- **Resources:** remove `fake_actuator.py`, `orca_interface.py`. Add `teensy_interface.py`, `data_logger.py`.
- **Python file:** `python_file ('cart_pendulum.py')` — same name, new directory.
- **Variables to add:**
  - `teensy_ip` (string), `teensy_port` (int), `local_udp_port` (int).
  - `mass_cart`, `mass_pendulum`, `mass_shaft`, `pendulum_length_m`, `gravity`, `max_force_mN`, `loop_period_us` — drive `CMD_CONFIG`.
  - The full set of `rendered_*` and `sample_*` variables listed above. All defaulted to 0.
- **Variables to remove:** anything specific to MuJoCo internals that no longer has meaning here (`qfrc_bias`, `qfrc_passive`, `qfrc_constraint`, `qfrc_applied`, `qfrc_actuator`, `mj_*` if you choose to retire them — but it's fine to keep `mj_cart_position_x` and `mj_pendulum_angle` as duplicates of `rendered_*` to avoid touching the visual stimulus declarations).
- **Trial flow:** 
1) Whatever logic in the reference handled "reset → go cue → trial → log" should work as-is since the Python-level API is preserved.
2) Add two states: one at the beginning of the experiment, and one at the end of the experiment. The first state is to do the first clock sync, and the second state is to do the last clock sync. Similar to haptic_bench.py. Do not render the scene when the clock sync is happening.
3)  Also make an `AUTOZERO` state in `cart_pendulum.mwel` that occurs before the experiment begins global clock sync begins. Do not render the pendulum variables while the autozero procedure commences.
- **Variable group `#group`** for the `sample_*` and `rendered_*` variables so they're easy to enable/disable in the MWorks event log.

## `data_logger.py` — Detailed Spec

Copy `cart_pendulum_chris/data_logger.py` verbatim into the new directory. The threading model, queue-based non-blocking `log()`, sentinel-based shutdown, and `_writer_loop` are all reusable as-is. **Only the `FIELDS` list at module top changes**, because the new schema is per-sample (not per-MuJoCo-step) and the MuJoCo-specific columns (`qfrc_*`, `cart_xddot` from MuJoCo, `motor_*` from `pyorcasdk`) no longer have meaning.

Also be sure it is possible to calculate post-hoc from the data logger output (or measure from the experiment) how long it takes for MWorks to process all the setvar calls after it receives a data packet from the teensy.

Replace `FIELDS` with the following list, in this order:

```python
FIELDS = [
    # Mac-side bookkeeping
    "received_at_s",        # wallclock seconds since session start, set in TeensyInterface
    "batch_index",          # monotonically increasing UDP batch counter
    "sample_offset",        # 0 = most recent in batch, -1 = next, etc.
    # Teensy-side authoritative timing
    "cycle",                # loop iteration counter from Teensy
    "t_meas_us",            # micros() on Teensy at moment xddot was measured
    "loop_us",              # measured duration of that Teensy loop iteration
    # Measured shaft state (SI)
    "x",
    "xdot",
    "xddot",
    # Integrated pendulum state
    "theta",
    "thetadot",
    # Forces
    "fx",                   # inferred human force (N), log only
    "fpc",                  # pendulum reaction force (N)
    "f_command_mN",         # commanded force after scaling + clip
    "force_sensed_mN",      # motor-reported force (lags ~3 frames), log only
]
```

`TeensyInterface` constructs one dict per *sample* (not per batch) with these keys and passes it to `logger.log(snapshot)`. `received_at_s` is `time.perf_counter() - session_t0`, set once per batch but written into every sample's row from that batch. `batch_index` is incremented by the RX thread when a `0xB0` packet arrives.

No other changes to `data_logger.py` are needed. Do not import from the old file at runtime — make a clean copy so the new directory is self-contained per the directory rule above.

## Logging Conventions

| Where | What | Granularity |
|---|---|---|
| MWorks event log (`.mwk2`) | All `setvar` calls — both `rendered_*` (once per batch) and `sample_*` (N times per batch). | One MWorks event per `setvar`. |
| `DataLogger` CSV | One row per *sample* with the `FIELDS` schema above. | High-rate, lossless. |
| Teensy summary (`0xB1`) | Final `Sample` + total cycle count at end of session. | Once per session. |

The `t_meas_us` field is the authoritative timing for any per-sample analysis. The MWorks event timestamp tells you when Python processed the batch (useful for measuring Mac-side latency); it is not the time the measurement was taken. The CSV's `received_at_s` plus `t_meas_us` together let you separate Teensy-side timing from transport latency.

## Acceptance Criteria

1. A trial starts cleanly: `AUTOZERO → CONFIG → INIT → BEGIN` all `ACK`'d, telemetry begins streaming within a second.
2. The visual scene animates at MWorks's display rate, using `rendered_cart_x` and `rendered_pendulum_angle` from the most recent sample of each batch. The variables should be rendered with live_queue as they currently are in the original cart_pendulum.mwel script.
3. The MWorks event log contains, per UDP batch, exactly one set of `rendered_*` updates plus `N` sets of `sample_*` updates. Filtering on `sample_offset == 0` yields a time series that matches `rendered_*` 1:1.
4. `sample_t_meas_us` is strictly increasing across a session, with consecutive deltas equal to the Teensy's `loop_period_us` ± measured loop jitter.
5. Sending `CMD_EXIT_MODE` (e.g. via experiment stop) reaches the Teensy, motor enters sleep, and the summary packet is logged.
6. A behavioral pilot reproduces the qualitative feel and the trial-level performance metrics of the legacy `cart_pendulum_chris/` setup.
7. No MuJoCo or `pyorcasdk` imports anywhere in the new directory.

## Out of Scope

- Friction-table loading on the Teensy (deferred to "Step 4 modified with Step 3's friction table" per `CLAUDE.md`).
- Force-sensor input.
- Any control-loop logic on the Mac. If you find yourself writing a control loop in `teensy_interface.py`, stop — that's a sign of architecture drift.

## Debugging & Changes (bring-up — completed 2026-06-02)

The first end-to-end MWorks runs showed two symptoms: the visual pendulum
progressively lagged behind the real haptic feel, and the Teensy aborted mid-trial
with `ERR_MODBUS_TIMEOUT`, which terminated the experiment early. Investigation of
the CSV/`.mwk2` logs showed the UDP/RX path had no lag — the problem was downstream
in MWorks event processing. The fixes below resolved both symptoms; the run now
completes the full trial and replays cleanly.

### 1. Visual lag — removed the per-sample `setvar` storm (supersedes §"Telemetry batch processing")

The original spec pushed all 13 `sample_*` fields through `setvar` for **every**
sample in **every** batch (~9,600 `setvar`/s). This floods the MWorks event stream;
the display falls further behind real time the longer a trial runs.

**Change:** `teensy_interface._process_telemetry()` now sets only the four
`rendered_*` variables once per batch (from the most-recent sample) for the visual
scene. The full per-sample history is written to the CSV `DataLogger` only. The
`sample_*` `var group` was removed from `cart_pendulum.mwel`. **The §"Telemetry
batch processing" step 4 (per-sample setvars) and the `sample_*` variable list are
obsolete — do not re-add a per-sample `setvar` loop.** Per-sample timing analysis
is done from the CSV (`received_at_s`, `batch_index`, `sample_offset`, `t_meas_us`),
which is lossless and doesn't load the MWorks event loop.

### 2. Re-run crash — re-entrant socket / `start()` (Mac-side)

The Python module loads once; module-level objects persist across protocol re-runs
(MWorks does not re-import on replay). `cleanup()` closed the UDP socket, so a second
run reused a dead file descriptor → `[Errno 9] Bad file descriptor`.

**Change:** factored socket creation into `_open_socket()` and made `start()`
re-entrant — it reopens the socket and clears the `_closed`/`_shutdown` flags (and
re-enables autozero, since the motor was slept on the prior cleanup) when a previously
closed interface is started again.

### 3. End-of-session clock sync — service `CMD_PING` inside the haptic loop (firmware)

The end-of-session `clock_sync()` runs *while the loop is still streaming*. The loop
didn't service `CMD_PING`, so no PING round succeeded and the second clock-sync anchor
was missing ("no PING round succeeded").

**Change:** the loop's step-11 UDP poll now answers `CMD_PING` inline
(`send_ping_ack()`) in addition to handling `CMD_EXIT_MODE`/`CMD_SLEEP`. Both clock-sync
anchors are now captured, enabling skew-corrected post-hoc alignment.

### 4. Modbus timeouts — hold-last-value instead of retry (firmware)

Occasional single-frame RS-422 glitches were aborting the whole run via
`ERR_MODBUS_TIMEOUT`. A bounded-retry approach was tried first but rejected: a retry
adds a whole extra transaction (~0.6 ms) of jitter to the affected cycle.

**Change:** `motor_exchange()` now does a **single attempt per transaction with
hold-last-value**. On a glitched/short readback frame it keeps the previous good value
for that field (`prev_position_um` / `prev_speed_mmps` / `prev_accel_mmpss`), bumps a
`modbus_glitch_count` diagnostic, and the physics steps forward with no added latency —
the motor still received the `0x64` force command (the TX went out fine; only the
readback frame glitched). The **only** abort condition is a motor-reported fault in
`ex.errors`; a genuine comms break surfaces there as the Orca's `COMMS_TIMEOUT`
(error bit 2048) in the next valid frame. A clean shutdown reports
`modbus glitches held-over: N`.

### 5. In-loop Modbus gap timeout kept small — distinct from the motor watchdog

`modbus_transact`'s `timeout_us` is the Teensy-side **inter-byte / wait-for-reply gap
timeout** (the timer resets on every received byte), *not* the Orca's `COMMS_TIMEOUT`
watchdog register. Because a no-reply transaction blocks for the whole timeout before
bailing, a large value here would stall the 1.4 ms loop for the full duration and
defeat the zero-latency hold-last-value design (and could itself trip the motor's
watchdog).

**Change:** the three in-loop transactions in `motor_exchange()` use a **5000 µs**
gap timeout, so a missing/truncated reply bails within ~5 ms and falls back to the
held value. The one-time setup transactions (autozero, `verify_motor_present`, etc.)
run outside the real-time loop and keep the longer `USER_COMMS_TIMEOUT_US`.

# Post Step 6 - Analyzing MWorks and csv files

## Goal
Create a jupyter notebook at `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/teensy_mworks_analysis.ipynb` that plots time distributions of key loops in the experiment (i.e., the haptic control loop, the rendering loop, the approximate time it takes to send data from teensy to MWorks, etc)

## Reference script
See `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_chris/explore_mwk2.ipynb` for an example interactive analysis and brainstorming script that used the previous FTDI + serial implementation of the control loop

## Location of most recent code runs
Mworks file: `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/mwk_logs/jazlabamini-cart_pendulum-20260602-162707.mwk2`

## CSV scripts associated with the mworks file listed above:
- `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/datalogger_logs/teensy_log_20260602_162709.csv`
- `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/datalogger_logs/teensy_log_20260602_162831.csv`
- `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/datalogger_logs/teensy_log_20260602_162943.csv`

NOTE that the pause/play button was hit three times in the experiment, which is why there are three csv files for this particular June 2nd run.
