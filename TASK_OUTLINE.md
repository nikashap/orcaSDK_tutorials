# Step 7 — Task Development

## Purpose

Grow the single base task in `cart_pendulum_teensy/` into a modular suite of task variants. Restructure the directory for modularity **while preserving the behavior of the existing Task 0 code**, then add Task 1 (center-out reaching) and Task 2 (pursuit). Tasks 3 and 4 are documented here for forward design but are **not built this round**.

Two codebases change: the MWorks side (`cart_pendulum_teensy/`) and the Teensy sketch (canonical: `cart_pendulum_task/Arduino/teensy_cartpole_haptic/teensy_cartpole_haptic.ino`; the `orcaSDK_tutorials/test_teensy_orca/teensy_cartpole_haptic/` copy is the kept historical original). The Teensy gains two new protocol modes — `COR` and `MOVE_TO` — needed by Tasks 1 and 2.

## Build Scope This Round

| Item | Status |
|---|---|
| Directory restructure for modularity | **Done** (06/12/26) |
| Task 0 (base haptic) — preserve behavior through the restructure | **Done** (06/12/26, committed) |
| Task 1 (center-out reaching) | **Done** (06/12/26) — built; pending rig verification |
| Task 2 (pursuit) | **Build** (next round) |
| Teensy `COR` protocol | **Done** (06/12/26) — pending bench verification |
| Teensy `MOVE_TO` protocol | **Done** (06/12/26) — pending bench verification |
| Task 3 (decoupled identical-mode dynamics) | **Document only** |
| Task 4 (endpoint trajectory tracking) | **Document only** |

If you find yourself implementing Task 3 or 4 Teensy modes, stop — they are out of scope.

### Completion log

- **Directory restructure + Task 0 (06/12/26, committed).** `cart_pendulum_teensy/`
  reorganized to the modular `mworks/` pattern with shared infra factored out and
  Task 0 behavior preserved. New layout (deviation from `mworks/`, noted below): the
  per-task entry `.mwel` stays at the package **root** (not a `protocols/` subdir) so
  the MWorks Python working directory is unchanged and the existing cwd-relative
  imports keep working — `mworks/` can use `protocols/` only because its Python files
  pin an absolute `_PWD` onto `sys.path`.
  - `CartPendulumHaptic.mwel` (entry, was `cart_pendulum.mwel`), `cart_pendulum.py`,
    `common/teensy_common.mwel`, `stimuli/cartpole_stimuli.mwel`,
    `task_configs/HapticConfig.mwel`, `interfaces/{teensy_interface,data_logger}.py`.
  - The reserved config (`shaft_min_um`, `shaft_max_um`, `friction_boost_gain`,
    `stiction_velocity_threshold`) is declared in `common/teensy_common.mwel`.

- **Task 1 + Teensy COR/MOVE_TO (06/12/26, pending hardware verification).** Defaults
  settled with user: **hold time = 100 ms**, **reach time limit = 5 s**.
  - Teensy `teensy_cartpole_haptic.ino`: `CMD_ENTER_COR=0x08`, `CMD_MOVE_TO=0x09`,
    `CMD_EXIT_MODE=0x0A`; `ERR_MOVE_FAILED=0x08`; `SampleCOR` (28 B) under
    `TELE_BATCH_COR=0xB2`; `cor_loop()` (force base 0, no RK4) and `do_move_to()`
    (quintic min-jerk via `SUB_POSITION_CTRL=0x1E`). Haptic `0x01`–`0x07` path
    untouched. (`SampleCOR` later extended to 36 B and `0x0B` allocated by Step 3
    Part C below — see that entry.)
  - `interfaces/teensy_interface.py`: `SampleCOR`, `0xB2` handler (drives
    `rendered_cart_x` only — pendulum angle owned by MWorks), `configure()`,
    `move_to()`, `enter_cor()`, `exit_cor()`; `ERR_MOVE_FAILED` treated as a
    trial-level event, not a fatal shutdown.
  - MWorks Task 1: `CenterOutReach.mwel` (entry), `task1.py`,
    `stimuli/target_stimuli.mwel`, `task_configs/CenterOutConfig.mwel`.
  - **Decisions / interpretation deviations:** (1) Friction params are
    MWorks/Python-plumbed and the Teensy has a 0-contribution hook, but are **not
    transmitted** this round (`CMD_CONFIG` payload left unchanged to avoid breaking
    Task 0; `CMD_ENTER_COR` left payload-less per the opcode table). (2) `MOVE_TO`
    leaves the motor **holding** the last position (no sleep) so a prompt
    `ENTER_COR` hands off within the motor's 500 ms comms watchdog. (3) `MOVE_TO`
    replies carry the achieved final position (int32 µm) on both ACK and ERROR.
  - **Unverified assumption (flag):** `SUB_POSITION_CTRL=0x1E` driving position mode
    is inferred from the SDK register map (`POS_CMD=30`), not yet bench-confirmed —
    the most likely failure point.

- **Step 3 Part C — `boost_human` in Task 1 (handle force-boost).** Spec:
  `STICTION_BOOST_README.md` (C.0–C.2, C.4). `boost_stiction` + the Task 0 combination
  and the standalone Parts A/B sketches are a later round.
  - Teensy `teensy_cartpole_haptic.ino`: `CMD_CALIBRATE_HANDLE=0x0B` (payload 5 × `float`
    boost shape params), `ERR_HANDLE_CAL=0x09`; reserved range narrowed to `0x0C`–`0x0F`.
    `read_handle_voltage()` (A0, 12-bit, `v=raw*3.3/4095`), `boost_human_mN()` (deadband
    → exp ramp → rail plateau, not velocity-gated, 0 until calibrated), `calibrate_handle()`
    (2 s baseline + validity gate). Boost added in the shared `cor_loop()` before the
    safety clip; gated on `handle_calibrated` so an uncalibrated session is base-0 as before.
    The 5 shape params (`par_boost_*`) are session-overridable via the calibration payload.
    `SampleCOR` extended 28→36 B (`+handle_voltage, +boost_human_mN`, `<II6fI`).
  - `interfaces/teensy_interface.py`: `CMD_CALIBRATE_HANDLE`/`ERR_HANDLE_CAL`, 36-B
    `SampleCOR` parse + CSV log, `calibrate_handle(boost_*)` (non-fatal rendezvous like
    `move_to`). `interfaces/data_logger.py`: `+handle_voltage, +boost_human_mN` columns.
  - MWorks: `common/teensy_common.mwel` `Force Handle` group (`handle_baseline_min/max`,
    `handle_cal_ok`, `boost_min_mN/boost_max_mN/boost_dv/boost_k/boost_m`); `task1.py`
    `setup()` calibrates after `configure()`, logs the band + outcome, non-fatal on reject.
  - **Pending bench verification:** handle hardware not yet wired; uncalibrated path
    (boost 0) is the safe default until confirmed.

## Step 0 (do this first): Conform to the existing modular layout

Before writing any task code, **read and follow the structure of `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/mworks/`**, which already organizes independent tasks as separate `.mwel` files (e.g. `CalibrateEncoder.mwel`, `PendulumTask.mwel`, `SmoothPursuit.mwel`).

1. Read `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/README.md` to understand how that directory is organized (shared resources, per-task `.mwel` files, common Python helpers, variable/group conventions).
2. **Ignore all "moog"-related code.** That API is not used here.
3. Mirror that layout inside `cart_pendulum_teensy/`: shared infrastructure (the UDP/Teensy interface, data logger, common variable declarations, rendering stimuli) factored out of the per-task files; one `.mwel` per task that includes the shared pieces and adds only its own trial logic.
4. Preserve the current Task 0 behavior exactly. After the restructure, the base task must still run end-to-end as it does today (autozero → config → init → begin → trial → end → clean shutdown). The restructure is a refactor, not a rewrite — move and factor, don't change semantics.

Do not invent a structure from first principles; match the conventions already in `mworks/` so the two directories feel consistent. If `mworks/` and this spec ever conflict on a naming or layout detail, follow `mworks/` and note the deviation.

## Shared Infrastructure (factored out, used by all tasks)

These already exist for Task 0 and should become shared modules after the restructure:

- The Teensy UDP interface (`teensy_interface.py`) — connection, opcode send/ack, telemetry RX thread, MWorks variable bridge.
- `data_logger.py` — per-sample CSV writer.
- Common MWorks variable declarations and visual stimuli (rail, cart, pendulum, bob).
- Shared config block (see below).

### Shared config block (all tasks)

Declare these in a common include so every task inherits them. Two are **declared-but-unused this round** — they exist so the wiring is ready when the force-sensing handle is built:

```
// Geometry / scene
rail_length, rail_height, cart_size, bob_size, pendulum_length_m, ...

// Shaft range (motor stroke limits, micrometers)
shaft_min_um, shaft_max_um

// Teensy connection
teensy_ip, teensy_port, local_udp_port

// Physics params (Task 0 haptic; ignored by fully-actuated tasks)
mass_cart, mass_pendulum, mass_shaft, gravity, max_force_mN, loop_period_us

// Friction boost — DECLARED BUT UNUSED THIS ROUND (handle not built yet)
friction_boost_gain            // gain on FSR-driven stiction boost
stiction_velocity_threshold    // |v| below which static-friction regime is assumed
```

Thread `friction_boost_gain` and `stiction_velocity_threshold` through the config path (MWorks → `CMD_CONFIG` payload or a dedicated config message) even though the Teensy ignores them for now. Mark them clearly as reserved.

## Teensy Protocol Additions

The existing opcode table (`HAPTIC_LOOP_README.md`) uses `0x01`–`0x07`. Add new opcodes starting at `0x08`. Keep the existing haptic protocol untouched — Task 0 continues to use it.

### New command opcodes (Mac → Teensy)

| Opcode | Name | Payload | Meaning |
|---|---|---|---|
| `0x08` | `CMD_ENTER_COR` | none | Enter center-out-reach mode (force mode, base force 0 + `boost_human` once calibrated, no physics). Used by Tasks 1 and 2. |
| `0x09` | `CMD_MOVE_TO` | 4 × `float` = `setpoint_um, duration_s, loop_period_us, error_threshold_um` | Position-mode move to a setpoint along a generated trajectory. |
| `0x0A` | `CMD_EXIT_MODE` | none | Leave COR or MOVE_TO, return motor to idle/sleep, ready for the next mode. |
| `0x0B` | `CMD_CALIBRATE_HANDLE` | 5 × `float` = `boost_min_mN, boost_max_mN, boost_dv, boost_k, boost_m` | Capture the force-handle resting baseline (2 s) and adopt the `boost_human` shape params shipped in the payload. Reply `ACK 0x0B` (or `ERROR 0x09`) carries the measured `[min, max]` voltage band as 2 × `float`. Cross-task (Step 3 Part C); see `STICTION_BOOST_README.md`. |

Reserve `0x0C`–`0x0F` for future task modes (Tasks 3, 4). Receiving a reserved opcode → `ERR_BADOP`.

### `COR` mode behavior

When `CMD_ENTER_COR` is received:

1. Set motor to **force mode**.
2. Internally command a **base force of 0**.
3. Add `boost_human` (handle-driven assist force) to the base command before the safety clip — implemented in Step 3 Part C (`CMD_CALIBRATE_HANDLE`); contributes 0 until a session calibrates the handle. The velocity-gated `boost_stiction` term (gated by `stiction_velocity_threshold`, scaled by `friction_boost_gain`) is still deferred to the Task 0 phase — see `STICTION_BOOST_README.md` C.3/C.5.
4. Keep running the **same motor exchange as the haptic loop** — command-stream run, acceleration read, velocity read, with timestamps — but **skip the RK4 physics step** entirely. There are no pendulum dynamics in this mode.
5. Stream a **reduced telemetry record** (below).

### Reduced `Sample` struct for COR (`SampleCOR`)

The COR telemetry is the haptic `Sample` with the pendulum-only fields removed (`theta`, `thetadot`, `fx`, `fpc`), plus the force-handle boost fields appended (Step 3 Part C) so `boost_human` can be verified on the bench. Use a distinct telemetry discriminator byte so the Mac can tell the two record types apart.

```
// Telemetry discriminator bytes:
//   0xB0 = full haptic Sample      (Task 0)
//   0xB2 = reduced COR sample      (Tasks 1, 2)
struct __attribute__((packed)) SampleCOR {     // 36 bytes (was 28 before Step 3)
  uint32_t cycle;
  uint32_t t_meas_us;     // micros() at moment xddot was measured
  float    x, xdot, xddot;// measured shaft state (SI)
  float    f_command_mN;  // commanded force after boost + safety clip
  float    handle_voltage;// raw handle ADC voltage this cycle (V)        <-- Step 3
  float    boost_human_mN;// boost_human before the safety clip (signed mN) <-- Step 3
  uint32_t loop_us;
};
```

Batch ~8 per packet exactly as the haptic stream does: leading `0xB2`, then `uint8` count, then that many `SampleCOR` records. Python format `<II6fI` (36 B); both fields are 0 until a successful `CMD_CALIBRATE_HANDLE` enables the boost.

### `MOVE_TO` mode behavior

When `CMD_MOVE_TO` is received with `(setpoint_um, duration_s, loop_period_us, error_threshold_um)`:

1. Read current shaft position as the trajectory start.
2. Generate a **quintic minimum-jerk position trajectory** from start to `setpoint_um` over `duration_s`, sampled at `loop_period_us`. (Minimum-jerk = zero velocity and zero acceleration at both endpoints; smooth and safe for moving the shaft to the stroke center between trials.)
3. Drive the motor in **Position Mode** to follow the trajectory.
4. On completion, check final position against `setpoint_um`. Success = within `error_threshold_um` (default base threshold **100 µm**).
5. Send a completion reply: `ACK 0x09` on success, or `ERROR` with a new `ERR_MOVE_FAILED` code (assign the next free error code) if the final position is outside threshold or the motor faulted. Include the achieved final position in the reply payload for logging.

`MOVE_TO` is used at trial start (Tasks 1, 2) to home the shaft to the middle of the stroke before the subject takes over.

### New error code

| Code | Name | Meaning |
|---|---|---|
| `0x08` | `ERR_MOVE_FAILED` | `MOVE_TO` ended outside the error threshold or faulted. Reply carries the achieved final position (`int32` µm). |
| `0x09` | `ERR_HANDLE_CAL` | `CMD_CALIBRATE_HANDLE` measured an implausible baseline (band > 0.4 V, or pinned below 0.5 V / above 3.0 V). Reply carries the measured `[min, max]` band (2 × `float`); boost stays disabled. |

## Task Specs

### Task 0 — Base haptic (preserve)

The existing implementation. After restructure, must run identically. Uses the unchanged haptic protocol (`0x01`–`0x07`) and full `Sample` telemetry (`0xB0`). No new work beyond factoring shared pieces out and leaving Task 0's trial logic in its own `.mwel`.

### Task 1 — Center-out reaching (fully actuated)

**Dynamics:** none. The pendulum is **rigidly fixed** at an angle relative to the cart. Moving the cart moves the bob; no pendulum swing, no force feedback. Teensy runs `COR` mode.

**Trial structure (per trial):**

1. **Randomize pendulum angle** (fixed for the trial; e.g. 0°, 25°, …). The pendulum renders at this constant angle relative to the cart.
2. **Home the shaft:** issue `CMD_MOVE_TO` to the middle of the stroke (`(shaft_min_um + shaft_max_um)/2`). Wait for success before proceeding.
3. **Randomize target:** place `target_bob` at a feasible location — **same y-coordinate as the pendulum bob** (since the bob's height is fixed by the constant pendulum angle), with an x-coordinate randomly chosen so the whole target stays within the rendered scene bounds. The bob's x is set by cart position, so the target must be reachable by translating the cart.
4. **Enter COR / hand to subject:** issue `CMD_ENTER_COR`; the subject now drives the shaft. MWorks renders cart and pendulum from the streamed motor position.
5. **Goal:** subject moves the cart so the pendulum bob overlaps `target_bob` within a time constraint.

**Rendering:**
- `target_bob` is a **yellow circle**, same size as the pendulum bob.
- Render cart + pendulum from the most-recent streamed motor position (`rendered_*`), exactly as Task 0 renders position.

**Reward (MWorks side, using `rendered_*`):**
- Success = center of the pendulum bob within **threshold = target_bob radius** of the target center, **held for ≥ `hold_time_ms`** (default 20 ms).
- Binary reward: 1 on success within the time constraint, 0 otherwise.
- On success: `target_bob` turns **green**. On failure (time constraint elapses without success): `target_bob` turns **red**.
- Implement the hold-time check on the MWorks side from the rendered bob position vs. target. (Reward logic lives in MWorks per project convention; the Teensy only streams position.)

### Task 2 — Pursuit (fully actuated)

**Dynamics:** none, same as Task 1 (pendulum fixed at an angle, COR mode, no force feedback).

**Trial structure:**

1. Randomize pendulum angle and home the shaft (as Task 1, steps 1–2).
2. **Moving target:** `target_bob` follows a **sinusoidal trajectory** along x. Randomize **frequency, amplitude, and number of periods** at trial start. Constrain amplitude so the target stays within the rendered scene bounds for the whole trajectory.
3. Enter COR; subject pursues the moving target with the cart.

**Reward (MWorks side, frame-weighted using `rendered_*`):**
- On each rendered frame, score 1 if the pendulum bob center is within threshold of the current `target_bob` position, else 0.
- Trial reward = **percentage of rendered frames within threshold** over the trial (frame-weighted fraction × 100). Document that this is frame-weighted (MWorks display rate), not Teensy-sample-weighted, since reward is computed from rendered position.

**Rendering:** same cart+pendulum rendering as Task 1; `target_bob` animates along its sinusoid (color convention can mirror Task 1 if you want live in/out-of-threshold feedback, otherwise neutral).

### Task 3 — Decoupled identical-mode dynamics (DOCUMENT ONLY — do not build)

Forward design notes for a later round. Not implemented now; do not add a Teensy mode for it.

System: `M(q) q̈ = B u`, with `M(q) = diag(m_c, m_p)`, `q̈ = [ẍ, θ̈]ᵀ`, `B = [1, 1]ᵀ`, scalar `u`. The cart and pendulum are driven by the **same** scalar control; they cannot be independently actuated, though they render with different spatial relationships. Some states are unreachable depending on initial conditions.

When the initial velocities are equal (`ẋ(0) = θ̇(0)`), the motion satisfies the algebraic relation
`θ(t) − θ(0) = (m_c / m_p) · (x(t) − x(0))`.
Because the pendulum exerts no force on the cart in this system, **there is no force feedback** — the planned rendering is **kinematic**: drive the pendulum angle from the measured cart displacement via the relation above, rather than integrating the ODE. Confirm this rendering choice when this task is scheduled.

### Task 4 — Endpoint trajectory tracking (DOCUMENT ONLY — do not build)

Forward design notes. The subject drives the **cart-pole** (real dynamics, like Task 0) to make the pendulum bob follow a prescribed sinusoidal endpoint motion (frequency + amplitude), kept within the rendered scene bounds. Reward likely tracking-error-based, analogous to Task 2 but on the underactuated system. Spec to be detailed when scheduled.

## Future / Deferred (mentioned, not this round)

- **Eye fixation.** Will be added to tasks later, modeled on `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/mworks/eye_interfaces`. Leave room in the task structure for it; do not implement now.
- **FSR friction boost.** The handle strain gauges are still being built. Keep the COR-mode hook and the two reserved config params in place, contributing 0.

## Acceptance Criteria

1. `cart_pendulum_teensy/` is reorganized to match the `mworks/` modular pattern; shared infra is factored out; each task is its own `.mwel`.
2. Task 0 runs end-to-end exactly as before the restructure (no behavioral change).
3. Task 1: pendulum renders at a randomized fixed angle; shaft homes to center via `MOVE_TO`; yellow target placed at a reachable same-y location in-bounds; bob-on-target held ≥20 ms yields a green target and reward 1; timeout yields a red target and reward 0.
4. Task 2: target follows a randomized in-bounds sinusoid; trial reward is the frame-weighted percentage of time the bob is within threshold.
5. Teensy: `CMD_ENTER_COR` runs the motor exchange with no RK4 and streams `SampleCOR` (`0xB2`); `CMD_MOVE_TO` follows a quintic profile and reports success/failure against a 100 µm default threshold.
6. `friction_boost_gain` and `stiction_velocity_threshold` are present in config, plumbed through, and unused (Teensy contribution 0).
7. No moog code; Tasks 3 and 4 are documented but not built.

## Open Items to Confirm With User

- `hold_time_ms` default (assumed 20 ms) and Task 1 time-constraint default.
- Whether Task 2 should show live in/out-of-threshold color feedback or a neutral target.
- The exact next-free values for `ERR_MOVE_FAILED` and any reserved opcodes, to avoid colliding with codes added since `HAPTIC_LOOP_README.md`.