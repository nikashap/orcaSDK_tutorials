# Step 3 — Static Friction Characterization & Force-Boost Integration

## Purpose

Three deliverables, built as **separate, modular Teensy sketches** plus force-boost integration into the task code:

- **Part A — Characterization sketch.** A fresh, standalone sketch that drives the Orca through a full-shaft breakaway sweep, collects static-friction-vs-position data, and (with a Python helper) fits the existing GP model. Output: a compiled-in header table of GP **posterior means** (and a constant fallback).
- **Part B — Validation sketch.** A second fresh, standalone sketch that loads the GP-mean header from Part A and replays pre-computed sinusoidal-control trajectories, testing whether the friction model improves open-loop trajectory tracking.
- **Part C — Force-boost integration.** Add the handle-driven `boost_human` to the task code, starting with **Task 1** (fully actuated, no pendulum dynamics — simplest to bench test). Only after Task 1 bench testing do we add `boost_human` to **Task 0**, then test `boost_stiction`, and the combined behavior to **Task 0** (the dynamics task).

Parts A and B are bench tools in their own sketches; keep them out of `teensy_cartpole_haptic.ino`. Part C edits the task code. First complete Part C, Task 1 related code for `boost_human`. Wait for confirmation from user to move on to the next code.

## Reference material

- Old stiction collection + training (FTDI + serial, no Teensy): `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/stiction`. **Read this directory and reuse the minimal functions needed** for a full-shaft sweep and GP fit; the codebase is clunky but organized, so scavenge the essential pieces rather than copying entirely.
- GP training code lives within that module (e.g. `.../stiction/training/train_stiction_model.py`). Reuse the fitting; only the data-collection front end changes (Teensy instead of FTDI).
- Stiction gating: `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/ilc/ilc_loop/ilc_runner.py` — the authoritative gating is reproduced below (B.3), taken from its `stiction_enabled` block.
- Handle read reference: `wheatstone.ino` (provided; reconciled below in B.1). Helpful for Part C.
- Trajectory generation for Part B: `/Users/Nikasha/Documents/GitHub/cartpole-target/orca/trajectories.ipynb`, section **"4) Trajectories of the cart (acceleration, velocity, and position) based on sinusoidal control inputs"**. Mirror that method.
- Example GP fit (expected output shape): `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/stiction/stiction_model/stiction_model_gaussian_process_fit.png` — two panels, Direction +1 (extend) and Direction −1 (retract), breakaway force in mN vs. position in mm. **Only the model mean (the black curve) is used at runtime**; the ±2σ band and per-point data are not.

### Protocol/opcode hygiene

Existing: `HAPTIC_LOOP_README.md` (`0x01`–`0x07`, telemetry `0xB0`), `TASK_OUTLINE.md` (`0x08`–`0x0A`, COR telemetry `0xB2`). Part C took `0x0B` = `CMD_CALIBRATE_HANDLE` and error `0x09` = `ERR_HANDLE_CAL`; `0x0C`–`0x0F` remain reserved. The characterization and validation sketches are **standalone** (their own simple command sets — they don't need to share the haptic opcode table), but if any new opcode is added to the haptic/task protocol, grep the current sketch first and use a free value (next free task opcode = `0x0C`).

---

## Part A — Static Friction Characterization (standalone sketch)

> **STATUS (06/23/26): code complete, bench-pending.** Built under
> `cart_pendulum_task/Arduino/stiction/`: `stiction.ino` (Teensy on-Teensy sweep,
> reusing the haptic sketch's Modbus/`motor_exchange`/min-jerk scaffolding),
> `stiction_logger.py` (Mac UDP driver/logger → `breakaway_data/sweep_<ts>/{sweep_data,summary}.csv`),
> `build_stiction_dataset.py` + `train_stiction_model.py` (copied from
> `orca_controllers`, paths localized, GP fit), and `export_stiction_header.py`
> (GP means + constant fallback → `stiction_model.h` with inline
> `stiction_lookup_mN`). The full fit→export pipeline is verified on synthetic
> data (660-node header, figure-matching ranges). UDP uses a self-contained text
> command set (`CONNECT`/`AUTOZERO`/`START_SWEEP …`/`ABORT`/`SLEEP`) + tagged
> binary stream — independent of the haptic opcode table. Sweep orchestration is
> on-Teensy (deterministic ramp). **TODO:** flash + bench-run a real sweep and
> regenerate `stiction_model.h` before Part B / C.3. See `Arduino/stiction/README.md`.

### Location

- Sketch: `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/Arduino/stiction/stiction.ino`
- All outputs (raw CSV, fitted model, exported header): `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/Arduino/stiction/`

This is a **fresh sketch**, not derived from the haptic sketch. It only needs the Modbus/motor-comms scaffolding (reuse the connection/transaction code style from `teensy_orca_bridge`) plus the sweep logic.

### What "static friction" means here

Static friction = **breakaway force**: the commanded force at which the stationary shaft just begins to move, as a function of position and push direction. The GP maps **(position_mm, direction) → breakaway force (mN)**, matching the example figure: Direction +1 (extend) gives positive breakaway forces (~5000–12000 mN across the stroke), Direction −1 (retract) gives negative breakaway forces (~−4500 to −11000 mN). This is the convention `train_stiction_model.py` expects.

### Collection protocol — full-shaft swept breakaway test

**Reference material:** Refer to `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/stiction/collection_scripts`. If a method differs between the description below and the reference material, confirm before implementing

**Input parameter:** `n_points` — the number of sample positions along the shaft. The sketch divides the usable stroke `[shaft_min_um, shaft_max_um]` into `n_points` evenly spaced bins. (Expose `n_points` as a serial/compile-time parameter; the example figure used ~30 positions × several repeats over a ~600 mm stroke.)

Per position bin, per direction (+1 extend, −1 retract):

1. **Home** to the bin position (Position Mode, smooth move).
2. **Switch to Force Mode**, command 0 N, confirm the shaft is at rest.
3. **Clear stale stream**, clear stale readings from the stream for 4 frames (make this a parameter). A new opcode will need to be written for this process. Refer to `advance_motor_stream` in `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/helpers.py`
4. **Quasi-static ramp:** increase commanded force slowly (low ramp rate so inertial effects are negligible — `ramp_rate_mN_s`, tunable) in the test direction until measured velocity exceeds a small breakaway threshold (`breakaway_mm_s`, e.g. a few mm/s).
5. **Record** the commanded force at breakaway = static friction at (bin, direction). Stream the full ramp (position, velocity, commanded force, timestamp), not just the breakaway point, so detection can be re-tuned offline.
6. **Relax** to 0 N, let it settle, repeat for the other direction.
7. **Step** to the next bin; repeat. Collect several repeats per (bin, direction) so the GP sees scatter (as in the figure's Train/Val points).

The sweep can be orchestrated entirely on the Teensy (driven by a `START_SWEEP n_points` serial command) streaming results back over serial/UDP to a small Python logger in the same folder, or the Python side can step it bin-by-bin — pick whichever matches the reused `orca_controllers` functions most cleanly, and keep it simple.

### Fitting and export

1. A Python script in `Arduino/stiction/` ingests the raw CSV and calls the reused `train_stiction_model.py` GP fit, producing the per-direction mean curve (the black line in the figure). Copy over the `train_stiction_model.py` file from `orca_controllers` into `Arduino/stiction/`

2. **Export only the posterior means** as a compiled-in C header (e.g. `stiction_model.h`) for use by Parts B and C. Format must support correct interpolation:
   ```c
   // stiction_model.h  — GP posterior means only (no variance)
   #define STICTION_POS_MIN_UM   <shaft_min_um>
   #define STICTION_POS_MAX_UM   <shaft_max_um>
   #define STICTION_N_NODES      <N>            // uniform grid over [MIN,MAX]
   // Breakaway force in mN at each node, per direction:
   static const float STICTION_EXTEND_MN[STICTION_N_NODES] = { ... };  // direction +1
   static const float STICTION_RETRACT_MN[STICTION_N_NODES] = { ... }; // direction -1
   ```
   The grid is uniform so runtime lookup is O(1): `f = (pos_um - MIN)/(MAX - MIN) * (N_NODES-1)`, then **linear interpolation** between the two bracketing nodes. Choose `N_NODES` fine enough (a few hundred to ~1000) that linear interpolation reproduces the GP mean curve faithfully — the curve in the figure has several gentle inflections, so ~1 mm node spacing is comfortable (~600 nodes for a 600 mm stroke, ~2.4 KB/direction in float).
3. Also export a **constant fallback** (e.g. median |breakaway| per direction) so the runtime can select constant vs. position-table via a flag.

A shared C helper (used by Parts B and C) interpolates the table:
```c
// returns signed breakaway force (mN) at position, for direction = +1 or -1
float stiction_lookup_mN(float pos_um, int direction);
```

---

## Part B — Controller Validation (standalone sketch)

> **STATUS (06/23/26): code complete, bench-pending.** Built under
> `cart_pendulum_task/Arduino/validate_controller/`: `validate_controller.ino`
> (Teensy replay sketch, reuses `stiction.ino`'s Modbus/`motor_exchange`/min-jerk
> scaffolding, `#include`s `stiction_model.h`; UDP text protocol
> `CONNECT`/`AUTOZERO`/`RUN_TRAJ …`/`ABORT`/`SLEEP` + tagged `ValSample` (34 B)
> stream), `validate_logger.py` (Mac driver: mirrors the notebook §4 rejection
> sampler, computes the desired free-mass trajectory, runs each trajectory
> comp-off then comp-on, reports position-tracking RMS off vs on →
> `validation_data/run_<ts>/{traj_<i>_{off,on}.csv, summary.csv}`), and
> `README.md`. Trajectory math + struct sizes verified offline; the sampler now
> rejects on the *actual* integrated path (the notebook's free-mass displacement
> estimate let some trajectories leave the stroke). `export_stiction_header.py`
> auto-copies `stiction_model.h` into the sketch folder. **TODO:** generate a real
> `stiction_model.h` from a full Part A sweep, then flash + bench-run; confirm the
> `sign(f_cmd)`↔table sign convention on hardware.

### Location

- Sketch: `/Users/Nikasha/Documents/GitHub/cart_pendulum_task/Arduino/validate_controller/validate_controller.ino`

A **fresh, standalone** sketch. It `#include`s the `stiction_model.h` header produced by Part A (compiled-in, so it interpolates the GP means directly — no UDP model loading needed) and replays pre-computed trajectories on the bench.

### Reference files

See `/Users/Nikasha/Documents/GitHub/orca_controllers/orca_controllers/ilc` directory for an older example that used the FTDI chip for finding a reasonable controller. Note that the method included an affine scaling factor to the force commands in addition to a static friction model in the config file. Also note that the reference trajectories are different from the Pre-computed trajectories I want.

### Pre-computed trajectories

Generate the reference trajectories the same way as `cartpole-target/orca/trajectories.ipynb` section 4 — **cart acceleration, velocity, and position from sinusoidal control inputs**. Concretely: choose sinusoidal control input(s) `u(t)` (amplitude/frequency) where the reference block (aka, the motor shaft itself; no pendulum dynamics) always starts from rest (zero velocity, zero acceleration). Integrate to get desired acceleration, velocity, and position profiles for the cart, and precompute the per-frame command stream. These can be computed offline and baked into the sketch as arrays, or generated on-Teensy from a few parameters — match the notebook's formulation either way. Be sure to use the same control timestep as the teensy.

### What the validation does

Open-loop trajectory-following test of the friction model's quality:

1. Move to the trajectory start position.
2. Force Mode. At each frame, command the feedforward force for the desired acceleration **plus** the gated stiction-compensation term from `stiction_lookup_mN(...)` (same gating as B.3), with a runtime toggle to disable compensation.
3. Read and log measured position/velocity/acceleration vs. the desired trajectory.
4. Compare tracking error **with vs. without** the friction compensation. Lower error with compensation enabled validates that the model means are useful.

Keep telemetry logging to the `Arduino/validate_controller/` folder (or stream to a small Python logger there).

---

## Part C — Force-Boost Integration (in the task code)

### Phasing (important)

1. **Now:** implement `boost_human` in **Task 1** code (fully actuated, no pendulum dynamics, COR mode). Bench test it.
2. **After bench test:** add the `boost_human` behavior to **Task 0**. Bench test it (the subject is looking for realistic feeling boost with the pendulum dynamics incorporated). Once confirmed from the subject, then implement `boost_stiction` and the combined `boost_human + boost_stiction` behavior in **Task 0** (the dynamics task). The combination modes (C.5) belong to this later phase.

Do not wire `boost_stiction` into Task 0 in this round — get `boost_human` feeling right in Task 1 and Task 0 first.

### C.0 — Handle calibration routine (run at start of every task)

Write a standalone function in `teensy_cartpole_haptic.ino` (so all tasks share it), invoked once at experiment start — either from the haptic-entry/setup path or via a `CMD_CALIBRATE_HANDLE` opcode (next free value) the Mac sends before `CMD_ENTER_HAPTIC`.

- For **2 seconds**, with the handle at rest, sample the handle ADC and record the **min and max resting voltage** → `baseline_min`, `baseline_max`. These are **measured**, not assumed (they will land near 1.50–1.70 V but must be captured fresh each session — the AD620 offset can drift).
- Report the measured baseline back to the Mac in an ACK payload so it is logged per session.
- If the band is implausibly wide (greater than 0.4V variation) or pinned to a rail (below 0.5 V or above 3.0 V), return `ERR_HANDLE_CAL` (next free error code) rather than proceeding.

### C.1 — Handle ADC read (reconciled with `wheatstone.ino`)

- Pin: `A0` (`FORCE_PIN` in `wheatstone.ino`).
- `analogReadResolution(12)` → range **0–4095**, and `voltage = raw * (3.3 / 4095.0)`. **Note:** a comment in `wheatstone.ino` says the 12-bit range is "0-8191" — that is incorrect; the actual range is 0–4095 and the file's own voltage math (line 17) uses 4095. Follow the 4095 convention; ignore the stale comment.
- `analogReadAveraging(1)` (no hardware averaging) in the reference, consistent with letting the boost mapping do the smoothing. Read **once per control cycle** in the existing loop.
- First pass uses no separate filter (the deadband + smooth ramp below provide smoothing). The exponential ramp **starts at 0 N at the band edge**, so threshold onset is naturally gentle and resists chatter. If chatter still appears, add edge hysteresis or bump `analogReadAveraging` — leave a hook, don't build it yet.

### C.2 — `boost_human` mapping (voltage → signed force)

Three zones per side, anchored on the **measured** baseline. Let `b_lo = baseline_min`, `b_hi = baseline_max`. Constants: `BOOST_MIN_MN = 12000` (12 N), `BOOST_MAX_MN = 15000` (fixed parameter), onset margin `DV = 0.20 V`.

**Rightward (push right → position increases), `V > b_hi`:**
- Deadband `b_lo ≤ V ≤ b_hi` → `0`.
- Ramp zone `b_hi < V ≤ b_hi + DV` → **exponential rise 0 → +BOOST_MIN_MN** (0 at `b_hi`, +12 N at `b_hi + DV`).
- Saturation zone `V > b_hi + DV` → from `+BOOST_MIN_MN`, **asymptotically plateau to +BOOST_MAX_MN at V = 3.3 V**.

**Leftward (push left → position decreases), `V < b_lo`:** mirror image, negative sign.
- Ramp zone `b_lo - DV ≤ V < b_lo` → 0 → **−BOOST_MIN_MN** (−12 N at `b_lo - DV`).
- Saturation zone `V < b_lo - DV` → from −12 N, **plateau to −BOOST_MAX_MN at V = 0 V**.

Continuous, monotonic, zero in the deadband, sign = push direction. Suggested forms (expose shape constants `k`, `m`):
- Ramp (`u = (V - b_hi)/DV ∈ [0,1]`): `boost = BOOST_MIN_MN * (exp(k*u) - 1)/(exp(k) - 1)`.
- Saturation (`w = (V - (b_hi+DV))/(3.3 - (b_hi+DV)) ∈ [0,1]`): `boost = BOOST_MAX_MN - (BOOST_MAX_MN - BOOST_MIN_MN)*exp(-m*w)`, clamp exactly to `BOOST_MAX_MN` at the rail (`m ≈ 4` start).

`boost_human` depends **only** on handle voltage; it is **not** velocity-gated — it applies whether or not the shaft is moving.

### C.3 — `boost_stiction` (Task 0 phase only; threshold + exponential-decay envelope)

> **STATUS (06/25/26): implemented, bench-pending.** `boost_stiction` + the C.5
> combine modes are wired into `teensy_cartpole_haptic.ino`'s `haptic_loop()`
> (Task 0 only; the COR loop is untouched). The sketch now `#include`s
> `stiction_model.h` (auto-copied by `export_stiction_header.py`). Magnitude =
> `stiction_lookup_mN(pos, dir) * stiction_mult * env`, where `env = 1` for
> `|v| <= stiction_v_thresh_mm_s` and `exp(-(|v|-v_thresh)/stiction_decay_mm_s)`
> above it (the "stiction_only" envelope from `Arduino/validate_controller`);
> `dir = sign(f_command)` at/below the threshold, `sign(v)` above. Six params ride
> in `CMD_CONFIG` (now `<13f>`, 52 B): `stiction_enable, stiction_mult,
> stiction_v_thresh_mm_s, stiction_decay_mm_s, boost_combine_mode, boost_cap_mN` —
> all adjustable in `common/teensy_common.mwel` ('Stiction Boost' group), read each
> `reset()`. Telemetry (`boost_stiction_mN`, `stiction_gate`, `boost_combined_mN`)
> already existed in the 68 B `Sample`; now populated. Mac: `teensy_interface.py`
> `CONFIG_FMT`, `helpers.build_config_params()`. Docs: `TEENSY_API.md` §7b + CONFIG
> table, `HAPTIC_LOOP_README.md`. **TODO:** flash + bench-run; confirm the boost
> feels right with the dynamics and the `sign(f_command)`↔table sign convention.

Use the design decision we formulated from validate_controller.
Decision: stiction is fully applied under a certain velocity threshold and exponentially decays.
Default parameters: v_thresh = 350, decay = 100
Be sure that these parameters are adjustable in teensy_common.mwel

> **Note (06/25/26):** the `exp(-(v/epsilon)^2)` gate below is the *old* ILC design;
> it has been **superseded** by the threshold + exponential-decay envelope above.
> Kept here only for provenance / sign conventions.

An old example gating, lifted from `ilc_runner.py` (the `stiction_enabled` block):

```python
sign_xddot = np.sign(f_command) if f_command != 0.0 else 1.0
f_s_mN = stiction_model.predict(x_um, sign_xddot, safety_margin=stiction_safety)  # ~1.1
g = np.exp(-(v_kf / epsilon) ** 2)        # epsilon = gating_epsilon_mm_s, default 5.0
f_stiction_term_N = (f_s_mN / 1000.0) * g
f_total += f_stiction_term_N
```
NOTE: the parameter `safety_margin` in the stiction_model.predict function is a misnomer. This is really a multiplier on the predicted force. Default to 1 (the amplification is whatever the calibration returned exactly). Rename this parameter to `stiction_mult`.

Teensy translation:
- **Magnitude/sign:** `f_s_mN = stiction_lookup_mN(pos_um, sign(f_command))` × `stiction_mult` (default **1.0**). Direction is the sign of the commanded haptic force `f_command` (the pendulum reaction). The lookup already returns signed values per the figure (positive for extend, negative for retract), so honor the table's sign convention and reconcile with `sign(f_command)`.
- **Velocity gate:** `g = exp(-(v / epsilon)^2)` with `epsilon = 5.0 mm/s` (`gating_epsilon_mm_s`). This is the exact form from the ILC code.
- `boost_stiction_mN = f_s_mN * safety_margin * g`.

This term is added only in the Task 0 phase, after `boost_human` is validated in Task 1 and Task 0.

### C.4 — Where the boost enters

The boost is added to `f_command` **before** the existing `MAX_FORCE_SAFETY_MN` clip and the mN conversion in the loop. In the Task 1 phase, "the boost" is just `boost_human`. In the Task 0 phase, it is the combination from C.5.

### C.5 — Combining the two boosts (Task 0 phase only — switchable, empirical)

The `boost_human` helps the subject start the shaft from rest (friction) and make the shaft feel less heavy. The `boost_stiction` helps maintain the fidelity of the theoretical cart-pendulum dynamics when the system is slowing down into the stiction regime. When both boosts are active, they can compensate the **same** physical static friction if they are aligned, so naively adding two large aligned terms can over-boost and lurch the shaft. Make the combination a runtime-selectable mode (`BOOST_COMBINE_MODE`) and A/B test. Compute the gated `boost_stiction` and the handle-driven `boost_human` first, then:

- **`ADDITIVE_CAPPED` (recommended default):** `boost = boost_stiction + boost_human`, then clip the **combined** magnitude to `BOOST_CAP_MN` (set near max expected breakaway, ~12000–13000 mN from the figure, or to `BOOST_MAX_MN`). Faithful to "independent additive terms"; the cap plus `MAX_FORCE_SAFETY_MN` prevents lurch.
- **`HUMAN_OVERRIDE`:** if the handle is out of deadband (`|boost_human| > 0`), use `boost_human` only; else `boost_stiction`. Matches "drop the stiction boost when the human is actively pushing."
- **`MAX_ALIGNED`:** same sign → take the larger magnitude (friction overcome once); opposite signs → add (partial cancel). Most physically principled.

### C.6 — Telemetry additions

Append to the haptic `Sample` (add fields at the end; bump a version byte if needed to keep `0xB0` parsers from breaking):
- `handle_voltage` (float, V)
- `boost_human_mN` (float)
- `boost_stiction_mN` (float, 0 until Task 0 phase)
- `stiction_gate` (float, 0–1)
- `boost_combined_mN` (float)

Update `data_logger.py` `FIELDS` and the Mac-side `struct` unpack to match.

### C.7 - Analysis script
**Existing Jupyter notebook:** /Users/Nikasha/Documents/GitHub/cart_pendulum_task/cart_pendulum_teensy/teensy_mworks_analysis.ipynb
- Make sure that the existing jupyter notebook can load in any of the generated mworks and csv files (from Task 0 and Task 1)
- Keep the code that plots how long the control loop takes on the teensy to track if there are any over runs
- If there's a foce boost due to the human, add a plot of the force boost command throughout time in the experiment.
---

## Acceptance Criteria

1. **Characterization sketch** (`Arduino/stiction/stiction.ino`) sweeps the full stroke over `n_points` positions × both directions via quasi-static breakaway, logs raw ramps + breakaway points to `Arduino/stiction/`, and a Python helper fits the GP (reusing `orca_controllers` code) and exports `stiction_model.h` with **means only**, plus a constant fallback.
2. **Validation sketch** (`Arduino/validate_controller/validate_controller.ino`) `#include`s `stiction_model.h`, interpolates the means, replays sinusoidal-control trajectories with `trajectories.ipynb` §4, and shows lower tracking error with friction compensation enabled than disabled.
3. **Handle calibration** runs 2 s at the start of every task, reports/logs measured `baseline_min/max`, and errors out on a bad baseline.
4. **`boost_human` in Task 1 and Task 0:** zero in the measured deadband; exponential ramp to ±12 N at ±0.20 V beyond the band; plateau toward ±`max_boost_N` at the 3.3 V / 0 V rails; sign matches push direction; not velocity-gated. Bench-tested before any Task 0 work, then bench-test in Task 0 **without** `boost_stiction`.
5. **`boost_stiction` (Task 0 phase):** magnitude from `stiction_lookup_mN` × stiction_mult, sign from `sign(f_command)`, gated by `exp(-(v/5.0)^2)` — matching `ilc_runner.py`.
6. **Combination (Task 0 phase):** `BOOST_COMBINE_MODE` switches `ADDITIVE_CAPPED` (default) / `HUMAN_OVERRIDE` / `MAX_ALIGNED`; combined boost added before the safety clip; all components in telemetry + CSV.
7. **Modularity:** characterization and validation live in their own sketches under `Arduino/`; the haptic/task sketch is not burdened with bench-only code.
8. **No regressions:** handle at baseline + `boost_stiction` disabled → Task 0 behaves as before.

## Open items to confirm
- **Opcodes** Make sure all opcodes are properly referenced in TEENSY_API.md (once that is created by CLAUDE).
- **`BOOST_CAP_MN`** for `ADDITIVE_CAPPED` — set to max expected breakaway or to `max_boost_N`?
- **Sweep parameters** — `n_points`, `ramp_rate_N_s`, `breakaway_mm_s`, repeats per bin; set so the ramp is quasi-static but not painfully slow. The figure suggests ~30 positions over ~600 mm worked.
- **Sign reconciliation** — the table returns signed breakaway (positive extend / negative retract). Confirm this matches `sign(f_command)` direction convention in the haptic loop so the stiction boost pushes the intended way (the C.3 self-check from the haptic sign-test still applies).