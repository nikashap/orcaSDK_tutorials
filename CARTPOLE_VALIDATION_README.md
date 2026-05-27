# Cart-Pole Physics Validation: Teensy + Python

## Purpose

Validate that a semi-implicit Euler integrator running on a Teensy 4.1 correctly integrates the full cart-pole equations of motion. This is a **standalone numerical-correctness test** — no motor, no force sensor, no real-time constraint. The Teensy runs the dynamics as fast as it can, streams the trajectory back to a Mac over UDP, and the Mac compares against a high-accuracy reference computed with `scipy.integrate.solve_ivp`.

The output is a confidence statement: "the Teensy reproduces the reference trajectory within errors consistent with semi-implicit Euler at the chosen timestep."

## Deliverables

Produce these files:

1. `test_teensy_orca/teensy_cartpole_sim/teensy_cartpole_sim.ino` — Arduino sketch for Teensy 4.1.
2. `test_teesny_orca/validate_cartpole.py` — Python driver: configures, runs, receives, compares, plots.
3. `test_teensy_orca/cartpole_reference.py` — Reference implementation of the dynamics for `solve_ivp`.
4. `test_teensy_orca/test_cases.py` — Definitions of the test scenarios (importable from the validator).
5. `test_teensy_orca/README.md` for the validation suite (concise, how to run).

Plots and a numerical summary should be written to a `test_teensy_orca/results/` directory created at run time.

## Physics

State vector $\mathbf{x} = (x, \theta, \dot x, \dot\theta)$, where $x$ is cart position (m), $\theta$ is pendulum angle measured counter-clockwise from straight-down (rad). Inputs: external horizontal force $f_x$ on the cart (N). Parameters: $m_c$ (cart mass, kg), $m_p$ (pendulum mass, kg), $l$ (pendulum length to point mass, m), $g$ (gravity, m/s², default 9.81).

### Accelerations (closed form, from Underactuated Robotics §3.2)

$$
\ddot{x} = \frac{f_x + m_p \sin\theta\,(l\,\dot\theta^2 + g\cos\theta)}{m_c + m_p \sin^2\theta}
$$

$$
\ddot{\theta} = \frac{-f_x\cos\theta - m_p l \dot\theta^2 \cos\theta \sin\theta - (m_c+m_p)\,g\sin\theta}{l\,(m_c + m_p \sin^2\theta)}
$$

The denominator $m_c + m_p\sin^2\theta$ is bounded below by $m_c > 0$; no singularity guarding is required.

### Integrator: semi-implicit Euler (MuJoCo's `Euler` with no joint damping)

Per the MuJoCo numerical-integration docs, semi-implicit Euler is:

$$
v_{t+h} = v_t + h\, a_t, \qquad q_{t+h} = q_t + h\, v_{t+h}
$$

With no joint damping in this model, MuJoCo's `Euler` integrator reduces exactly to the formula above. Concretely, per step of size $h = \Delta t$:

1. Evaluate $(\ddot x_k, \ddot\theta_k)$ from current state $(x_k, \theta_k, \dot x_k, \dot\theta_k)$ and current $f_x(t_k)$.
2. $\dot x_{k+1} = \dot x_k + h\,\ddot x_k$ and $\dot\theta_{k+1} = \dot\theta_k + h\,\ddot\theta_k$.
3. $x_{k+1} = x_k + h\,\dot x_{k+1}$ and $\theta_{k+1} = \theta_k + h\,\dot\theta_{k+1}$ (note: **new** velocities).

Use `double` precision throughout (the Teensy 4.1 Cortex-M7 has a hardware double-precision FPU).

### Forcing functions $f_x(t)$

Supported modes (chosen by an integer code on the wire):

| Mode | Name | $f_x(t)$ |
|---|---|---|
| 0 | zero | 0 |
| 1 | constant | `param1` |
| 2 | sinusoidal | `param1 * sin(2π * param2 * t)` |
| 3 | step | `0` for `t < param2`, else `param1` |
| 4 | impulse | `param1` for `t < param2`, else `0` |

The Python and Teensy implementations must produce identical $f_x(t)$ values for the same parameters. Use `t = step_index * dt` (not accumulated `t += dt`, which drifts).

## Architecture

```
+-----------------+   UDP (Ethernet)   +-------------------+
|  Mac (Python)   | <----------------> |  Teensy 4.1       |
|  validate_*.py  |                    |  cartpole_sim.ino |
|  scipy ref sim  |                    |  semi-impl. Euler |
+-----------------+                    +-------------------+
```

Reuse the QNEthernet + UDP text-protocol style from `teensy_orca_bridge.ino` (Teensy IP `192.168.1.177`, listens on UDP port `8888`, replies to whatever port the Mac sent from). No motor, no `Serial1` traffic, no Modbus — this sketch is purely simulation + networking.

## Communication Protocol

Text lines, one command per UDP packet, newline-terminated. Style matches `teensy_orca_bridge.ino`.

### Commands (Mac → Teensy)

```
PING
SIM_CONFIG <m_c> <m_p> <l> <g>
SIM_INIT   <x0> <theta0> <xdot0> <thetadot0>
SIM_RUN    <duration_s> <dt_s> <fx_mode> <fx_param1> <fx_param2> <sample_stride>
ABORT
```

All numeric values are SI units and base-10 ASCII floats (parse with `strtod`). `sample_stride` is an integer ≥ 1: emit one sample every `sample_stride` integration steps.

### Responses (Teensy → Mac)

```
ACK <command_name>
ERROR <message>
SIM_START <total_steps> <wall_time_start_us>
SIM_SAMPLE <step_index> <t> <x> <theta> <xdot> <thetadot> <fx>
SIM_DONE  <total_steps> <wall_time_elapsed_us>
```

`wall_time_*` values come from `micros()` and let the Mac sanity-check that the Teensy did not silently throttle or hang. `SIM_SAMPLE` lines are emitted during the run. To avoid UDP packet fragmentation and improve throughput, **batch up to ~8 samples per UDP packet**, one per line, separated by `\n`.

Format `t`, `x`, `theta`, `xdot`, `thetadot`, `fx` with `printf("%.10g")` so the Python side can recover full double precision.

## Teensy Implementation Requirements (`teensy_cartpole_sim.ino`)

- Use QNEthernet, same setup pattern as `teensy_orca_bridge.ino`.
- State variables: `double m_c, m_p, l, g; double x, theta, xdot, thetadot; double dt; uint64_t total_steps; int sample_stride;` plus the force-mode params.
- Implement a single function `void step()` that performs one semi-implicit Euler update using the equations above. Inline the math; do not over-abstract.
- Run the simulation in a tight loop after receiving `SIM_RUN`. The simulation is **not** real-time. Wall time should be << simulated time; expected throughput is hundreds of thousands of steps per second.
- After each step, if `step_index % sample_stride == 0`, append the sample to an in-RAM buffer. When the buffer holds ~8 samples or the simulation ends, send via UDP and reset the buffer.
- Yield to UDP RX (`Udp.parsePacket()`) at least every ~1 ms of wall time so an `ABORT` command can interrupt.
- On `SIM_RUN` completion, send `SIM_DONE`.
- Errors (bad command, NaN in state, etc.) → `ERROR <reason>`.

### Numerical hygiene

- Compute $\sin\theta, \cos\theta$ once per step (call `sincos` if available, otherwise two separate calls), store in locals, and reuse.
- Compute the denominator $D = m_c + m_p \sin^2\theta$ once, divide twice.
- Wrap $\theta$ into $(-\pi, \pi]$ **only for the output**, not for the integration state. The integrator should work on unwrapped $\theta$.
- After each step, check `isfinite(x) && isfinite(theta) && isfinite(xdot) && isfinite(thetadot)`; if any fail, send `ERROR NAN_STATE` and abort.

## Python Implementation Requirements

### `cartpole_reference.py`

Single function:

```python
def cartpole_rhs(t, state, m_c, m_p, l, g, fx_func):
    x, theta, xdot, thetadot = state
    fx = fx_func(t)
    s, c = math.sin(theta), math.cos(theta)
    D = m_c + m_p * s * s
    xddot = (fx + m_p * s * (l * thetadot * thetadot + g * c)) / D
    thetaddot = (-fx * c - m_p * l * thetadot * thetadot * c * s - (m_c + m_p) * g * s) / (l * D)
    return [xdot, thetadot, xddot, thetaddot]
```

Plus a helper that returns the same $f_x(t)$ given a mode and params, so Python and Teensy stay synchronized.

### `test_cases.py`

Each test case is a dict / dataclass with: name, parameters $(m_c, m_p, l, g)$, initial state, duration, dt, force mode + params, sample stride, and a tolerance dict.

Required cases (run all by default):

1. **`equilibrium`** — $x_0 = 0.25$, $\theta_0 = 0$, $\dot x_0 = \dot\theta_0 = 0$, $f_x \equiv 0$, $T = 2$ s, $dt = 1$ ms. Expectation: state changes are at the level of floating-point round-off.

2. **`small_oscillation`** — $\theta_0 = 0.05$ rad, all else zero, $f_x \equiv 0$, $T = 5$ s, $dt = 1$ ms. Expectation: small oscillation; compare to reference and to the linearized period $T_\text{lin} = 2\pi\sqrt{\tfrac{m_c l}{(m_c+m_p)g}}$ (small-angle).

3. **`large_swing`** — $\theta_0 = \pi/3$, all else zero, $f_x \equiv 0$, $T = 10$ s, $dt = 1$ ms.

4. **`high_initial_velocity`** — $\theta_0 = 0$, $\dot\theta_0 = 2$ rad/s, all else zero, $f_x \equiv 0$, $T = 5$ s, $dt = 1$ ms.

5. **`energy_conservation`** — $\theta_0 = \pi/3$, all else zero, $f_x \equiv 0$, $T = 30$ s, $dt = 1$ ms. Track total mechanical energy and horizontal momentum over time; both should be conserved by the **continuous** dynamics. SIE is symplectic, so energy should oscillate with bounded amplitude rather than drift monotonically; momentum should be conserved to ~machine precision.

6. **`constant_force`** — Zero initial state, $f_x = 1$ N constant, $T = 2$ s, $dt = 1$ ms.

7. **`sinusoidal_drive`** — Zero initial state, $f_x = 2\sin(2\pi \cdot 0.5\, t)$ N, $T = 10$ s, $dt = 1$ ms.

8. **`near_upright`** — $\theta_0 = \pi - 0.01$, all else zero, $f_x \equiv 0$, $T = 2$ s, $dt = 1$ ms. Falls away from the unstable equilibrium.

9. **`x_offset_sweep`** — Repeat case `small_oscillation` for $x_0 \in \{0, 0.1, 0.2, 0.3, 0.4, 0.5\}$ m. Trajectories must be identical up to a constant offset in $x$ (translational invariance check).

10. **`convergence`** — Repeat `large_swing` at $dt \in \{2, 1, 0.5, 0.25, 0.125\}$ ms. Verify that the error vs. reference scales like $O(dt)$ (first-order accuracy). This is the strongest single check that the integrator is implemented correctly.

### `validate_cartpole.py`

For each test case:

1. Send `SIM_CONFIG`, `SIM_INIT`, then `SIM_RUN`.
2. Receive `SIM_SAMPLE` packets until `SIM_DONE` (with a generous wall-time timeout).
3. Run the reference simulation with `solve_ivp(..., method='DOP853', rtol=1e-12, atol=1e-14, dense_output=True)` over the same time span and parameters.
4. Evaluate the reference at the same sample times the Teensy reported.
5. Compute per-state error metrics:
   - max absolute error in $x, \theta, \dot x, \dot\theta$
   - RMS error in same
   - for `energy_conservation`: $\Delta E / E_0$ and $\Delta p$ over time
   - for `convergence`: error vs. $dt$ on log-log axes; fit slope
   - for `x_offset_sweep`: residual of (Teensy trajectory − constant) across runs
6. Save plots to `results/<case_name>.png` (one figure per case, multi-panel: state overlays, errors vs. time, and where relevant energy/momentum vs. time).
7. Print a single summary line per case: `PASS` or `FAIL` with the dominant error number.
8. Exit nonzero if any case fails.

### Energy and momentum (for reference computation)

$$
E = \tfrac{1}{2}(m_c + m_p)\dot x^2 + m_p \dot x \dot\theta l \cos\theta + \tfrac{1}{2}m_p l^2 \dot\theta^2 - m_p g l \cos\theta
$$

$$
p = (m_c + m_p)\dot x + m_p l \dot\theta \cos\theta
$$

## Success Criteria

For semi-implicit Euler at $dt = 1$ ms, with reference from `DOP853` at `rtol=1e-12`, over the cases above (excluding `convergence`):

- **`equilibrium`**: all state errors $< 10^{-12}$.
- **`x_offset_sweep`**: residuals across runs $< 10^{-12}$ (translational invariance is exact in the equations, so deviation indicates a bug).
- **All other cases**: max absolute position error $< 5\times 10^{-3}$ m (5 mm) and $< 5\times 10^{-3}$ rad over the duration; max velocity errors $< 5\times 10^{-2}$ m/s and rad/s. These thresholds correspond to roughly 1% of the trajectory amplitude for the test parameters; tighten if observed errors are much smaller.
- **`energy_conservation`**: $|\Delta E / E_0| < 10^{-2}$ over 30 s with bounded (non-secular) drift; $|\Delta p| < 10^{-10}$ (momentum is preserved exactly by the continuous equations and very nearly by SIE since $f_x = 0$).
- **`convergence`**: log-log slope of max error vs. $dt$ in the range $[0.9, 1.2]$ (semi-implicit Euler is first-order in global error).

A failure that is small in magnitude but the **wrong scaling** in the `convergence` test (e.g., slope of 0 or 2) is a stronger signal of a bug than a marginally-out-of-bounds absolute error.

## Notes on Style

- Match the style of `teensy_orca_bridge.ino`: ASCII text protocol, ALL_CAPS commands, `ACK`/`ERROR` responses, `IPAddress`/`EthernetUDP` types.
- Do not add features beyond what is specified. This is a validator, not a product. No web UI, no command queue, no logging to SD card.
- Python target: 3.10+, dependencies `numpy`, `scipy`, `matplotlib`. Use `argparse` for any user-facing options (Teensy IP, port, which cases to run). Default to running all cases.
- Keep the Python single-threaded; UDP receive can use a 200 ms socket timeout and a simple while-loop.

## Out of Scope (do not implement)

- Force sensor I/O.
- Modbus / Orca motor traffic.
- Friction lookup table.
- Haptic-rendering force feedback.
- Real-time scheduling, `IntervalTimer`-paced loops.
- Any RK4 variant (semi-implicit Euler only).

These belong to later phases of the project and would confound the numerical validation if mixed in here.
