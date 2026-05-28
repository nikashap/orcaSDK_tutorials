# Step 4 — Integrated Haptic Loop (Bench)

## Purpose

Merge the validated cart-pole physics (Step 1) into the working Modbus/Ethernet bridge sketch (Step 2) to produce the first real haptic loop. A human manually drives the Orca shaft (the "cart"); the Teensy integrates the pendulum dynamics from the measured shaft acceleration and commands the motor to render the pendulum's reaction force. No MWorks yet — this is a bench test driven by a Python script over UDP.

Build on `test_teensy_orca/teensy_orca_bridge` (QNEthernet, ALL_CAPS UDP command protocol, `ACK`/`ERROR` replies, Modbus over `Serial1` at 1 Mbaud). Do not start from scratch.

Deliverable: `teensy_cartpole_haptic/teensy_cartpole_haptic.ino` plus a Python bench client `haptic_bench.py`.

## Physics (authoritative — supersedes any earlier draft)

State: $(x, \theta, \dot x, \dot\theta)$. $x$ = cart position (m), $\theta$ = pendulum angle CCW from straight-down (rad). Parameters: $m_c$ (virtual cart mass), $m_p$ (pendulum mass), $m_s$ (real shaft mass), $l$ (pendulum length), $g = 9.81$.

### Each cycle, the cart kinematics come from the motor, not from integration

Set $\ddot x$, $\dot x$, $x$ in the physics state to the **measured** shaft acceleration, velocity, and position from the motor's extended stream. The cart is not integrated — it is driven by the human and observed.

### Pendulum angular acceleration (from the second EoM)

$$
\ddot\theta = -\frac{1}{l}\big(\ddot x \cos\theta + g\sin\theta\big)
$$

using the **previous** cycle's $\theta, \dot\theta$ and the current measured $\ddot x$.

### Implied human force (log only)

From the first EoM, the force the simulation infers the human applied:

$$
f_x = (m_c + m_p)\,\ddot x + m_p l\,\ddot\theta\cos\theta - m_p l\,\dot\theta^2\sin\theta
$$

This is computed for logging/diagnostics only; it is never used in the command path.

### Force of pendulum on cart

Define $P \equiv m_p l\,(\ddot\theta\cos\theta - \dot\theta^2\sin\theta)$. The horizontal reaction force the pendulum exerts on the cart is the negative of this:

$$
f_{pc} = -P = -m_p l\,\big(\ddot\theta\cos\theta - \dot\theta^2\sin\theta\big)
$$

### Motor command (mass-scaled)

To make the real shaft of mass $m_s$ accelerate as the virtual cart of mass $(m_c + m_p)$ would under the same human input, scale the reaction force by the mass ratio:

$$
f_\text{command} = \frac{m_s}{m_c + m_p}\,f_{pc}
$$

Derivation, for the record: the virtual system gives $\ddot x = (f_x - P)/(m_c+m_p)$. Requiring the real shaft to match, $m_s\ddot x = \tfrac{m_s}{m_c+m_p}f_x - \tfrac{m_s}{m_c+m_p}P$. The first term is supplied by the human's hand (this is the modeling assumption — the human is a pure force source and $f_x$ is inferred, never measured), leaving the command as the second term, $\tfrac{m_s}{m_c+m_p}f_{pc}$.

No force sensor is required in this phase; $f_x$ is inferred from the measured $\ddot x$.

### Integration of the pendulum state

Advance only $\theta, \dot\theta$ by one RK4 step of $h = 1.8$ ms. **The measured $\ddot x$ is held constant (zero-order hold) across all four RK4 stages** — there is only one motor reading per cycle, so do not attempt to re-read the motor mid-step. The RK4 right-hand side is the single equation $\ddot\theta = -(\ddot x\cos\theta + g\sin\theta)/l$ with $\ddot x$ fixed. (At this step size RK4 buys little over semi-implicit Euler given the ZOH, but keep RK4 for consistency with Step 1.)

Do not integrate $x, \dot x$ — overwrite them from the motor each cycle.

## Sign Verification (MANDATORY startup self-check)

Before entering the loop, run a one-time numeric check in code and refuse to start if it fails. With the pendulum hanging ($\theta = 0$, $\dot\theta = 0$) and a small positive cart acceleration ($\ddot x = +1.0\ \text{m/s}^2$):

- $\ddot\theta = -(\ddot x\cdot 1 + 0)/l = -\ddot x/l < 0$.
- $P = m_p l(\ddot\theta\cdot 1 - 0) = m_p l\ddot\theta < 0$, so $f_{pc} = -P > 0$... 

**Stop and think about the physical expectation:** when the human shoves the cart in $+x$, the hanging pendulum lags behind and should pull the cart back toward $-x$. So the *rendered force the human feels* must oppose the shove, i.e. be negative for a positive shove.

Compute $f_\text{command}$ for $\ddot x = +1.0$, $\theta = 0$, $\dot\theta = 0$ with the current parameters and assert:

```
assert f_command < 0   // positive shove must produce restoring (negative) force
```

If this assertion fails, send `ERROR` with code `ERR_SIGN_CHECK` (`0xA2 0x04`) and put the motor to sleep — do not run the loop. This catches a flipped sign in $f_{pc}$ or the command scaling before any force reaches the human. (Resolve the apparent tension in the hand-computation above by trusting this physical test; it is the ground truth.)

## Command Protocol (Mac → Teensy)

Binary, not text. Every command is a UDP packet whose first byte is a one-byte opcode, optionally followed by a fixed little-endian payload. No delimiters, no whitespace, no newline. The opcode alone fully determines the payload length, so the Teensy reads byte 0, then reads exactly the expected number of payload bytes. A packet whose length does not match the opcode's expected length is rejected with `ERR_BADLEN`.

All multi-byte payload fields are little-endian. Floating-point parameters are `float` (4 bytes) in SI units unless noted.

### Opcode table

| Opcode (hex) | Name | Payload | Bytes | Meaning |
|---|---|---|---|---|
| `0x01` | `CMD_PING` | none | 0 | Liveness check. Teensy replies `ACK 0x01`. |
| `0x02` | `CMD_AUTOZERO` | none | 0 | Run motor autozero (see below). |
| `0x03` | `CMD_CONFIG` | 7 × `float` | 28 | Set parameters, in order: `m_c, m_p, m_s, l, g, max_force_mN, loop_period_us, x_init, theta_init, xdot_init, thetadot_init`. (`loop_period_us` is a float for wire uniformity; round to integer µs on receipt.) |
| `0x04` | `CMD_INIT` | 4 × `float` | 16 | Initial pendulum state, in order: `x0, theta0, xdot0, thetadot0`. |
| `0x05` | `CMD_BEGIN` | none | 0 | Enter the haptic loop (see below). |
| `0x06` | `CMD_END` | none | 0 | Exit the loop, sleep the motor (see below). |
| `0x07` | `CMD_SLEEP` | none | 0 | Immediate motor-sleep + idle, valid any time. |

Opcodes `0x00` and `0x08`–`0xFF` are reserved; receiving one yields `ERR_BADOP`.

Payload byte offsets for reference (all little-endian):
- `CMD_CONFIG`: `m_c`@0, `m_p`@4, `m_s`@8, `l`@12, `g`@16, `max_force_mN`@20, `loop_period_us`@24.
- `CMD_INIT`: `x0`@0, `theta0`@4, `xdot0`@8, `thetadot0`@12.

### Command notes

- `CMD_AUTOZERO` — command the motor's autozero procedure; allow 6 s, max force 50 N, max speed 100 mm/s. Block (or poll) until complete, then reply `ACK 0x02`. Reply `ERROR` if the motor reports a fault.
- `CMD_CONFIG` — `max_force_mN` is the safety clip magnitude; `loop_period_us` is the target loop period (default 1800). Reply `ACK 0x03`.
- `CMD_INIT` — $x, \dot x$ are overwritten by the first motor read; only $\theta, \dot\theta$ are honored. Reply `ACK 0x04`.
- `CMD_BEGIN`, `CMD_END`, `CMD_SLEEP` — see below.

## Responses (Teensy → Mac)

Keep packets compact. Control replies are short; the high-rate stream is packed binary.

- **Control replies** begin with a one-byte reply opcode:
  - `0xA1 ACK` — followed by one byte echoing the acknowledged command opcode (e.g. `0xA1 0x03` = "CONFIG accepted").
  - `0xA2 ERROR` — followed by one byte error code (see error-code table below).
- **Per-cycle telemetry**: a packed binary record (little-endian), batched ~8 per UDP packet. Each telemetry packet begins with the byte `0xB0` so the receiver can distinguish it from control replies, followed by a `uint8` count of records, then that many `Sample` structs back-to-back.

```
struct __attribute__((packed)) Sample {
  uint32_t cycle;          // loop iteration counter
  uint32_t t_meas_us;      // micros() at the moment xdd was measured
  float    x, xdot, xddot; // measured shaft state (SI)
  float    theta, thetadot;// integrated pendulum state
  float    fx;             // inferred human force (log only)
  float    fpc;            // pendulum reaction force
  float    f_command_mN;   // commanded force after scaling + clip
  uint32_t loop_us;        // measured duration of this loop iteration
};
```

Document the struct layout in a comment so the Python side can `struct.unpack` it. Flush any partial batch on `CMD_END`.

### Error-code table (one byte, follows `0xA2`)

| Code | Name | Meaning |
|---|---|---|
| `0x01` | `ERR_BADOP` | Unknown/reserved opcode received. |
| `0x02` | `ERR_BADLEN` | Packet length did not match opcode's expected payload. |
| `0x03` | `ERR_MOTOR_FAULT` | Motor reported an active fault (fault bits follow as `uint16`). |
| `0x04` | `ERR_SIGN_CHECK` | Startup sign-verification failed; loop refused to start. |
| `0x05` | `ERR_NAN_STATE` | A physics state went non-finite. |
| `0x06` | `ERR_MODBUS_TIMEOUT` | Motor exchange timed out. |
| `0x07` | `ERR_NOT_CONFIGURED` | `CMD_BEGIN` received before a valid `CMD_CONFIG`. |

## CMD_BEGIN — entry to the haptic loop

1. Refuse to start if no valid `CMD_CONFIG` has been received: send `ERROR` with `ERR_NOT_CONFIGURED`.
2. Refuse to start if the motor reports any active error: send `ERROR` with `ERR_MOTOR_FAULT` followed by the fault bits (`uint16`), and stay idle.
3. Run the mandatory sign-verification self-check above; on failure send `ERR_SIGN_CHECK`.
4. Zero the cycle counter, set the loop timer, and enter the main loop.
5. Set the initial values of the simulation (`x_init, theta_init, xdot_init, thetadot_init`), step forward with $f_x = 0$ to calculate $\ddot{\theta}$ and subsequently ${f_pc}$, and stream the initial ${f_pc}$ command to the motor before the rest of the loop commences
6. Send `ACK 0x05` once the loop is running.

## Main Simulation Loop (one iteration)

In order:

1. **Motor exchange (extended mode):** stream the *previous* iteration's force command, and read shaft position, velocity, and acceleration with timestamps. Budget ~1.54 ms worst case (three ~500 µs operations). Timestamp `t_meas_us = micros()` at the moment the acceleration read completes.
2. **Note the position lag:** the streamed position trails by three control frames. For this step, feed the lagged position as-is and document it; do not extrapolate. (Extrapolation belongs to the later friction-table phase, where position accuracy matters more.)
3. **Overwrite cart state:** set physics $\ddot x, \dot x, x$ to the measured values.
4. **Compute** $\ddot\theta$ from the current measured $\ddot x$ and previous $\theta, \dot\theta$.
5. **Compute** $f_x$ (log only), $P$, and $f_{pc} = -P$.
6. **Scale:** $f_\text{command} = \tfrac{m_s}{m_c+m_p} f_{pc}$.
7. **Convert to mN** and **clip** to $\pm$`max_force_mN`. The clipped value is what gets streamed next cycle.
8. **Integrate** $\theta, \dot\theta$ one RK4 step ($h = 1.8$ ms, $\ddot x$ ZOH).
9. **Record** `loop_us` for this iteration.
10. **Push** a `Sample` to the telemetry batch.
11. **Check for UDP commands** (non-blocking) so `CMD_END`/`CMD_SLEEP` can interrupt.
12. **Pace the loop:** if the iteration took less than `loop_period_us`, sleep until the period boundary (fixed-interval schedule). If it took longer, proceed to the next iteration immediately and (optionally) count the overrun.

### Error handling inside the loop

If the motor reports a fault, or any state goes non-finite, or a Modbus exchange times out: stop the loop, put the motor into sleep mode immediately, send `ERROR` with the appropriate code (`ERR_MOTOR_FAULT`, `ERR_NAN_STATE`, or `ERR_MODBUS_TIMEOUT`), and return to idle. Safety takes priority over telemetry.

## CMD_END / CMD_SLEEP

- On `CMD_END` (UDP) or any error: put the motor into sleep mode safely, flush the telemetry batch, then send a final summary packet beginning with byte `0xB1` followed by the most recent `Sample` struct and the total cycle count (`uint32`). Then send `ACK 0x06`.
- `CMD_SLEEP` is an immediate motor-sleep + idle, usable any time. Reply `ACK 0x07`.

## Build log and analysis code
- Flush data received from the motor to an h5 log with teensy timestamps (maybe use teensy as the global clock?)
- Write an accompanying jupyter notebook that loads in the h5 log and visualizes plots of time distribution for commands

## Out of Scope for Step 4

- Friction compensation / GP stiction boost (added in the modified Step 4 after Step 3).
- Force-sensor input and mass scaling that differs from the $\tfrac{m_s}{m_c+m_p}$ factor above.
- Position extrapolation for the three-frame lag.
- MWorks integration.

## Acceptance Criteria

- Sign self-check passes; a deliberate sign flip is caught and refuses to run.
- Manually shoving the shaft in $+x$ produces a felt force opposing the shove (and vice versa); releasing lets the virtual pendulum swing and the felt force tracks it.
- Sustained loop rate ≥ 500 Hz (period ≤ 1.8 ms) for several minutes with no overruns logged under normal operation.
- Telemetry arrives at the Mac with monotonic `cycle` counters and no sustained gaps; timestamps are consistent with the loop period.
- On any injected fault (unplug, force a Modbus timeout), the motor goes to sleep and the Mac receives an `ERROR`.