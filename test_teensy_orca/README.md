# Cart-Pole Physics Validation Suite

Validates the Teensy 4.1 RK4 integrator against a high-accuracy
`scipy.integrate.solve_ivp` (DOP853) reference for the cart-pole dynamics.

## Hardware

- Teensy 4.1 with Ethernet kit, connected to Mac via Ethernet.
- No motor or Modbus wiring required — this is a pure simulation test.
- Teensy IP: `192.168.1.177`, UDP port `8888`.

## Setup

1. Flash `teensy_cartpole_sim/teensy_cartpole_sim.ino` to the Teensy 4.1
   (requires QNEthernet library).
2. Connect Teensy to the Mac via Ethernet (direct or through a switch).
3. Install Python dependencies: `pip install numpy scipy matplotlib`

## Running

```bash
# Run all test cases
python3 validate_cartpole.py

# Run specific cases
python3 validate_cartpole.py --cases equilibrium small_oscillation convergence

# Validate pendulum-joint damping (sweeps b over [1e-4 … 1.0])
python3 validate_cartpole.py --cases damping_sweep

# Use a different Teensy IP
python3 validate_cartpole.py --ip 192.168.1.100
```

Plots and results are saved to `results/`.

## Test Cases

| # | Name | What it checks |
|---|------|----------------|
| 1 | `equilibrium` | Roundoff-level errors at stable rest |
| 2 | `small_oscillation` | Small-angle oscillation |
| 3 | `large_swing` | Nonlinear regime (theta0 = 60 deg) |
| 4 | `high_initial_velocity` | Large initial angular velocity |
| 5 | `energy_conservation` | Bounded energy drift, exact momentum conservation |
| 6 | `constant_force` | Driven dynamics (f_x = 1 N) |
| 7 | `sinusoidal_drive` | Time-varying forcing |
| 8 | `near_upright` | Unstable equilibrium fallaway |
| 9 | `x_offset_sweep` | Translational invariance (exact) |
| 10 | `convergence` | First-order error scaling (log-log slope) | NOTE: this convergence test is currently incorrect because it's based on Euler and not RK4
| 11 | `damping_sweep` | Pendulum-joint damping vs. b-aware reference (sweeps `b`) |

## Pendulum-joint damping

`SIM_CONFIG` takes an optional 5th argument, the pendulum-joint viscous damping
`b` (omit it or pass `0` for the undamped model — backward compatible):

```
SIM_CONFIG <m_c> <m_p> <l> <g> [b]
```

This mirrors MuJoCo's joint `damping` on `shaft_rotate` in
`cartpole-target/orca/cartpendulum-orca.xml`: a generalized torque `-b·θ̇` on the
hinge DOF. In the coupled cart-pole it perturbs both accelerations and reduces to
`θ̈ -= b·θ̇/(m_p·l²)` in the heavy-cart (prescribed-acceleration) limit the haptic
loop uses — so the Step 3 port to `teensy_cartpole_haptic.ino` is the same physics.
`damping_sweep` sweeps `b ∈ {1e-4, 5e-4, 5e-3, 5e-2, 5e-1, 1.0}` (the production
range is `[0, 1]`, rounded to the nearest 1e-4) on a 60° free swing and checks each
against the reference solved with the same `b`; it also writes
`results/damping_sweep_overlay.png` showing the decay grow with `b`.

## Files

- `teensy_cartpole_sim/teensy_cartpole_sim.ino` — Teensy sketch
- `validate_cartpole.py` — Python driver (configure, run, compare, plot)
- `cartpole_reference.py` — Reference dynamics for `solve_ivp`
- `test_cases.py` — Test scenario definitions and tolerances
