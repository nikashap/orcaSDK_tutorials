# Cart-Pole Physics Validation Suite

Validates the Teensy 4.1 semi-implicit Euler integrator against a high-accuracy
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
# Run all 10 test cases
python3 validate_cartpole.py

# Run specific cases
python3 validate_cartpole.py --cases equilibrium small_oscillation convergence

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
| 10 | `convergence` | First-order error scaling (log-log slope) |

## Files

- `teensy_cartpole_sim/teensy_cartpole_sim.ino` — Teensy sketch
- `validate_cartpole.py` — Python driver (configure, run, compare, plot)
- `cartpole_reference.py` — Reference dynamics for `solve_ivp`
- `test_cases.py` — Test scenario definitions and tolerances
