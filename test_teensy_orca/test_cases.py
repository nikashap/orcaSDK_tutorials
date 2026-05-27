"""test_cases.py — Test scenario definitions for cart-pole physics validation."""

import math
from dataclasses import dataclass


@dataclass
class TestCase:
    name: str
    m_c: float = 1.0
    m_p: float = 0.1
    l: float = 0.5
    g: float = 9.81
    x0: float = 0.0
    theta0: float = 0.0
    xdot0: float = 0.0
    thetadot0: float = 0.0
    duration: float = 5.0
    dt: float = 0.001
    fx_mode: int = 0
    fx_param1: float = 0.0
    fx_param2: float = 0.0
    sample_stride: int = 1
    tol_pos: float = 5e-3
    tol_angle: float = 5e-3
    tol_vel: float = 5e-2
    tol_angvel: float = 5e-2


CASES = [
    TestCase("equilibrium",
             x0=0.25, duration=2.0,
             tol_pos=1e-12, tol_angle=1e-12, tol_vel=1e-12, tol_angvel=1e-12),

    TestCase("small_oscillation",
             theta0=0.05, duration=5.0),

    TestCase("large_swing",
             theta0=math.pi / 3, duration=10.0),

    TestCase("high_initial_velocity",
             thetadot0=2.0, duration=5.0),

    TestCase("energy_conservation",
             theta0=math.pi / 3, duration=30.0),

    TestCase("constant_force",
             fx_mode=1, fx_param1=1.0, duration=2.0),

    TestCase("sinusoidal_drive",
             fx_mode=2, fx_param1=2.0, fx_param2=0.5, duration=10.0),

    TestCase("near_upright",
             theta0=math.pi - 0.01, duration=2.0),
]

SWEEP_X0_VALUES = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
SWEEP_RESIDUAL_TOL = 1e-12

CONVERGENCE_DTS = [0.002, 0.001, 0.0005, 0.00025, 0.000125]
CONVERGENCE_SLOPE_RANGE = (0.9, 1.2)

ENERGY_MAX_DRIFT = 1e-2
ENERGY_MAX_MOMENTUM = 1e-10


def get_case(name):
    """Look up a test case by name."""
    for c in CASES:
        if c.name == name:
            return c
    raise KeyError(f"Unknown test case: {name}")
