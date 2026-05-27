"""cartpole_reference.py — High-accuracy cart-pole reference for validation."""

import math
import numpy as np
from scipy.integrate import solve_ivp


def make_fx_func(mode, param1=0.0, param2=0.0):
    """Build a force function f_x(t) matching the Teensy's forcing modes."""
    if mode == 0:
        return lambda t: 0.0
    elif mode == 1:
        return lambda t: param1
    elif mode == 2:
        amp, freq = param1, param2
        return lambda t: amp * math.sin(2.0 * math.pi * freq * t)
    elif mode == 3:
        level, t_step = param1, param2
        return lambda t: 0.0 if t < t_step else level
    elif mode == 4:
        level, t_end = param1, param2
        return lambda t: level if t < t_end else 0.0
    else:
        raise ValueError(f"Unknown force mode: {mode}")


def cartpole_rhs(t, state, m_c, m_p, l, g, fx_func):
    """ODE right-hand side for cart-pole dynamics (Underactuated Robotics §3.2)."""
    x, theta, xdot, thetadot = state
    fx = fx_func(t)
    s, c = math.sin(theta), math.cos(theta)
    D = m_c + m_p * s * s
    xddot = (fx + m_p * s * (l * thetadot**2 + g * c)) / D
    thetaddot = (-fx * c - m_p * l * thetadot**2 * c * s
                 - (m_c + m_p) * g * s) / (l * D)
    return [xdot, thetadot, xddot, thetaddot]


def solve_reference(state0, duration, m_c, m_p, l, g, fx_func):
    """High-accuracy reference solution via DOP853 with dense output."""
    sol = solve_ivp(
        cartpole_rhs, [0.0, duration], state0,
        method='DOP853', rtol=1e-12, atol=1e-14,
        dense_output=True, args=(m_c, m_p, l, g, fx_func))
    if not sol.success:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")
    return sol


def compute_energy(states, m_c, m_p, l, g):
    """Total mechanical energy. states shape: (N, 4) or (4,)."""
    s = np.atleast_2d(states)
    theta, xdot, thetadot = s[:, 1], s[:, 2], s[:, 3]
    return (0.5 * (m_c + m_p) * xdot**2
            + m_p * xdot * thetadot * l * np.cos(theta)
            + 0.5 * m_p * l**2 * thetadot**2
            - m_p * g * l * np.cos(theta))


def compute_momentum(states, m_c, m_p, l, g):
    """Horizontal momentum. states shape: (N, 4) or (4,)."""
    s = np.atleast_2d(states)
    theta, xdot, thetadot = s[:, 1], s[:, 2], s[:, 3]
    return (m_c + m_p) * xdot + m_p * l * thetadot * np.cos(theta)
