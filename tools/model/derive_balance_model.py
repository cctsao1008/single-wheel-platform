#!/usr/bin/env python3
"""Symbolically derive the canonical upright/straight-line balance model.

No numeric plant parameters are embedded here. The script derives M(q),
c(q,q_dot), g(q), the physical input map, upright linearization structure,
open-loop unstable modal rates, and generic controllability conditions.

Coordinates:
    q = [s, theta, phi, psi]^T

Inputs:
    u = [tau_drive, tau_reaction]^T

`psi` is the reaction-wheel angle relative to the robot body.
"""

from __future__ import annotations

import sympy as sp


def coriolis_vector(mass: sp.Matrix, q: sp.Matrix, q_dot: sp.Matrix) -> sp.Matrix:
    """Construct c(q,q_dot) from Christoffel symbols of the first kind."""
    out = []
    n = len(q)
    for i in range(n):
        value = 0
        for j in range(n):
            for k in range(n):
                gamma = sp.Rational(1, 2) * (
                    sp.diff(mass[i, j], q[k])
                    + sp.diff(mass[i, k], q[j])
                    - sp.diff(mass[j, k], q[i])
                )
                value += gamma * q_dot[j] * q_dot[k]
        out.append(sp.trigsimp(value))
    return sp.Matrix(out)


def main() -> None:
    s, theta, phi, psi = sp.symbols("s theta phi psi", real=True)
    s_dot, theta_dot, phi_dot, psi_dot = sp.symbols(
        "s_dot theta_dot phi_dot psi_dot", real=True
    )

    m_s, h_first, s_second = sp.symbols("M_s H S", positive=True)
    i_bx, i_by, i_bz = sp.symbols("I_bx I_by I_bz", positive=True)
    j_r, j_t = sp.symbols("J_r J_t", positive=True)
    gravity, radius = sp.symbols("g r", positive=True)

    q = sp.Matrix([s, theta, phi, psi])
    q_dot = sp.Matrix([s_dot, theta_dot, phi_dot, psi_dot])

    kinetic = (
        sp.Rational(1, 2) * m_s * s_dot**2
        + h_first * s_dot * theta_dot * sp.cos(phi) * sp.cos(theta)
        - h_first * s_dot * phi_dot * sp.sin(phi) * sp.sin(theta)
        + sp.Rational(1, 2) * s_second * phi_dot**2
        + sp.Rational(1, 2) * s_second * theta_dot**2 * sp.cos(phi) ** 2
        + sp.Rational(1, 2) * i_bx * phi_dot**2
        + sp.Rational(1, 2) * i_by * theta_dot**2 * sp.cos(phi) ** 2
        + sp.Rational(1, 2) * i_bz * theta_dot**2 * sp.sin(phi) ** 2
        + sp.Rational(1, 2) * j_t * theta_dot**2
        + sp.Rational(1, 2) * j_r * (phi_dot + psi_dot) ** 2
    )

    potential = gravity * h_first * sp.cos(theta) * sp.cos(phi)

    mass = sp.hessian(kinetic, q_dot)
    coriolis = coriolis_vector(mass, q, q_dot)
    gravity_vector = sp.Matrix([sp.diff(potential, item) for item in q])
    input_map = sp.Matrix(
        [
            [1 / radius, 0],
            [-1, 0],
            [0, 0],
            [0, 1],
        ]
    )

    # Mechanical sanity check: a kinetic-energy Hessian must be symmetric.
    assert mass == mass.T

    j_theta = s_second + i_by + j_t
    j_phi = s_second + i_bx
    delta_pitch = m_s * j_theta - h_first**2

    # Upright linearization.
    mass_upright = mass.subs({theta: 0, phi: 0})
    gravity_stiffness = gravity_vector.jacobian(q).subs({theta: 0, phi: 0})

    # Compact pitch subsystem: [s, s_dot, theta, theta_dot].
    pitch_a = sp.Matrix(
        [
            [0, 1, 0, 0],
            [0, 0, -h_first**2 * gravity / delta_pitch, 0],
            [0, 0, 0, 1],
            [0, 0, h_first * m_s * gravity / delta_pitch, 0],
        ]
    )
    pitch_b = sp.Matrix(
        [
            0,
            (j_theta / radius + h_first) / delta_pitch,
            0,
            -(h_first / radius + m_s) / delta_pitch,
        ]
    )
    pitch_ctrb = sp.Matrix.hstack(
        pitch_b,
        pitch_a * pitch_b,
        pitch_a**2 * pitch_b,
        pitch_a**3 * pitch_b,
    )
    det_pitch = sp.factor(pitch_ctrb.det(method="domain-ge"))

    # Compact roll/momentum subsystem: [phi, phi_dot, psi_dot].
    roll_a = sp.Matrix(
        [
            [0, 1, 0],
            [h_first * gravity / j_phi, 0, 0],
            [-h_first * gravity / j_phi, 0, 0],
        ]
    )
    roll_b = sp.Matrix(
        [
            0,
            -1 / j_phi,
            (j_phi + j_r) / (j_r * j_phi),
        ]
    )
    roll_ctrb = sp.Matrix.hstack(roll_b, roll_a * roll_b, roll_a**2 * roll_b)
    det_roll = sp.factor(roll_ctrb.det(method="domain-ge"))

    omega_pitch_sq = sp.factor(h_first * m_s * gravity / delta_pitch)
    omega_roll_sq = sp.factor(h_first * gravity / j_phi)

    print("M(q) =")
    sp.print_latex(mass)
    print("\nc(q, q_dot) =")
    sp.print_latex(coriolis)
    print("\ng(q) =")
    sp.print_latex(gravity_vector)
    print("\nB =")
    sp.print_latex(input_map)
    print("\nM(0) =")
    sp.print_latex(mass_upright)
    print("\nK_g(0) =")
    sp.print_latex(gravity_stiffness)
    print("\ndet(C_pitch) =")
    sp.print_latex(det_pitch)
    print("\ndet(C_roll) =")
    sp.print_latex(det_roll)
    print("\nopen-loop unstable modal rates squared:")
    print("pitch:")
    sp.print_latex(omega_pitch_sq)
    print("roll:")
    sp.print_latex(omega_roll_sq)


if __name__ == "__main__":
    main()
