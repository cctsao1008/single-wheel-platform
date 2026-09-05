#!/usr/bin/env python3
"""Symbolically derive the canonical reduced balance model.

This tool is deliberately host-side.  It derives the mechanics from the same
coordinate contract used by `crates/plant-model` and prints the nonlinear mass,
Coriolis/centrifugal and gravity terms, the upright continuous state-space
model, and analytic controllability determinants.

No numeric platform parameters are invented here.
"""

from __future__ import annotations

import sympy as sp


def main() -> None:
    # Generalized coordinates q = [s, theta, phi, psi].
    s, theta, phi, psi = sp.symbols("s theta phi psi", real=True)
    ds, dtheta, dphi, dpsi = sp.symbols("s_dot theta_dot phi_dot psi_dot", real=True)

    # Canonical aggregate parameters.
    m_s, h, s2 = sp.symbols("M_s H S", positive=True, finite=True)
    i_bx, i_by, i_bz = sp.symbols("I_bx I_by I_bz", positive=True, finite=True)
    j_t, j_r, radius, gravity = sp.symbols(
        "J_t J_r r g", positive=True, finite=True
    )

    q = sp.Matrix([s, theta, phi, psi])
    dq = sp.Matrix([ds, dtheta, dphi, dpsi])

    j_phi = s2 + i_bx
    j_theta = (s2 + i_by) * sp.cos(phi) ** 2 + i_bz * sp.sin(phi) ** 2 + j_t

    # Mass matrix derived from the kinetic energy in docs/architecture/plant_model.md.
    mass = sp.Matrix(
        [
            [m_s, h * sp.cos(phi) * sp.cos(theta), -h * sp.sin(phi) * sp.sin(theta), 0],
            [h * sp.cos(phi) * sp.cos(theta), j_theta, 0, 0],
            [-h * sp.sin(phi) * sp.sin(theta), 0, j_phi + j_r, j_r],
            [0, 0, j_r, j_r],
        ]
    )

    # c_i = sum_jk Gamma_ijk qdot_j qdot_k.
    coriolis = []
    for i in range(4):
        term = 0
        for j in range(4):
            for k in range(4):
                gamma_ijk = sp.Rational(1, 2) * (
                    sp.diff(mass[i, j], q[k])
                    + sp.diff(mass[i, k], q[j])
                    - sp.diff(mass[j, k], q[i])
                )
                term += gamma_ijk * dq[j] * dq[k]
        coriolis.append(sp.trigsimp(sp.simplify(term)))
    coriolis = sp.Matrix(coriolis)

    potential = gravity * h * sp.cos(theta) * sp.cos(phi)
    gravity_vector = sp.Matrix([sp.diff(potential, qi) for qi in q])

    # Virtual-work input mapping.  psi is already relative to the body.
    input_map = sp.Matrix(
        [
            [1 / radius, 0],
            [-1, 0],
            [0, 0],
            [0, 1],
        ]
    )

    print("M(q) =")
    sp.print_latex(mass)
    print("\nc(q, qdot) =")
    sp.print_latex(coriolis)
    print("\ng(q) =")
    sp.print_latex(gravity_vector)
    print("\nB_q =")
    sp.print_latex(input_map)

    # Upright linearization.
    j_theta_0 = s2 + i_by + j_t
    delta_pitch = sp.factor(m_s * j_theta_0 - h**2)

    # Reduced state order:
    # [s, s_dot, theta, theta_dot, phi, phi_dot, psi_dot].
    a = sp.zeros(7, 7)
    b = sp.zeros(7, 2)

    a[0, 1] = 1
    a[1, 2] = -gravity * h**2 / delta_pitch
    a[2, 3] = 1
    a[3, 2] = m_s * gravity * h / delta_pitch

    a[4, 5] = 1
    a[5, 4] = gravity * h / j_phi
    a[6, 4] = -gravity * h / j_phi

    b[1, 0] = (j_theta_0 / radius + h) / delta_pitch
    b[3, 0] = -(h / radius + m_s) / delta_pitch
    b[5, 1] = -1 / j_phi
    b[6, 1] = 1 / j_r + 1 / j_phi

    print("\nA_c (reduced upright state) =")
    sp.print_latex(a)
    print("\nB_c (reduced upright state) =")
    sp.print_latex(b)

    # Analytic controllability certificates for the two first-order-decoupled
    # upright subsystems.  These are not assumptions of global decoupling.
    a_pitch = a.extract([0, 1, 2, 3], [0, 1, 2, 3])
    b_pitch = b.extract([0, 1, 2, 3], [0])
    c_pitch = sp.Matrix.hstack(
        b_pitch,
        a_pitch * b_pitch,
        a_pitch**2 * b_pitch,
        a_pitch**3 * b_pitch,
    )

    a_roll = a.extract([4, 5, 6], [4, 5, 6])
    b_roll = b.extract([4, 5, 6], [1])
    c_roll = sp.Matrix.hstack(b_roll, a_roll * b_roll, a_roll**2 * b_roll)

    print("\ndet(C_pitch) =")
    sp.print_latex(sp.factor(c_pitch.det()))
    print("\ndet(C_roll) =")
    sp.print_latex(sp.factor(c_roll.det()))

    print("\nInterpretation:")
    print("  - The nonlinear model is coupled away from upright.")
    print("  - The stationary-upright first-order model decouples into")
    print("    translation/pitch and roll/reaction-wheel blocks.")
    print("  - For positive physical parameters and nonsingular inertia matrices,")
    print("    both upright blocks are controllable.")


if __name__ == "__main__":
    main()
