"""
Reduced-Order CasADi Model for REMUS 100 NMPC
=================================================
State:  x = [z, theta, psi, u_surge, q, r]  (6 states)
Control: u = [delta_s, delta_r, n_rpm]       (3 inputs)

Design philosophy: this is still a reduced-order prediction model, but it is
used by a full trajectory NMPC optimiser.  Disturbance/coupling terms are kept
conservative: over-modelled coupling caused artificial rudder kicks in the
previous version.

Current effects captured:
  - Relative speed for drag and fin effectiveness (u_r = u_s - u_c)
  - Surge drag/thrust dependence on propeller RPM
"""

import numpy as np
import math


def build_casadi_dynamics(vehicle):
    import casadi as ca

    rho = vehicle.rho
    m11 = vehicle.M[0][0]
    I55 = vehicle.M[4][4]
    I66 = vehicle.M[5][5]

    d_surge = m11 / vehicle.T_surge
    d_pitch = I55 * 2 * vehicle.zeta_pitch * vehicle.w_pitch
    d_yaw = I66 / vehicle.T_yaw

    g_theta = vehicle.W * (vehicle.r_bg[2] - vehicle.r_bb[2])

    A_s = vehicle.A_s;  CL_s = vehicle.CL_delta_s;  x_s = vehicle.x_s
    A_r = vehicle.A_r;  CL_r = vehicle.CL_delta_r;  x_r = vehicle.x_r

    M_s_coeff = -(-x_s) * 0.5 * rho * A_s * CL_s
    N_r_coeff = x_r * (-0.5 * rho * A_r * CL_r)

    # Propeller: same Wageningen linearisation as the Fossen truth model.
    # KT falls with advance number Ja = Va/(n*D); omitting this term makes the
    # predictor believe it has ~2.5x the real thrust at cruise speed, so the
    # NMPC under-commands RPM and the vehicle runs visibly slower than 2.5 m/s.
    D_prop = 0.14;  KT_0 = 0.4566;  t_prop = 0.1
    KT_max = 0.1798;  Ja_max = 0.6632;  w_wake = 0.944   # Va = 0.944 * U
    S = vehicle.S;  CD_0 = vehicle.CD_0

    # CasADi symbols
    x = ca.SX.sym('x', 6)
    u = ca.SX.sym('u', 3)
    V_c = ca.SX.sym('V_c')
    beta_c = ca.SX.sym('beta_c')

    z = x[0]; theta = x[1]; psi = x[2]
    u_s = x[3]; q = x[4]; r = x[5]
    delta_s = u[0]; delta_r = u[1]; n_rpm = u[2]

    n_rps = n_rpm / 60.0

    # Current decomposition into body frame
    u_c = V_c * ca.cos(beta_c - psi)

    # Relative surge speed (current affects drag and fin effectiveness)
    u_r = u_s - u_c
    U_r = ca.fmax(ca.fabs(u_r), 0.1)

    # ─── DEPTH ───────────────────────────────────────────────────
    z_dot = -u_s * ca.sin(theta)

    # ─── KINEMATICS ──────────────────────────────────────────────
    theta_dot = q
    psi_dot = r

    # ─── SURGE ───────────────────────────────────────────────────
    # Thrust with the advance-ratio loss (matches remus100.dynamics for n >= 0):
    #   X_prop = rho D^4 [ KT_0 |n|n + (KT_max-KT_0)/Ja_max * (Va/D) |n| ]
    Va = w_wake * ca.fabs(u_s)
    X_prop = (1 - t_prop) * rho * D_prop**4 * (
        KT_0 * ca.fabs(n_rps) * n_rps
        + (KT_max - KT_0) / Ja_max * (Va / D_prop) * ca.fabs(n_rps))
    # Linear damping vanishes at speed (exp(-3 U_r), as in the truth model);
    # quadratic hull drag dominates at cruise.
    X_drag = d_surge * ca.exp(-3.0 * U_r) * u_r \
        + 0.5 * rho * S * CD_0 * u_r * ca.fabs(u_r)
    u_dot = (X_prop - X_drag) / m11

    # ─── PITCH ───────────────────────────────────────────────────
    M_stern = M_s_coeff * U_r**2 * delta_s
    q_dot = (M_stern - d_pitch * q - g_theta * ca.sin(theta)) / I55

    # ─── YAW ─────────────────────────────────────────────────────
    # Rudder moment (uses relative speed for fin effectiveness)
    N_rudder = N_r_coeff * U_r**2 * delta_r

    # NO cross-flow yaw moment from current — this was 262% of max
    # rudder authority and caused massive overcompensation. The real
    # Fossen crossFlowDrag uses relative sway velocity (much smaller)
    # and 20-strip integration. The residual is best handled by the
    # receding-horizon feedback, not the predictor.

    # Pitch-yaw coupling deliberately disabled in the reduced predictor.
    # The full Fossen model contains nonlinear 6-DOF coupling terms, but this
    # reduced z/theta/psi model does not include sway/heave states with enough
    # fidelity to reproduce that coupling correctly.  The previous heuristic
    # term created artificial rudder kicks during pure depth manoeuvres.
    N_coupling = 0.0

    r_dot = (N_rudder + N_coupling - d_yaw * r) / I66

    x_dot = ca.vertcat(z_dot, theta_dot, psi_dot, u_dot, q_dot, r_dot)

    f = ca.Function('remus100_reduced',
                     [x, u, V_c, beta_c], [x_dot],
                     ['state', 'control', 'V_c', 'beta_c'], ['state_dot'])

    return f