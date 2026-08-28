"""
PATCHED Trajectory NMPC for REMUS 100  —  offset-free, anti-overshoot variant
=============================================================================

This is a drop-in replacement for ``controllers.nmpc_remus.NMPC_REMUS100`` that
targets the two transient failures identified in the Scenario-6 analysis:

  (1) the ~58 deg overshoot during the sharp heading reversal (t~520 s), and
  (2) the ~31 deg heading drift during the pure depth change (t~280 s).

It exposes the EXACT same public interface as the original controller
(``compute``, ``set_reference_provider``, ``set_current_estimate``, ``reset``,
``name``, ``solve_times``, ``n_rpm``, ``dt_mpc``), so anywhere the framework
builds an NMPC you can swap this class in unchanged.

What is different from the original, and why
--------------------------------------------
A. OFFSET-FREE YAW DISTURBANCE OBSERVER  (fixes the depth-coupling drift).
   The reduced predictor in ``casadi_model.py`` sets the pitch->yaw coupling and
   the cross-flow yaw moment to zero on purpose (the old heuristic was badly
   scaled and kicked the rudder).  Consequently, during an ascent the real
   Fossen hull produces a yaw moment the predictor models as exactly 0, so the
   controller only reacts *after* the heading has already drifted.

   The fix is the textbook offset-free MPC construction: augment the model with
   an estimated yaw angular-acceleration disturbance ``d_r`` [rad/s^2], estimate
   it online from the one-step yaw-rate prediction error with a simple
   disturbance observer, and add it into the predicted yaw dynamics
   (held constant over the horizon).  The optimiser then *plans* rudder to
   cancel the moment instead of chasing it.  Because the observer lumps ALL
   unmodelled yaw moment into ``d_r``, it also removes the small persistent
   current-induced heading bias.  No physical coupling coefficient has to be
   identified — the residual is estimated, not modelled.

B. ANTI-OVERSHOOT RE-TUNING  (fixes the reversal overshoot).
   - Longer default horizon: N = 30 (6 s at dt_mpc = 0.2 s) instead of 20 (4 s).
     A 75 deg+ reversal at <=18 deg/s takes most of a 4 s window, so the old
     horizon could not "see" the settle and planned the braking too late.
   - Heavier penalty on residual yaw rate, especially terminal:
     Q[r]: 55 -> 100,  P[r]: 95 -> 300.  Arriving at the target still rotating
     is the signature of overshoot; this discourages it directly.
   - More rudder braking authority: the per-step rudder slew limit is relaxed
     4 deg -> 6 deg, and the rudder move-suppression Rd is lowered 520 -> 320 so
     the rudder can reverse fast enough to brake the turn.  (Rprev, which keeps
     the *first* applied move smooth, is left high, so this does not reintroduce
     chatter on holds.)

C. OPTIONAL calibrated coupling feed-forward (OFF by default).
   ``k_couple`` injects a physically-motivated pitch-rate * surge yaw term as a
   feed-forward.  It is 0 by default because, like the term the original
   removed, a wrong coefficient causes rudder kicks.  If you want to use it,
   identify ``k_couple`` by matching the full Fossen yaw response during a pure
   ascent on data you already have, then set it small.  The observer in (A) is
   the safe, self-tuning alternative and is enabled by default.

Internal state order:   x = [z, theta, psi, u_surge, q, r]
Internal control order: U = [delta_s, delta_r, n_rpm]
Simulator command order: [delta_r, delta_s, n_rpm]
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import numpy as np


def _build_augmented_dynamics(vehicle, *, k_couple: float = 0.0):
    """Reduced REMUS dynamics with an additive yaw-disturbance input ``d_r``.

    Mirrors ``adapters.casadi_model.build_casadi_dynamics`` exactly, then adds:
      * ``d_r`` (a CasADi parameter): estimated unmodelled yaw angular
        acceleration [rad/s^2], entering r_dot directly and held constant over
        the horizon (the offset-free disturbance);
      * an OPTIONAL feed-forward coupling term scaled by ``k_couple`` (0 = off).

    Returns f(state, control, V_c, beta_c, d_r) -> state_dot.
    """
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

    D_prop = 0.14;  KT_0 = 0.4566;  t_prop = 0.1
    S = vehicle.S;  CD_0 = vehicle.CD_0

    x = ca.SX.sym('x', 6)
    u = ca.SX.sym('u', 3)
    V_c = ca.SX.sym('V_c')
    beta_c = ca.SX.sym('beta_c')
    d_r = ca.SX.sym('d_r')               # <-- offset-free yaw disturbance

    z = x[0]; theta = x[1]; psi = x[2]
    u_s = x[3]; q = x[4]; r = x[5]
    delta_s = u[0]; delta_r = u[1]; n_rpm = u[2]

    n_rps = n_rpm / 60.0
    u_c = V_c * ca.cos(beta_c - psi)
    u_r = u_s - u_c
    U_r = ca.fmax(ca.fabs(u_r), 0.1)

    z_dot = -u_s * ca.sin(theta)
    theta_dot = q
    psi_dot = r

    X_prop = (1 - t_prop) * rho * D_prop**4 * KT_0 * ca.fabs(n_rps) * n_rps
    X_drag = d_surge * u_r + 0.5 * rho * S * CD_0 * u_r * ca.fabs(u_r)
    u_dot = (X_prop - X_drag) / m11

    M_stern = M_s_coeff * U_r**2 * delta_s
    q_dot = (M_stern - d_pitch * q - g_theta * ca.sin(theta)) / I55

    N_rudder = N_r_coeff * U_r**2 * delta_r
    # Optional, OFF by default (k_couple=0). A pitch-rate * surge feed-forward
    # approximating the dominant ascent-induced yaw moment.  Identify k_couple
    # from full-Fossen ascent data before enabling.
    N_coupling_ff = k_couple * q * u_s
    # Offset-free disturbance enters as an acceleration (rad/s^2), held constant
    # across the horizon; the observer in the controller updates it each step.
    r_dot = (N_rudder + N_coupling_ff - d_yaw * r) / I66 + d_r

    x_dot = ca.vertcat(z_dot, theta_dot, psi_dot, u_dot, q_dot, r_dot)
    return ca.Function('remus100_reduced_offsetfree',
                       [x, u, V_c, beta_c, d_r], [x_dot],
                       ['state', 'control', 'V_c', 'beta_c', 'd_r'],
                       ['state_dot'])


class NMPC_REMUS100_Patched:
    """Offset-free, anti-overshoot trajectory NMPC for REMUS 100 (drop-in)."""

    def __init__(self, vehicle, N: int = 30, dt_mpc: float = 0.2, n_rpm: float = 1525,
                 *,
                 use_disturbance_observer: bool = True,
                 obs_gain: float = 0.7,
                 obs_lp: float = 0.65,
                 obs_leak: float = 0.005,
                 d_r_max_deg: float = 30.0,
                 k_couple: float = 0.0):
        import casadi as ca

        self.N = int(N)
        self.dt_mpc = float(dt_mpc)
        self.n_x = 6
        self.n_u = 3
        self.n_rpm = float(n_rpm)
        self.name = f"NMPC offset-free (N={self.N})"
        self.solve_times: list[float] = []

        # --- disturbance observer config ---
        self.use_obs = bool(use_disturbance_observer)
        self.obs_gain = float(obs_gain)          # innovation gain L in (0, 1]
        self.obs_lp = float(obs_lp)              # EMA smoothing of the estimate
        self.obs_leak = float(obs_leak)          # per-step fractional decay toward 0
        self.d_r_max = math.radians(float(d_r_max_deg))   # clip [rad/s^2]
        self.d_r_est = 0.0                       # current yaw-disturbance estimate

        self.f_dyn = _build_augmented_dynamics(vehicle, k_couple=k_couple)

        self.ds_max = float(vehicle.deltaMax_s)
        self.dr_max = float(vehicle.deltaMax_r)
        self.n_min = max(0.0, float(getattr(vehicle, "nMin", 0.0)))
        self.n_max = max(float(getattr(vehicle, "nMax", self.n_rpm)), self.n_rpm)

        self.u_min = np.array([-self.ds_max, -self.dr_max, self.n_min], dtype=float)
        self.u_max = np.array([ self.ds_max,  self.dr_max, self.n_max], dtype=float)

        # (B) more rudder braking authority: 4 deg -> 6 deg per step on the rudder.
        self.du_max = np.array([math.radians(4.0), math.radians(6.0), 100.0], dtype=float)

        self.theta_max = math.radians(25.0)
        self.q_max = math.radians(18.0)
        self.r_max = math.radians(18.0)

        # (B) heavier yaw-rate penalty, especially terminal, to kill overshoot.
        #     order: [z, theta, psi, u_surge, q, r]
        Q_diag = np.array([120.0, 90.0, 260.0, 12.0, 28.0, 100.0], dtype=float)   # r: 55 -> 100
        P_diag = np.array([350.0, 150.0, 520.0, 20.0, 45.0, 300.0], dtype=float)  # r: 95 -> 300

        R_diag = np.array([18.0, 24.0, 1.0e-6], dtype=float)
        # (B) lower rudder move-suppression so it can reverse to brake (520 -> 320),
        #     but keep Rprev (first-move smoothness) high to avoid hold chatter.
        Rd_diag = np.array([380.0, 320.0, 5.0e-5], dtype=float)
        Rprev_diag = np.array([650.0, 820.0, 8.0e-5], dtype=float)

        opti = ca.Opti()
        X = opti.variable(self.n_x, self.N + 1)
        U = opti.variable(self.n_u, self.N)

        x0_p = opti.parameter(self.n_x)
        X_ref_p = opti.parameter(self.n_x, self.N + 1)
        U_ref_p = opti.parameter(self.n_u, self.N)
        u_prev_p = opti.parameter(self.n_u)
        Vc_p = opti.parameter()
        bc_p = opti.parameter()
        dr_p = opti.parameter()              # <-- yaw-disturbance estimate

        Q = ca.DM(np.diag(Q_diag))
        P = ca.DM(np.diag(P_diag))
        R = ca.DM(np.diag(R_diag))
        Rd = ca.DM(np.diag(Rd_diag))
        Rprev = ca.DM(np.diag(Rprev_diag))

        def wrapped_state_error(x_col, xr_col):
            e_z = x_col[0] - xr_col[0]
            e_theta = ca.atan2(ca.sin(x_col[1] - xr_col[1]), ca.cos(x_col[1] - xr_col[1]))
            e_psi = ca.atan2(ca.sin(x_col[2] - xr_col[2]), ca.cos(x_col[2] - xr_col[2]))
            e_u = x_col[3] - xr_col[3]
            e_q = x_col[4] - xr_col[4]
            e_r = x_col[5] - xr_col[5]
            return ca.vertcat(e_z, e_theta, e_psi, e_u, e_q, e_r)

        def rk4(x_, u_):
            dt = self.dt_mpc
            k1 = self.f_dyn(x_, u_, Vc_p, bc_p, dr_p)
            k2 = self.f_dyn(x_ + dt / 2 * k1, u_, Vc_p, bc_p, dr_p)
            k3 = self.f_dyn(x_ + dt / 2 * k2, u_, Vc_p, bc_p, dr_p)
            k4 = self.f_dyn(x_ + dt * k3, u_, Vc_p, bc_p, dr_p)
            return x_ + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        J = 0
        for k in range(self.N):
            x_err = wrapped_state_error(X[:, k], X_ref_p[:, k])
            u_err = U[:, k] - U_ref_p[:, k]
            J += x_err.T @ Q @ x_err + u_err.T @ R @ u_err
            prev_u = u_prev_p if k == 0 else U[:, k - 1]
            du = U[:, k] - prev_u
            J += du.T @ (Rprev if k == 0 else Rd) @ du

        x_err_N = wrapped_state_error(X[:, self.N], X_ref_p[:, self.N])
        J += x_err_N.T @ P @ x_err_N
        opti.minimize(J)

        opti.subject_to(X[:, 0] == x0_p)
        for k in range(self.N):
            opti.subject_to(X[:, k + 1] == rk4(X[:, k], U[:, k]))
            opti.subject_to(opti.bounded(-self.ds_max, U[0, k], self.ds_max))
            opti.subject_to(opti.bounded(-self.dr_max, U[1, k], self.dr_max))
            opti.subject_to(opti.bounded(self.n_min, U[2, k], self.n_max))
            prev_u = u_prev_p if k == 0 else U[:, k - 1]
            opti.subject_to(opti.bounded(-self.du_max[0], U[0, k] - prev_u[0], self.du_max[0]))
            opti.subject_to(opti.bounded(-self.du_max[1], U[1, k] - prev_u[1], self.du_max[1]))
            opti.subject_to(opti.bounded(-self.du_max[2], U[2, k] - prev_u[2], self.du_max[2]))

        for k in range(self.N + 1):
            opti.subject_to(opti.bounded(-self.theta_max, X[1, k], self.theta_max))
            opti.subject_to(opti.bounded(-self.q_max, X[4, k], self.q_max))
            opti.subject_to(opti.bounded(-self.r_max, X[5, k], self.r_max))

        opts = {
            "ipopt.print_level": 0,
            "ipopt.max_iter": 130,
            "ipopt.tol": 1e-3,
            "ipopt.acceptable_tol": 3e-3,
            "ipopt.acceptable_iter": 4,
            "ipopt.mu_strategy": "adaptive",
            "ipopt.warm_start_init_point": "yes",
            "ipopt.warm_start_bound_push": 1e-6,
            "ipopt.warm_start_mult_bound_push": 1e-6,
            "print_time": False,
        }
        opti.solver("ipopt", opts)

        self.opti = opti
        self.X = X
        self.U = U
        self.x0_p = x0_p
        self.X_ref_p = X_ref_p
        self.U_ref_p = U_ref_p
        self.u_prev_p = u_prev_p
        self.Vc_p = Vc_p
        self.bc_p = bc_p
        self.dr_p = dr_p

        # numeric RK4 (numpy) for the observer's one-step prediction
        self._f_np = self.f_dyn

        self._prev_X: Optional[np.ndarray] = None
        self._prev_U: Optional[np.ndarray] = None
        self.prev_u = np.array([0.0, 0.0, self.n_rpm], dtype=float)
        self.V_c_est = 0.0
        self.beta_c_est = 0.0
        self.reference_provider: Optional[Callable[[float], tuple[np.ndarray, np.ndarray]]] = None

        # observer memory: last measured state and the control actually applied
        self._x_meas_prev: Optional[np.ndarray] = None
        self._u_applied_prev: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    #  numeric RK4 used only by the disturbance observer
    # ------------------------------------------------------------------ #
    def _rk4_np(self, x, u, d_r):
        dt = self.dt_mpc
        f = lambda xx: np.asarray(self._f_np(xx, u, self.V_c_est, self.beta_c_est, d_r)).reshape(self.n_x)
        k1 = f(x)
        k2 = f(x + dt / 2 * k1)
        k3 = f(x + dt / 2 * k2)
        k4 = f(x + dt * k3)
        return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def _update_disturbance(self, x_now: np.ndarray) -> None:
        """One-step yaw-disturbance observer.

        Predicts r one control step ahead from the previous measured state and
        the control actually applied, using the current estimate, then corrects
        the estimate by the yaw-rate innovation.  This is an integral action on
        the yaw-rate prediction error -> offset-free for constant yaw moments.
        """
        if not self.use_obs:
            return
        if self._x_meas_prev is None or self._u_applied_prev is None:
            return
        try:
            x_pred = self._rk4_np(self._x_meas_prev, self._u_applied_prev, self.d_r_est)
            nu_r = float(x_now[5] - x_pred[5])          # yaw-rate innovation [rad/s]
            d_r_raw = self.d_r_est + self.obs_gain * nu_r / self.dt_mpc
            # EMA smoothing then leak: decays stale estimates toward 0 (τ ≈ dt/obs_leak)
            # so a disturbance estimate from one dive doesn't persist into later turns.
            d_r_filt = (1.0 - self.obs_lp) * self.d_r_est + self.obs_lp * d_r_raw
            self.d_r_est = float(np.clip(d_r_filt * (1.0 - self.obs_leak), -self.d_r_max, self.d_r_max))
        except Exception:
            pass  # never let the observer break the control loop

    # ------------------------------------------------------------------ #
    #  public interface (identical to the original controller)
    # ------------------------------------------------------------------ #
    @staticmethod
    def _ssa(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def set_reference_provider(self, reference_provider) -> None:
        self.reference_provider = reference_provider

    def set_current_estimate(self, V_c: float, beta_c_rad: float) -> None:
        self.V_c_est = float(V_c)
        self.beta_c_est = float(beta_c_rad)

    def _single_reference_to_state(self, eta_d, nu_d):
        eta_d = np.asarray(eta_d, dtype=float)
        nu_d = np.asarray(nu_d, dtype=float)
        z_ref = float(eta_d[2])
        theta_ref = float(eta_d[4]) if eta_d.size > 4 else 0.0
        psi_ref = float(eta_d[5])
        u_ref = float(nu_d[0]) if nu_d.size > 0 and np.isfinite(nu_d[0]) else 2.5
        q_ref = float(nu_d[4]) if nu_d.size > 4 and np.isfinite(nu_d[4]) else 0.0
        r_ref = float(nu_d[5]) if nu_d.size > 5 and np.isfinite(nu_d[5]) else 0.0
        theta_ref = float(np.clip(theta_ref, -self.theta_max, self.theta_max))
        q_ref = float(np.clip(q_ref, -self.q_max, self.q_max))
        r_ref = float(np.clip(r_ref, -self.r_max, self.r_max))
        return np.array([z_ref, theta_ref, psi_ref, u_ref, q_ref, r_ref], dtype=float)

    def _build_reference_horizon(self, eta_d, nu_d, t: float):
        X_ref = np.zeros((self.n_x, self.N + 1), dtype=float)
        U_ref = np.zeros((self.n_u, self.N), dtype=float)
        provider = self.reference_provider
        for k in range(self.N + 1):
            if provider is not None:
                eta_k, nu_k = provider(float(t + k * self.dt_mpc))
            else:
                eta_k, nu_k = eta_d, nu_d
            X_ref[:, k] = self._single_reference_to_state(eta_k, nu_k)
        X_ref[2, :] = np.unwrap(X_ref[2, :])
        for k in range(self.N):
            U_ref[:, k] = np.array([0.0, 0.0, self.n_rpm], dtype=float)
        return X_ref, U_ref

    def _apply_output_safety_limits(self, u_opt):
        u_opt = np.asarray(u_opt, dtype=float).reshape(self.n_u)
        u_opt = np.clip(u_opt, self.u_min, self.u_max)
        du = np.clip(u_opt - self.prev_u, -self.du_max, self.du_max)
        return np.clip(self.prev_u + du, self.u_min, self.u_max)

    def _set_cold_start(self, x0, X_ref, U_ref):
        X_init = X_ref.copy()
        X_init[:, 0] = x0
        for k in range(1, self.N + 1):
            a = k / max(self.N, 1)
            X_init[0, k] = (1.0 - a) * x0[0] + a * X_ref[0, k]
            X_init[1, k] = (1.0 - a) * x0[1] + a * X_ref[1, k]
            X_init[2, k] = x0[2] + a * self._ssa(X_ref[2, k] - x0[2])
            X_init[3, k] = (1.0 - a) * x0[3] + a * X_ref[3, k]
            X_init[4, k] = (1.0 - a) * x0[4] + a * X_ref[4, k]
            X_init[5, k] = (1.0 - a) * x0[5] + a * X_ref[5, k]
        U_init = U_ref.copy()
        if U_init.shape[1] > 0:
            U_init[:, 0] = self.prev_u
        self.opti.set_initial(self.X, X_init)
        self.opti.set_initial(self.U, U_init)

    def compute(self, eta, nu, eta_d, nu_d, t):
        t_start = time.perf_counter()

        x0 = np.array([eta[2], eta[4], eta[5], nu[0], nu[4], nu[5]], dtype=float)

        # (A) update the yaw-disturbance estimate from the realised motion BEFORE
        #     planning, so this solve already compensates for the moment.
        self._update_disturbance(x0)

        X_ref, U_ref = self._build_reference_horizon(eta_d, nu_d, float(t))
        branch_shift = 2.0 * math.pi * round((x0[2] - X_ref[2, 0]) / (2.0 * math.pi))
        X_ref[2, :] += branch_shift

        self.opti.set_value(self.x0_p, x0)
        self.opti.set_value(self.X_ref_p, X_ref)
        self.opti.set_value(self.U_ref_p, U_ref)
        self.opti.set_value(self.u_prev_p, self.prev_u)
        self.opti.set_value(self.Vc_p, self.V_c_est)
        self.opti.set_value(self.bc_p, self.beta_c_est)
        self.opti.set_value(self.dr_p, self.d_r_est)

        if self._prev_X is not None and self._prev_U is not None:
            try:
                X_init = np.hstack([self._prev_X[:, 1:], self._prev_X[:, -1:]])
                U_init = np.hstack([self._prev_U[:, 1:], self._prev_U[:, -1:]])
                X_init[:, 0] = x0
                U_init[:, 0] = self.prev_u
                # Always override psi and r from a direction-correct cold-start
                # interpolation.  The psi channel has two equally valid local minima
                # (short path vs long path); the shifted warm start can point toward
                # the wrong one after a near-infeasible solve.  Initialising psi via
                # SSA guarantees the correct turn direction without touching any other
                # state, so IPOPT still warm-starts efficiently.
                psi_delta = self._ssa(X_ref[2, self.N] - x0[2])
                for k in range(self.N + 1):
                    X_init[2, k] = x0[2] + (k / self.N) * psi_delta
                X_init[5, :] = X_ref[5, :]  # r from reference
                X_init[5, 0] = x0[5]
                self.opti.set_initial(self.X, X_init)
                self.opti.set_initial(self.U, U_init)
            except Exception:
                self._set_cold_start(x0, X_ref, U_ref)
        else:
            self._set_cold_start(x0, X_ref, U_ref)

        try:
            sol = self.opti.solve()
            X_opt = sol.value(self.X)
            U_opt = sol.value(self.U)
            u_opt = U_opt[:, 0]
            self._prev_X = X_opt
            self._prev_U = U_opt
        except Exception:
            u_opt = self.prev_u.copy()
            # Clear warm start so the next solve uses a direction-correct cold start
            # instead of the stale (now potentially wrong-direction) solution.
            self._prev_X = None
            self._prev_U = None

        u_opt = self._apply_output_safety_limits(u_opt)
        if self._prev_U is not None:
            try:
                self._prev_U[:, 0] = u_opt
            except Exception:
                pass

        self.prev_u = u_opt.copy()
        # remember for the next observer step (internal order [ds, dr, n])
        self._x_meas_prev = x0.copy()
        self._u_applied_prev = u_opt.copy()
        self.solve_times.append(time.perf_counter() - t_start)

        # Fossen expects [delta_r, delta_s, n_rpm].
        return np.array([u_opt[1], u_opt[0], u_opt[2]], dtype=float)

    def reset(self) -> None:
        self._prev_X = None
        self._prev_U = None
        self.prev_u = np.array([0.0, 0.0, self.n_rpm], dtype=float)
        self.solve_times = []
        self.d_r_est = 0.0
        self._x_meas_prev = None
        self._u_applied_prev = None
