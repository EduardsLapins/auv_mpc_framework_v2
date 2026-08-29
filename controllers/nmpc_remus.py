"""
Trajectory NMPC for REMUS 100 depth + heading missions
======================================================

This controller is a proper receding-horizon trajectory NMPC for the reduced
REMUS depth/heading model used in this project.

Main differences from the old setpoint-style controller
------------------------------------------------------
1. The optimiser receives a whole reference trajectory X_ref[:, 0:N], not only
   the current eta_d sample.  It can therefore see waypoint transitions before
   the first control sample is applied.
2. The optimiser controls all three available REMUS inputs in the prediction
   model: stern plane, rudder, and propeller RPM.
3. The reference includes desired pitch, pitch-rate, and yaw-rate when the
   reference generator provides them.  The old controller always used r_ref=0,
   even during heading changes, which created a conflict inside the objective.
4. Actuator amplitude and slew-rate constraints are hard constraints inside the
   optimisation problem.  The first move is also penalised against the last
   command actually sent to the simulator.
5. Heading and pitch errors are wrapped inside the CasADi objective.

Internal control order:
    U = [delta_s, delta_r, n_rpm]

Simulator/Fossen command order:
    output = [delta_r, delta_s, n_rpm]
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import numpy as np


class NMPC_REMUS100:
    """Reduced-order trajectory NMPC controller for REMUS 100."""

    def __init__(self, vehicle, N: int = 20, dt_mpc: float = 0.2, n_rpm: float = 1525):
        import casadi as ca
        from adapters.casadi_model import build_casadi_dynamics

        self.N = int(N)
        self.dt_mpc = float(dt_mpc)
        self.n_x = 6                         # [z, theta, psi, u, q, r]
        self.n_u = 3                         # [delta_s, delta_r, n]
        self.n_rpm = float(n_rpm)
        self.name = f"NMPC traj. (N={self.N})"
        self.solve_times: list[float] = []

        self.f_dyn = build_casadi_dynamics(vehicle)

        self.ds_max = float(vehicle.deltaMax_s)
        self.dr_max = float(vehicle.deltaMax_r)
        self.n_min = float(getattr(vehicle, "nMin", 0.0))
        self.n_max = float(getattr(vehicle, "nMax", self.n_rpm))
        # Some remus100 versions do not expose nMin.  In this project the
        # propeller is used as a forward cruise actuator, so keep it nonnegative.
        self.n_min = max(0.0, self.n_min)
        self.n_max = max(self.n_max, self.n_rpm)

        self.u_min = np.array([-self.ds_max, -self.dr_max, self.n_min], dtype=float)
        self.u_max = np.array([ self.ds_max,  self.dr_max, self.n_max], dtype=float)

        # Hard per-MPC-step slew limits.  These are intentionally calmer than
        # the previous patch because the controller now has preview and should
        # not need to kick the rudder at every waypoint boundary.
        self.du_max = np.array([math.radians(4.0), math.radians(4.0), 100.0], dtype=float)

        # Soft state safety limits inside the prediction horizon.  These help
        # the optimiser avoid unrealistic pitch/yaw-rate plans, but still leave
        # enough authority for the manoeuvres in the provided scenarios.
        self.theta_max = math.radians(25.0)
        self.q_max = math.radians(18.0)
        self.r_max = math.radians(18.0)

        # State cost order: [z, theta, psi, u_surge, q, r].
        Q_diag = np.array([120.0, 90.0, 260.0, 12.0, 28.0, 55.0], dtype=float)
        P_diag = np.array([350.0, 150.0, 520.0, 20.0, 45.0, 95.0], dtype=float)

        # Control effort around the nominal/reference control.  RPM has a tiny
        # weight because it is measured in raw rpm, unlike fins measured in rad.
        R_diag = np.array([18.0, 24.0, 1.0e-6], dtype=float)

        # Move suppression.  These terms are the main anti-chatter mechanism.
        Rd_diag = np.array([380.0, 520.0, 5.0e-5], dtype=float)
        Rprev_diag = np.array([650.0, 820.0, 8.0e-5], dtype=float)

        ca = ca
        opti = ca.Opti()
        X = opti.variable(self.n_x, self.N + 1)
        U = opti.variable(self.n_u, self.N)
        # Slack variables that soften the theta/q/r state limits.  The initial
        # state is pinned to the measurement, so hard state bounds make the NLP
        # structurally infeasible whenever the real vehicle exceeds a limit
        # (e.g. pitch overshoot in a steep climb) — every solve then fails and
        # the stale-command fallback lets the vehicle diverge.  An exact
        # (L1 + L2) penalty keeps the slacks at zero except when a violation
        # is unavoidable, so the optimiser can always plan a recovery.
        S_state = opti.variable(3)           # [s_theta, s_q, s_r] >= 0
        w_slack_lin = 1.0e3
        w_slack_quad = 1.0e4

        x0_p = opti.parameter(self.n_x)
        X_ref_p = opti.parameter(self.n_x, self.N + 1)
        U_ref_p = opti.parameter(self.n_u, self.N)
        u_prev_p = opti.parameter(self.n_u)
        Vc_p = opti.parameter()
        bc_p = opti.parameter()

        Q = ca.DM(np.diag(Q_diag))
        P = ca.DM(np.diag(P_diag))
        R = ca.DM(np.diag(R_diag))
        Rd = ca.DM(np.diag(Rd_diag))
        Rprev = ca.DM(np.diag(Rprev_diag))

        def wrapped_state_error(x_col, xr_col):
            """Return x_col - xr_col with theta/psi wrapped to [-pi, pi]."""
            e_z = x_col[0] - xr_col[0]
            e_theta = ca.atan2(ca.sin(x_col[1] - xr_col[1]), ca.cos(x_col[1] - xr_col[1]))
            e_psi = ca.atan2(ca.sin(x_col[2] - xr_col[2]), ca.cos(x_col[2] - xr_col[2]))
            e_u = x_col[3] - xr_col[3]
            e_q = x_col[4] - xr_col[4]
            e_r = x_col[5] - xr_col[5]
            return ca.vertcat(e_z, e_theta, e_psi, e_u, e_q, e_r)

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
        J += w_slack_lin * ca.sum1(S_state) + w_slack_quad * ca.sumsqr(S_state)
        opti.minimize(J)

        opti.subject_to(X[:, 0] == x0_p)

        for k in range(self.N):
            x_next = self._rk4(X[:, k], U[:, k], Vc_p, bc_p)
            opti.subject_to(X[:, k + 1] == x_next)

            # Actuator limits.
            opti.subject_to(opti.bounded(-self.ds_max, U[0, k], self.ds_max))
            opti.subject_to(opti.bounded(-self.dr_max, U[1, k], self.dr_max))
            opti.subject_to(opti.bounded(self.n_min, U[2, k], self.n_max))

            # Slew-rate limits.
            prev_u = u_prev_p if k == 0 else U[:, k - 1]
            opti.subject_to(opti.bounded(-self.du_max[0], U[0, k] - prev_u[0], self.du_max[0]))
            opti.subject_to(opti.bounded(-self.du_max[1], U[1, k] - prev_u[1], self.du_max[1]))
            opti.subject_to(opti.bounded(-self.du_max[2], U[2, k] - prev_u[2], self.du_max[2]))

        opti.subject_to(S_state >= 0)
        for k in range(self.N + 1):
            opti.subject_to(opti.bounded(-self.theta_max - S_state[0], X[1, k],
                                         self.theta_max + S_state[0]))
            opti.subject_to(opti.bounded(-self.q_max - S_state[1], X[4, k],
                                         self.q_max + S_state[1]))
            opti.subject_to(opti.bounded(-self.r_max - S_state[2], X[5, k],
                                         self.r_max + S_state[2]))

        opts = {
            "ipopt.print_level": 0,
            "ipopt.max_iter": 110,
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

        self._prev_X: Optional[np.ndarray] = None
        self._prev_U: Optional[np.ndarray] = None
        self.prev_u = np.array([0.0, 0.0, self.n_rpm], dtype=float)
        self.V_c_est = 0.0
        self.beta_c_est = 0.0
        self.reference_provider: Optional[Callable[[float], tuple[np.ndarray, np.ndarray]]] = None

    def _rk4(self, x, u, Vc, bc):
        dt = self.dt_mpc
        k1 = self.f_dyn(x, u, Vc, bc)
        k2 = self.f_dyn(x + dt / 2 * k1, u, Vc, bc)
        k3 = self.f_dyn(x + dt / 2 * k2, u, Vc, bc)
        k4 = self.f_dyn(x + dt * k3, u, Vc, bc)
        return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    @staticmethod
    def _ssa(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def set_reference_provider(self, reference_provider: Callable[[float], tuple[np.ndarray, np.ndarray]] | None) -> None:
        """Attach the same reference function used by the simulator.

        This is what turns the controller from setpoint MPC into trajectory
        NMPC: inside compute() the optimiser samples this provider at
        t, t+dt_mpc, ..., t+N*dt_mpc.
        """
        self.reference_provider = reference_provider

    def _single_reference_to_state(self, eta_d: np.ndarray, nu_d: np.ndarray) -> np.ndarray:
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

    def _build_reference_horizon(self, eta_d, nu_d, t: float) -> tuple[np.ndarray, np.ndarray]:
        X_ref = np.zeros((self.n_x, self.N + 1), dtype=float)
        U_ref = np.zeros((self.n_u, self.N), dtype=float)

        provider = self.reference_provider
        for k in range(self.N + 1):
            if provider is not None:
                eta_k, nu_k = provider(float(t + k * self.dt_mpc))
            else:
                eta_k, nu_k = eta_d, nu_d
            X_ref[:, k] = self._single_reference_to_state(eta_k, nu_k)

        # Keep heading references on a continuous branch across the prediction
        # horizon.  The objective wraps the error, but this improves warm starts
        # and removes artificial 0/360 discontinuities inside the initial guess.
        X_ref[2, :] = np.unwrap(X_ref[2, :])

        for k in range(self.N):
            U_ref[:, k] = np.array([0.0, 0.0, self.n_rpm], dtype=float)

        return X_ref, U_ref

    def _apply_output_safety_limits(self, u_opt: np.ndarray) -> np.ndarray:
        u_opt = np.asarray(u_opt, dtype=float).reshape(self.n_u)
        u_opt = np.clip(u_opt, self.u_min, self.u_max)
        du = np.clip(u_opt - self.prev_u, -self.du_max, self.du_max)
        return np.clip(self.prev_u + du, self.u_min, self.u_max)

    def compute(self, eta, nu, eta_d, nu_d, t):
        t_start = time.perf_counter()

        x0 = np.array([eta[2], eta[4], eta[5], nu[0], nu[4], nu[5]], dtype=float)
        X_ref, U_ref = self._build_reference_horizon(eta_d, nu_d, float(t))

        # Shift the reference branch near the current heading.  The error is
        # wrapped anyway, but this keeps the internal NLP numerically gentle.
        branch_shift = 2.0 * math.pi * round((x0[2] - X_ref[2, 0]) / (2.0 * math.pi))
        X_ref[2, :] += branch_shift

        self.opti.set_value(self.x0_p, x0)
        self.opti.set_value(self.X_ref_p, X_ref)
        self.opti.set_value(self.U_ref_p, U_ref)
        self.opti.set_value(self.u_prev_p, self.prev_u)
        self.opti.set_value(self.Vc_p, self.V_c_est)
        self.opti.set_value(self.bc_p, self.beta_c_est)

        if self._prev_X is not None and self._prev_U is not None:
            try:
                X_init = np.hstack([self._prev_X[:, 1:], self._prev_X[:, -1:]])
                U_init = np.hstack([self._prev_U[:, 1:], self._prev_U[:, -1:]])
                X_init[:, 0] = x0
                U_init[:, 0] = self.prev_u
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
            # Safe fallback: keep the last physical command.  Do not use
            # partially converged debug values; those tend to be single-sample
            # spikes at waypoint boundaries.
            u_opt = self.prev_u.copy()

        u_opt = self._apply_output_safety_limits(u_opt)
        if self._prev_U is not None:
            try:
                self._prev_U[:, 0] = u_opt
            except Exception:
                pass

        self.prev_u = u_opt.copy()
        self.solve_times.append(time.perf_counter() - t_start)

        # Fossen expects [delta_r, delta_s, n_rpm].
        return np.array([u_opt[1], u_opt[0], u_opt[2]], dtype=float)

    def _set_cold_start(self, x0: np.ndarray, X_ref: np.ndarray, U_ref: np.ndarray) -> None:
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

    def set_current_estimate(self, V_c: float, beta_c_rad: float) -> None:
        self.V_c_est = float(V_c)
        self.beta_c_est = float(beta_c_rad)

    def reset(self) -> None:
        self._prev_X = None
        self._prev_U = None
        self.prev_u = np.array([0.0, 0.0, self.n_rpm], dtype=float)
        self.solve_times = []