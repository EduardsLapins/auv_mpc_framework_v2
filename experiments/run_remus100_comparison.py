"""
Full REMUS 100 experiment suite — Fossen PID/SMC vs tuned PID vs NMPC.
All figure text is looked up via config.T() to support English / Latvian output.

Changes from the original version
----------------------------------
1. NMPC is now a trajectory NMPC: the optimiser receives the full future reference
   over the horizon, not only the current setpoint.
2. make_nmpc() reads USE_PATCHED_NMPC from config.py to choose between the
   original (N=20) and patched offset-free (N=30) variant.
3. Reference uses minimum-jerk / S-curve transitions with rate limits, which
   avoids impulsive reference rates at waypoint boundaries.
4. Reference includes theta_ref, q_ref, r_ref so NMPC does not penalise yaw
   rate as zero during heading changes.
5. Heading plots are drawn as continuous (unwrapped) angles.
6. Scenario 6 exports a heading-error CSV and a 6-panel comparison figure.
"""

from __future__ import annotations

import csv
import math
import os
import sys
from typing import Iterable

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from adapters.fossen_adapter import FossenVehicleAdapter
from config import T, USE_PATCHED_NMPC

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
COLORS = {
    "Fossen PID/SMC": "#e74c3c",
    "PID (tuned)":    "#3498db",   # 50 Hz — standard embedded rate
    "PID (5 Hz)":     "#e67e22",   # 5 Hz — rate-matched to NMPC
}
NMPC_COLOR = "#27ae60"


def _color(name: str) -> str:
    return COLORS.get(name, NMPC_COLOR)


def _ssa(angle: float) -> float:
    """Smallest signed angle in radians, scalar version."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_to_pi(angle):
    """Smallest signed angle in radians, scalar or NumPy array."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def _angle_delta(start_rad: float, target_rad: float) -> float:
    """Shortest angular delta from start to target.

    For the exact 180 deg case, choose +pi instead of -pi so a labelled
    0 -> 180 deg step is plotted in the intuitive positive direction.
    """
    d = _ssa(target_rad - start_rad)
    if abs(d + math.pi) < 1e-12:
        d = math.pi
    return d


def heading_deg(angle_rad) -> np.ndarray:
    """Convert radian heading to compass degrees [0, 360)."""
    return np.mod(np.degrees(angle_rad), 360.0)


def heading_continuous_deg(angle_rad, *, anchor_deg: float = 0.0) -> np.ndarray:
    """Unwrap a heading time series into a continuous degree signal.

    A compass heading of -1 deg and 359 deg is physically the same direction,
    but plotting it as [0, 360) makes the graph jump from the bottom to the
    top.  This function preserves physical continuity instead:

        0, -1, -2, ...        instead of      0, 359, 358, ...
        330, 360, 405         instead of      330, 0, 45
    """
    y = np.degrees(np.unwrap(np.asarray(angle_rad, dtype=float)))
    if y.size:
        y = y + 360.0 * round((anchor_deg - y[0]) / 360.0)
    return y


def heading_error_deg(eta_psi, eta_d_psi) -> np.ndarray:
    """Signed heading tracking error in degrees, wrapped to [-180, 180]."""
    return np.degrees(_wrap_to_pi(np.asarray(eta_psi) - np.asarray(eta_d_psi)))


def plot_heading_continuous(ax, time, angle_rad, *, label, color, lw=1.5, ls="-",
                            alpha=1.0, anchor_deg: float = 0.0):
    """Plot heading as a continuous signal instead of compass-modulo [0, 360)."""
    y = heading_continuous_deg(angle_rad, anchor_deg=anchor_deg)
    ax.plot(time, y, color=color, label=label, lw=lw, ls=ls, alpha=alpha)


# Backwards-compatible alias.
def plot_heading_line(ax, time, angle_rad, *, label, color, lw=1.5, ls="-", alpha=1.0):
    plot_heading_continuous(ax, time, angle_rad, label=label, color=color,
                            lw=lw, ls=ls, alpha=alpha)


def varying_current(seed: int, t_final: float, V_mean: float, beta_mean_deg: float, *,
                    dt: float = 0.02, V_sigma: float = 0.12, tau: float = 60.0,
                    wave_amp: float = 0.08, wave_period: float = 9.0,
                    beta_sigma_deg: float = 12.0, beta_tau: float = 90.0):
    """Seeded time-varying ocean current for the standard scenarios.

    Same construction as ``analysis.run_advanced_experiments.gauss_markov_current``
    (mean-reverting Gauss–Markov speed + wave orbital component + slowly
    wandering direction), with milder defaults: the scenarios stop being
    idealised constant-current runs, but the figures stay legible.  The whole
    realisation is pre-sampled from one seeded RNG, so every rerun produces
    identical figures.  Returns ``fn(t) -> (V_c, beta_c_deg)`` for
    ``FossenVehicleAdapter.run(disturbance_fn=...)``; controllers are still
    told only the nominal mean (V_mean, beta_mean) — the realisation is unknown
    to them, which keeps the comparison honest.
    """
    rng = np.random.default_rng(int(seed))
    n = int(round(t_final / dt)) + 2
    t_grid = np.arange(n) * dt

    a_v = math.exp(-dt / tau)
    q_v = V_sigma * math.sqrt(max(0.0, 1.0 - a_v * a_v))
    V = np.empty(n)
    V[0] = V_mean
    wn = rng.standard_normal(n)
    for k in range(1, n):
        V[k] = V_mean + a_v * (V[k - 1] - V_mean) + q_v * wn[k]
    V = V + wave_amp * np.sin(2.0 * np.pi * t_grid / wave_period
                              + rng.uniform(0.0, 2.0 * np.pi))
    V = np.clip(V, 0.0, V_mean + 0.5)

    a_b = math.exp(-dt / beta_tau)
    q_b = beta_sigma_deg * math.sqrt(max(0.0, 1.0 - a_b * a_b))
    B = np.empty(n)
    B[0] = beta_mean_deg
    bn = rng.standard_normal(n)
    for k in range(1, n):
        B[k] = beta_mean_deg + a_b * (B[k - 1] - beta_mean_deg) + q_b * bn[k]

    def fn(t: float):
        i = min(max(int(t / dt), 0), n - 1)
        return float(V[i]), float(B[i])

    fn.mean_speed = float(V_mean)
    fn.t_grid = t_grid
    fn.V = V
    fn.beta_deg = B
    return fn


def make_nmpc(V_c: float = 0.0, beta_c: float = 0.0):
    """Build the NMPC controller, using the variant set in config.py."""
    from python_vehicle_simulator.vehicles.remus100 import remus100
    v = remus100("stepInput", V_current=V_c, beta_current=beta_c)
    # N=12: the horizon sweep (EXP-C) and the S6 horizon check showed accuracy
    # saturates by N~12, at less than half the compute of N=30.
    if USE_PATCHED_NMPC:
        from controllers.nmpc_remus_patched import NMPC_REMUS100_Patched
        nmpc = NMPC_REMUS100_Patched(v, N=12, n_rpm=1525)
    else:
        from controllers.nmpc_remus import NMPC_REMUS100
        nmpc = NMPC_REMUS100(v, N=12, n_rpm=1525)
    nmpc.set_current_estimate(V_c, beta_c * D2R)
    return nmpc


def make_pid():
    from controllers.pid_remus import PID_REMUS100
    return PID_REMUS100(n_rpm=1525)


def wrap_nmpc(nmpc, reference_fn=None, dt_mpc: float = 0.2):
    """Wrap NMPC to run at a slower MPC rate than the 50 Hz simulator.

    reference_fn is passed into the controller so the NLP can sample the
    future reference at t, t+dt, ..., t+N*dt.
    """
    if hasattr(nmpc, "set_reference_provider"):
        nmpc.set_reference_provider(reference_fn)

    last_t = [-1.0]
    last_u = [np.array([0.0, 0.0, nmpc.n_rpm], dtype=float)]

    def fn(eta, nu, eta_d, nu_d, t):
        if t - last_t[0] >= dt_mpc - 1e-9:
            last_u[0] = nmpc.compute(eta, nu, eta_d, nu_d, t)
            last_t[0] = t
        return last_u[0]

    fn.name = nmpc.name
    fn.solve_times = nmpc.solve_times
    return fn


def wrap_pid(pid):
    """Wrap PID at full 50 Hz simulator rate."""
    def fn(eta, nu, eta_d, nu_d, t):
        return pid.compute(eta, nu, eta_d, nu_d, t)

    fn.name = pid.name
    fn.solve_times = pid.solve_times
    return fn


def wrap_pid_5hz(pid, dt_mpc: float = 0.2):
    """Wrap PID as a true 5 Hz controller (matches NMPC update rate).

    Unlike a ZOH wrapper, this only calls compute() once per dt_mpc second,
    passing dt=dt_mpc so the integral and reference smoothing step correctly.
    The integral accumulation rate is mathematically identical to 50 Hz
    for dynamics slower than 5 Hz (which applies to REMUS 100 heading/depth).
    """
    last_t = [-1.0]
    last_u = [None]

    def fn(eta, nu, eta_d, nu_d, t):
        if t - last_t[0] >= dt_mpc - 1e-9:
            u = pid.compute(eta, nu, eta_d, nu_d, t, dt=dt_mpc)
            last_u[0] = u.copy()
            last_t[0] = t
        return last_u[0] if last_u[0] is not None else pid.compute(eta, nu, eta_d, nu_d, t, dt=dt_mpc)

    fn.name = pid.name + " (5 Hz)"
    fn.solve_times = pid.solve_times
    return fn


def wrap_pid_hz(pid, dt: float):
    """Run PID at any rate by passing the matching dt to compute().

    Use with sampleTime=dt in adapter.run() so the simulation steps and
    the PID update are synchronised (call compute() once per physics step).
    """
    def fn(eta, nu, eta_d, nu_d, t):
        return pid.compute(eta, nu, eta_d, nu_d, t, dt=dt)
    fn.name = pid.name
    fn.solve_times = pid.solve_times
    return fn


def downsample_result(r, factor: int):
    """Return a new SimulationResult keeping every `factor`-th time step.

    Used to bring a high-rate run (e.g. 500 Hz) back to the standard 50 Hz
    grid so it can be saved alongside other results in the same CSV.
    """
    from adapters.fossen_adapter import SimulationResult
    idx = np.arange(0, len(r.time), factor)
    return SimulationResult(
        time=r.time[idx],
        eta=r.eta[idx],
        nu=r.nu[idx],
        u_control=r.u_control[idx],
        u_actual=r.u_actual[idx],
        eta_d=r.eta_d[idx],
        controller_name=r.controller_name,
        solve_times=r.solve_times,
    )


def _minimum_jerk(s: float) -> tuple[float, float, float]:
    """Return alpha, d(alpha)/ds, d2(alpha)/ds2 for 10s^3-15s^4+6s^5."""
    s = max(0.0, min(1.0, float(s)))
    a = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    da = 30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4
    dda = 60.0 * s - 180.0 * s**2 + 120.0 * s**3
    return a, da, dda


def _transition_duration(
    z0: float,
    z1: float,
    psi0: float,
    psi1: float,
    tau_rise: float,
    *,
    z_rate_max: float,
    psi_rate_max: float,
) -> float:
    """Choose an S-curve duration that respects approximate peak rates."""
    dz = abs(float(z1) - float(z0))
    dpsi = abs(_angle_delta(float(psi0), float(psi1)))
    T_base = max(0.1, 2.5 * float(tau_rise))
    T_z = 1.875 * dz / max(z_rate_max, 1e-6)
    T_psi = 1.875 * dpsi / max(psi_rate_max, 1e-6)
    return max(T_base, T_z, T_psi, 0.1)


def smooth_ref(
    targets: list[tuple[float, float]],
    switch_times: list[float],
    tau_rise: float = 10.0,
    *,
    u_ref: float = 2.5,
    z_rate_max: float = 1.0,
    psi_rate_max_deg: float = 5.0,
    theta_max_deg: float = 18.0,
    q_ref_max_deg_s: float = 10.0,
    r_ref_max_deg_s: float = 8.0,
):
    """Create a physically smooth depth/heading reference.

    targets are (depth_m, heading_deg).  The heading is interpolated on the
    shortest angular path, and the transition profile is minimum-jerk.
    """
    if len(targets) != len(switch_times) + 1:
        raise ValueError("targets length must be switch_times length + 1")

    psi_rate_max = psi_rate_max_deg * D2R
    theta_max = theta_max_deg * D2R
    q_ref_max = q_ref_max_deg_s * D2R
    r_ref_max = r_ref_max_deg_s * D2R

    durations = []
    for i in range(1, len(targets)):
        z0, psi0_deg = targets[i - 1]
        z1, psi1_deg = targets[i]
        durations.append(
            _transition_duration(
                z0, z1, psi0_deg * D2R, psi1_deg * D2R, tau_rise,
                z_rate_max=z_rate_max, psi_rate_max=psi_rate_max,
            )
        )

    def ref(t: float):
        z_d, psi_deg = targets[0]
        psi_rad = psi_deg * D2R
        z_dot = 0.0
        z_ddot = 0.0

        for i in range(1, len(targets)):
            t_switch = switch_times[i - 1]
            if t >= t_switch:
                z0, psi0_deg = targets[i - 1]
                z1, psi1_deg = targets[i]
                psi0 = psi0_deg * D2R
                psi1 = psi1_deg * D2R
                dz = z1 - z0
                dpsi = _angle_delta(psi0, psi1)

                T = durations[i - 1]
                s_loc = (float(t) - t_switch) / T
                alpha, dalpha_ds, ddalpha_ds2 = _minimum_jerk(s_loc)
                alpha_dot = dalpha_ds / T if 0.0 <= s_loc <= 1.0 else 0.0
                alpha_ddot = ddalpha_ds2 / (T * T) if 0.0 <= s_loc <= 1.0 else 0.0

                z_d = z0 + dz * alpha
                z_dot = dz * alpha_dot
                z_ddot = dz * alpha_ddot
                psi_rad = _ssa(psi0 + dpsi * alpha)
                psi_dot = float(np.clip(dpsi * alpha_dot, -r_ref_max, r_ref_max))

        u_safe = max(abs(u_ref), 0.3)
        theta_arg = float(np.clip(-z_dot / u_safe, -math.sin(theta_max), math.sin(theta_max)))
        theta_ref = math.asin(theta_arg)

        denom = max(1e-3, math.sqrt(max(1e-6, 1.0 - theta_arg * theta_arg)))
        q_ref = (-z_ddot / u_safe) / denom
        q_ref = float(np.clip(q_ref, -q_ref_max, q_ref_max))

        # psi_dot may have been overwritten in the loop; guard against UnboundLocalError
        try:
            psi_dot  # noqa: B018
        except NameError:
            psi_dot = 0.0

        eta_d = np.array([0.0, 0.0, z_d, 0.0, theta_ref, psi_rad], dtype=float)
        nu_d = np.array([u_ref, 0.0, z_dot, 0.0, q_ref, psi_dot], dtype=float)
        return eta_d, nu_d

    ref.transition_durations = durations
    return ref


def tracking_errors(result):
    """Return tracking error arrays and summary statistics for one result."""
    if result.eta_d is None:
        raise ValueError("result.eta_d is required for tracking error analysis")

    e_z = result.eta[:, 2] - result.eta_d[:, 2]
    e_psi_rad = _wrap_to_pi(result.eta[:, 5] - result.eta_d[:, 5])
    e_psi_deg = np.degrees(e_psi_rad)
    dt = result.time[1] - result.time[0] if len(result.time) > 1 else 0.02

    return {
        "e_z": e_z,
        "abs_e_z": np.abs(e_z),
        "e_psi_rad": e_psi_rad,
        "e_psi_deg": e_psi_deg,
        "abs_e_psi_deg": np.abs(e_psi_deg),
        "cum_abs_z": np.cumsum(np.abs(e_z)) * dt,
        "cum_abs_psi_deg": np.cumsum(np.abs(e_psi_deg)) * dt,
        "dt": dt,
        "z_rmse": float(np.sqrt(np.mean(e_z**2))),
        "z_mae": float(np.mean(np.abs(e_z))),
        "z_iae": float(np.sum(np.abs(e_z)) * dt),
        "psi_rmse_deg": float(np.sqrt(np.mean(e_psi_deg**2))),
        "psi_mae_deg": float(np.mean(np.abs(e_psi_deg))),
        "psi_max_deg": float(np.max(np.abs(e_psi_deg))),
        "psi_iae_deg_s": float(np.sum(np.abs(e_psi_deg)) * dt),
    }


def compute_metrics(result, tz: float | None = None, tp_deg: float | None = None,
                    t0: float = 0.0, ref=None):
    """Whole-run tracking metrics for scenarios 1-5.

    When ``ref`` (callable t -> (eta_d, nu_d)) is given, errors are measured
    against the time-varying mission reference over the full run, transient
    included.  Otherwise errors fall back to the recorded eta_d or the
    constant final target.  ``t0`` optionally skips the start of the run.
    """
    mask = result.time >= t0
    if not mask.any():
        mask = np.ones(len(result.time), dtype=bool)

    if ref is not None:
        ref_poses = np.array([ref(t)[0] for t in result.time])
        z_ref = ref_poses[:, 2]
        psi_ref = ref_poses[:, 5]
    else:
        if tz is None and result.eta_d is not None:
            z_ref = result.eta_d[:, 2]
        else:
            z_ref = np.full_like(result.time, float(tz))

        if tp_deg is None and result.eta_d is not None:
            psi_ref = result.eta_d[:, 5]
        else:
            psi_ref = np.full_like(result.time, float(tp_deg) * D2R)

    ze = result.eta[mask, 2] - z_ref[mask]
    pe = _wrap_to_pi(result.eta[mask, 5] - psi_ref[mask])
    return {
        "z_rmse": float(np.sqrt(np.mean(ze**2))),
        "psi_rmse_deg": float(np.degrees(np.sqrt(np.mean(pe**2)))),
    }


def plot_scenario(results, path: str, title: str):
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle(title, fontsize=14, fontweight="bold")

    for r in results:
        c = _color(r.controller_name)
        axes[0].plot(r.time, r.eta[:, 2], color=c, label=r.controller_name, lw=1.5)
        plot_heading_continuous(axes[1], r.time, r.eta[:, 5], label=r.controller_name,
                                color=c, lw=1.5, anchor_deg=0.0)
        spd = np.sqrt(np.sum(r.nu[:, :3] ** 2, axis=1))
        axes[2].plot(r.time, spd, color=c, label=r.controller_name, lw=1.5)

    # Common mission reference (all controllers track the same trajectory)
    ref_r = results[-1]
    if ref_r.eta_d is not None:
        axes[0].plot(ref_r.time, ref_r.eta_d[:, 2], color="k", ls="--", lw=1,
                     alpha=0.6, label=T("reference"))
        plot_heading_continuous(axes[1], ref_r.time, ref_r.eta_d[:, 5],
                                label=T("reference"), color="k", lw=1, ls="--",
                                alpha=0.6, anchor_deg=0.0)

    axes[0].set_ylabel(T("depth_m"))
    axes[0].set_title(T("depth_tracking"))
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel(T("heading_deg"))
    axes[1].set_title(T("heading_tracking"))
    axes[1].margins(y=0.08)
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_ylabel(T("speed_ms"))
    axes[2].set_xlabel(T("time_s"))
    axes[2].set_title(T("speed"))
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    # leave headroom so the suptitle does not overlap the first panel title
    plt.tight_layout(rect=(0, 0, 1, 0.955))
    plt.savefig(path, dpi=200)
    print(f"    Saved: {path}")
    plt.close()


FOSSEN_NAME = "Fossen PID/SMC"

# Rows accumulated by every scenario run, written by main() to one CSV.
SCENARIO_METRICS: list[dict] = []


def record_scenario_metrics(tag: str, results):
    for r in results:
        e = tracking_errors(r)
        SCENARIO_METRICS.append({
            "scenario": tag,
            "controller": r.controller_name,
            "z_rmse_m": f"{e['z_rmse']:.4f}",
            "z_mae_m": f"{e['z_mae']:.4f}",
            "z_iae_m_s": f"{e['z_iae']:.2f}",
            "psi_rmse_deg": f"{e['psi_rmse_deg']:.4f}",
            "psi_mae_deg": f"{e['psi_mae_deg']:.4f}",
            "psi_max_deg": f"{e['psi_max_deg']:.4f}",
            "psi_iae_deg_s": f"{e['psi_iae_deg_s']:.2f}",
        })


def write_scenario_metrics_csv(out: str) -> str:
    path = os.path.join(out, "scenario_metrics.csv")
    cols = ["scenario", "controller", "z_rmse_m", "z_mae_m", "z_iae_m_s",
            "psi_rmse_deg", "psi_mae_deg", "psi_max_deg", "psi_iae_deg_s"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(SCENARIO_METRICS)
    print(f"    Saved: {path}")
    return path


def run_three(t_final, z_d, psi_d, Vc, bc, ref, tag, out, title, *, seed=0,
              eta0=None, nu0=None):
    """Run the three-controller comparison for one scenario.

    ``eta0``/``nu0`` optionally start every controller from the same non-zero
    initial state (e.g. already at depth), instead of surfaced and at rest.
    """
    # Every scenario runs against a seeded time-varying current realisation
    # (identical for all three controllers); controllers only know the mean.
    dist = varying_current(seed, t_final, Vc, bc)
    results = []

    print("    Fossen autopilot...")
    a = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r1 = a.run_builtin_autopilot(t_final, z_d, psi_d, 1525, Vc, bc,
                                 disturbance_fn=dist, reference_fn=ref,
                                 eta0=eta0, nu0=nu0)
    r1.controller_name = "Fossen PID/SMC"
    results.append(r1)

    print("    Tuned PID (50 Hz)...")
    pid = make_pid()
    a2 = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r2 = a2.run(t_final, wrap_pid(pid), ref, eta0=eta0, nu0=nu0,
                sampleTime=0.02, disturbance_fn=dist)
    r2.controller_name = "PID (tuned)"
    results.append(r2)

    print("    NMPC...")
    nmpc = make_nmpc(Vc, bc)
    a3 = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r3 = a3.run(t_final, wrap_nmpc(nmpc, ref), ref, eta0=eta0, nu0=nu0,
                sampleTime=0.02, disturbance_fn=dist)
    r3.controller_name = nmpc.name
    results.append(r3)

    for r in results:
        m = compute_metrics(r, z_d, psi_d, ref=ref)
        print(f"      {r.controller_name:28s}: z_RMSE={m['z_rmse']:.3f} m, "
              f"psi_RMSE={m['psi_rmse_deg']:.2f} deg")
    record_scenario_metrics(tag, results)

    plot_scenario(results, f"{out}/{tag}.png", title)
    plot_scenario([r for r in results if r.controller_name != FOSSEN_NAME],
                  f"{out}/{tag}_bez_fossen.png", title)
    return results


def scenario_1(out):
    print("\n" + "=" * 70 + "\n  SCENARIO 1: Standard depth+heading (Vc=0.5)\n" + "=" * 70)
    return run_three(
        200, 30, 50, 0.5, 170,
        smooth_ref([(0, 0), (30, 50)], [2.0]),
        "s1_standarts", out,
        T("s1_title"),
        seed=1,
    )


def scenario_2(out):
    print("\n" + "=" * 70 + "\n  SCENARIO 2: Disturbed environment\n" + "=" * 70)
    for i, (Vc, bc) in enumerate([(0.0, 0), (0.5, 170), (1.0, 170)]):
        print(f"\n  Current: {Vc} m/s @ {bc} deg")
        tag = f"s2_Vc{Vc:.1f}".replace(".", "_")
        run_three(
            200, 30, 50, Vc, bc,
            smooth_ref([(0, 0), (30, 50)], [2.0]),
            tag, out,
            T("s2_title").format(vc=Vc),
            seed=21 + i,
        )


S3_DEPTH = 20.0   # scenario 3 runs entirely at this depth


def scenario_3(out):
    """Pure heading manoeuvres: the vehicle starts trimmed at S3_DEPTH and the
    depth reference is held constant, so nothing in the depth channel (dive
    pitch, stern-plane action) can contaminate the yaw response."""
    print("\n" + "=" * 70 + "\n  SCENARIO 3: Large heading changes\n" + "=" * 70)
    eta0 = np.array([0.0, 0.0, S3_DEPTH, 0.0, 0.0, 0.0])
    for j, pd in enumerate([90, 180]):
        print(f"\n  Heading: 0 -> {pd} deg (constant depth {S3_DEPTH:.0f} m)")
        run_three(
            200, S3_DEPTH, pd, 0.3, 90,
            smooth_ref([(S3_DEPTH, 0), (S3_DEPTH, pd)], [2.0], tau_rise=15.0),
            f"s3_kurss{pd}", out,
            T("s3_title").format(pd=pd),
            seed=31 + j,
            eta0=eta0,
        )


def scenario_4(out):
    print("\n" + "=" * 70 + "\n  SCENARIO 4: Multi-waypoint mission\n" + "=" * 70)
    return run_three(
        400, 20, 30, 0.5, 170,
        smooth_ref([(0, 0), (20, 30), (40, 120), (20, 30)], [5.0, 100.0, 200.0],
                   tau_rise=15.0),
        "s4_daudzpunkti", out,
        T("s4_title"),
        seed=4,
    )


def scenario_5(out):
    print("\n" + "=" * 70 + "\n  SCENARIO 5: Aggressive descent 0->50m\n" + "=" * 70)
    return run_three(
        150, 50, 0, 0.0, 0,
        smooth_ref([(0, 0), (50, 0)], [2.0], tau_rise=5.0),
        "s5_descent", out,
        T("s5_title"),
        seed=5,
    )


def _segment_bounds(switch_times: Iterable[float], t_final: float):
    return [0.0, *list(switch_times), float(t_final)]


def print_heading_segment_analysis(result, switch_times, t_final, *, top_n: int = 4):
    errs = tracking_errors(result)
    rows = []
    bounds = _segment_bounds(switch_times, t_final)
    for a, b in zip(bounds[:-1], bounds[1:]):
        mask = (result.time >= a) & (result.time < b)
        if not mask.any():
            continue
        dt = errs["dt"]
        abs_e = errs["abs_e_psi_deg"][mask]
        rows.append(
            {
                "segment": f"{a:.0f}-{b:.0f}s",
                "iae": float(np.sum(abs_e) * dt),
                "mae": float(np.mean(abs_e)),
                "max": float(np.max(abs_e)),
            }
        )

    rows.sort(key=lambda x: x["iae"], reverse=True)
    print(f"\n    {result.controller_name} — worst heading error segments:")
    for row in rows[:top_n]:
        print(
            f"      {row['segment']:>9s}: int|e_psi|dt={row['iae']:7.1f} deg*s, "
            f"MAE={row['mae']:5.2f} deg, max={row['max']:5.2f} deg"
        )


def _safe_col(name: str) -> str:
    """Controller name -> CSV column prefix (see analyze_s6.CONTROLLERS)."""
    return (name
            .replace(" ", "_")
            .replace("(", "").replace(")", "")
            .replace("=", "").replace("-", "-"))


def save_s6_error_csv(results, out: str) -> str:
    """Save time-history of heading and depth (plus their errors) for scenario 6.

    Depth columns are appended after the heading block, so every consumer that
    selects columns by name (analyze_s6, thesis_analysis) keeps working; they
    are what lets the deep-dive analysis score depth as well as heading.
    """
    path = os.path.join(out, "s6_kursa_kludas_analize.csv")
    base = results[0]
    ref_cont = heading_continuous_deg(base.eta_d[:, 5], anchor_deg=0.0)
    result_cont = {id(r): heading_continuous_deg(r.eta[:, 5], anchor_deg=0.0)
                   for r in results}

    headers = ["time_s", "reference_heading_deg", "reference_heading_continuous_deg"]
    for r in results:
        safe = _safe_col(r.controller_name)
        headers += [
            f"{safe}_heading_deg",
            f"{safe}_heading_continuous_deg",
            f"{safe}_heading_error_deg",
        ]
    headers.append("reference_depth_m")
    for r in results:
        safe = _safe_col(r.controller_name)
        headers += [f"{safe}_depth_m", f"{safe}_depth_error_m"]

    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for i, t in enumerate(base.time):
            row = [
                f"{t:.3f}",
                f"{heading_deg(base.eta_d[i, 5]):.6f}",
                f"{ref_cont[i]:.6f}",
            ]
            for r in results:
                row.append(f"{heading_deg(r.eta[i, 5]):.6f}")
                row.append(f"{result_cont[id(r)][i]:.6f}")
                row.append(f"{heading_error_deg(r.eta[i, 5], r.eta_d[i, 5]):.6f}")
            row.append(f"{base.eta_d[i, 2]:.6f}")
            for r in results:
                row.append(f"{r.eta[i, 2]:.6f}")
                row.append(f"{r.eta[i, 2] - r.eta_d[i, 2]:.6f}")
            writer.writerow(row)

    print(f"    Saved: {path}")
    return path


def scenario_6_complex(out):
    """SCENARIO 6: complex 8-segment mission (600 s) — long-horizon comparison."""
    print("\n" + "=" * 70)
    print("  SCENARIO 6: Complex trajectory (600 s mission)")
    print("  8 segments, current 0.6 m/s @ 150 deg")
    print("=" * 70)

    Vc = 0.6
    bc = 150
    t_final = 600.0
    waypoints = [(0, 0), (15, 0), (25, 90), (40, 90), (40, 200),
                 (10, 200), (30, 330), (45, 45), (20, 0)]
    switch_times = [5, 60, 120, 180, 260, 340, 420, 500]
    # z_rate_max capped at u_ref * sin(theta_max) ≈ 2.5 * sin(25°) ≈ 1.05 m/s.
    # Without this limit the S-curve commands depth rates up to 2 m/s (pitch ~53°),
    # far above the NMPC predictor's pitch constraint (25°), causing a large
    # pitch→yaw coupling mismatch that neither the model nor the observer can cancel.
    ref = smooth_ref(waypoints, switch_times, tau_rise=12.0, z_rate_max=1.0)
    # Same seeded time-varying current for all three controllers (seed 6);
    # analysis.compare_nmpc_patch rebuilds the identical realisation.
    dist = varying_current(6, t_final, Vc, bc)
    results = []

    # PID at 50 Hz — standard embedded-system rate
    print("    PID (50 Hz)...")
    pid_50 = make_pid()
    a_pid50 = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r_pid50 = a_pid50.run(t_final, wrap_pid(pid_50), ref, sampleTime=0.02,
                          disturbance_fn=dist)
    r_pid50.controller_name = "PID (tuned)"
    results.append(r_pid50)

    # Fossen builtin autopilot — baseline, follows the same mission reference
    print("    Fossen autopilot...")
    a_fossen = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r_fossen = a_fossen.run_builtin_autopilot(t_final, 0.0, 0.0, 1525, Vc, bc,
                                              disturbance_fn=dist,
                                              reference_fn=ref)
    r_fossen.controller_name = "Fossen PID/SMC"
    results.append(r_fossen)

    # PID at 5 Hz — rate-matched to NMPC (same gains, slower feedback loop)
    print("    PID (5 Hz) — rate-matched to NMPC...")
    pid_5 = make_pid()
    a_pid5 = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r_pid5 = a_pid5.run(t_final, wrap_pid_5hz(pid_5), ref, sampleTime=0.02,
                        disturbance_fn=dist)
    r_pid5.controller_name = "PID (5 Hz)"
    results.append(r_pid5)

    # NMPC at 5 Hz — constrained by solver time (~25 ms / call)
    print("    NMPC (5 Hz)...")
    nmpc = make_nmpc(Vc, bc)
    a3 = FossenVehicleAdapter(V_current=Vc, beta_current=bc)
    r3 = a3.run(t_final, wrap_nmpc(nmpc, ref), ref, sampleTime=0.02,
                disturbance_fn=dist)
    r3.controller_name = nmpc.name
    results.append(r3)

    print("\n  Cumulative tracking errors:")
    for r in results:
        e = tracking_errors(r)
        print(
            f"    {r.controller_name:28s}: "
            f"int|e_z|dt={e['z_iae']:7.1f} m*s, z_RMSE={e['z_rmse']:5.2f} m, "
            f"int|e_psi|dt={e['psi_iae_deg_s']:8.1f} deg*s, "
            f"psi_RMSE={e['psi_rmse_deg']:5.2f} deg, "
            f"psi_MAE={e['psi_mae_deg']:5.2f} deg, max={e['psi_max_deg']:5.2f} deg"
        )
        print_heading_segment_analysis(r, switch_times, t_final)

    save_s6_error_csv(results, out)

    # Save controller solve times for computational performance analysis.
    # NMPC runs at 5 Hz (dt_mpc=0.2 s); PID runs at 50 Hz (sampleTime=0.02 s).
    times_path = os.path.join(out, "nmpc_solve_times_s6.csv")
    with open(times_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["call_index", "mission_time_s",
                    "nmpc_solve_time_ms", "pid50_solve_time_ms", "pid5_solve_time_ms"])
        n_nmpc  = len(nmpc.solve_times)
        n_pid50 = len(pid_50.solve_times)
        n_pid5  = len(pid_5.solve_times)
        # One row per NMPC call (every 0.2 s = 5 Hz).
        # PID-50: 10 calls per row; PID-5: 1 call per row.
        n_rows = max(n_nmpc, n_pid50 // 10 if n_pid50 else 0, n_pid5)
        for i in range(n_rows):
            t_s = i * 0.2
            nmpc_ms = nmpc.solve_times[i] * 1000 if i < n_nmpc else ""

            sl50 = pid_50.solve_times[i * 10:(i + 1) * 10]
            pid50_ms = float(np.mean(sl50)) * 1000 if sl50 else ""

            pid5_ms = pid_5.solve_times[i] * 1000 if i < n_pid5 else ""

            w.writerow([
                i, f"{t_s:.3f}",
                f"{nmpc_ms:.4f}"  if nmpc_ms  != "" else "",
                f"{pid50_ms:.4f}" if pid50_ms != "" else "",
                f"{pid5_ms:.4f}"  if pid5_ms  != "" else "",
            ])
    print(f"    Saved: {times_path}")

    record_scenario_metrics("s6_kompleksa_trajektorija", results)
    _plot_s6_figures(results, switch_times, out)
    _plot_s6_figures([r for r in results if r.controller_name != FOSSEN_NAME],
                     switch_times, out, suffix="_bez_fossen")

    print(
        "\n  Interpretation: if heading error is large immediately after switch points, "
        "this is typically a transient-regime effect.  The heading error is computed as "
        "the shortest angular difference wrap(heading - reference) in [-180, 180], "
        "and the figure shows where this error actually accumulates."
    )

    return results


def _plot_s6_figures(results, switch_times, out, suffix=""):
    """Scenario-6 six-panel + top-view figures for the given controller subset."""
    fig, axes = plt.subplots(6, 1, figsize=(16, 21), sharex=True)
    fig.suptitle(T("s6_title"), fontsize=14, fontweight="bold")

    for r in results:
        c = _color(r.controller_name)
        t = r.time
        e = tracking_errors(r)
        spd = np.sqrt(np.sum(r.nu[:, :3] ** 2, axis=1))

        axes[0].plot(t, r.eta[:, 2], color=c, label=r.controller_name, lw=1.5)
        plot_heading_continuous(axes[1], t, r.eta[:, 5], label=r.controller_name,
                                color=c, lw=1.5, anchor_deg=0.0)
        axes[2].plot(t, e["e_psi_deg"], color=c, label=r.controller_name, lw=1.2)
        axes[3].plot(t, spd, color=c, label=r.controller_name, lw=1.5)
        if r.controller_name == "PID (5 Hz)":
            # PID (5 Hz) alternates between ±15° saturation at every update —
            # 3 000 square-wave cycles over 600 s cannot be plotted as a readable
            # line.  Omit from this panel; the instability is discussed in text.
            pass
        else:
            axes[4].plot(t, np.degrees(r.u_actual[:, 0]), color=c, ls="-",
                         label=f"{r.controller_name} {T('rudder')}", lw=1)
            axes[4].plot(t, np.degrees(r.u_actual[:, 1]), color=c, ls="--",
                         label=f"{r.controller_name} {T('stern')}", lw=1, alpha=0.7)
        axes[5].plot(t, e["cum_abs_z"], color=c,
                     label=f"{r.controller_name} {T('int_ez')}", lw=1.5)

    ax5b = axes[5].twinx()
    for r in results:
        c = _color(r.controller_name)
        e = tracking_errors(r)
        ax5b.plot(r.time, e["cum_abs_psi_deg"], color=c, ls="--",
                  label=f"{r.controller_name} {T('int_epsi')}", lw=1.2, alpha=0.8)

    ref_r = results[-1]
    if ref_r.eta_d is not None:
        axes[0].plot(ref_r.time, ref_r.eta_d[:, 2], "k--", lw=1, alpha=0.4,
                     label=T("reference"))
        plot_heading_continuous(axes[1], ref_r.time, ref_r.eta_d[:, 5],
                                label=T("reference"), color="k", lw=1, ls="--",
                                alpha=0.4, anchor_deg=0.0)

    for ts in switch_times:
        for ax in axes:
            ax.axvline(x=ts, color="gray", ls=":", alpha=0.3)

    axes[0].set_ylabel(T("depth_m"))
    axes[0].set_title(T("depth_tracking"))
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel(T("heading_deg"))
    axes[1].set_title(T("heading_tracking"))
    axes[1].margins(y=0.08)
    axes[1].legend(loc="best", fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].axhline(0, color="gray", lw=1, alpha=0.4)
    axes[2].set_ylabel(T("heading_err_short"))
    axes[2].set_title(T("heading_error_wrap"))
    axes[2].legend(loc="best", fontsize=8)
    axes[2].grid(True, alpha=0.3)

    axes[3].set_ylabel(T("speed_ms"))
    axes[3].set_title(T("speed"))
    axes[3].legend(loc="best", fontsize=8)
    axes[3].grid(True, alpha=0.3)

    axes[4].set_ylabel(T("ctrl_surface_angles"))
    axes[4].set_title(T("ctrl_surfaces"))
    axes[4].legend(loc="best", ncol=2, fontsize=7)
    axes[4].grid(True, alpha=0.3)

    axes[5].set_ylabel(T("cum_depth_err"))
    ax5b.set_ylabel(T("cum_hdg_err"))
    axes[5].set_xlabel(T("time_s"))
    axes[5].set_title(T("cumulative_errors"))
    axes[5].grid(True, alpha=0.3)

    lines_a, labels_a = axes[5].get_legend_handles_labels()
    lines_b, labels_b = ax5b.get_legend_handles_labels()
    axes[5].legend(lines_a + lines_b, labels_a + labels_b,
                   loc="best", fontsize=7, ncol=2)

    plt.tight_layout(rect=(0, 0, 1, 0.975))
    path = f"{out}/s6_kompleksa_trajektorija{suffix}.png"
    plt.savefig(path, dpi=200)
    print(f"    Saved: {path}")
    plt.close()

    # ---- horizontal trajectory (top view, illustrative only) ------------
    # The mission defines depth/heading/speed references, NOT an x/y route,
    # so no desired x/y trajectory exists and none is constructed here.
    # x/y is not fed back to either controller; the current drift is
    # identical for both, so path differences reflect heading/speed
    # tracking differences.  Evaluation metrics remain depth/heading.
    r_nmpc = results[-1]
    t_grid = np.asarray(r_nmpc.time, float)
    dt_s = float(t_grid[1] - t_grid[0]) if len(t_grid) > 1 else 0.02

    fig, ax = plt.subplots(figsize=(9, 8.5))
    for r in results:
        if r.controller_name == "PID (5 Hz)":
            continue  # visually indistinguishable from PID (tuned) at this scale
        c = _color(r.controller_name)
        ax.plot(r.eta[:, 1], r.eta[:, 0], color=c, lw=1.6, label=r.controller_name)
        ax.scatter([r.eta[-1, 1]], [r.eta[-1, 0]], color=c, s=42, zorder=5,
                   edgecolor="white")
    first_switch = True
    for ts in switch_times:
        i = min(int(ts / dt_s), len(t_grid) - 1)
        ax.scatter([r_nmpc.eta[i, 1]], [r_nmpc.eta[i, 0]], color="k", s=14,
                   zorder=6, label=T("seg_switches") if first_switch else None)
        first_switch = False
    ax.scatter([0], [0], marker="s", color="k", s=55, zorder=6, label=T("start"))
    ax.set_xlabel(T("east_m"))
    ax.set_ylabel(T("north_m"))
    ax.set_title(T("s6_xy_title"), fontsize=13, fontweight="bold")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8.5)
    plt.tight_layout()
    path_xy = f"{out}/s6_trajektorija_xy{suffix}.png"
    plt.savefig(path_xy, dpi=200)
    print(f"    Saved: {path_xy}")
    plt.close()


def main():
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "results")
    os.makedirs(out, exist_ok=True)

    variant = "patched offset-free (N=12)" if USE_PATCHED_NMPC else "original (N=12)"
    print("\n" + "#" * 70)
    print("#  REMUS 100 — THREE-WAY COMPARISON")
    print(f"#  Fossen PID/SMC  vs  tuned PID  vs  NMPC [{variant}]")
    print("#" * 70)

    scenario_1(out)
    try:
        scenario_2(out)
        scenario_3(out)
        scenario_4(out)
        scenario_5(out)
        scenario_6_complex(out)
    except Exception as e:
        print(f"\n  Error: {e}")
        import traceback
        traceback.print_exc()

    write_scenario_metrics_csv(out)

    print("\n" + "=" * 70 + "\n  ALL SCENARIOS COMPLETE\n  Results: " + out +
          "\n" + "=" * 70)


if __name__ == "__main__":
    main()
