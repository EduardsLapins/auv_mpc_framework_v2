"""
run_advanced_experiments.py  —  ADVANCED EXPERIMENT HARNESS (RUN LOCALLY)
=========================================================================

PURPOSE
-------
The thesis presents *single deterministic runs* per scenario.  For a master's
thesis the three weakest methodological points are:

  (1) no statistical treatment of disturbances — one current realisation only,
      so the reader cannot tell a real performance gap from a lucky seed;
  (2) the real-time claim for NMPC (5 Hz, IPOPT) is asserted but never measured;
  (3) the prediction horizon N=20 is fixed with no accuracy/compute trade-off
      study to justify it.

This script adds the three experiments that close those gaps.  Each one uses
the *existing* framework controllers and the *real* Fossen 6-DOF REMUS 100
dynamics, so its outputs are scientifically consistent with the figures already
in the thesis (unlike a reduced re-simulation would be).

  EXP-A  Monte-Carlo robustness  -> metric distributions + paired statistics
  EXP-B  NMPC solver timing       -> real-time feasibility against the deadline
  EXP-C  Horizon sweep            -> accuracy vs compute Pareto front

-------------------------------------------------------------------------------
!! WHY THIS IS A SEPARATE, RUN-LOCALLY FILE !!

The closed-loop simulations require two packages that are NOT importable in the
review sandbox where the analysis library was developed:

    casadi                       (NMPC optimiser, IPOPT backend)
    python_vehicle_simulator     (Fossen REMUS 100 dynamics)

Therefore every framework import in this file is deliberately placed *inside*
the functions, never at module top level.  The module itself imports only
numpy + the analysis package, so it loads cleanly anywhere; the heavy imports
fire only when you actually run an experiment in an environment that has
casadi and the Fossen simulator installed (i.e. the same environment that
produced the existing thesis figures).

USAGE (in your local environment)
---------------------------------
    pip install casadi
    # python_vehicle_simulator must be importable (as it was for your figures)

    python -m analysis.run_advanced_experiments --all
    python -m analysis.run_advanced_experiments --montecarlo --seeds 40
    python -m analysis.run_advanced_experiments --timing
    python -m analysis.run_advanced_experiments --horizon
    python -m analysis.run_advanced_experiments --all --quick   # fast smoke test

All figures and tables are written to results/advanced/ and are in Latvian,
ready to drop into the thesis.  Re-using --seeds N with the same N reproduces
the identical disturbance set (the seeds are fixed integers 0..N-1).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

# Analysis package is pure numpy/scipy/matplotlib -> always importable.
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from analysis import metrics, plotting, report

R2D = 180.0 / math.pi
D2R = math.pi / 180.0
OUT_DEFAULT = os.path.join(ROOT, "results", "advanced")


# --------------------------------------------------------------------------- #
#  Framework availability guard
# --------------------------------------------------------------------------- #
def _require_framework():
    """Import the framework lazily and fail with a clear, actionable message.

    Returns a small namespace object exposing the framework callables the
    experiments need.  Raising here (rather than at import time) is what lets
    this module be imported in an environment without casadi.
    """
    try:
        import casadi  # noqa: F401
    except Exception as exc:  # pragma: no cover - environment dependent
        raise SystemExit(
            "\n[run_advanced_experiments] casadi is not installed.\n"
            "  These experiments drive the real NMPC + Fossen dynamics, so they\n"
            "  must run where the thesis figures were produced.\n"
            "  Fix:  pip install casadi\n"
            f"  (import error: {exc})\n"
        )
    try:
        from adapters.fossen_adapter import FossenVehicleAdapter
        from experiments.run_remus100_comparison import (
            make_nmpc, make_pid, wrap_nmpc, wrap_pid, smooth_ref,
        )
    except Exception as exc:  # pragma: no cover - environment dependent
        raise SystemExit(
            "\n[run_advanced_experiments] could not import the framework.\n"
            "  Make sure python_vehicle_simulator is importable (the same\n"
            "  package that produced your existing results).\n"
            f"  (import error: {exc})\n"
        )

    class _FW:
        pass

    fw = _FW()
    fw.FossenVehicleAdapter = FossenVehicleAdapter
    fw.make_nmpc = make_nmpc
    fw.make_pid = make_pid
    fw.wrap_nmpc = wrap_nmpc
    fw.wrap_pid = wrap_pid
    fw.smooth_ref = smooth_ref
    return fw


# --------------------------------------------------------------------------- #
#  Time-varying disturbance generator  (pure numpy -> safe at module level)
# --------------------------------------------------------------------------- #
def gauss_markov_current(
    seed: int,
    t_final: float,
    *,
    dt: float = 0.02,
    V_mean: float = 0.5,
    V_sigma: float = 0.25,
    tau: float = 60.0,
    V_max: float = 1.2,
    wave_amp: float = 0.12,
    wave_period: float = 9.0,
    beta_mean_deg: float = 170.0,
    beta_sigma_deg: float = 25.0,
    beta_tau: float = 90.0,
):
    """Deterministic-from-seed time-varying ocean current.

    Builds a realistic disturbance that the thesis' fixed (V_c, beta_c) cannot:
      * current SPEED  : first-order Gauss-Markov process (mean-reverting random
        walk, correlation time ``tau``) clipped to [0, V_max];
      * WAVE component : a sinusoidal orbital-velocity term added on the speed,
        modelling shallow-water wave forcing (thesis §1.4);
      * current HEADING: a slower Gauss-Markov process about ``beta_mean_deg``.

    The whole realisation is pre-sampled on the simulator's time grid from a
    single seeded RNG, so it is fully reproducible and independent of how often
    the integrator queries it.  Returns ``disturbance_fn(t) -> (V_c, beta_deg)``
    suitable for ``FossenVehicleAdapter.run(disturbance_fn=...)``.

    Rationale: a mean-reverting (Ornstein-Uhlenbeck-type) model is the standard
    way to represent slowly varying ocean currents in marine control studies,
    while the additive sinusoid captures the periodic wave disturbance the
    thesis describes but never injects.
    """
    rng = np.random.default_rng(int(seed))
    n = int(round(t_final / dt)) + 2
    t_grid = np.arange(n) * dt

    # --- Gauss-Markov speed ---------------------------------------------------
    a_v = math.exp(-dt / tau)
    q_v = V_sigma * math.sqrt(max(0.0, 1.0 - a_v * a_v))
    V = np.empty(n)
    V[0] = V_mean
    noise_v = rng.standard_normal(n)
    for k in range(1, n):
        V[k] = V_mean + a_v * (V[k - 1] - V_mean) + q_v * noise_v[k]
    # additive wave orbital velocity (random phase per seed)
    phase = rng.uniform(0.0, 2.0 * math.pi)
    V = V + wave_amp * np.sin(2.0 * math.pi * t_grid / wave_period + phase)
    V = np.clip(V, 0.0, V_max)

    # --- Gauss-Markov heading -------------------------------------------------
    a_b = math.exp(-dt / beta_tau)
    q_b = beta_sigma_deg * math.sqrt(max(0.0, 1.0 - a_b * a_b))
    B = np.empty(n)
    B[0] = beta_mean_deg
    noise_b = rng.standard_normal(n)
    for k in range(1, n):
        B[k] = beta_mean_deg + a_b * (B[k - 1] - beta_mean_deg) + q_b * noise_b[k]

    def disturbance_fn(t: float):
        idx = int(t / dt)
        if idx < 0:
            idx = 0
        elif idx >= n:
            idx = n - 1
        return float(V[idx]), float(B[idx])

    # expose the realisation for plotting / inspection
    disturbance_fn.t_grid = t_grid
    disturbance_fn.V = V
    disturbance_fn.beta_deg = B
    disturbance_fn.mean_speed = float(np.mean(V))
    return disturbance_fn


# --------------------------------------------------------------------------- #
#  Shared mission used by the robustness study
# --------------------------------------------------------------------------- #
def _robustness_mission(fw):
    """A depth+heading acquisition with a mid-mission heading change.

    Returns (t_final, z_d, psi_d_deg, switch_times, only_changing, ref_fn).
    Kept short enough that a 40-seed sweep is feasible on a laptop while still
    exercising one acquisition transient and one manoeuvre transient.
    """
    t_final = 220.0
    z_d, psi_d = 30.0, 60.0
    # targets are (depth_m, heading_deg); switch_times are absolute [s].
    #   t=2   : begin acquiring 30 m / 60 deg   (acquisition transient)
    #   t=160 : change heading 60 -> 200 deg    (manoeuvre transient)
    ref = fw.smooth_ref([(0, 0), (z_d, psi_d), (z_d, 200)],
                        [2.0, 160.0], tau_rise=12.0)
    switch_times = [2.0, 160.0]
    only_changing = [True, True]
    return t_final, z_d, psi_d, switch_times, only_changing, ref


def _extract_signals(result):
    """Pull (t, z, z_ref, psi_rad, psi_ref_rad) arrays from a SimulationResult."""
    t = np.asarray(result.time, float)
    z = np.asarray(result.eta[:, 2], float)
    psi = np.asarray(result.eta[:, 5], float)
    z_ref = np.asarray(result.eta_d[:, 2], float)
    psi_ref = np.asarray(result.eta_d[:, 5], float)
    return t, z, z_ref, psi, psi_ref


# --------------------------------------------------------------------------- #
#  EXP-A : Monte-Carlo robustness
# --------------------------------------------------------------------------- #
def run_monte_carlo(n_seeds: int, out_dir: str, *, quick: bool = False) -> dict:
    """Run PID and NMPC across ``n_seeds`` randomised current realisations.

    Produces, per controller, the *distribution* of depth-RMSE, heading-RMSE
    and settled heading-RMSE, then a paired statistical comparison (same seed
    drives both controllers, so the test is paired).  This is the single most
    important addition for the thesis: it converts "NMPC looks better in one
    run" into a defensible statistical statement with a confidence interval
    and an effect size.
    """
    fw = _require_framework()
    os.makedirs(out_dir, exist_ok=True)
    if quick:
        n_seeds = min(n_seeds, 4)

    t_final, z_d, psi_d, switch_times, only_changing, ref = _robustness_mission(fw)
    print(f"[EXP-A] Monte-Carlo robustness: {n_seeds} seeds, mission {t_final:.0f}s")

    # Keyed by the controller names the runs actually report (PID first,
    # NMPC second — insertion order is relied on below for the paired stats).
    acc: dict = {}

    for s in range(n_seeds):
        dist = gauss_markov_current(s, t_final)
        print(f"  seed {s:2d}  mean current {dist.mean_speed:.2f} m/s ... ", end="", flush=True)

        # --- PID ---
        pid = fw.make_pid()
        a_pid = fw.FossenVehicleAdapter()
        r_pid = a_pid.run(t_final, fw.wrap_pid(pid), ref, sampleTime=0.02,
                          disturbance_fn=dist)
        r_pid.controller_name = "PID (pielāgots)"

        # --- NMPC (told the *mean* current, not the realisation -> honest) ---
        nmpc = fw.make_nmpc(dist.mean_speed, 170.0)
        a_nmpc = fw.FossenVehicleAdapter()
        r_nmpc = a_nmpc.run(t_final, fw.wrap_nmpc(nmpc, ref), ref, sampleTime=0.02,
                            disturbance_fn=dist)
        r_nmpc.controller_name = nmpc.name

        for r in (r_pid, r_nmpc):
            t, z, z_ref, psi, psi_ref = _extract_signals(r)
            zm = metrics.tracking_metrics(t, z, z_ref)
            pm = metrics.tracking_metrics(t, psi, psi_ref, angular=True)
            split = metrics.split_transient_steady(
                t, psi, psi_ref, switch_times, angular=True,
                only_changing=only_changing, transient_window=35.0)
            d = acc.setdefault(r.controller_name, {
                "z_rmse": [], "psi_rmse": [], "psi_settled": [], "psi_max": []})
            d["z_rmse"].append(zm.rmse)
            d["psi_rmse"].append(pm.rmse * R2D)
            d["psi_max"].append(pm.max_abs * R2D)
            d["psi_settled"].append(split["steady_rmse"] * R2D)
        print("done")

    # ---- summarise + paired stats -------------------------------------------
    names = list(acc)
    pid_name, nmpc_name = names
    rows = []
    for metric_key, label, unit in [
        ("z_rmse", "Dziļuma RMSE", "m"),
        ("psi_rmse", "Kursa RMSE", "°"),
        ("psi_settled", "Miera stāvokļa kursa RMSE", "°"),
        ("psi_max", "Kursa max|e|", "°"),
    ]:
        for nm in names:
            st = metrics.summarize_runs(acc[nm][metric_key])
            rows.append({
                "Metrika": f"{label} [{unit}]",
                "Kontrolieris": nm,
                "Vidējais": st["mean"],
                "Std": st["std"],
                "Mediāna": st["median"],
                "CI_zem": st["ci_low"],
                "CI_virs": st["ci_high"],
                "n": st["n"],
            })

    cmp_psi = metrics.paired_comparison(
        acc[pid_name]["psi_rmse"],
        acc[nmpc_name]["psi_rmse"],
        labels=("PID", "NMPC"))
    cmp_settled = metrics.paired_comparison(
        acc[pid_name]["psi_settled"],
        acc[nmpc_name]["psi_settled"],
        labels=("PID", "NMPC"))

    # ---- figures ------------------------------------------------------------
    f_rmse = plotting.plot_metric_boxes(
        {nm: acc[nm]["psi_rmse"] for nm in names},
        os.path.join(out_dir, "mc_heading_rmse_box.png"),
        ylabel="Kursa RMSE [°]",
        title=f"Kursa RMSE sadalījums {n_seeds} testos ar nejaušiem trokšņiem")
    f_settled = plotting.plot_metric_boxes(
        {nm: acc[nm]["psi_settled"] for nm in names},
        os.path.join(out_dir, "mc_heading_settled_box.png"),
        ylabel="Miera stāvokļa kursa RMSE [°]",
        title=f"Miera stāvokļa precizitāte {n_seeds} testos ar nejaušiem trokšņiem")
    f_depth = plotting.plot_metric_boxes(
        {nm: acc[nm]["z_rmse"] for nm in names},
        os.path.join(out_dir, "mc_depth_rmse_box.png"),
        ylabel="Dziļuma RMSE [m]",
        title=f"Dziļuma RMSE sadalījums {n_seeds} testos ar nejaušiem trokšņiem")

    # ---- tables -------------------------------------------------------------
    csv_path = report.write_csv(
        rows,
        ["Metrika", "Kontrolieris", "Vidējais", "Std", "Mediāna",
         "CI_zem", "CI_virs", "n"],
        os.path.join(out_dir, "mc_summary.csv"))

    def _fmt_cmp(c):
        return (f"vidējā starpība (PID−NMPC) = {c['mean_diff']:+.3f}, "
                f"Cohen dz = {c['cohen_dz']:.2f}, "
                f"paired-t p = {c['paired_t_p']:.2e}, "
                f"Wilcoxon p = {c['wilcoxon_p']:.2e} (n={c['n']})")

    body = (
        f"Veikti {n_seeds} testi ar neatkarīgiem, ar sēklu noteiktiem nejaušiem "
        f"straumes trokšņiem (Gausa–Markova ātrums + viļņu komponente + "
        f"Gausa–Markova virziens). Katrā testā abi kontrolieri saņem identisku "
        f"trokšņa realizāciju, tāpēc salīdzinājums ir pārī (paired).\n\n"
        f"- Kopējais kursa RMSE: {_fmt_cmp(cmp_psi)}.\n"
        f"- Miera stāvokļa kursa RMSE: {_fmt_cmp(cmp_settled)}.\n\n"
        f"Pozitīva vidējā starpība nozīmē, ka PID kļūda ir lielāka (NMPC labāks); "
        f"negatīva — pretēji. |Cohen dz| ≳ 0,8 norāda lielu efektu.\n"
    )
    md_path = report.write_markdown_report(
        os.path.join(out_dir, "mc_results.md"),
        "EXP-A — Montekarlo noturības analīze",
        [("Kopsavilkums", body),
         ("Metriku tabula", report.metrics_table_md(
             rows,
             ["Metrika", "Kontrolieris", "Vidējais", "Std", "Mediāna",
              "CI_zem", "CI_virs"]))])

    print(f"[EXP-A] heading RMSE: {_fmt_cmp(cmp_psi)}")
    print(f"[EXP-A] settled RMSE: {_fmt_cmp(cmp_settled)}")
    print(f"[EXP-A] wrote {f_rmse}, {f_settled}, {f_depth}, {csv_path}, {md_path}")
    return {"rows": rows, "paired_psi": cmp_psi, "paired_settled": cmp_settled,
            "figures": [f_rmse, f_settled, f_depth], "csv": csv_path, "md": md_path}


# --------------------------------------------------------------------------- #
#  EXP-B : NMPC solver timing / real-time feasibility
# --------------------------------------------------------------------------- #
def run_compute_cost(out_dir: str, *, quick: bool = False) -> dict:
    """Measure the NMPC per-step solve time and test it against the deadline.

    The thesis claims a 5 Hz NMPC (dt_mpc = 0.2 s) runs in real time.  Whether
    that holds is governed by the WORST-CASE solve, not the mean.  Here we run
    one full mission, harvest the controller's logged ``solve_times`` and report
    the fraction of solves that beat the 0.2 s deadline plus the worst overrun.
    """
    fw = _require_framework()
    os.makedirs(out_dir, exist_ok=True)

    t_final, z_d, psi_d, switch_times, only_changing, ref = _robustness_mission(fw)
    if quick:
        t_final = 80.0
        ref = fw.smooth_ref([(0, 0), (z_d, psi_d), (z_d, 200)], [2.0, 40.0],
                            tau_rise=12.0)
    print(f"[EXP-B] solver timing on a {t_final:.0f}s mission")

    dist = gauss_markov_current(0, t_final)
    nmpc = fw.make_nmpc(dist.mean_speed, 170.0)
    dt_mpc = float(getattr(nmpc, "dt_mpc", 0.2))
    a = fw.FossenVehicleAdapter()
    r = a.run(t_final, fw.wrap_nmpc(nmpc, ref, dt_mpc=dt_mpc), ref,
              sampleTime=0.02, disturbance_fn=dist)
    r.controller_name = nmpc.name

    solve_times = list(getattr(nmpc, "solve_times", []))
    if not solve_times:
        raise SystemExit("[EXP-B] no solve_times logged — NMPC did not solve?")

    timing = metrics.solver_timing(solve_times, dt_control=dt_mpc)
    fig = plotting.plot_solver_timing(
        solve_times, deadline_ms=dt_mpc * 1e3,
        path=os.path.join(out_dir, "nmpc_solver_timing.png"))

    rows = [{
        "Mērvienība": "ms",
        "Vidējais": timing["mean_ms"],
        "Mediāna": timing["median_ms"],
        "P95": timing["p95_ms"],
        "P99": timing["p99_ms"],
        "Maks.": timing["max_ms"],
        "Periods": timing["deadline_ms"],
        "Daļa_periodā": timing["frac_realtime"],
        "Maks_pārsniegums": timing["worst_overrun_ms"],
    }]
    csv_path = report.write_csv(
        rows,
        ["Mērvienība", "Vidējais", "Mediāna", "P95", "P99", "Maks.",
         "Periods", "Daļa_periodā", "Maks_pārsniegums"],
        os.path.join(out_dir, "nmpc_solver_timing.csv"))

    print(f"[EXP-B] mean {timing['mean_ms']:.1f} ms, P99 {timing['p99_ms']:.1f} ms, "
          f"max {timing['max_ms']:.1f} ms, deadline {timing['deadline_ms']:.0f} ms, "
          f"{100*timing['frac_realtime']:.1f}% in real time")
    print(f"[EXP-B] wrote {fig}, {csv_path}")
    return {"timing": timing, "figure": fig, "csv": csv_path}


# --------------------------------------------------------------------------- #
#  EXP-C : horizon sweep  (accuracy vs compute Pareto)
# --------------------------------------------------------------------------- #
def run_horizon_sweep(out_dir: str, *, horizons=None, quick: bool = False) -> dict:
    """Sweep the NMPC prediction horizon N and chart accuracy vs compute.

    Justifies the choice of N=12: too short loses preview (worse tracking), too
    long costs compute with diminishing returns.  Plots heading RMSE against the
    P99 solve time so the knee of the trade-off is visible.
    """
    fw = _require_framework()
    os.makedirs(out_dir, exist_ok=True)
    if horizons is None:
        horizons = [8, 12, 16, 20, 30, 40]
    if quick:
        horizons = [8, 20, 40]

    t_final, z_d, psi_d, switch_times, only_changing, ref = _robustness_mission(fw)
    if quick:
        t_final = 120.0
        ref = fw.smooth_ref([(0, 0), (z_d, psi_d), (z_d, 200)], [2.0, 60.0],
                            tau_rise=12.0)
    print(f"[EXP-C] horizon sweep over N={horizons}")

    dist = gauss_markov_current(0, t_final)
    points = []   # (label, p99_ms, heading_rmse_deg)
    rows = []
    for N in horizons:
        # build an NMPC with this horizon (constructor takes N)
        from controllers.nmpc_remus import NMPC_REMUS100
        from python_vehicle_simulator.vehicles.remus100 import remus100
        veh = remus100("stepInput", V_current=dist.mean_speed, beta_current=170.0)
        nmpc = NMPC_REMUS100(veh, N=int(N), n_rpm=1525)
        nmpc.set_current_estimate(dist.mean_speed, 170.0 * D2R)
        dt_mpc = float(getattr(nmpc, "dt_mpc", 0.2))

        a = fw.FossenVehicleAdapter()
        r = a.run(t_final, fw.wrap_nmpc(nmpc, ref, dt_mpc=dt_mpc), ref,
                  sampleTime=0.02, disturbance_fn=dist)

        t, z, z_ref, psi, psi_ref = _extract_signals(r)
        pm = metrics.tracking_metrics(t, psi, psi_ref, angular=True)
        timing = metrics.solver_timing(getattr(nmpc, "solve_times", []),
                                       dt_control=dt_mpc)
        rmse_deg = pm.rmse * R2D
        p99 = timing.get("p99_ms", float("nan"))
        points.append((f"N={N}", p99, rmse_deg))
        rows.append({
            "N": N,
            "Kursa_RMSE_deg": rmse_deg,
            "P99_ms": p99,
            "Maks_ms": timing.get("max_ms", float("nan")),
            "Daļa_periodā": timing.get("frac_realtime", float("nan")),
        })
        print(f"  N={N:2d}: heading RMSE {rmse_deg:6.2f}°, P99 {p99:6.1f} ms, "
              f"{100*timing.get('frac_realtime', float('nan')):.0f}% real time")

    fig = plotting.plot_pareto(
        points, os.path.join(out_dir, "horizon_pareto.png"),
        xlabel="P99 risinātāja laiks [ms]", ylabel="Kursa RMSE [°]",
        title="Horizonta N kompromiss: precizitāte pret skaitļošanas izmaksām")
    csv_path = report.write_csv(
        rows, ["N", "Kursa_RMSE_deg", "P99_ms", "Maks_ms", "Daļa_periodā"],
        os.path.join(out_dir, "horizon_sweep.csv"))

    print(f"[EXP-C] wrote {fig}, {csv_path}")
    return {"points": points, "rows": rows, "figure": fig, "csv": csv_path}


# --------------------------------------------------------------------------- #
#  CLI
# --------------------------------------------------------------------------- #
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Advanced AUV-MPC experiments (run where casadi is installed).")
    p.add_argument("--all", action="store_true", help="run every experiment")
    p.add_argument("--montecarlo", action="store_true", help="EXP-A robustness")
    p.add_argument("--timing", action="store_true", help="EXP-B solver timing")
    p.add_argument("--horizon", action="store_true", help="EXP-C horizon sweep")
    p.add_argument("--seeds", type=int, default=30, help="Monte-Carlo seed count")
    p.add_argument("--out", default=OUT_DEFAULT, help="output directory")
    p.add_argument("--quick", action="store_true",
                   help="short missions / few seeds for a fast smoke test")
    args = p.parse_args(argv)

    if not (args.all or args.montecarlo or args.timing or args.horizon):
        p.print_help()
        return

    os.makedirs(args.out, exist_ok=True)
    if args.all or args.montecarlo:
        run_monte_carlo(args.seeds, args.out, quick=args.quick)
    if args.all or args.timing:
        run_compute_cost(args.out, quick=args.quick)
    if args.all or args.horizon:
        run_horizon_sweep(args.out, quick=args.quick)
    print("\n[run_advanced_experiments] done -> " + args.out)


if __name__ == "__main__":
    main()
