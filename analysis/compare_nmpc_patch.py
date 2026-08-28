"""
compare_nmpc_patch.py  —  A/B test: original vs patched NMPC
=============================================================

Reruns the EXACT Scenario-6 mission (same waypoints, switch times, and current
as ``experiments.run_remus100_comparison.scenario_6_complex``) with three
controllers:

    * PID (tuned)                 — the tuned baseline, for reference
    * NMPC traj. (N=20)           — the ORIGINAL controller (controllers.nmpc_remus)
    * NMPC offset-free (N=30)     — the PATCHED controller (controllers.nmpc_remus_patched)

and quantifies whether the patch shrinks the two transient failures:

    reversal overshoot   ~58 deg in the 500–600 s segment
    depth-coupling drift ~31 deg in the 260–340 s segment

All figure text uses config.T() for language toggling.

Run:
    python -m analysis.compare_nmpc_patch
    python -m analysis.compare_nmpc_patch --quick    # to t=360 s (depth-coupling only)
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from analysis import metrics as M
from analysis import plotting as P
from analysis import report as R
from config import T

D2R = math.pi / 180.0
R2D = 180.0 / math.pi
OUTDIR = os.path.join(ROOT, "results", "advanced")

# ---- exact Scenario-6 mission ----
WAYPOINTS = [(0, 0), (15, 0), (25, 90), (40, 90), (40, 200),
             (10, 200), (30, 330), (45, 45), (20, 0)]
SWITCH = [5, 60, 120, 180, 260, 340, 420, 500]
T_FINAL_FULL = 600.0
V_C, BETA_C = 0.6, 150.0

WIN_DEPTH    = (260.0, 340.0)
WIN_REVERSAL = (500.0, 600.0)

# Stable internal names for colour lookup (distinct colours for before/after)
COL = {
    "PID (tuned)":             "#2e86ab",
    "NMPC traj. (N=20)":       "#1b998b",
    "NMPC offset-free (N=30)": "#e8893a",
    "reference":               "#5a5a66",
}


def _ssa_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _turn_labels(bounds):
    labels = []
    for k in range(len(bounds) - 1):
        if k == 0:
            labels.append(T("seg_holding"))
            continue
        turn = _ssa_deg(WAYPOINTS[k][1] - WAYPOINTS[k - 1][1])
        if abs(turn) < 1e-6:
            labels.append(T("seg_holding"))
        else:
            labels.append(f"{T('seg_turn')} {turn:+.0f}°")
    return labels


# --------------------------------------------------------------------------- #
#  Run the three controllers on the mission
# --------------------------------------------------------------------------- #
def _run(t_final: float):
    """Return ref + per-controller heading series. Needs casadi."""
    try:
        import casadi  # noqa: F401
    except Exception as exc:
        raise SystemExit(
            "\n[compare_nmpc_patch] casadi is not installed.\n"
            f"  Fix: pip install casadi  (import error: {exc})\n")
    try:
        from adapters.fossen_adapter import FossenVehicleAdapter
        from experiments.run_remus100_comparison import (
            make_pid, wrap_pid, wrap_nmpc, smooth_ref,
            heading_continuous_deg, heading_error_deg,
        )
        from controllers.nmpc_remus import NMPC_REMUS100
        from controllers.nmpc_remus_patched import NMPC_REMUS100_Patched
        from python_vehicle_simulator.vehicles.remus100 import remus100
    except Exception as exc:
        raise SystemExit(f"[compare_nmpc_patch] framework import failed: {exc}")

    ref = smooth_ref(WAYPOINTS, SWITCH, tau_rise=12.0)
    runs = {}

    print("  PID (tuned) ...")
    pid = make_pid()
    a = FossenVehicleAdapter(V_current=V_C, beta_current=BETA_C)
    r_pid = a.run(t_final, wrap_pid(pid), ref, sampleTime=0.02)

    print("  NMPC traj. (N=20)  [original] ...")
    veh0 = remus100("stepInput", V_current=V_C, beta_current=BETA_C)
    nmpc0 = NMPC_REMUS100(veh0, N=20, n_rpm=1525)
    nmpc0.set_current_estimate(V_C, BETA_C * D2R)
    a0 = FossenVehicleAdapter(V_current=V_C, beta_current=BETA_C)
    r0 = a0.run(t_final, wrap_nmpc(nmpc0, ref), ref, sampleTime=0.02)

    print("  NMPC offset-free (N=30)  [patched] ...")
    veh1 = remus100("stepInput", V_current=V_C, beta_current=BETA_C)
    nmpc1 = NMPC_REMUS100_Patched(veh1, N=30, n_rpm=1525)
    nmpc1.set_current_estimate(V_C, BETA_C * D2R)
    a1 = FossenVehicleAdapter(V_current=V_C, beta_current=BETA_C)
    r1 = a1.run(t_final, wrap_nmpc(nmpc1, ref), ref, sampleTime=0.02)

    t = np.asarray(r_pid.time, float)
    ref_cont = heading_continuous_deg(r_pid.eta_d[:, 5], anchor_deg=0.0)

    solve = {}
    for name, res, ctrl in [
        ("PID (tuned)",             r_pid, pid),
        ("NMPC traj. (N=20)",       r0,   nmpc0),
        ("NMPC offset-free (N=30)", r1,   nmpc1),
    ]:
        runs[name] = {
            "t": t,
            "heading_cont_deg": heading_continuous_deg(res.eta[:, 5], anchor_deg=0.0),
            "err_deg": np.array([
                heading_error_deg(res.eta[i, 5], res.eta_d[i, 5])
                for i in range(len(t))
            ], float),
        }
        solve[name] = list(getattr(ctrl, "solve_times", []))

    return t, ref_cont, runs, solve


# --------------------------------------------------------------------------- #
#  Metrics
# --------------------------------------------------------------------------- #
def _compute_metrics(t, ref_cont_deg, runs, bounds):
    ref_rad = ref_cont_deg * D2R
    changing = [WAYPOINTS[i + 1][1] != WAYPOINTS[i][1]
                for i in range(len([s for s in SWITCH if s < bounds[-1]]))]
    active_switch = [s for s in SWITCH if s < bounds[-1]]

    settled_mask = np.zeros(len(t), bool)
    for a, b in zip(bounds[:-1], bounds[1:]):
        settled_mask |= (t >= a + 0.6 * (b - a)) & (t < b)

    regimes, agg, seg = {}, {}, {}
    for name, d in runs.items():
        sig_rad = d["heading_cont_deg"] * D2R
        e = M.angle_error(sig_rad, ref_rad) * R2D
        tm = M.tracking_metrics(t, sig_rad, ref_rad, angular=True)
        sp = M.split_transient_steady(t, sig_rad, ref_rad, active_switch,
                                      angular=True, transient_window=35.0,
                                      only_changing=changing)
        agg[name] = {"rmse": tm.rmse * R2D, "mae": tm.mae * R2D,
                     "max": tm.max_abs * R2D, "iae": tm.iae * R2D}
        regimes[name] = {
            T("regime_aggregate"): float(np.sqrt(np.mean(e ** 2))),
            T("regime_transient"): sp["transient_rmse"] * R2D,
            T("regime_settled"):   float(np.sqrt(np.mean(e[settled_mask] ** 2))),
        }
        rows = M.segment_decompose(t, sig_rad, ref_rad, bounds, angular=True,
                                   steady_frac=0.4)
        for rr in rows:
            for k in ("iae", "mae", "rmse", "max_abs", "steady_state"):
                rr[k] *= R2D
        seg[name] = rows
    return agg, regimes, seg


def _window_peak(t, runs, window):
    a, b = window
    mask = (t >= a) & (t < b)
    out = {}
    for name, d in runs.items():
        out[name] = float(np.max(np.abs(d["err_deg"][mask]))) if mask.any() else float("nan")
    return out


# --------------------------------------------------------------------------- #
#  Before/after zoom overlay
# --------------------------------------------------------------------------- #
def _zoom_overlay(t, ref_cont_deg, runs, window, path, *, title):
    P.setup_style()
    import matplotlib.pyplot as plt
    a, b = window
    mask = (t >= a) & (t < b)
    fig, (ax_h, ax_e) = plt.subplots(2, 1, figsize=(11, 7.2), sharex=True,
                                     gridspec_kw={"height_ratios": [2, 1]})
    ax_h.plot(t[mask], ref_cont_deg[mask], color=COL["reference"], lw=1.6,
              ls="--", label=T("reference"))
    for name in ("NMPC traj. (N=20)", "NMPC offset-free (N=30)"):
        if name not in runs:
            continue
        ax_h.plot(t[mask], runs[name]["heading_cont_deg"][mask],
                  color=COL[name], lw=2.0, label=name)
    ax_h.set_ylabel(T("hdg_continuous"))
    ax_h.set_title(title, fontsize=12, fontweight="bold")
    ax_h.legend(loc="best")

    for name in ("NMPC traj. (N=20)", "NMPC offset-free (N=30)"):
        if name not in runs:
            continue
        e = np.abs(runs[name]["err_deg"][mask])
        ax_e.plot(t[mask], e, color=COL[name], lw=1.8, label=name)
        pk_i = int(np.argmax(e))
        ax_e.annotate(f"max {e[pk_i]:.1f}°",
                      xy=(t[mask][pk_i], e[pk_i]),
                      xytext=(8, 6), textcoords="offset points",
                      fontsize=9, color=COL[name], fontweight="bold")
    ax_e.set_ylabel(T("hdg_err_abs"))
    ax_e.set_xlabel(T("time_s"))
    ax_e.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  Main
# --------------------------------------------------------------------------- #
def main(argv=None):
    p = argparse.ArgumentParser(description="A/B test original vs patched NMPC.")
    p.add_argument("--quick", action="store_true",
                   help="stop at t=360 s (covers depth-coupling only)")
    p.add_argument("--out", default=OUTDIR)
    args = p.parse_args(argv)

    os.makedirs(args.out, exist_ok=True)
    t_final = 360.0 if args.quick else T_FINAL_FULL
    bounds = [0.0] + [s for s in SWITCH if s < t_final] + [t_final]

    print(f"[compare_nmpc_patch] running mission to t={t_final:.0f}s "
          f"(current {V_C} m/s @ {BETA_C}°)")
    t, ref_cont, runs, solve = _run(t_final)

    agg, regimes, seg = _compute_metrics(t, ref_cont, runs, bounds)
    peak_depth = _window_peak(t, runs, WIN_DEPTH)
    peak_rev   = _window_peak(t, runs, WIN_REVERSAL) if not args.quick else None

    # ---- 3-way regime + segment figures ----
    f_regime = P.plot_regime_comparison(
        regimes, os.path.join(args.out, "patch_regime_comparison.png"),
        title="Heading RMSE by regime: original vs. patched NMPC")
    f_seg = P.plot_segment_breakdown(
        seg, os.path.join(args.out, "patch_segment_breakdown.png"),
        turn_labels=_turn_labels(bounds), metric="iae", unit="°·s")

    figs = [f_regime, f_seg]
    f_depth = _zoom_overlay(
        t, ref_cont, runs, WIN_DEPTH,
        os.path.join(args.out, "patch_zoom_depthcoupling.png"),
        title="Depth-induced heading excursion (260–340 s): original vs. patched")
    figs.append(f_depth)
    if not args.quick:
        f_rev = _zoom_overlay(
            t, ref_cont, runs, WIN_REVERSAL,
            os.path.join(args.out, "patch_zoom_reversal.png"),
            title="Sharp reversal overshoot (500–600 s): original vs. patched")
        figs.append(f_rev)

    # ---- save time-series CSV for thesis_analysis.py ----
    ts_path = os.path.join(args.out, "patch_compare_timeseries.csv")
    import csv as _csv
    with open(ts_path, "w", newline="", encoding="utf-8") as fh:
        w = _csv.writer(fh)
        names = list(runs)
        w.writerow(["time_s", "reference_cont_deg"]
                   + [f"{n}_cont_deg" for n in names]
                   + [f"{n}_err_deg" for n in names])
        for i in range(len(t)):
            w.writerow([f"{t[i]:.3f}", f"{ref_cont[i]:.6f}"]
                       + [f"{runs[n]['heading_cont_deg'][i]:.6f}" for n in names]
                       + [f"{runs[n]['err_deg'][i]:.6f}" for n in names])

    # ---- summary table ----
    o = "NMPC traj. (N=20)"
    q = "NMPC offset-free (N=30)"
    pid = "PID (tuned)"

    def _red(orig, new):
        return (1.0 - new / orig) * 100.0 if orig else float("nan")

    rows = []
    rows.append({T("col_metric"): "Aggregate heading RMSE [°]",
                 "PID": agg[pid]["rmse"],
                 "NMPC_orig": agg[o]["rmse"], "NMPC_patch": agg[q]["rmse"],
                 T("col_improvement"): _red(agg[o]["rmse"], agg[q]["rmse"])})
    rows.append({T("col_metric"): "Settled heading RMSE [°]",
                 "PID": regimes[pid][T("regime_settled")],
                 "NMPC_orig": regimes[o][T("regime_settled")],
                 "NMPC_patch": regimes[q][T("regime_settled")],
                 T("col_improvement"): _red(regimes[o][T("regime_settled")],
                                            regimes[q][T("regime_settled")])})
    rows.append({T("col_metric"): "Depth-coupling max|e| (260–340 s) [°]",
                 "PID": peak_depth[pid], "NMPC_orig": peak_depth[o],
                 "NMPC_patch": peak_depth[q],
                 T("col_improvement"): _red(peak_depth[o], peak_depth[q])})
    if peak_rev is not None:
        rows.append({T("col_metric"): "Reversal overshoot max|e| (500–600 s) [°]",
                     "PID": peak_rev[pid], "NMPC_orig": peak_rev[o],
                     "NMPC_patch": peak_rev[q],
                     T("col_improvement"): _red(peak_rev[o], peak_rev[q])})

    cols = [T("col_metric"), "PID", "NMPC_orig", "NMPC_patch", T("col_improvement")]
    csv_path = R.write_csv(rows, cols, os.path.join(args.out, "patch_compare.csv"))

    # solver timing
    timing_note = ""
    if solve.get(q):
        tinfo = M.solver_timing(solve[q], dt_control=0.2)
        timing_note = (f"Patched NMPC (N=30) solver time: mean {tinfo['mean_ms']:.1f} ms, "
                       f"P99 {tinfo['p99_ms']:.1f} ms, max {tinfo['max_ms']:.1f} ms; "
                       f"{100*tinfo['frac_realtime']:.1f}% of solves within deadline (200 ms).")

    md = R.write_markdown_report(
        os.path.join(args.out, "patch_compare.md"),
        "NMPC patch A/B comparison (Scenario 6)",
        [("Summary",
          "Identical Scenario-6 mission with three controllers. The patched NMPC adds "
          "an offset-free yaw disturbance observer (for depth-coupling drift), a heavier "
          "terminal yaw-rate penalty, and a longer horizon (for reversal overshoot).\n\n"
          + timing_note),
         ("Results", R.metrics_table_md(rows, cols)),
         ("Note",
          "Positive 'Improvement [%]' means the patch reduced the metric. "
          "All values use the same definitions as `analyze_s6.py`. "
          "Figures in `results/advanced/` with prefix `patch_`.")])

    # ---- console summary ----
    print("\n================  RESULTS  ================")
    print(f"  {T('col_metric'):42s} {'orig':>8s} {'patch':>8s} {'Δ%':>7s}")
    for r in rows:
        print(f"  {r[T('col_metric')]:42s} {r['NMPC_orig']:8.2f} "
              f"{r['NMPC_patch']:8.2f} {r[T('col_improvement')]:6.1f}%")
    if timing_note:
        print("\n  " + timing_note)
    print(f"\n  Figures + tables -> {args.out}")
    print("  Figures:", ", ".join(os.path.basename(f) for f in figs))
    print(f"  Tables: {os.path.basename(csv_path)}, patch_compare.md, "
          f"{os.path.basename(ts_path)}")


if __name__ == "__main__":
    main()
