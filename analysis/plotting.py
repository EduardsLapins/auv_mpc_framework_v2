"""
Publication-quality figures for AUV control analysis
====================================================

A small plotting toolkit that produces thesis-ready figures with a consistent
visual identity.  All on-figure text is looked up via ``config.T(key)`` so that
switching between English and Latvian only requires changing ``LANGUAGE`` in
``config.py``.

Design choices
--------------
* One muted accent per controller, reused everywhere (legible, colour-blind safe).
* Generous white space, light gridlines, no chartjunk.
* Every figure that makes a *claim* annotates the number behind the claim
  directly on the axes, so a reader does not have to cross-reference a table.

The functions take generic inputs (arrays / dicts), not framework objects, so
they can be called from the experiment runner, from a notebook, or from the
``analyze_s6`` demonstration that ships with this package.
"""

from __future__ import annotations

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import Mapping, Sequence

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

from config import T

# --------------------------------------------------------------------------- #
#  Visual identity
# --------------------------------------------------------------------------- #
# Keys are stable English internal names; display names use T() in legends.
PALETTE = {
    "Fossen PID/SMC":   "#d1495b",   # muted red
    "PID (tuned)":      "#2e86ab",   # muted blue  (50 Hz)
    "PID (5 Hz)":       "#e67e22",   # orange — rate-matched to NMPC
    "NMPC":             "#1b998b",   # teal-green — the main (offset-free) NMPC
    "NMPC original":    "#7d5ba6",   # purple — original N=20 variant, so the
                                     # two NMPC curves are distinguishable
    "reference":        "#5a5a66",   # neutral grey
}
_DEFAULT = "#1b998b"


def controller_color(name: str) -> str:
    """Stable colour for a controller, matching prefixes like 'NMPC traj.'."""
    if name in PALETTE:
        return PALETTE[name]
    if name.upper().startswith("NMPC"):
        if "traj" in name:
            return PALETTE["NMPC original"]
        return PALETTE["NMPC"]
    if "5 Hz" in name and name.startswith("PID"):
        return PALETTE["PID (5 Hz)"]
    if "tuned" in name or name.startswith("PID"):
        return PALETTE["PID (tuned)"]
    if "Fossen" in name:
        return PALETTE["Fossen PID/SMC"]
    return _DEFAULT


def setup_style():
    """Apply the package matplotlib style. Call once before plotting."""
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#9aa0a6",
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "axes.axisbelow": True,
            "grid.color": "#dfe1e5",
            "grid.linewidth": 0.7,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.titlesize": 11,
            "axes.titleweight": "bold",
            "axes.labelsize": 10,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 8.5,
            "legend.frameon": True,
            "legend.framealpha": 0.92,
            "legend.edgecolor": "#dfe1e5",
            "font.size": 10,
            "figure.dpi": 120,
        }
    )


# --------------------------------------------------------------------------- #
#  1. Steady-state vs transient reframe
# --------------------------------------------------------------------------- #
def plot_steady_vs_transient(
    split_by_controller: Mapping[str, Mapping[str, float]],
    path: str,
    *,
    unit: str = "°",
    ylabel: str | None = None,
    title: str | None = None,
):
    """Grouped bars: transient RMSE vs steady-state RMSE per controller.

    This is the single most important reframing figure: the aggregate RMSE is
    dominated by short transients, but the steady-state holding accuracy tells
    the opposite story.  Plotted on a log y-axis because the two regimes differ
    by more than an order of magnitude.
    """
    if title is None:
        title = T("steady_vs_transient_title")
    setup_style()
    names = list(split_by_controller)
    trans = [split_by_controller[n]["transient_rmse"] for n in names]
    steady = [split_by_controller[n]["steady_rmse"] for n in names]

    x = np.arange(len(names))
    w = 0.36
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    b1 = ax.bar(x - w / 2, trans, w, label=T("transient_bar"),
                color="#e0a458", edgecolor="white")
    b2 = ax.bar(x + w / 2, steady, w, label=T("steadystate_bar"),
                color=[controller_color(n) for n in names], edgecolor="white")

    ax.set_yscale("log")
    ax.set_ylabel(ylabel or T("rmse_log"))
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_title(title)
    for bars in (b1, b2):
        for r in bars:
            h = r.get_height()
            ax.text(r.get_x() + r.get_width() / 2, h * 1.05, f"{h:.2f}",
                    ha="center", va="bottom", fontsize=8)
    ax.legend(loc="upper right")
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  1b. Regime comparison (aggregate / transient / settled)
# --------------------------------------------------------------------------- #
def plot_regime_comparison(
    regimes_by_controller: "Mapping[str, Mapping[str, float]]",
    path: str,
    *,
    regime_order: Sequence[str] | None = None,
    unit: str = "°",
    ylabel: str | None = None,
    title: str | None = None,
):
    """Grouped bars of an error metric across operating regimes per controller.

    ``regimes_by_controller[name]`` maps a regime label (e.g. T("regime_aggregate"),
    T("regime_transient"), T("regime_settled")) to a metric value.  The whole point
    is to show that a single aggregate number conflates regimes in which the ranking
    of two controllers actually flips.  Log y-axis because regimes differ by an order
    of magnitude or more.
    """
    if title is None:
        title = T("regime_comparison_title")
    setup_style()
    names = list(regimes_by_controller)
    regimes = list(regime_order or next(iter(regimes_by_controller.values())).keys())
    x = np.arange(len(regimes))
    w = 0.8 / len(names)
    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    for i, n in enumerate(names):
        vals = [regimes_by_controller[n][r] for r in regimes]
        bars = ax.bar(x + (i - (len(names) - 1) / 2) * w, vals, w,
                      label=n, color=controller_color(n), edgecolor="white")
        for r in bars:
            h = r.get_height()
            ax.text(r.get_x() + r.get_width() / 2, h * 1.04, f"{h:.2f}",
                    ha="center", va="bottom", fontsize=8)
    ax.set_yscale("log")
    ax.set_ylabel(ylabel or T("rmse_log"))
    ax.set_xticks(x)
    ax.set_xticklabels(regimes)
    ax.set_ylim(top=ax.get_ylim()[1] * 6.0)
    ax.set_title(title)
    ax.legend(loc="upper right", framealpha=0.95)
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  2. Per-segment breakdown
# --------------------------------------------------------------------------- #
def plot_segment_breakdown(
    segments_by_controller: Mapping[str, Sequence[dict]],
    path: str,
    *,
    turn_labels: Sequence[str] | None = None,
    metric: str = "iae",
    unit: str = "°·s",
    title: str | None = None,
):
    """Horizontal grouped bars of a per-segment metric for each controller.

    Localises *where* in the mission each controller wins or loses, which a
    single global number hides.  ``turn_labels`` (optional) annotates each
    segment with the commanded manoeuvre.
    """
    if title is None:
        title = T("segment_breakdown_title")
    setup_style()
    names = list(segments_by_controller)
    n_seg = len(next(iter(segments_by_controller.values())))
    seg_labels = [
        f"{int(s['start'])}–{int(s['end'])} s"
        for s in next(iter(segments_by_controller.values()))
    ]
    if turn_labels is not None:
        seg_labels = [f"{lab}\n{turn}" for lab, turn in zip(seg_labels, turn_labels)]

    y = np.arange(n_seg)
    h = 0.8 / len(names)
    fig, ax = plt.subplots(figsize=(8.6, 0.62 * n_seg + 1.8))
    for i, n in enumerate(names):
        vals = [s[metric] for s in segments_by_controller[n]]
        ax.barh(y + (i - (len(names) - 1) / 2) * h, vals, h,
                label=n, color=controller_color(n), edgecolor="white")
    ax.set_yticks(y)
    ax.set_yticklabels(seg_labels, fontsize=8)
    ax.invert_yaxis()
    ax.set_xlabel(f"{metric.upper()} [{unit}]")
    ax.set_title(title)
    ax.legend(loc="lower right")
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  3. Error distribution (ECDF)
# --------------------------------------------------------------------------- #
def plot_error_ecdf(
    errors_by_controller: Mapping[str, np.ndarray],
    path: str,
    *,
    unit: str = "°",
    xlabel: str | None = None,
    linthresh: float = 0.5,
    title: str | None = None,
):
    """Empirical CDF of |error| for each controller.

    Reveals the heavy-tail behaviour that mean/RMSE hide: a controller can be
    better for 95% of the run yet have a damaging worst-case excursion.
    """
    if title is None:
        title = T("ecdf_title")
    setup_style()
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    for j, (n, e) in enumerate(errors_by_controller.items()):
        ae = np.sort(np.abs(np.asarray(e, float)))
        cdf = np.arange(1, ae.size + 1) / ae.size
        ax.plot(ae, cdf, label=n, color=controller_color(n), lw=2)
        p95 = np.percentile(ae, 95)
        ax.scatter([p95], [0.95], color=controller_color(n), zorder=5, s=28)
        nd = 1 if p95 >= 1.0 else 3
        ax.annotate(f"P95 = {p95:.{nd}f}{unit}", (p95, 0.95),
                    textcoords="offset points", xytext=(7, -14 - 12 * j),
                    fontsize=8, color=controller_color(n))
    ax.axhline(0.95, color="#9aa0a6", ls=":", lw=1, alpha=0.7)
    ax.set_xlabel(xlabel or T("hdg_err_abs"))
    ax.set_ylabel(T("cum_prob"))
    ax.set_xscale("symlog", linthresh=linthresh)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.02)
    ax.set_title(title)
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  4. Spike anatomy (zoom)
# --------------------------------------------------------------------------- #
def plot_spike_zoom(
    t,
    series_by_controller: Mapping[str, np.ndarray],
    ref,
    window: tuple[float, float],
    path: str,
    *,
    unit: str = "°",
    ylabel: str | None = None,
    title: str = "",
    note: str = "",
):
    """Zoom on a time window to dissect a transient event.

    Plots each controller and the reference over ``window`` and marks the peak
    deviation of the worst controller.  Use to show *why* an aggregate metric
    spiked (overshoot, coupling excursion, etc.).
    """
    if ylabel is None:
        ylabel = T("heading_deg")
    setup_style()
    t = np.asarray(t, float)
    m = (t >= window[0]) & (t <= window[1])
    fig, ax = plt.subplots(figsize=(7.6, 4.2))
    ax.plot(t[m], np.asarray(ref, float)[m], color=PALETTE["reference"], ls="--",
            lw=1.6, label=T("reference"), alpha=0.9)
    for n, s in series_by_controller.items():
        s = np.asarray(s, float)
        ax.plot(t[m], s[m], color=controller_color(n), lw=2, label=n)
    ax.set_xlim(*window)
    # absolute tick labels (e.g. 200.05), not matplotlib's "+2e2" offset form
    ax.ticklabel_format(axis="y", useOffset=False)
    ax.set_xlabel(T("time_s"))
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    if note:
        ax.text(0.01, -0.22, note, transform=ax.transAxes, fontsize=8,
                color="#5a5a66", va="top", wrap=True)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  5. Mission overview with annotated events
# --------------------------------------------------------------------------- #
def plot_mission_overview(
    t,
    heading_by_controller: Mapping[str, np.ndarray],
    ref_heading,
    error_by_controller: Mapping[str, np.ndarray],
    switch_times: Sequence[float],
    path: str,
    *,
    events: Sequence[tuple[float, float, str]] = (),
    ylabel: str | None = None,
    err_ylabel: str | None = None,
    title: str | None = None,
):
    """Two-panel mission overview: tracked signal (top), error (bottom).

    Cleaner than stacking six panels: segment switches are light vertical
    guides, and ``events`` (t, value, label) annotate notable excursions on the
    error panel.
    """
    if title is None:
        title = T("mission_overview_title")
    setup_style()
    t = np.asarray(t, float)
    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [1.25, 1]})

    ax0.plot(t, np.asarray(ref_heading, float), color=PALETTE["reference"], ls="--",
             lw=1.4, alpha=0.9, label=T("reference"))
    for n, h in heading_by_controller.items():
        ax0.plot(t, np.asarray(h, float), color=controller_color(n), lw=1.6, label=n)
    ax0.set_ylabel(ylabel or T("heading_deg"))
    ax0.set_title(title)
    ax0.legend(loc="upper left", ncol=3)

    ax1.axhline(0, color="#9aa0a6", lw=1, alpha=0.7)
    for n, e in error_by_controller.items():
        ax1.plot(t, np.asarray(e, float), color=controller_color(n), lw=1.3, label=n)
    ax1.set_ylabel(err_ylabel or T("heading_error_deg"))
    ax1.set_xlabel(T("time_s"))
    ax1.legend(loc="upper left", ncol=2)

    for ts in switch_times:
        for ax in (ax0, ax1):
            ax.axvline(ts, color="#c2c5cc", ls=":", lw=0.9, alpha=0.7)
    for (et, ev, lab) in events:
        ax1.scatter([et], [ev], color="#d1495b", zorder=6, s=34)
        late = et > 0.78 * float(t[-1])
        ax1.annotate(lab.strip(), (et, ev), textcoords="offset points",
                     xytext=(-10 if late else 10, 0),
                     ha="right" if late else "left",
                     fontsize=8, color="#d1495b", va="center")

    ax0.yaxis.set_major_locator(MaxNLocator(6))
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  7. Monte-Carlo metric distributions
# --------------------------------------------------------------------------- #
def plot_metric_boxes(
    values_by_controller: Mapping[str, Sequence[float]],
    path: str,
    *,
    ylabel: str = "",
    title: str = "",
    show_points: bool = True,
):
    """Box plot of a metric across Monte-Carlo seeds, one box per controller.

    Shows the *distribution* of a metric over randomised disturbance seeds,
    which a single deterministic run cannot convey (median, spread, outliers).
    """
    setup_style()
    names = list(values_by_controller)
    data = [np.asarray(values_by_controller[n], float) for n in names]
    fig, ax = plt.subplots(figsize=(1.7 * len(names) + 2.5, 4.4))
    bp = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=False,
                    medianprops=dict(color="#202124", lw=1.5))
    for patch, n in zip(bp["boxes"], names):
        patch.set_facecolor(controller_color(n))
        patch.set_alpha(0.55)
        patch.set_edgecolor(controller_color(n))
    if show_points:
        for i, d in enumerate(data, start=1):
            jit = (np.random.default_rng(i).random(d.size) - 0.5) * 0.18
            ax.scatter(np.full(d.size, i) + jit, d, s=14,
                       color=controller_color(names[i - 1]), alpha=0.7, zorder=3,
                       edgecolor="white", linewidth=0.4)
    ax.set_xticks(range(1, len(names) + 1))
    ax.set_xticklabels(names)
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)
    ax.grid(axis="x", visible=False)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  8. Solver timing / real-time feasibility
# --------------------------------------------------------------------------- #
def plot_solver_timing(solve_times, deadline_ms: float, path: str,
                       *, title: str | None = None):
    """Histogram + ECDF of NMPC solve times against the real-time deadline."""
    if title is None:
        title = T("solver_timing_title")
    setup_style()
    s_ms = np.asarray(list(solve_times), float) * 1e3
    s_ms = s_ms[np.isfinite(s_ms)]
    fig, (axh, axc) = plt.subplots(1, 2, figsize=(11, 4.2))

    axh.hist(s_ms, bins=40, color=PALETTE["NMPC"], alpha=0.75, edgecolor="white")
    axh.axvline(deadline_ms, color="#d1495b", ls="--", lw=1.6,
                label=f"{T('deadline')} {deadline_ms:.0f} ms")
    axh.axvline(np.percentile(s_ms, 99), color="#202124", ls=":", lw=1.3,
                label=f"P99 {np.percentile(s_ms, 99):.0f} ms")
    axh.set_xlabel(T("solve_time_ms"))
    axh.set_ylabel(T("solve_count"))
    axh.set_title(T("distribution"))
    axh.legend(loc="upper right")

    ss = np.sort(s_ms)
    cdf = np.arange(1, ss.size + 1) / ss.size
    axc.plot(ss, cdf, color=PALETTE["NMPC"], lw=2)
    axc.axvline(deadline_ms, color="#d1495b", ls="--", lw=1.6)
    frac = float(np.mean(s_ms <= deadline_ms))
    axc.set_xlabel(T("solve_time_ms"))
    axc.set_ylabel(T("cum_prob"))
    axc.set_title(f"ECDF — {100 * frac:.1f}% {T('pct_on_time')}")
    axc.set_ylim(0, 1.02)

    fig.suptitle(title, fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  9. Accuracy / compute Pareto
# --------------------------------------------------------------------------- #
def plot_pareto(points: Sequence[tuple[str, float, float]], path: str,
                *, xlabel: str | None = None,
                ylabel: str | None = None,
                title: str | None = None):
    """Scatter of (label, cost, accuracy) points — e.g. an NMPC horizon sweep."""
    if xlabel is None:
        xlabel = T("pareto_xlabel")
    if ylabel is None:
        ylabel = T("pareto_ylabel")
    if title is None:
        title = T("pareto_title")
    setup_style()
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    ax.plot(xs, ys, "-o", color=PALETTE["NMPC"], lw=1.4, ms=7,
            markeredgecolor="white")
    for lab, x, y in points:
        ax.annotate(str(lab), (x, y), textcoords="offset points", xytext=(8, 4),
                    fontsize=8.5, color="#202124")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path


# --------------------------------------------------------------------------- #
#  Generic step-response comparison (depth / heading / speed)
# --------------------------------------------------------------------------- #
def plot_step_comparison(
    results: Sequence,
    path: str,
    *,
    z_target: float | None = None,
    psi_target_deg: float | None = None,
    title: str = "",
    settle_band_m: float = 0.6,
):
    """Annotated 3-panel step comparison for scenarios 1–5.

    ``results`` is a sequence of objects exposing ``.time``, ``.eta`` (N×6),
    ``.nu`` (N×6) and ``.controller_name`` — i.e. the framework's
    ``SimulationResult``.  Adds a depth tolerance band and annotates each
    controller's depth settling time, which the original plots omitted.
    """
    from analysis.metrics import step_metrics

    setup_style()
    fig, axes = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    if title:
        fig.suptitle(title, fontsize=12, fontweight="bold")

    for r in results:
        c = controller_color(r.controller_name)
        t = np.asarray(r.time, float)
        axes[0].plot(t, r.eta[:, 2], color=c, lw=1.6, label=r.controller_name)
        axes[1].plot(t, np.degrees(np.unwrap(r.eta[:, 5])), color=c, lw=1.6,
                     label=r.controller_name)
        spd = np.sqrt(np.sum(r.nu[:, :3] ** 2, axis=1))
        axes[2].plot(t, spd, color=c, lw=1.6, label=r.controller_name)

        if z_target is not None:
            sm = step_metrics(t, r.eta[:, 2], z_target, y0=float(r.eta[0, 2]),
                              settle_tol=settle_band_m / max(abs(z_target), 1e-6))
            if sm.settled and np.isfinite(sm.settling_time):
                axes[0].scatter([sm.settling_time], [z_target], color=c, s=26, zorder=5)

    if z_target is not None:
        axes[0].axhline(z_target, color="#9aa0a6", ls="--", alpha=0.6, label=T("target"))
        axes[0].axhspan(z_target - settle_band_m, z_target + settle_band_m,
                        color="#9aa0a6", alpha=0.10)
    if psi_target_deg is not None:
        axes[1].axhline(psi_target_deg, color="#9aa0a6", ls="--", alpha=0.6, label=T("target"))

    axes[0].set_ylabel(T("depth_m"))
    axes[0].set_title(T("depth_tracking"))
    axes[1].set_ylabel(T("heading_deg"))
    axes[1].set_title(T("heading_tracking"))
    axes[2].set_ylabel(T("speed_ms"))
    axes[2].set_title(T("speed"))
    axes[2].set_xlabel(T("time_s"))
    for ax in axes:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return path
