"""
Control-performance metrics for AUV trajectory-tracking analysis
================================================================

A self-contained, dependency-light metrics library (numpy + scipy only).
It is written to plug directly into the existing framework: every function
operates on plain time/signal/reference arrays, so it works equally well on a
``SimulationResult`` from ``adapters.fossen_adapter`` and on a CSV exported by
``experiments.run_remus100_comparison``.

The library deliberately separates three layers of analysis that the original
experiment script collapsed into a single RMSE number:

1.  Aggregate tracking quality        -> tracking_metrics()
2.  Step / transition characterisation -> step_metrics()
3.  Steady-state vs transient split    -> segment_decompose(), split_transient_steady()

plus actuator-activity metrics (chatter) and Monte-Carlo summary statistics.

All angular quantities are handled in radians internally; helpers are provided
to wrap errors to [-pi, pi] so that a 359 deg / -1 deg pair is treated as a
2 deg error rather than a 358 deg error.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Iterable, Sequence

import numpy as np

try:                                  # scipy is optional; only stats helpers need it
    from scipy import stats as _sps
    _HAVE_SCIPY = True
except Exception:                     # pragma: no cover
    _HAVE_SCIPY = False


# --------------------------------------------------------------------------- #
#  Angle helpers
# --------------------------------------------------------------------------- #
def wrap_to_pi(angle):
    """Wrap an angle (rad), scalar or array, to the interval [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_error(y_rad, ref_rad):
    """Signed tracking error (rad) wrapped to [-pi, pi]."""
    return wrap_to_pi(np.asarray(y_rad, float) - np.asarray(ref_rad, float))


# --------------------------------------------------------------------------- #
#  Aggregate tracking metrics
# --------------------------------------------------------------------------- #
@dataclass
class TrackingMetrics:
    rmse: float          # root-mean-square error
    mae: float           # mean absolute error
    iae: float           # integral of |e| dt
    itae: float          # integral of t*|e| dt  (penalises late errors)
    max_abs: float       # worst-case absolute error
    final: float         # |error| in the last sample
    std: float           # standard deviation of the error

    def as_dict(self):
        return asdict(self)


def tracking_metrics(t, y, ref, *, angular: bool = False) -> TrackingMetrics:
    """Aggregate tracking error metrics for one signal against its reference.

    Parameters
    ----------
    t : array        time vector [s] (uniform or non-uniform)
    y : array        measured signal
    ref : array      reference signal (same shape as ``y``)
    angular : bool   if True, errors are wrapped to [-pi, pi] before scoring
                     (use for heading / pitch in radians)
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    ref = np.asarray(ref, float)
    e = angle_error(y, ref) if angular else (y - ref)
    ae = np.abs(e)
    dt = np.gradient(t)
    return TrackingMetrics(
        rmse=float(np.sqrt(np.mean(e**2))),
        mae=float(np.mean(ae)),
        iae=float(np.sum(ae * dt)),
        itae=float(np.sum(t * ae * dt)),
        max_abs=float(np.max(ae)),
        final=float(ae[-1]),
        std=float(np.std(e)),
    )


# --------------------------------------------------------------------------- #
#  Step / transition characterisation
# --------------------------------------------------------------------------- #
@dataclass
class StepMetrics:
    rise_time: float           # 10% -> 90% of the commanded change [s]
    settling_time: float       # time to enter & stay within tol band [s]
    overshoot_pct: float       # peak excursion beyond target, % of step size
    peak: float                # peak value reached
    peak_time: float           # time of the peak [s]
    steady_state_error: float  # |y - target| averaged over the tail
    settled: bool              # whether a settling time was found

    def as_dict(self):
        return asdict(self)


def step_metrics(
    t,
    y,
    target: float,
    *,
    y0: float | None = None,
    settle_tol: float = 0.02,
    tail_frac: float = 0.1,
    angular: bool = False,
) -> StepMetrics:
    """Classic step-response descriptors for a single transition.

    ``settle_tol`` is a *fraction of the step size* (e.g. 0.02 = 2% band).
    For angular signals the y/target/y0 values are still expressed in the same
    unit (rad); errors are wrapped so a turn through +pi is handled correctly.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if y0 is None:
        y0 = float(y[0])

    if angular:
        step = float(wrap_to_pi(target - y0))
        err = wrap_to_pi(y - target)                 # distance to target
        prog = wrap_to_pi(y - y0)                     # progress from start
    else:
        step = float(target - y0)
        err = y - target
        prog = y - y0

    span = abs(step) if abs(step) > 1e-9 else 1.0
    band = settle_tol * span

    # Rise time: 10% -> 90% of the commanded change (sign-aware).
    rise_time = float("nan")
    if abs(step) > 1e-9:
        frac = prog / step                            # 0 at start, 1 at target
        try:
            i10 = np.argmax(frac >= 0.1)
            i90 = np.argmax(frac >= 0.9)
            if i90 > i10:
                rise_time = float(t[i90] - t[i10])
        except Exception:
            pass

    # Settling time: last instant the error leaves the band, +1 sample.
    settled = False
    settling_time = float("nan")
    outside = np.abs(err) > band
    if outside.any():
        last_out = np.max(np.where(outside)[0])
        if last_out + 1 < len(t):
            settling_time = float(t[last_out + 1] - t[0])
            settled = True
    else:
        settling_time = 0.0
        settled = True

    # Overshoot: maximum progress beyond the target, as % of the step.
    if abs(step) > 1e-9:
        over = np.max(prog / step) - 1.0
        overshoot_pct = float(max(0.0, over) * 100.0)
        ipk = int(np.argmax(prog / step))
    else:
        overshoot_pct = 0.0
        ipk = int(np.argmax(np.abs(prog)))
    peak = float(y[ipk])
    peak_time = float(t[ipk])

    n_tail = max(1, int(tail_frac * len(t)))
    sse = float(np.mean(np.abs(err[-n_tail:])))

    return StepMetrics(
        rise_time=rise_time,
        settling_time=settling_time,
        overshoot_pct=overshoot_pct,
        peak=peak,
        peak_time=peak_time,
        steady_state_error=sse,
        settled=settled,
    )


# --------------------------------------------------------------------------- #
#  Actuator activity (chatter) metrics
# --------------------------------------------------------------------------- #
def actuator_activity(t, u) -> dict:
    """Activity / smoothness metrics for one actuator channel.

    total_variation : sum of |du| over the run  (lower = smoother)
    rms_rate        : RMS of du/dt
    reversals       : number of sign changes of du/dt  (chatter proxy)
    """
    t = np.asarray(t, float)
    u = np.asarray(u, float)
    du = np.diff(u)
    dt = np.diff(t)
    rate = du / np.where(dt == 0, np.nan, dt)
    rate = rate[np.isfinite(rate)]
    sign = np.sign(rate)
    reversals = int(np.sum(np.abs(np.diff(sign)) > 1e-9))
    return {
        "total_variation": float(np.sum(np.abs(du))),
        "rms_rate": float(np.sqrt(np.mean(rate**2))) if rate.size else 0.0,
        "reversals": reversals,
    }


# --------------------------------------------------------------------------- #
#  Segment-aware decomposition
# --------------------------------------------------------------------------- #
def segment_decompose(
    t,
    y,
    ref,
    segment_bounds: Sequence[float],
    *,
    angular: bool = False,
    steady_frac: float = 0.4,
) -> list[dict]:
    """Split a mission into segments and score each one independently.

    ``segment_bounds`` is the list of segment edges, e.g.
    ``[0, 5, 60, 120, ..., t_final]``.  For each segment the function reports
    aggregate IAE/MAE/max and, separately, the *steady-state* error averaged
    over the final ``steady_frac`` of the segment.  This is what separates
    "good at holding" from "good at manoeuvring".
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    ref = np.asarray(ref, float)
    e = angle_error(y, ref) if angular else (y - ref)
    ae = np.abs(e)
    dt_med = float(np.median(np.diff(t))) if len(t) > 1 else 0.02

    rows = []
    for a, b in zip(segment_bounds[:-1], segment_bounds[1:]):
        m = (t >= a) & (t < b)
        if not m.any():
            continue
        sdur = b - a
        sm = (t >= a + (1.0 - steady_frac) * sdur) & (t < b)
        rows.append(
            {
                "start": float(a),
                "end": float(b),
                "iae": float(np.sum(ae[m]) * dt_med),
                "mae": float(np.mean(ae[m])),
                "rmse": float(np.sqrt(np.mean(e[m] ** 2))),
                "max_abs": float(np.max(ae[m])),
                "steady_state": float(np.mean(ae[sm])) if sm.any() else float("nan"),
            }
        )
    return rows


def split_transient_steady(
    t,
    y,
    ref,
    switch_times: Sequence[float],
    *,
    angular: bool = False,
    transient_window: float = 35.0,
    only_changing: Sequence[bool] | None = None,
) -> dict:
    """Split the whole run into 'transient' and 'steady' samples and score each.

    A sample is *transient* if it falls within ``transient_window`` seconds
    after a manoeuvre-inducing switch.  ``only_changing`` (optional, one bool
    per switch) lets the caller mark which switches actually command a change
    so that pure holds are not counted as transients.
    """
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    ref = np.asarray(ref, float)
    e = angle_error(y, ref) if angular else (y - ref)

    trans = np.zeros(len(t), bool)
    for i, ts in enumerate(switch_times):
        if only_changing is not None and not only_changing[i]:
            continue
        trans |= (t >= ts) & (t < ts + transient_window)
    steady = ~trans

    def _rmse(mask):
        return float(np.sqrt(np.mean(e[mask] ** 2))) if mask.any() else float("nan")

    return {
        "transient_rmse": _rmse(trans),
        "steady_rmse": _rmse(steady),
        "transient_max": float(np.max(np.abs(e[trans]))) if trans.any() else float("nan"),
        "steady_max": float(np.max(np.abs(e[steady]))) if steady.any() else float("nan"),
        "transient_frac": float(np.mean(trans)),
    }


# --------------------------------------------------------------------------- #
#  Monte-Carlo / multi-run statistics
# --------------------------------------------------------------------------- #
def summarize_runs(values: Iterable[float], *, confidence: float = 0.95) -> dict:
    """Mean, std, median, IQR and a confidence interval for a set of runs.

    Uses a Student-t interval when scipy is available and n>1, otherwise a
    normal approximation.  Designed for summarising a metric (e.g. RMSE)
    across Monte-Carlo seeds.
    """
    v = np.asarray(list(values), float)
    v = v[np.isfinite(v)]
    n = v.size
    out = {
        "n": int(n),
        "mean": float(np.mean(v)) if n else float("nan"),
        "std": float(np.std(v, ddof=1)) if n > 1 else 0.0,
        "median": float(np.median(v)) if n else float("nan"),
        "q25": float(np.percentile(v, 25)) if n else float("nan"),
        "q75": float(np.percentile(v, 75)) if n else float("nan"),
        "min": float(np.min(v)) if n else float("nan"),
        "max": float(np.max(v)) if n else float("nan"),
    }
    if n > 1:
        sem = out["std"] / np.sqrt(n)
        if _HAVE_SCIPY:
            tcrit = _sps.t.ppf(0.5 + confidence / 2.0, df=n - 1)
        else:
            tcrit = 1.96
        out["ci_low"] = out["mean"] - tcrit * sem
        out["ci_high"] = out["mean"] + tcrit * sem
    else:
        out["ci_low"] = out["mean"]
        out["ci_high"] = out["mean"]
    out["confidence"] = confidence
    return out


def paired_comparison(a: Sequence[float], b: Sequence[float], *, labels=("A", "B")) -> dict:
    """Paired statistical comparison of two controllers across matched runs.

    Returns a paired-t p-value, a Wilcoxon signed-rank p-value (when scipy is
    present), the mean paired difference and Cohen's d_z effect size.  Use
    when the same set of disturbance seeds was applied to both controllers.
    """
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    d = a - b
    out = {
        "label_a": labels[0],
        "label_b": labels[1],
        "mean_a": float(np.mean(a)),
        "mean_b": float(np.mean(b)),
        "mean_diff": float(np.mean(d)),
        "n": int(d.size),
    }
    if d.size > 1 and np.std(d) > 0:
        out["cohen_dz"] = float(np.mean(d) / np.std(d, ddof=1))
    else:
        out["cohen_dz"] = float("nan")
    if _HAVE_SCIPY and d.size > 1:
        try:
            out["paired_t_p"] = float(_sps.ttest_rel(a, b).pvalue)
        except Exception:
            out["paired_t_p"] = float("nan")
        try:
            out["wilcoxon_p"] = float(_sps.wilcoxon(a, b).pvalue)
        except Exception:
            out["wilcoxon_p"] = float("nan")
    else:
        out["paired_t_p"] = float("nan")
        out["wilcoxon_p"] = float("nan")
    return out


# --------------------------------------------------------------------------- #
#  Solver / real-time feasibility
# --------------------------------------------------------------------------- #
def solver_timing(solve_times: Sequence[float], *, dt_control: float) -> dict:
    """Summarise NMPC solve-time samples and assess real-time feasibility.

    ``dt_control`` is the control period the solver must beat (e.g. 0.2 s for
    a 5 Hz NMPC).  Reports the fraction of solves that met the deadline and the
    worst case, which is the number that actually governs real-time safety.
    """
    s = np.asarray(list(solve_times), float)
    s = s[np.isfinite(s)]
    if s.size == 0:
        return {"n": 0}
    return {
        "n": int(s.size),
        "mean_ms": float(np.mean(s) * 1e3),
        "median_ms": float(np.median(s) * 1e3),
        "p95_ms": float(np.percentile(s, 95) * 1e3),
        "p99_ms": float(np.percentile(s, 99) * 1e3),
        "max_ms": float(np.max(s) * 1e3),
        "deadline_ms": float(dt_control * 1e3),
        "frac_realtime": float(np.mean(s <= dt_control)),
        "worst_overrun_ms": float(max(0.0, np.max(s) - dt_control) * 1e3),
    }
