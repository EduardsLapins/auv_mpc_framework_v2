# AUV NMPC Thesis Analysis — Scenario 6 Comprehensive Report

## §1 Executive Summary

Scenario 6 (600 s, 8-waypoint complex mission, current 0.6 m/s @ 150°) was used to compare a tuned cascaded PID and trajectory NMPC controllers on the REMUS 100 AUV.  The aggregate heading RMSE of the patched NMPC (0.25°) is lower than the PID (1.43°), but this aggregate figure is dominated by two specific transient failure modes.  In settled holding, NMPC is 11× more accurate than PID (0.052° vs 0.585° RMSE).  The patched NMPC (offset-free disturbance observer) directly addresses both failure modes identified in this analysis.

## §2 Root Cause Analysis

> **Historical note.** The two failure modes below were identified on earlier
> development runs (original N=20 controller, and the patched controller before
> the surge-channel model fix).  They document *why* the patched controller and
> the corrected CasADi predictor exist.  In the current results (tables below
> and §3) both windows show only small residual errors: the surge predictor now
> includes the advance-ratio thrust loss, state limits are soft constraints,
> and the disturbance observer has a decay term.

### Failure Mode 1 — Reversal Overshoot (490–570 s)

**Physical cause — disturbance observer with no decay:**
The offset-free observer estimates a yaw angular-acceleration disturbance
`d_r` [rad/s²] by accumulating the one-step yaw-rate prediction error:

    d_r[k] = (1−α)·d_r[k−1] + α·(d_r[k−1] + L·ν_r[k] / dt)

Without a decay term, `d_r` acts as a pure integrator that holds its value
indefinitely once the disturbance ends.  During the depth-coupling window
(260–340 s), the unmodelled pitch→yaw coupling drives `d_r` to its negative
saturation limit (large counter-clockwise angular-acceleration estimate).
After the ascent completes, `ν_r → 0` (the NMPC compensates in closed-loop),
so `d_r` never decays — it remains locked at the stale saturated value for
all subsequent segments.

At t = 500 s, the mission commands a −45° (counter-clockwise) heading turn
from 45° to 0° while simultaneously initiating a second depth change (45 m →
20 m).  The stale `d_r` (large CCW) causes the NMPC predictor to expect rapid
counter-clockwise yaw acceleration even with no rudder.  To prevent exceeding
the yaw-rate constraint, IPOPT responds by commanding strong clockwise (positive)
rudder — exactly the opposite of what the turn requires.  The AUV heads clock-
wise instead of counter-clockwise, producing the observed 74° overshoot.

**Secondary cause — excessive depth rate creating modelling mismatch:**
The default `z_rate_max = 2.0 m/s` allows depth transitions at rates that
require pitch angles up to asin(2.0/2.5) ≈ 53°, far exceeding the NMPC
predictor's `θ_max = 25°` pitch constraint.  This large predictor–plant
mismatch amplifies the coupling disturbance during steep dives, driving `d_r`
to saturation faster and keeping it there longer.

**Fixes applied:**

1. **Observer leak** (`obs_leak = 0.005` per step): applies a per-step
   multiplicative decay to `d_r` so stale estimates decay with time constant
   τ = dt / obs_leak ≈ 0.2 / 0.005 = 40 steps ≈ 8 s.  After 160 s without
   active coupling the estimate decays to < 2% of its peak value.

2. **Depth-rate cap** (`z_rate_max = 1.0 m/s`): limits depth transitions to
   rates requiring pitch ≤ asin(1.0/2.5) ≈ 24°, consistent with the NMPC
   predictor's pitch constraint and therefore eliminating the modelling
   mismatch that saturated the observer.

3. **SSA warm-start override**: at each MPC call the initial guess for ψ is
   overridden with a linear interpolation in the SSA-shortest direction to the
   horizon-end reference heading, so IPOPT starts in the correct turn direction
   regardless of the previous warm-start state.

### Failure Mode 2 — Depth-Coupling Excursion (260–340 s)

**Physical cause:** The CasADi reduced-order predictor sets pitch→yaw coupling
to zero.  During ascent, the real Fossen hull generates a yaw moment through
pitch-yaw inertial coupling:

    r_dot_true = (N_rudder − d_yaw·r) / I₆₆ + (N_vr·v·r + N_wp·w·p + ...)/ I₆₆

The terms in parentheses are absent from the predictor (δ_r ≠ 0 throughout
the dive).  With `z_rate_max = 2.0 m/s` the required pitch angle (≈ 53°)
far exceeds `θ_max = 25°`, so the coupling moment is also much larger than
the observer can track (which only estimates a single scalar `d_r`).

**Fixes applied (same two fixes as above):**
The depth-rate cap limits the pitch angle to ≤ 24°, dramatically reducing the
coupling moment; the observer with leakage quickly tracks the remaining smaller
disturbance and the heading excursion is substantially reduced.

### Quantification

**PID (tuned):**
| Window | Peak error | Window IAE | % of total IAE |
|--------|-----------|------------|----------------|
| Depth coupling (260–340 s) | 0.7° | 32.1 °·s | 5.6% |
| Reversal (490–570 s) | 2.1° | 67.2 °·s | 11.8% |

**NMPC traj. (N=12) (original, reference):**
| Window | Peak error | Window IAE | % of total IAE |
|--------|-----------|------------|----------------|
| Depth coupling (260–340 s) | 0.0° | 1.2 °·s | 0.3% |
| Reversal (490–570 s) | 2.0° | 27.9 °·s | 6.7% |

**NMPC offset-free (N=12) (patched):**
| Window | Peak error | Window IAE | % of total IAE |
|--------|-----------|------------|----------------|
| Depth coupling (260–340 s) | 0.0° | 1.2 °·s | 1.5% |
| Reversal (490–570 s) | 0.9° | 9.9 °·s | 12.7% |



## §3 Full Performance Metrics

| Controller | Regime | RMSE [°] | MAE [°] | IAE [°·s] | P95 [°] | P99 [°] | Max |e| [°] |
|---|---|---|---|---|---|---|---|
| PID (tuned) | aggregate | 1.435 | 0.947 | 568.4 | 3.82 | 4.71 | 4.91 |
| PID (tuned) | transient | 2.498 | 2.053 | 359.4 | 4.59 | 4.88 | 4.91 |
| PID (tuned) | settled | 0.585 | 0.511 | 122.6 | 0.92 | 0.98 | 0.99 |
| NMPC traj. (N=12) | aggregate | 1.609 | 0.694 | 416.5 | 4.50 | 5.90 | 6.28 |
| NMPC traj. (N=12) | transient | 2.926 | 2.134 | 373.4 | 5.81 | 6.12 | 6.28 |
| NMPC traj. (N=12) | settled | 0.053 | 0.034 | 8.0 | 0.10 | 0.10 | 0.17 |
| NMPC offset-free (N=12) | aggregate | 0.249 | 0.130 | 77.9 | 0.63 | 0.98 | 1.44 |
| NMPC offset-free (N=12) | transient | 0.444 | 0.327 | 57.3 | 0.91 | 1.28 | 1.44 |
| NMPC offset-free (N=12) | settled | 0.052 | 0.033 | 7.9 | 0.10 | 0.10 | 0.19 |

## §4 Statistical Significance

#### PID (tuned)  vs  NMPC traj. (N=12)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | 0.204 |
| 95% CI [°] | [0.020, 0.388] |
| Paired t-test p-value | 0.0337 |
| Wilcoxon p-value | 0.0547 |
| Cohen's d_z | 0.853 (large) |

#### PID (tuned)  vs  NMPC offset-free (N=12)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | 0.698 |
| 95% CI [°] | [0.248, 1.148] |
| Paired t-test p-value | 0.0072 |
| Wilcoxon p-value | 0.0195 |
| Cohen's d_z | 1.193 (large) |

#### NMPC traj. (N=12)  vs  NMPC offset-free (N=12)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | 0.494 |
| 95% CI [°] | [0.035, 0.953] |
| Paired t-test p-value | 0.0381 |
| Wilcoxon p-value | 0.0273 |
| Cohen's d_z | 0.827 (large) |


## §5 Per-Segment Breakdown

| Segment | Manoeuvre | PID (tuned) IAE [°·s] | PID (tuned) SS [°] | NMPC traj. (N=12) IAE [°·s] | NMPC traj. (N=12) SS [°] | NMPC offset-free (N=12) IAE [°·s] | NMPC offset-free (N=12) SS [°] |
|---|---|---|---|---|---|---|---|
| 0–5 s | miera stāvoklis | 0.0 | 0.006 | 0.3 | 0.142 | 0.4 | 0.162 |
| 5–60 s | miera stāvoklis | 0.6 | 0.005 | 6.0 | 0.100 | 5.9 | 0.098 |
| 60–120 s | pagrieziens +90° | 87.6 | 0.450 | 74.5 | 0.005 | 11.3 | 0.005 |
| 120–180 s | miera stāvoklis | 20.5 | 0.300 | 1.3 | 0.005 | 1.2 | 0.005 |
| 180–260 s | pagrieziens +110° | 124.8 | 0.751 | 111.1 | 0.005 | 16.1 | 0.005 |
| 260–340 s | miera stāvoklis | 41.7 | 0.435 | 1.4 | 0.007 | 1.3 | 0.006 |
| 340–420 s | pagrieziens +130° | 126.6 | 0.889 | 133.6 | 0.002 | 14.9 | 0.002 |
| 420–500 s | pagrieziens +75° | 91.9 | 0.834 | 56.7 | 0.033 | 13.4 | 0.032 |
| 500–600 s | pagrieziens -45° | 74.8 | 0.284 | 31.6 | 0.096 | 13.5 | 0.093 |

## §6 Failure Window Dissection

### Depth-coupling excursion (255–310 s)

Pure ascent (40 m → 10 m) at constant heading 200°.  Heading drift is caused entirely by the unmodelled pitch→yaw coupling in the reduced predictor.

| Controller | Peak error [°] | Window IAE [°·s] | % of total IAE |
|------------|----------------|-----------------|----------------|
| PID (tuned) | 0.7 | 32.1 | 5.6% |
| NMPC traj. (N=12) | 0.0 | 1.2 | 0.3% |
| NMPC offset-free (N=12) | 0.0 | 1.2 | 1.5% |

### Reversal (495–560 s)

Sharp heading reversal (330°→45°→0°) combined with a 45 m → 20 m climb.

| Controller | Peak error [°] | Window IAE [°·s] | % of total IAE |
|------------|----------------|-----------------|----------------|
| PID (tuned) | 2.1 | 67.2 | 11.8% |
| NMPC traj. (N=12) | 2.0 | 27.9 | 6.7% |
| NMPC offset-free (N=12) | 0.9 | 9.9 | 12.7% |

## §7 Conclusions

1. **Root cause of the 74° reversal overshoot was a missing decay term in the disturbance observer**, not the prediction horizon.  The observer's integral action accumulated a large counter-clockwise disturbance estimate during the first depth change (260–340 s) and held it indefinitely — through 160 s and two subsequent heading turns — until it directly sabotaged the counter-clockwise turn at t = 500 s.

2. **Two targeted fixes eliminate both failure modes:**
   - Observer leak (τ ≈ 8 s) prevents stale disturbance estimates from persisting across unrelated mission segments.
   - Depth-rate cap (`z_rate_max = 1.0 m/s`) keeps pitch within the NMPC predictor's constraint, eliminating the modelling mismatch that had saturated the observer and amplified the coupling.

3. **Regime decomposition** is essential for fair comparison: aggregate RMSE is dominated by brief transient events.  In steady-state heading holding, the NMPC is significantly more accurate than PID (0.052° vs 0.585° RMSE).

4. **Patched NMPC aggregate RMSE = 0.25°**, settled RMSE = 0.052°.  PID aggregate RMSE = 1.43°.  Original NMPC aggregate RMSE = 1.61°. 

5. **Limitations:** All results come from a single deterministic simulation run.  A Monte-Carlo robustness study (see `analysis/run_advanced_experiments.py`) is required to make statistical claims about the full disturbance distribution.

## Data sources

- CSV: `results/advanced/patch_compare_timeseries.csv` or `results/s6_kursa_kludas_analize.csv`
- Figures: `results/advanced/thesis_*.png`
- This report: `results/advanced/thesis_report.md`
