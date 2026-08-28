# AUV NMPC Thesis Analysis — Scenario 6 Comprehensive Report

## §1 Executive Summary

Scenario 6 (600 s, 8-waypoint complex mission, current 0.6 m/s @ 150°) was used to compare a tuned cascaded PID and trajectory NMPC controllers on the REMUS 100 AUV.  The aggregate heading RMSE of the patched NMPC (0.89°) is lower than the PID (1.44°), but this aggregate figure is dominated by two specific transient failure modes.  In settled holding, NMPC is 20× more accurate than PID (0.029° vs 0.587° RMSE).  The patched NMPC (offset-free disturbance observer, N=30 horizon) directly addresses both failure modes identified in this analysis.

## §2 Root Cause Analysis

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
| Depth coupling (260–340 s) | 0.7° | 42.9 °·s | 7.5% |
| Reversal (490–570 s) | 1.9° | 74.1 °·s | 13.0% |

**NMPC offset-free (N=30) (patched):**
| Window | Peak error | Window IAE | % of total IAE |
|--------|-----------|------------|----------------|
| Depth coupling (260–340 s) | 0.1° | 4.3 °·s | 2.4% |
| Reversal (490–570 s) | 0.3° | 5.5 °·s | 3.1% |



## §3 Full Performance Metrics

| Controller | Regime | RMSE [°] | MAE [°] | IAE [°·s] | P95 [°] | P99 [°] | Max |e| [°] |
|---|---|---|---|---|---|---|---|
| PID (tuned) | aggregate | 1.440 | 0.952 | 570.9 | 3.81 | 4.75 | 5.05 |
| PID (tuned) | transient | 2.508 | 2.061 | 360.6 | 4.56 | 5.02 | 5.05 |
| PID (tuned) | settled | 0.587 | 0.513 | 123.2 | 0.91 | 0.97 | 0.98 |
| PID (5 Hz) | aggregate | 1.450 | 0.959 | 575.4 | 3.80 | 4.77 | 5.09 |
| PID (5 Hz) | transient | 2.525 | 2.075 | 363.1 | 4.61 | 5.04 | 5.09 |
| PID (5 Hz) | settled | 0.590 | 0.518 | 124.2 | 0.92 | 0.98 | 1.02 |
| NMPC offset-free (N=30) | aggregate | 0.886 | 0.294 | 176.2 | 1.36 | 5.39 | 6.18 |
| NMPC offset-free (N=30) | transient | 1.638 | 0.906 | 158.5 | 4.60 | 6.11 | 6.18 |
| NMPC offset-free (N=30) | settled | 0.029 | 0.026 | 6.2 | 0.04 | 0.05 | 0.08 |

## §4 Statistical Significance

#### PID (tuned)  vs  PID (5 Hz)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | -0.007 |
| 95% CI [°] | [-0.011, -0.003] |
| Paired t-test p-value | 0.0040 |
| Wilcoxon p-value | 0.0039 |
| Cohen's d_z | 1.328 (large) |

#### PID (tuned)  vs  NMPC offset-free (N=30)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | 0.565 |
| 95% CI [°] | [0.217, 0.914] |
| Paired t-test p-value | 0.0057 |
| Wilcoxon p-value | 0.0195 |
| Cohen's d_z | 1.248 (large) |

#### PID (5 Hz)  vs  NMPC offset-free (N=30)

| Statistic | Value |
|-----------|-------|
| n (mission segments) | 9 |
| Mean difference [°] | 0.572 |
| 95% CI [°] | [0.223, 0.922] |
| Paired t-test p-value | 0.0054 |
| Wilcoxon p-value | 0.0195 |
| Cohen's d_z | 1.258 (large) |


## §5 Per-Segment Breakdown

| Segment | Manoeuvre | PID (tuned) IAE [°·s] | PID (tuned) SS [°] | PID (5 Hz) IAE [°·s] | PID (5 Hz) SS [°] | NMPC offset-free (N=30) IAE [°·s] | NMPC offset-free (N=30) SS [°] |
|---|---|---|---|---|---|---|---|
| 0–5 s | balansēšana | 0.0 | 0.006 | 0.0 | 0.006 | 0.1 | 0.028 |
| 5–60 s | balansēšana | 0.6 | 0.005 | 1.4 | 0.023 | 1.7 | 0.016 |
| 60–120 s | pagrieziens +90° | 88.9 | 0.459 | 89.5 | 0.461 | 23.8 | 0.041 |
| 120–180 s | balansēšana | 20.9 | 0.306 | 21.0 | 0.308 | 3.0 | 0.038 |
| 180–260 s | pagrieziens +110° | 128.9 | 0.773 | 129.3 | 0.775 | 97.5 | 0.035 |
| 260–340 s | balansēšana | 42.9 | 0.448 | 43.0 | 0.450 | 4.3 | 0.032 |
| 340–420 s | pagrieziens +130° | 122.7 | 0.879 | 123.4 | 0.883 | 27.6 | 0.002 |
| 420–500 s | pagrieziens +75° | 91.6 | 0.826 | 92.6 | 0.831 | 12.4 | 0.036 |
| 500–600 s | pagrieziens -45° | 74.5 | 0.278 | 75.0 | 0.279 | 5.6 | 0.016 |

## §6 Failure Window Dissection

### Depth-coupling excursion (260–340 s)

Pure ascent (40 m → 10 m) at constant heading 200°.  Heading drift is caused entirely by the unmodelled pitch→yaw coupling in the reduced predictor.

| Controller | Peak error [°] | Window IAE [°·s] | % of total IAE |
|------------|----------------|-----------------|----------------|
| PID (tuned) | 0.7 | 42.9 | 7.5% |
| PID (5 Hz) | 0.7 | 43.0 | 7.5% |
| NMPC offset-free (N=30) | 0.1 | 4.3 | 2.4% |

### Reversal overshoot (490–570 s)

Sharp heading reversal (330°→45°→0°).  Overshoot is caused by the prediction horizon being shorter than the physical manoeuvre time.

| Controller | Peak error [°] | Window IAE [°·s] | % of total IAE |
|------------|----------------|-----------------|----------------|
| PID (tuned) | 1.9 | 74.1 | 13.0% |
| PID (5 Hz) | 2.0 | 74.7 | 13.0% |
| NMPC offset-free (N=30) | 0.3 | 5.5 | 3.1% |

## §7 Conclusions

1. **Root cause of the 74° reversal overshoot was a missing decay term in the disturbance observer**, not the prediction horizon.  The observer's integral action accumulated a large counter-clockwise disturbance estimate during the first depth change (260–340 s) and held it indefinitely — through 160 s and two subsequent heading turns — until it directly sabotaged the counter-clockwise turn at t = 500 s.

2. **Two targeted fixes eliminate both failure modes:**
   - Observer leak (τ ≈ 8 s) prevents stale disturbance estimates from persisting across unrelated mission segments.
   - Depth-rate cap (`z_rate_max = 1.0 m/s`) keeps pitch within the NMPC predictor's constraint, eliminating the modelling mismatch that had saturated the observer and amplified the coupling.

3. **Regime decomposition** is essential for fair comparison: aggregate RMSE is dominated by brief transient events.  In steady-state heading holding, the NMPC is significantly more accurate than PID (0.029° vs 0.587° RMSE).

4. **Patched NMPC aggregate RMSE = 0.89°**, settled RMSE = 0.029°.  PID aggregate RMSE = 1.44°.  

5. **Limitations:** All results come from a single deterministic simulation run.  A Monte-Carlo robustness study (see `analysis/run_advanced_experiments.py`) is required to make statistical claims about the full disturbance distribution.

## Data sources

- CSV: `results/advanced/patch_compare_timeseries.csv` or `results/s6_kursa_kludas_analize.csv`
- Figures: `results/advanced/thesis_*.png`
- This report: `results/advanced/thesis_report.md`
