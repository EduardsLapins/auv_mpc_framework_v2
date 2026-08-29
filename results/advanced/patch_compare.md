# NMPC patch A/B comparison (Scenario 6)

## Summary

Identical Scenario-6 mission with three controllers. The patched NMPC adds an offset-free yaw disturbance observer (for depth-coupling drift), a heavier terminal yaw-rate penalty, and a longer horizon (for reversal overshoot).

Patched NMPC (N=12) solver time: mean 13.7 ms, P99 27.3 ms, max 55.3 ms; 100.0% of solves within deadline (200 ms).

## Results

| Mērījums | PID | NMPC_orig | NMPC_patch | Uzlabojums [%] |
|---|---|---|---|---|
| Aggregate heading RMSE [°] | 1.435 | 1.609 | 0.249 | 84.506 |
| Settled heading RMSE [°] | 0.585 | 0.053 | 0.052 | 1.151 |
| Depth-coupling max|e| (255–310 s) [°] | 0.696 | 0.049 | 0.047 | 4.311 |
| Reversal max|e| (495–560 s) [°] | 2.063 | 1.953 | 0.918 | 52.999 |

## Note

Positive 'Improvement [%]' means the patch reduced the metric. All values use the same definitions as `analyze_s6.py`. Figures in `results/advanced/` with prefix `patch_`.
