# Scenario 6 in-depth analysis (heading and depth tracking)

## Summary

- **Aggregate heading RMSE:** PID (tuned) = 1.52°, NMPC offset-free (N=12) = 0.25° — aggregate metric makes the second controller appear worse.
- **Steady-state holding error (mean across segments):** PID (tuned) = 0.040°, NMPC offset-free (N=12) = 0.045° — NMPC is ~1× more precise in holding.
- **Root cause:** aggregate RMSE is dominated by two transient events — reversal overshoot (~1°, t≈195 s) and depth-coupling excursion (~0°, t≈273 s).
- **Aggregate depth RMSE:** PID (tuned) = 1.989 m, NMPC offset-free (N=12) = 0.047 m; holding error 0.387 m vs 0.018 m — the same transient-vs-holding split as heading, in the depth channel.


## Aggregate metrics (heading)

| Vadības algoritms | RMSE [°] | MAE [°] | IAE [°·s] | maks |e| [°] |
|---|---|---|---|---|
| Fossen PID/SMC | 33.62 | 22.29 | 13371.63 | 91.54 |
| PID (tuned) | 1.52 | 0.70 | 422.58 | 5.44 |
| NMPC offset-free (N=12) | 0.25 | 0.13 | 77.85 | 1.44 |

## Heading error by regime

| Vadības algoritms | Kopējā RMSE [°] | Pārejas RMSE [°] | Miera stāvokļa RMSE [°] |
|---|---|---|---|
| Fossen PID/SMC | 33.622 | 44.294 | 23.349 |
| PID (tuned) | 1.524 | 2.785 | 0.049 |
| NMPC offset-free (N=12) | 0.249 | 0.444 | 0.052 |

## Depth tracking (aggregate and by regime)

| Vadības algoritms | RMSE [m] | MAE [m] | IAE [m·s] | maks |e| [m] |
|---|---|---|---|---|
| Fossen PID/SMC | 9.911 | 8.679 | 5207.649 | 20.960 |
| PID (tuned) | 1.989 | 1.248 | 748.930 | 6.898 |
| NMPC offset-free (N=12) | 0.047 | 0.027 | 16.407 | 0.319 |

| Vadības algoritms | Kopējā RMSE [m] | Pārejas RMSE [m] | Miera stāvokļa RMSE [m] |
|---|---|---|---|
| Fossen PID/SMC | 9.9106 | 9.7073 | 9.2539 |
| PID (tuned) | 1.9892 | 2.6679 | 0.5003 |
| NMPC offset-free (N=12) | 0.0475 | 0.0726 | 0.0135 |

## Note

All values from `s6_kursa_kludas_analize.csv` (600 s, 50 Hz). Figures in `results/advanced/`; the depth counterparts carry the `_dzilums` suffix.
