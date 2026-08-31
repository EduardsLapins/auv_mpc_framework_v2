# Scenario 6 in-depth analysis (heading tracking)

## Summary

- **Aggregate heading RMSE:** PID (tuned) = 1.52°, NMPC offset-free (N=12) = 0.25° — aggregate metric makes the second controller appear worse.
- **Steady-state holding error (mean across segments):** PID (tuned) = 0.040°, NMPC offset-free (N=12) = 0.045° — NMPC is ~1× more precise in holding.
- **Root cause:** aggregate RMSE is dominated by two transient events — reversal overshoot (~1°, t≈195 s) and depth-coupling excursion (~0°, t≈273 s).


## Aggregate metrics

| Vadības algoritms | RMSE [°] | MAE [°] | IAE [°·s] | maks |e| [°] |
|---|---|---|---|---|
| Fossen PID/SMC | 33.62 | 22.29 | 13371.63 | 91.54 |
| PID (tuned) | 1.52 | 0.70 | 422.58 | 5.44 |
| NMPC offset-free (N=12) | 0.25 | 0.13 | 77.85 | 1.44 |

## Tracking error by regime

| Vadības algoritms | Kopējā RMSE [°] | Pārejas RMSE [°] | Miera stāvokļa RMSE [°] |
|---|---|---|---|
| Fossen PID/SMC | 33.622 | 44.294 | 23.349 |
| PID (tuned) | 1.524 | 2.785 | 0.049 |
| NMPC offset-free (N=12) | 0.249 | 0.444 | 0.052 |

## Note

All values from `s6_kursa_kludas_analize.csv` (600 s, 50 Hz). Figures in `results/advanced/`.
