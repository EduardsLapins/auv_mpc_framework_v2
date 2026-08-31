# EXP-A — Montekarlo noturības analīze

## Kopsavilkums

Veikti 30 testi ar neatkarīgiem, ar sēklu noteiktiem nejaušiem straumes trokšņiem (Gausa–Markova ātrums + viļņu komponente + Gausa–Markova virziens). Katrā testā visi kontrolieri saņem identisku trokšņa realizāciju, tāpēc salīdzinājums ir pārī (paired).

- Kopējais kursa RMSE (Fossen PID/SMC pret NMPC): vidējā starpība (Fossen PID/SMC−NMPC) = +36.091, Cohen dz = 394.78, paired-t p = 4.56e-77, Wilcoxon p = 1.86e-09 (n=30).
- Miera stāvokļa kursa RMSE (Fossen PID/SMC pret NMPC): vidējā starpība (Fossen PID/SMC−NMPC) = +36.802, Cohen dz = 408.53, paired-t p = 1.69e-77, Wilcoxon p = 1.86e-09 (n=30).
- Kopējais kursa RMSE (PID (pielāgots) pret NMPC): vidējā starpība (PID (pielāgots)−NMPC) = +1.304, Cohen dz = 19.39, paired-t p = 3.95e-39, Wilcoxon p = 1.86e-09 (n=30).
- Miera stāvokļa kursa RMSE (PID (pielāgots) pret NMPC): vidējā starpība (PID (pielāgots)−NMPC) = +0.636, Cohen dz = 16.11, paired-t p = 8.44e-37, Wilcoxon p = 1.86e-09 (n=30).

Pozitīva vidējā starpība nozīmē, ka PID kļūda ir lielāka (NMPC labāks); negatīva — pretēji. |Cohen dz| ≳ 0,8 norāda lielu efektu.


## Metriku tabula

| Metrika | Kontrolieris | Vidējais | Std | Mediāna | CI_zem | CI_virs |
|---|---|---|---|---|---|---|
| Dziļuma RMSE [m] | Fossen PID/SMC | 9.519 | 0.024 | 9.518 | 9.510 | 9.528 |
| Dziļuma RMSE [m] | PID (pielāgots) | 2.128 | 0.214 | 2.100 | 2.048 | 2.208 |
| Dziļuma RMSE [m] | NMPC offset-free (N=12) | 0.036 | 0.017 | 0.028 | 0.029 | 0.042 |
| Kursa RMSE [°] | Fossen PID/SMC | 36.430 | 0.034 | 36.424 | 36.417 | 36.442 |
| Kursa RMSE [°] | PID (pielāgots) | 1.643 | 0.064 | 1.626 | 1.619 | 1.667 |
| Kursa RMSE [°] | NMPC offset-free (N=12) | 0.339 | 0.078 | 0.347 | 0.309 | 0.368 |
| Miera stāvokļa kursa RMSE [°] | Fossen PID/SMC | 37.002 | 0.041 | 36.997 | 36.986 | 37.017 |
| Miera stāvokļa kursa RMSE [°] | PID (pielāgots) | 0.836 | 0.048 | 0.823 | 0.818 | 0.854 |
| Miera stāvokļa kursa RMSE [°] | NMPC offset-free (N=12) | 0.200 | 0.072 | 0.199 | 0.173 | 0.227 |
| Kursa max|e| [°] | Fossen PID/SMC | 95.349 | 0.225 | 95.377 | 95.265 | 95.433 |
| Kursa max|e| [°] | PID (pielāgots) | 5.610 | 0.315 | 5.575 | 5.492 | 5.728 |
| Kursa max|e| [°] | NMPC offset-free (N=12) | 1.648 | 0.437 | 1.524 | 1.485 | 1.811 |
