# EXP-A — Montekarlo noturības analīze

## Kopsavilkums

Veikti 30 testi ar neatkarīgiem, ar sēklu noteiktiem nejaušiem straumes trokšņiem (Gausa–Markova ātrums + viļņu komponente + Gausa–Markova virziens). Katrā testā abi kontrolieri saņem identisku trokšņa realizāciju, tāpēc salīdzinājums ir pārī (paired).

- Kopējais kursa RMSE: vidējā starpība (PID−NMPC) = +1.152, Cohen dz = 17.58, paired-t p = 6.77e-38, Wilcoxon p = 1.86e-09 (n=30).
- Miera stāvokļa kursa RMSE: vidējā starpība (PID−NMPC) = +0.500, Cohen dz = 11.76, paired-t p = 7.40e-33, Wilcoxon p = 1.86e-09 (n=30).

Pozitīva vidējā starpība nozīmē, ka PID kļūda ir lielāka (NMPC labāks); negatīva — pretēji. |Cohen dz| ≳ 0,8 norāda lielu efektu.


## Metriku tabula

| Metrika | Kontrolieris | Vidējais | Std | Mediāna | CI_zem | CI_virs |
|---|---|---|---|---|---|---|
| Dziļuma RMSE [m] | PID (pielāgots) | 1.772 | 0.211 | 1.737 | 1.694 | 1.851 |
| Dziļuma RMSE [m] | NMPC offset-free (N=12) | 0.036 | 0.017 | 0.028 | 0.029 | 0.042 |
| Kursa RMSE [°] | PID (pielāgots) | 1.491 | 0.059 | 1.475 | 1.469 | 1.513 |
| Kursa RMSE [°] | NMPC offset-free (N=12) | 0.339 | 0.078 | 0.347 | 0.309 | 0.368 |
| Miera stāvokļa kursa RMSE [°] | PID (pielāgots) | 0.700 | 0.043 | 0.692 | 0.684 | 0.716 |
| Miera stāvokļa kursa RMSE [°] | NMPC offset-free (N=12) | 0.200 | 0.072 | 0.199 | 0.173 | 0.227 |
| Kursa max|e| [°] | PID (pielāgots) | 5.174 | 0.300 | 5.159 | 5.063 | 5.286 |
| Kursa max|e| [°] | NMPC offset-free (N=12) | 1.648 | 0.437 | 1.524 | 1.485 | 1.811 |
