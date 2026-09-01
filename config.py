"""
AUV MPC Framework v2 — User Configuration
==========================================

Edit the two settings below to control language and controller variant.
Everything else in this file is the translation dictionary — do not edit it
unless you want to add or correct a translation.
"""

# ============================================================
#  USER SETTINGS  (edit these)
# ============================================================

# Language for all figure labels and plot text.
#   "en" = English  (use this to produce figures you can translate yourself)
#   "lv" = Latvian  (use this for the final thesis submission)
LANGUAGE = "lv"

# NMPC controller variant.
#   True  = patched offset-free NMPC (N=12, disturbance observer)  — RECOMMENDED
#   False = original NMPC (N=12, no observer)  — for comparison / reference only
USE_PATCHED_NMPC = True

# ============================================================
#  TRANSLATION DICTIONARY  (do not edit unless correcting)
# ============================================================

_STRINGS: dict[str, dict[str, str]] = {
    # ---- axis labels ----
    "depth_m":               {"en": "Depth [m]",              "lv": "Dziļums [m]"},
    "heading_deg":           {"en": "Heading [°]",            "lv": "Kurss [°]"},
    "heading_error_deg":     {"en": "Heading error [°]",      "lv": "Kursa kļūda [°]"},
    "speed_ms":              {"en": "Speed [m/s]",            "lv": "Ātrums [m/s]"},
    "time_s":                {"en": "Time [s]",               "lv": "Laiks [s]"},
    "rmse_log":              {"en": "RMSE [°] (log scale)",   "lv": "RMSE [°] (logaritmiskā ass)"},
    "cum_depth_err":         {"en": "Cumulative depth error [m·s]",   "lv": "Kumulatīvā dziļuma kļūda [m·s]"},
    "cum_hdg_err":           {"en": "Cumulative heading error [°·s]", "lv": "Kumulatīvā kursa kļūda [°·s]"},
    "ctrl_surface_angles":   {"en": "Control surface angles [°]",     "lv": "Stūres leņķis [°]"},
    "hdg_err_abs":           {"en": "|Heading error| [°]",    "lv": "|kursa kļūda| [°]"},
    "depth_error_m":         {"en": "Depth error [m]",        "lv": "Dziļuma kļūda [m]"},
    "depth_err_abs":         {"en": "|Depth error| [m]",      "lv": "|dziļuma kļūda| [m]"},
    "rmse_log_m":            {"en": "RMSE [m] (log scale)",   "lv": "RMSE [m] (logaritmiskā ass)"},
    "solve_time_ms":         {"en": "Solver time [ms]",       "lv": "Aprēķina laiks [ms]"},
    "solve_count":           {"en": "Solve count",            "lv": "Risinājumu skaits"},
    "cum_prob":              {"en": "Cumulative probability",  "lv": "Kumulatīvā varbūtība"},
    "hdg_continuous":        {"en": "Heading [°] (continuous)","lv": "Kurss [°] (nepārtraukts)"},

    # ---- subplot titles ----
    "depth_tracking":        {"en": "Depth tracking",         "lv": "Dziļums"},
    "heading_tracking":      {"en": "Heading tracking",       "lv": "Kurss"},
    "speed":                 {"en": "Speed",                  "lv": "Ātrums"},
    "ctrl_surfaces":         {"en": "Control surfaces",       "lv": "Stūres leņķis"},
    "cumulative_errors":     {"en": "Cumulative tracking errors", "lv": "Kumulatīvās kļūdas"},
    "heading_error_wrap":    {"en": "Heading error: wrap(heading − reference)",
                              "lv": "Kursa kļūda"},
    "distribution":          {"en": "Distribution",           "lv": "Sadalījums"},

    # ---- legend / annotation words ----
    "target":                {"en": "target",                 "lv": "mērķis"},
    "reference":             {"en": "reference",              "lv": "uzdotā vērtība"},
    "rudder":                {"en": "rudder",                 "lv": "spārns"},
    "stern":                 {"en": "stern",                  "lv": "dziļuma spārns"},
    "max_deviation":         {"en": "max. deviation",         "lv": "maks. novirze"},
    "deadline":              {"en": "deadline",               "lv": "periods"},
    "pct_on_time":           {"en": "% solves within deadline","lv": "% aprēķinu periodā"},

    # ---- regime / segment labels ----
    "regime_aggregate":      {"en": "Aggregate\n(full mission)",   "lv": "Kopā\n(visa misija)"},
    "regime_transient":      {"en": "Transient\n(±35 s)",          "lv": "Pārejas\n(±35 s)"},
    "regime_settled":        {"en": "Settled\n(holding)",          "lv": "Miera stāvoklis\n"},
    "seg_holding":           {"en": "holding",                     "lv": "miera stāvoklis"},
    "seg_turn":              {"en": "turn",                        "lv": "pagrieziens"},
    "transient_bar":         {"en": "Transient (±35 s after manoeuvre)",
                              "lv": "Pāreja (±35 s pēc manevra)"},
    "steadystate_bar":       {"en": "Steady-state (holding)",      "lv": "Miera stāvoklis"},

    # ---- cumulative error legend suffixes ----
    "int_ez":                {"en": "int|e_z|",              "lv": "int|kļ_z|"},
    "int_epsi":              {"en": "int|e_ψ|",              "lv": "int|kļ_ψ|"},
    "depth_label":           {"en": "depth",                 "lv": "dziļums"},
    "heading_label":         {"en": "heading",               "lv": "kurss"},
    "pid5_omitted":          {"en": "PID (5 Hz) omitted — saturates at ±15° in limit-cycle oscillation",
                              "lv": "PID (5 Hz) nav parādāts, jo pārsātina grafiku ar lielām svārstībām"},

    # ---- heading error y-label ----
    "heading_err_short":     {"en": "e_ψ [°]",              "lv": "kļ_ψ [°]"},

    # ---- scenario titles ----
    "s1_title":   {"en": "Scenario 1: Standard depth+heading (Vc=0.5 m/s)",
                   "lv": "1. scenārijs: Standarta dziļums+kurss (Vc=0.5 m/s)"},
    "s2_title":   {"en": "Scenario 2: Current Vc={vc} m/s",
                   "lv": "2. scenārijs: Straume Vc={vc} m/s"},
    "s3_title":   {"en": "Scenario 3: Heading change 0→{pd}° at constant 20 m depth (Vc=0.3 m/s)",
                   "lv": "3. scenārijs: Kursa maiņa 0→{pd}° nemainīgā 20 m dziļumā (Vc=0.3 m/s)"},
    "s4_title":   {"en": "Scenario 4: Multi-waypoint mission",
                   "lv": "4. scenārijs: Vairāku punktu misija"},
    "s5_title":   {"en": "Scenario 5: Depth step 0→50 m",
                   "lv": "5. scenārijs: Iegrimšana 0→50 m"},
    "s6_title":   {"en": "Scenario 6: Complex 8-segment mission\n(current 0.6 m/s from 150°, 600 s)",
                   "lv": "6. scenārijs: Sarežģīta 8-posmu misija\n(straume 0.6 m/s no 150°, 600 s)"},

    # ---- timing figure stats / legend text ----
    "deadline_label":    {"en": "Deadline {ms:.0f} ms",      "lv": "Periods {ms:.0f} ms"},
    "ecdf_mean_label":   {"en": "mean",                      "lv": "vidējais"},
    "stats_mean":        {"en": "Mean",                      "lv": "Vidējais"},
    "stats_deadline_pct":{"en": "≤ deadline",                "lv": "≤ periods"},
    "stats_ratio":       {"en": "NMPC / PID ratio",          "lv": "NMPC / PID attiecība"},

    # ---- timing figure titles ----
    "timing_main_title":  {"en": "Controller Compute Time Distribution — Scenario 6",
                           "lv": "Vadības aprēķinu laika sadalījums 6. scenārijā"},
    "timing_hist_title":  {"en": "Histogram (log scale)",   "lv": "Histogramma (log)"},
    "timing_ecdf_title":  {"en": "ECDF (log scale)",        "lv": "ECDF (log)"},
    "timing_xlab":        {"en": "Compute time [ms] (log scale)",
                           "lv": "Aprēķinu laiks [ms] (log)"},
    "timing_ylab_hist":   {"en": "Count",                   "lv": "Skaits"},
    "timing_ylab_ecdf":   {"en": "Cumulative probability [%]",
                           "lv": "Summārā varbūtība [%]"},
    "timing_time_xlab":   {"en": "Mission time [s]",        "lv": "Misijas laiks [s]"},
    "timing_time_ylab":   {"en": "Compute time [ms] (log scale)",
                           "lv": "Aprēķinu laiks [ms] (log)"},

    # ---- mission overview event annotations ----
    "event_depth_coupling":  {"en": "depth coupling {val:+.0f}°",
                              "lv": "dziļuma sasaiste {val:+.0f}°"},
    "event_reversal":        {"en": "reversal overshoot {val:+.0f}°",
                              "lv": "ass pagrieziens {val:+.0f}°"},

    # ---- thesis analysis figure titles ----
    "thesis_regime_title":    {"en": "Heading RMSE by regime: aggregate masks the true controller ranking",
                               "lv": "Kursa RMSE pa darbības režīmiem"},
    "thesis_ecdf_title":      {"en": "Heading error ECDF — heavy-tail behaviour",
                               "lv": "Kursa kļūdas ECDF"},
    "thesis_segment_title":   {"en": "Heading IAE by mission segment",
                               "lv": "Kursa IAE pa misijas posmiem"},
    "thesis_zoom_rev_title":  {"en": "Sharp heading reversal (495–560 s)",
                               "lv": "Ass pagrieziens (495–560 s)"},
    "thesis_zoom_depth_title":{"en": "Depth-coupling heading excursion (255–310 s)",
                               "lv": "Kursa novirze dziļuma maiņas laikā (255–310 s)"},
    "thesis_overview_title":  {"en": "Scenario 6: Heading over 600 s complex mission",
                               "lv": "6. scenārijs: Kurss 600 s sarežģītā misijā"},

    # ---- thesis_analysis: depth-channel twins ----
    "thesis_regime_depth_title":  {"en": "Depth RMSE by regime: aggregate masks the true controller ranking",
                                   "lv": "Dziļuma RMSE pa darbības režīmiem: kopējā vērtība slēpj patieso secību"},
    "thesis_ecdf_depth_title":    {"en": "Depth error ECDF — heavy-tail behaviour",
                                   "lv": "Dziļuma kļūdas ECDF — smagās astes"},
    "thesis_segment_depth_title": {"en": "Depth IAE by mission segment",
                                   "lv": "Dziļuma IAE pa misijas posmiem"},
    "thesis_zoom_rev_depth_title":  {"en": "Depth during the sharp heading reversal (495–560 s)",
                                     "lv": "Dziļums ass pagrieziena laikā (495–560 s)"},
    "thesis_zoom_depth_dz_title":   {"en": "Depth during the 40→10 m ascent (255–310 s)",
                                     "lv": "Dziļums 40→10 m manevra laikā (255–310 s)"},
    "thesis_overview_depth_title":  {"en": "Scenario 6: Depth over 600 s complex mission",
                                     "lv": "6. scenārijs: Dziļums 600 s sarežģītā misijā"},
    # ---- figure titles ----
    "steady_vs_transient_title":  {"en": "Steady-state vs. transient tracking error",
                                   "lv": "Miera stāvokļa un pārejas režīma kļūdas"},
    "regime_comparison_title":    {"en": "Tracking error by operating regime",
                                   "lv": "Kļūda pa darbības režīmiem"},
    "segment_breakdown_title":    {"en": "Heading error by mission segment",
                                   "lv": "Kursa kļūdas sadalījums pa misijas posmiem"},
    "ecdf_title":                 {"en": "Heading error empirical CDF",
                                   "lv": "Kursa kļūdas sadalījuma funkcija (ECDF)"},
    "mission_overview_title":     {"en": "Complex mission: heading and heading error",
                                   "lv": "Sarežģīta misija: kurss un kursa kļūda"},
    "solver_timing_title":        {"en": "NMPC solver time vs. real-time deadline",
                                   "lv": "NMPC risinātāja laiks un reāllaika periods"},
    "pareto_title":               {"en": "Accuracy vs. compute cost trade-off",
                                   "lv": "Precizitātes un skaitļošanas izmaksu kompromiss"},
    "pareto_xlabel":              {"en": "P99 solver time [ms]",
                                   "lv": "P99 risinājuma laiks [ms]"},
    "pareto_ylabel":              {"en": "Heading RMSE [°]",
                                   "lv": "Kursa RMSE [°]"},

    # ---- analyze_s6 figure titles / notes ----
    "s6_regime_title":     {"en": "Heading RMSE by regime: aggregate masks the regime switch",
                            "lv": "Kursa RMSE pa darbības režīmiem"},
    # ---- depth counterparts of the scenario-6 deep-dive figures ----
    "s6_regime_depth_title":  {"en": "Depth RMSE by operating regime",
                               "lv": "Dziļuma RMSE pa darbības režīmiem"},
    "s6_segment_depth_title": {"en": "Depth error by mission segment",
                               "lv": "Dziļuma kļūdas sadalījums pa misijas posmiem"},
    "s6_ecdf_depth_title":    {"en": "Depth error empirical CDF",
                               "lv": "Dziļuma kļūdas sadalījuma funkcija (ECDF)"},
    "s6_overview_depth_title": {"en": "Complex mission: depth and depth error",
                                "lv": "Sarežģīta misija: dziļums un dziļuma kļūda"},
    "s6_zoom_rev_title":   {"en": "Sharp heading reversal (330°→45°→0°)",
                            "lv": "Ass pagrieziens (330°→45°→0°)"},
    "s6_zoom_rev_note":    {"en": "With reference preview the NMPC follows the turn almost exactly; "
                                  "the PID lags slightly behind the reference during the manoeuvre.",
                            "lv": "Pateicoties uzdotās trajektorijas prognozei, NMPC pagriezienam seko "
                                  "gandrīz precīzi; PID manevra laikā nedaudz atpaliek no uzdotās vērtības."},
    "s6_zoom_depth_title": {"en": "Depth-coupling heading excursion (constant heading 200°)",
                            "lv": "Kursa novirze dziļuma sasaistes dēļ (konstants kurss 200°)"},
    "s6_zoom_depth_note":  {"en": "Heading is constant (200°) in this segment, but depth changes 40→10 m. "
                                  "The reduced predictor ignores pitch→yaw coupling; the disturbance "
                                  "observer compensates it, keeping the NMPC heading within ~0.05°, "
                                  "while the PID slowly drifts.",
                            "lv": "Kurss šajā posmā ir konstants (200°), bet dziļums mainās 40→10 m. "
                                  "Reducētais prognozes modelis neietver garensveres–kursa sasaisti — "
                                  "to kompensē traucējumu novērotājs: NMPC kursa novirze paliek ~0,05° "
                                  "robežās, kamēr PID lēnām dreifē."},

    # ---- compare_nmpc_patch figure titles ----
    "patch_regime_title":     {"en": "Heading RMSE by regime: original vs. patched NMPC",
                               "lv": "Kursa RMSE pa darbības režīmiem: sākotnējais pret uzlaboto NMPC"},
    "patch_zoom_depth_title": {"en": "Depth-induced heading excursion (255–310 s): original vs. patched",
                               "lv": "Kursa novirze dziļuma manevra laikā (255–310 s): sākotnējais pret uzlaboto"},
    "patch_zoom_rev_title":   {"en": "Sharp reversal (495–560 s): original vs. patched",
                               "lv": "Ass pagrieziens (495–560 s): sākotnējais pret uzlaboto"},

    # ---- compare_nmpc_patch: depth-channel twins ----
    "patch_regime_depth_title":  {"en": "Depth RMSE by regime: original vs. patched NMPC",
                                  "lv": "Dziļuma RMSE pa darbības režīmiem: sākotnējais pret uzlaboto NMPC"},
    "patch_segment_depth_title": {"en": "Depth error by mission segment: original vs. patched NMPC",
                                  "lv": "Dziļuma kļūda pa misijas posmiem: sākotnējais pret uzlaboto NMPC"},
    "patch_zoom_depth_dz_title": {"en": "Depth tracking during the 40→10 m ascent (255–310 s): original vs. patched",
                                  "lv": "Dziļuma noturēšana 40→10 m manevra laikā (255–310 s): sākotnējais pret uzlaboto"},
    "patch_zoom_rev_dz_title":   {"en": "Depth tracking during the sharp reversal (495–560 s): original vs. patched",
                                  "lv": "Dziļuma noturēšana ass pagrieziena laikā (495–560 s): sākotnējais pret uzlaboto"},

    # ---- solver timing timeline (thesis_analysis) ----
    "timing_timeline_title": {"en": "Controller Compute Time — Scenario 6 (600 s mission)\n"
                                    "NMPC at 5 Hz · PID at 50 Hz · PID at 5 Hz (per-call, one row per 0.2 s)",
                              "lv": "Vadības aprēķina laiks — 6. scenārijs (600 s misija)\n"
                                    "NMPC 5 Hz · PID 50 Hz · PID 5 Hz (pa izsaukumiem, viens punkts uz 0,2 s)"},
    "timing_all_ok":         {"en": "All {n} NMPC calls within {ms:.0f} ms deadline",
                              "lv": "Visi {n} NMPC aprēķini iekļaujas {ms:.0f} ms periodā"},
    "timing_tick_ok":        {"en": "ok",    "lv": "periodā"},
    "timing_tick_over":      {"en": "over",  "lv": "pāri"},
    "rolling_mean_label":    {"en": "rolling mean (20 calls)",
                              "lv": "slīdošais vidējais (20 izsaukumi)"},
    "seg_ascent":            {"en": "ascent", "lv": "pacelšanās"},

    # ---- horizontal trajectory (top view) ----
    "s6_xy_title":  {"en": "Scenario 6: horizontal trajectory (top view)",
                     "lv": "6. scenārijs: horizontālā trajektorija (skats no augšas)"},
    "north_m":      {"en": "North x [m]",  "lv": "x [m]"},
    "east_m":       {"en": "East y [m]",   "lv": "y [m]"},
    "start":        {"en": "start",        "lv": "starts"},
    "s6_xy_note":   {"en": "Illustration only: the mission defines depth/heading/speed references, not an\n"
                           "x/y route, so no desired x/y trajectory exists and none is evaluated.\n"
                           "x/y is not fed back; the current drift is identical for both controllers.\n"
                           "Final-position separation PID vs NMPC: {d:.0f} m.",
                     "lv": "Tikai ilustrācija: misija definē dziļuma/kursa/ātruma atsauces, nevis x/y\n"
                           "maršrutu, tāpēc vēlamā x/y trajektorija neeksistē un netiek vērtēta.\n"
                           "x/y netiek padots atgriezeniskajā saitē; straumes nese abiem ir identiska.\n"
                           "PID un NMPC beigu pozīciju attālums: {d:.0f} m.",},
    "seg_switches": {"en": "segment switches",  "lv": "posmu maiņas"},

    # ---- report / table column headers ----
    "col_controller":        {"en": "Controller",             "lv": "Vadības algoritms"},
    "col_max_e":             {"en": "max |e| [°]",            "lv": "maks |e| [°]"},
    "col_aggregate_rmse":    {"en": "Aggregate RMSE [°]",     "lv": "Kopējā RMSE [°]"},
    "col_transient_rmse":    {"en": "Transient RMSE [°]",     "lv": "Pārejas RMSE [°]"},
    "col_settled_rmse":      {"en": "Settled RMSE [°]",       "lv": "Miera stāvokļa RMSE [°]"},
    # metre-unit twins for the depth channel tables
    "col_max_e_m":           {"en": "max |e| [m]",            "lv": "maks |e| [m]"},
    "col_aggregate_rmse_m":  {"en": "Aggregate RMSE [m]",     "lv": "Kopējā RMSE [m]"},
    "col_transient_rmse_m":  {"en": "Transient RMSE [m]",     "lv": "Pārejas RMSE [m]"},
    "col_settled_rmse_m":    {"en": "Settled RMSE [m]",       "lv": "Miera stāvokļa RMSE [m]"},
    "col_metric":            {"en": "Metric",                 "lv": "Mērījums"},
    "col_improvement":       {"en": "Improvement [%]",        "lv": "Uzlabojums [%]"},
}


def T(key: str) -> str:
    """Return the translation for *key* in the active LANGUAGE.

    Falls back to English if the key or the active language is missing.
    Returns the key itself if no translation exists at all, so missing keys
    are visible in figures rather than raising an exception.
    """
    entry = _STRINGS.get(key)
    if entry is None:
        return key
    return entry.get(LANGUAGE) or entry.get("en") or key
