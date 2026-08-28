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
#   True  = patched offset-free NMPC (N=30, disturbance observer)  — RECOMMENDED
#   False = original NMPC (N=20, no observer)  — for comparison / reference only
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
    "rmse_log":              {"en": "RMSE [°] (log scale)",   "lv": "RMSE [°] (logaritmiska ass)"},
    "cum_depth_err":         {"en": "Cumulative depth error [m·s]",   "lv": "Summārā dziļuma kļūda [m*s]"},
    "cum_hdg_err":           {"en": "Cumulative heading error [°·s]", "lv": "Summārā kursa kļūda [deg*s]"},
    "ctrl_surface_angles":   {"en": "Control surface angles [°]",     "lv": "Stūres leņķi [deg]"},
    "hdg_err_abs":           {"en": "|Heading error| [°]",    "lv": "|kursa kļūda| [°]"},
    "solve_time_ms":         {"en": "Solver time [ms]",       "lv": "Aprēķinu periods [ms]"},
    "solve_count":           {"en": "Solve count",            "lv": "Solu skaits"},
    "cum_prob":              {"en": "Cumulative probability",  "lv": "Summārā varbūtība"},
    "hdg_continuous":        {"en": "Heading [°] (continuous)","lv": "Kurss [°] (nepārtraukts)"},

    # ---- subplot titles ----
    "depth_tracking":        {"en": "Depth tracking",         "lv": "Dziļuma novirze"},
    "heading_tracking":      {"en": "Heading tracking",       "lv": "Kursa novirze"},
    "speed":                 {"en": "Speed",                  "lv": "Ātrums"},
    "ctrl_surfaces":         {"en": "Control surfaces",       "lv": "Vadības ievades leņķi"},
    "cumulative_errors":     {"en": "Cumulative tracking errors", "lv": "Summārā novirze"},
    "heading_error_wrap":    {"en": "Heading error: wrap(heading − reference)",
                              "lv": "Kursa kļūda"},
    "distribution":          {"en": "Distribution",           "lv": "Sadalījums"},

    # ---- legend / annotation words ----
    "target":                {"en": "target",                 "lv": "mērķis"},
    "reference":             {"en": "reference",              "lv": "atsauce"},
    "rudder":                {"en": "rudder",                 "lv": "spārns"},
    "stern":                 {"en": "stern",                  "lv": "pakaļ."},
    "max_deviation":         {"en": "max. deviation",         "lv": "maks. novirze"},
    "deadline":              {"en": "deadline",               "lv": "maks. termiņš"},
    "pct_on_time":           {"en": "% solves within deadline","lv": "% atrisin. termiņā"},

    # ---- regime / segment labels ----
    "regime_aggregate":      {"en": "Aggregate\n(full mission)",   "lv": "Kopā\n(visa misija)"},
    "regime_transient":      {"en": "Transient\n(±35 s)",          "lv": "Pārejas\n(±35 s)"},
    "regime_settled":        {"en": "Settled\n(holding)",          "lv": "Balansa stāvoklis\n"},
    "seg_holding":           {"en": "holding",                     "lv": "balansēšana"},
    "seg_turn":              {"en": "turn",                        "lv": "pagrieziens"},
    "transient_bar":         {"en": "Transient (±35 s after manoeuvre)",
                              "lv": "Pāreja (±35 s pēc manevra)"},
    "steadystate_bar":       {"en": "Steady-state (holding)",      "lv": "Balansa stāvoklis"},

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
    "s2_title":   {"en": "Scenario 2: Disturbance Vc={vc} m/s",
                   "lv": "2. scenārijs: Traucējums Vc={vc} m/s"},
    "s3_title":   {"en": "Scenario 3: Heading change 0→{pd}° (Vc=0.3 m/s)",
                   "lv": "3. scenārijs: Kursa maiņa 0→{pd}° (Vc=0.3 m/s)"},
    "s4_title":   {"en": "Scenario 4: Multi-waypoint mission ({name})",
                   "lv": "4. scenārijs: Vairāku punktu misija ({name})"},
    "s5_title":   {"en": "Scenario 5: Depth step 0→50 m",
                   "lv": "5. scenārijs: Iegrimšana 0→50 m"},
    "s6_title":   {"en": "Scenario 6: Complex 8-segment mission\n(current 0.6 m/s from 150°, 600 s)",
                   "lv": "6. scenārijs: Sarežģīta 8-posmu misija\n(straume 0.6 m/s no 150°, 600 s)"},

    # ---- timing figure stats / legend text ----
    "deadline_label":    {"en": "Deadline {ms:.0f} ms",      "lv": "Termiņš {ms:.0f} ms"},
    "ecdf_mean_label":   {"en": "mean",                      "lv": "vidējais"},
    "stats_mean":        {"en": "Mean",                      "lv": "Vidējais"},
    "stats_deadline_pct":{"en": "≤ deadline",                "lv": "≤ termiņš"},
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
                              "lv": "dziļuma balanss {val:+.0f}°"},
    "event_reversal":        {"en": "reversal overshoot {val:+.0f}°",
                              "lv": "lielākā novirze {val:+.0f}°"},

    # ---- thesis analysis figure titles ----
    "thesis_regime_title":    {"en": "Heading RMSE by regime: aggregate masks the true controller ranking",
                               "lv": "Kursa RMSE pa modeļiem"},
    "thesis_ecdf_title":      {"en": "Heading error ECDF — heavy-tail behaviour",
                               "lv": "Kursa kļūda ECDF"},
    "thesis_segment_title":   {"en": "Heading IAE by mission segment",
                               "lv": "Kursa IAE pa misijas posmiem"},
    "thesis_zoom_rev_title":  {"en": "Sharp heading reversal overshoot (490–570 s)",
                               "lv": "Kursa novirze pie asa pagrieziena (490–570 s)"},
    "thesis_zoom_depth_title":{"en": "Depth-coupling heading excursion (260–340 s)",
                               "lv": "Kursa novirze iegrimšanas laikā (260–340 s)"},
    "thesis_overview_title":  {"en": "Scenario 6: Heading tracking over 600 s complex mission",
                               "lv": "6. scenārijs: Kursa novirze 600 s sarežģītā misijā"},

    # ---- figure titles ----
    "steady_vs_transient_title":  {"en": "Steady-state vs. transient tracking error",
                                   "lv": "Balansēšanas un pārejas stāvokļa novirzes"},
    "regime_comparison_title":    {"en": "Tracking error by operating regime",
                                   "lv": "Modeļa novirze"},
    "segment_breakdown_title":    {"en": "Heading error by mission segment",
                                   "lv": "Kursa kļūdas sadalījums pa misijas posmiem"},
    "ecdf_title":                 {"en": "Heading error empirical CDF",
                                   "lv": "Kursa kļūdas sadalījuma funkcija (ECDF)"},
    "mission_overview_title":     {"en": "Complex mission: heading tracking and error",
                                   "lv": "Kursa novirze sarežģītas misijas laikā"},
    "solver_timing_title":        {"en": "NMPC solver time vs. real-time deadline",
                                   "lv": "NMPC algoritma laiks un reāllaika ierobežojums"},
    "pareto_title":               {"en": "Accuracy vs. compute cost trade-off",
                                   "lv": "Precizitātes un skaitļošanas izmaksu salīdzinājums"},
    "pareto_xlabel":              {"en": "P99 solver time [ms]",
                                   "lv": "P99 risinājuma laiks [ms]"},
    "pareto_ylabel":              {"en": "Heading RMSE [°]",
                                   "lv": "Kursa RMSE [°]"},

    # ---- report / table column headers ----
    "col_controller":        {"en": "Controller",             "lv": "Vadības algoritms"},
    "col_max_e":             {"en": "max |e| [°]",            "lv": "maks |e| [°]"},
    "col_aggregate_rmse":    {"en": "Aggregate RMSE [°]",     "lv": "Kopējā RMSE [°]"},
    "col_transient_rmse":    {"en": "Transient RMSE [°]",     "lv": "Pārejas RMSE [°]"},
    "col_settled_rmse":      {"en": "Settled RMSE [°]",       "lv": "Balansēšanas RMSE [°]"},
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
