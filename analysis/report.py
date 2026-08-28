"""
Result-report generation
=========================

Helpers that turn the dictionaries produced by ``analysis.metrics`` into the
two artefacts a thesis actually needs: a machine-readable CSV and a
human-readable Markdown table.  Keeping this separate from the metric
computation means the same numbers can be re-formatted without re-running any
simulation.
"""

from __future__ import annotations

import csv
from typing import Mapping, Sequence

import numpy as np


def _fmt(x, nd=3):
    if isinstance(x, float):
        if not np.isfinite(x):
            return "—"
        return f"{x:.{nd}f}"
    return str(x)


def metrics_table_md(
    rows: Sequence[Mapping[str, object]],
    columns: Sequence[str],
    *,
    headers: Sequence[str] | None = None,
    nd: int = 3,
) -> str:
    """Render a list of metric dicts as a GitHub-flavoured Markdown table."""
    headers = list(headers or columns)
    out = ["| " + " | ".join(headers) + " |",
           "|" + "|".join(["---"] * len(headers)) + "|"]
    for r in rows:
        out.append("| " + " | ".join(_fmt(r.get(c, ""), nd) for c in columns) + " |")
    return "\n".join(out)


def write_csv(rows: Sequence[Mapping[str, object]], columns: Sequence[str], path: str) -> str:
    """Write metric dicts to a CSV with an explicit column order."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(columns), extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return path


def write_markdown_report(path: str, title: str, sections: Sequence[tuple[str, str]]) -> str:
    """Assemble a Markdown report from (heading, body) sections."""
    parts = [f"# {title}", ""]
    for heading, body in sections:
        parts += [f"## {heading}", "", body, ""]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    return path
