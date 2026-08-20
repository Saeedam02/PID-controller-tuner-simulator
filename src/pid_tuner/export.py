"""Small helpers for exporting simulation data and configuration.

The Streamlit UI uses in-memory strings for download buttons, while the CLI can
write the same data to disk.  Keeping export logic here prevents duplicated CSV
column ordering and JSON formatting.
"""

from __future__ import annotations

import csv
import io
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any


DEFAULT_RESULT_COLUMNS = (
    "t",
    "setpoint",
    "output",
    "measurement",
    "control",
    "p",
    "i",
    "d",
    "error",
    "saturated",
    "disturbance",
)


def results_to_csv(
    results: Mapping[str, Sequence[Any]],
    columns: Sequence[str] = DEFAULT_RESULT_COLUMNS,
) -> str:
    """Serialize parallel simulation result series to CSV text."""
    missing = [column for column in columns if column not in results]
    if missing:
        raise KeyError(f"Missing result columns: {', '.join(missing)}")

    lengths = {len(results[column]) for column in columns}
    if len(lengths) != 1:
        raise ValueError("All exported result columns must have the same length")

    buffer = io.StringIO()
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(columns)
    for row in zip(*(results[column] for column in columns), strict=True):
        writer.writerow(row)
    return buffer.getvalue()


def configuration_to_json(configuration: Mapping[str, Any]) -> str:
    """Serialize a run configuration with stable human-readable formatting."""
    return json.dumps(configuration, indent=2, sort_keys=True, ensure_ascii=False)


def write_text(path: str | Path, content: str) -> Path:
    """Write UTF-8 text and return the resolved path for CLI reporting."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content, encoding="utf-8")
    return target.resolve()
