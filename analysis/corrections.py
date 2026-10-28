"""Parse a batch's corrected/CORRECTIONS.md table.

Row shape: | date_utc | file | session_id | column | old value | new value | reason | by |
The session_id cell takes several forms (D8):
  `fmb2_s3c4_run01`      a run name            -> ref_kind = "run_id"
  `20261019T230955Z`     a session timestamp   -> ref_kind = "session_id"
  *(all rows)*           batch-wide            -> ref_kind = "all_rows"
  `(blank run_id)`       the blank obs row     -> ref_kind = "blank_run_id"
  *(example)* ...        template row          -> skipped
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

_SESSION_RX = re.compile(r"^2026\d{4}T\d{6}Z$")
_ROW_RX = re.compile(r"^\|\s*2026-\d\d-\d\d\s*\|")


@dataclass(frozen=True)
class Correction:
    batch_id: str
    date_utc: str
    file: str
    ref_kind: str          # run_id | session_id | all_rows | blank_run_id
    ref: str               # the cleaned reference text
    columns: tuple[str, ...]
    column_unspecified: bool
    old_value: str
    new_value: str
    reason: str
    by: str
    line_no: int

    def as_record(self) -> dict:
        return {
            "date": self.date_utc, "file": self.file, "columns": list(self.columns),
            "old": self.old_value, "new": self.new_value, "line": self.line_no,
            "column_unspecified": self.column_unspecified,
        }


@dataclass
class CorrectionsParse:
    corrections: list[Correction] = field(default_factory=list)
    skipped_template_rows: int = 0
    malformed_rows: list[tuple[int, str]] = field(default_factory=list)


def _clean(cell: str) -> str:
    return cell.strip().strip("`").strip()


def _split_columns(cell: str) -> tuple[tuple[str, ...], bool]:
    c = cell.strip()
    if c in {"-", "", "—"}:
        return tuple(), True
    cols = tuple(_clean(x) for x in c.split(",") if _clean(x))
    return cols, False


def parse_corrections(path: Path, batch_id: str) -> CorrectionsParse:
    out = CorrectionsParse()
    if not path.exists():
        return out
    for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not _ROW_RX.match(line):
            continue
        cells = [c for c in line.strip().strip("|").split("|")]
        if len(cells) < 8:
            out.malformed_rows.append((i, line[:120]))
            continue
        if len(cells) > 8:  # a '|' inside the reason cell: re-join the middle
            cells = cells[:6] + ["|".join(cells[6:-1])] + [cells[-1]]
        date, file_, sid, col, old, new, reason, by = (c.strip() for c in cells)
        if "*(example)*" in sid:
            out.skipped_template_rows += 1
            continue
        sid_clean = _clean(sid)
        if "*(all rows)*" in sid or sid_clean.lower() == "(all rows)":
            kind, ref = "all_rows", "*"
        elif "blank run_id" in sid_clean:
            kind, ref = "blank_run_id", ""
        elif _SESSION_RX.match(sid_clean):
            kind, ref = "session_id", sid_clean
        else:
            kind, ref = "run_id", sid_clean
        cols, unspecified = _split_columns(col)
        out.corrections.append(Correction(
            batch_id=batch_id, date_utc=date, file=_clean(file_), ref_kind=kind, ref=ref,
            columns=cols, column_unspecified=unspecified, old_value=old, new_value=new,
            reason=reason, by=by, line_no=i,
        ))
    return out


def corrections_for_row(parsed: CorrectionsParse, run_id: str, session_id: str,
                        obs_blank_row: bool = False) -> list[Correction]:
    """Every correction whose reference resolves to this run (any file)."""
    hits = []
    for c in parsed.corrections:
        if c.ref_kind == "all_rows":
            hits.append(c)
        elif c.ref_kind == "run_id" and c.ref == run_id:
            hits.append(c)
        elif c.ref_kind == "session_id" and c.ref == session_id:
            hits.append(c)
        elif c.ref_kind == "blank_run_id" and obs_blank_row:
            hits.append(c)
    return hits
