"""Load every batch into one row per run, keyed (batch_id, run_id).

Join rule (D7): results[_s4].csv row -> session timestamp parsed out of ``label_used``
(absolute or repo-relative; the timestamp is always present) -> observation_results.csv
``run_id`` (which IS the session timestamp).  A results row without an observation row is
legitimate (no cube queued) and passes with ``has_observation_row = False``.  An observation
row that matches no results row is a hard failure, with one recorded exception: a row with a
BLANK run_id and ``exclude`` set (an operator paste error) is logged as an orphan and skipped.

Observation columns are prefixed ``obs_`` because both CSVs carry ``run_id``, ``scenario``,
``exclude``, ``manual_note`` and ``n_observation_fires``.
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml

from .config import (ADJUDICATING_ROLE, FRACTION_COLUMNS, ORACLE_BATCHES, PARAM_ACCURACY_SOURCE)
from .corrections import CorrectionsParse, corrections_for_row, parse_corrections
from .fractions import Frac, parse_frac

_SESSION_RX = re.compile(r"(2026\d{4}T\d{6}Z)")
_TRUE = {"true", "1", "yes"}


def parse_bool(cell: Any) -> Optional[bool]:
    if cell is None:
        return None
    s = str(cell).strip().lower()
    if s in _TRUE:
        return True
    if s in {"false", "0", "no"}:
        return False
    return None


def parse_int(cell: Any) -> Optional[int]:
    if cell is None:
        return None
    s = str(cell).strip()
    if s == "" or s.lower() in {"none", "n/a", "na", "null"}:
        return None
    return int(float(s))


def parse_float(cell: Any) -> Optional[float]:
    if cell is None:
        return None
    s = str(cell).strip()
    if s == "" or s.lower() in {"none", "n/a", "na", "null"}:
        return None
    return float(s)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh, skipinitialspace=True))
    return [{(k or "").strip(): (v or "").strip() for k, v in r.items()} for r in rows]


class LoadError(RuntimeError):
    pass


@dataclass
class Batch:
    batch_id: str
    dir: Path
    yaml: dict
    is_oracle: bool
    results_path: Optional[Path]
    observation_path: Path
    corrections_path: Path
    corrections: CorrectionsParse
    files_read: list[Path] = field(default_factory=list)

    @property
    def model(self) -> dict[str, dict]:
        return self.yaml.get("per_role") or {}

    @property
    def session_entries(self) -> dict[str, dict]:
        return {s["session_id"]: s for s in (self.yaml.get("sessions") or []) if s.get("session_id")}

    @property
    def known_invalid(self) -> list[dict]:
        return list(self.yaml.get("known_invalid") or [])


@dataclass
class Run:
    """One run.  ``results`` and ``obs`` hold the ORIGINAL cells, untouched."""
    batch_id: str
    run_id: str
    session_id: str
    scenario: str
    cls: Optional[int]
    role: str
    model_executor: Optional[str]
    effort_executor: Optional[str]
    model_watchdog: Optional[str]
    effort_watchdog: Optional[str]
    is_oracle: bool
    results: dict[str, str]
    obs: Optional[dict[str, str]]
    fracs: dict[str, Optional[Frac]]
    excluded: bool
    exclude_reason: str
    obs_excluded: bool
    obs_exclude_reason: str
    corrections: list
    derived: dict[str, Any] = field(default_factory=dict)
    na_reasons: dict[str, str] = field(default_factory=dict)
    flags: dict[str, str] = field(default_factory=dict)
    session_entry: dict = field(default_factory=dict)

    @property
    def key(self) -> tuple[str, str]:
        return (self.batch_id, self.run_id)

    @property
    def has_observation_row(self) -> bool:
        return self.obs is not None

    def set_na(self, col: str, reason: str) -> None:
        self.derived[col] = None
        self.na_reasons[col] = reason

    # convenience accessors over the original cells
    def rint(self, col: str) -> Optional[int]:
        return parse_int(self.results.get(col))

    def rbool(self, col: str) -> Optional[bool]:
        return parse_bool(self.results.get(col))

    def oint(self, col: str) -> Optional[int]:
        return None if self.obs is None else parse_int(self.obs.get(col))


@dataclass
class Loaded:
    batches: dict[str, Batch]
    runs: list[Run]
    orphan_observation_rows: list[dict]
    notes: list[str] = field(default_factory=list)

    def by_batch(self, batch_id: str) -> list[Run]:
        return [r for r in self.runs if r.batch_id == batch_id]


def load_batch(batches_root: Path, batch_id: str) -> Batch:
    d = batches_root / batch_id
    ypath = d / "batch.yaml"
    y = yaml.safe_load(ypath.read_text(encoding="utf-8"))
    corr_dir = d / "corrected"
    res = None
    for name in ("results.csv", "results_s4.csv"):
        if (corr_dir / name).exists():
            res = corr_dir / name
    cpath = corr_dir / "CORRECTIONS.md"
    b = Batch(batch_id=batch_id, dir=d, yaml=y, is_oracle=batch_id in ORACLE_BATCHES,
              results_path=res, observation_path=corr_dir / "observation_results.csv",
              corrections_path=cpath, corrections=parse_corrections(cpath, batch_id))
    b.files_read = [ypath, cpath] + ([res] if res else []) + ([b.observation_path] if b.observation_path.exists() else [])
    if b.is_oracle and res is not None:
        raise LoadError(f"{batch_id}: oracle batch unexpectedly has a results CSV")
    if not b.is_oracle and res is None:
        raise LoadError(f"{batch_id}: no results.csv / results_s4.csv under corrected/")
    return b


def _obs_excluded(o: dict[str, str]) -> tuple[bool, str]:
    reasons = []
    if parse_bool(o.get("exclude")) or (o.get("exclude") or "").strip() not in ("", "0"):
        reasons.append("exclude_flag")
    if parse_bool(o.get("excluded")):
        reasons.append("excluded_leak_audit")
    if (o.get("warning") or "").strip():
        reasons.append("warning:" + o["warning"][:60])
    return bool(reasons), ";".join(reasons)


def load_runs(batch: Batch) -> tuple[list[Run], list[dict]]:
    per_role = batch.model
    ex = per_role.get("executor") or {}
    wd = per_role.get("watchdog") or {}
    entries = batch.session_entries
    obs_rows = read_csv(batch.observation_path) if batch.observation_path.exists() else []
    obs_by_sid: dict[str, dict] = {}
    orphans: list[dict] = []
    blank_rows: list[dict] = []
    for o in obs_rows:
        sid = o.get("run_id", "")
        if sid == "":
            blank_rows.append(o)
            continue
        if sid in obs_by_sid:
            raise LoadError(f"{batch.batch_id}: duplicate observation row for {sid}")
        obs_by_sid[sid] = o

    runs: list[Run] = []
    if batch.is_oracle:
        for sid, o in obs_by_sid.items():
            ent = entries.get(sid, {})
            oex, oreason = _obs_excluded(o)
            runs.append(Run(
                batch_id=batch.batch_id, run_id=sid, session_id=sid,
                scenario=str(o.get("scenario") or ent.get("scenario") or ""), cls=ent.get("class"),
                role=str(o.get("role") or "watchdog"),
                model_executor=ex.get("model"), effort_executor=ex.get("effort"),
                model_watchdog=wd.get("model"), effort_watchdog=wd.get("effort"),
                is_oracle=True, results={}, obs=o, fracs={}, excluded=False, exclude_reason="",
                obs_excluded=oex, obs_exclude_reason=oreason,
                corrections=corrections_for_row(batch.corrections, sid, sid), session_entry=ent,
            ))
            if sid not in entries:
                runs[-1].flags["session_not_in_batch_yaml"] = "1"
        used = set(obs_by_sid)
    else:
        rows = read_csv(batch.results_path)  # type: ignore[arg-type]
        seen: set[tuple[str, str]] = set()
        used = set()
        for r in rows:
            run_id = r.get("run_id", "")
            m = _SESSION_RX.search(r.get("label_used", ""))
            if not m:
                raise LoadError(f"{batch.batch_id}/{run_id}: label_used carries no session timestamp: {r.get('label_used')!r}")
            sid = m.group(1)
            if (batch.batch_id, run_id) in seen:
                raise LoadError(f"{batch.batch_id}: duplicate run_id {run_id}")
            seen.add((batch.batch_id, run_id))
            ent = entries.get(sid)
            if ent is None:
                raise LoadError(f"{batch.batch_id}/{run_id}: session {sid} is not listed in batch.yaml")
            o = obs_by_sid.get(sid)
            if o is not None:
                used.add(sid)
            scenario = str(r.get("scenario") or ent.get("scenario") or "")
            cls_csv = parse_int(r.get("class"))
            cls_yaml = ent.get("class")
            excl = (r.get("exclude") or "").strip() not in ("", "0")
            oex, oreason = _obs_excluded(o) if o is not None else (False, "")
            fr = {c: parse_frac(r.get(c)) for c in FRACTION_COLUMNS}
            run = Run(
                batch_id=batch.batch_id, run_id=run_id, session_id=sid, scenario=scenario,
                cls=cls_csv if cls_csv is not None else cls_yaml, role=ADJUDICATING_ROLE.get(scenario, ""),
                model_executor=ex.get("model"), effort_executor=ex.get("effort"),
                model_watchdog=wd.get("model"), effort_watchdog=wd.get("effort"),
                is_oracle=False, results=r, obs=o, fracs=fr, excluded=excl,
                exclude_reason=("exclude_flag:" + (r.get("manual_note") or "")[:80]) if excl else "",
                obs_excluded=oex, obs_exclude_reason=oreason,
                corrections=corrections_for_row(batch.corrections, run_id, sid), session_entry=ent,
            )
            if cls_csv is not None and cls_yaml is not None and cls_csv != cls_yaml:
                run.flags["class_mismatch_csv_vs_yaml"] = f"csv={cls_csv},yaml={cls_yaml}"
            if scenario != str(ent.get("scenario")):
                run.flags["scenario_mismatch_csv_vs_yaml"] = f"csv={scenario},yaml={ent.get('scenario')}"
            run.derived["param_accuracy_source"] = PARAM_ACCURACY_SOURCE.get(scenario, "")
            runs.append(run)
        missing = set(entries) - {r.session_id for r in runs}
        if missing:
            raise LoadError(f"{batch.batch_id}: batch.yaml sessions without a results row: {sorted(missing)}")
    unmatched = set(obs_by_sid) - used
    if unmatched:
        raise LoadError(f"{batch.batch_id}: observation rows with no results match: {sorted(unmatched)}")
    for o in blank_rows:
        oex, _ = _obs_excluded(o)
        if not oex:
            raise LoadError(f"{batch.batch_id}: observation row with blank run_id and no exclude flag")
        orphans.append({"batch_id": batch.batch_id, "reason": "blank_run_id_excluded_operator_row",
                        "manual_note": o.get("manual_note", ""),
                        "corrections": [c.as_record() for c in corrections_for_row(batch.corrections, "", "", obs_blank_row=True)]})
    return runs, orphans


def load_all(batches_root: Path, batch_ids: tuple[str, ...]) -> Loaded:
    batches: dict[str, Batch] = {}
    runs: list[Run] = []
    orphans: list[dict] = []
    for b in batch_ids:
        batch = load_batch(batches_root, b)
        batches[b] = batch
        rs, orph = load_runs(batch)
        runs.extend(rs)
        orphans.extend(orph)
    return Loaded(batches=batches, runs=runs, orphan_observation_rows=orphans)
