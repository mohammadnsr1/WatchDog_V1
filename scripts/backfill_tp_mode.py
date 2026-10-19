"""Add fault_TP_by_fire / fault_TP_by_check to a batch's corrected/results.csv.

The two columns were added to watchdog_scorer.py on 2026-10-19 (additive: no
other column changes). Rows scored before that lack them. This recomputes
them per session with the current scorer, in memory, and writes ONLY those
two columns into corrected/results.csv. raw/ is never touched.

    python3 scripts/backfill_tp_mode.py paper_results/batches/S2 [--init]

--init  creates corrected/results.csv and corrected/observation_results.csv
        as copies of raw/ first (BATCH_RUNBOOK Phase 6 step 1) when absent.

Safety: a row is filled only when the recomputed fault_TP equals the CSV's
fault_TP. A row whose fault_TP was hand-corrected (or whose session is
missing) is left blank and listed, for a hand decision.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
import watchdog_scorer as wd  # noqa: E402

NEW = ["fault_TP_by_fire", "fault_TP_by_check"]
SESSIONS = REPO / "agents" / "transcripts" / "sessions"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch_dir")
    ap.add_argument("--init", action="store_true")
    a = ap.parse_args()
    batch = Path(a.batch_dir)
    target = batch / "corrected" / "results.csv"
    if not target.exists():
        if not a.init:
            sys.exit(f"{target} does not exist; pass --init to copy raw/ first.")
        for name in ("results.csv", "observation_results.csv"):
            src = batch / "raw" / name
            if src.exists():
                shutil.copyfile(src, batch / "corrected" / name)
                print(f"copied raw/{name} -> corrected/{name}")

    rows = wd._read_existing(target)
    with open(target, newline="") as fh:
        header = [c.strip() for c in next(iter(fh)).split(",")]
    cols = [c for c in header if c not in NEW]
    at = cols.index("fault_FN_list") + 1
    cols[at:at] = NEW

    by_run = {}
    d = yaml.safe_load(open(batch / "batch.yaml"))
    for s in d.get("sessions") or []:
        sd = SESSIONS / s["session_id"]
        labels = sorted(sd.glob("gt_*.yaml"))
        if len(labels) != 1:
            continue
        label = wd.load_label(str(labels[0]))
        row = wd.score_run(str(sd), label)
        by_run[row["run_id"]] = (len(row["detection"]["true_positives"]),
                                 row["fault_TP_by_fire"], row["fault_TP_by_check"])

    filled, skipped = 0, []
    for r in rows:
        got = by_run.get(r.get("run_id"))
        if got is None or str(got[0]) != str(r.get("fault_TP", "")).strip():
            skipped.append((r.get("run_id"), "no session" if got is None
                            else f"csv fault_TP {r.get('fault_TP')} != recomputed {got[0]}"))
            r.setdefault("fault_TP_by_fire", "")
            r.setdefault("fault_TP_by_check", "")
            continue
        r["fault_TP_by_fire"], r["fault_TP_by_check"] = got[1], got[2]
        filled += 1
    wd._write_aligned(target, rows, cols)
    print(f"{target}: filled {filled}/{len(rows)} rows")
    for s in skipped:
        print(f"  left blank: {s[0]} ({s[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
