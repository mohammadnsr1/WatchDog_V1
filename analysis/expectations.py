"""PROVISIONAL regression expectations, hand-computed from the corrected CSVs.

These are NOT authoritative.  If the pipeline disagrees, one side has a bug; the CLI reports
the disagreement in audit.md and the test module fails, and nobody tunes either to match.
"""
from __future__ import annotations

from .fractions import Frac

# Baseline, all 23 runs per scenario (Table 1).
BASELINE: dict[str, dict[str, Frac]] = {
    "S1": {"arming": Frac(2296, 2478), "grip": Frac(169, 174), "verdict": Frac(22, 23),
           "fault_recall": Frac(33, 36), "fault_precision": Frac(33, 37), "obs_detection": Frac(0, 16),
           "c4_caught": Frac(3, 3)},
    "S2": {"arming": Frac(2236, 2414), "grip": Frac(158, 164), "verdict": Frac(19, 23),
           "fault_recall": Frac(33, 33), "fault_precision": Frac(33, 33), "obs_detection": Frac(0, 18),
           "c4_caught": Frac(3, 3)},
    # grip and precision corrected 2026-10-28: the first hand count omitted the two Class 3
    # grip cells (8/12, 8/10) and mixed adjusted fault_TP with scorer_raw_fault_FP.
    "S3": {"arming": Frac(2370, 2626), "grip": Frac(107, 178), "verdict": Frac(22, 23),
           "fault_recall": Frac(25, 33), "fault_precision": Frac(25, 26), "obs_detection": Frac(0, 22),
           "c4_caught": Frac(3, 3)},
    "S4": {"arming": Frac(1809, 2726), "grip": Frac(113, 188), "verdict": Frac(18, 23),
           "fault_recall": Frac(29, 49), "fault_precision": Frac(29, 36), "obs_detection": Frac(0, 22),
           "c4_caught": Frac(1, 3)},
}

# Table 3, class 1 and 2 only.
CAPABILITY: dict[str, dict[str, object]] = {
    # S3 precision and S4_opus_medium recall corrected 2026-10-28 (adjusted fault_* columns throughout).
    "S3":             {"n": 15, "verdict": Frac(15, 15), "fault_recall": Frac(14, 21), "fault_precision": Frac(14, 15), "arming": Frac(1436, 1611), "grip": Frac(71, 108)},
    "S3_opus_low":    {"n": 6,  "verdict": Frac(6, 6),   "fault_recall": Frac(4, 4),   "fault_precision": Frac(4, 4),   "arming": Frac(641, 667),   "grip": Frac(46, 46)},
    "S3_opus_medium": {"n": 5,  "verdict": Frac(5, 5),   "fault_recall": Frac(9, 9),   "fault_precision": Frac(9, 9),   "arming": Frac(613, 616),   "grip": Frac(40, 40)},
    "S4":             {"n": 15, "verdict": Frac(13, 15), "fault_recall": Frac(23, 35), "fault_precision": Frac(23, 29), "arming": Frac(1191, 1740), "grip": Frac(80, 120)},
    "S4_opus_medium": {"n": 5,  "verdict": Frac(5, 5),   "fault_recall": Frac(12, 14), "fault_precision": Frac(12, 14), "arming": Frac(503, 504),   "grip": Frac(36, 36)},
}

# Mapping from expectation keys to table column names.
T1_KEYS = {"arming": "arming_coverage", "grip": "grip_coverage", "verdict": "verdict_correct",
           "fault_recall": "fault_recall", "fault_precision": "fault_precision", "obs_detection": "obs_detection"}
T3_KEYS = {"arming": "arming_coverage", "grip": "grip_coverage", "verdict": "verdict_correct",
           "fault_recall": "fault_recall", "fault_precision": "fault_precision"}
