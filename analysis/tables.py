"""Table builders.  Every pooled cell is sum-of-numerators / sum-of-denominators and records
its numerator and denominator in the table's provenance list (pitfall 1, Phase 5 audit)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from .config import (BASELINE_BATCHES, CAMPAIGN_BATCHES, CAPABILITY_BATCHES, GATED_SCENARIOS,
                     PARAM_ACCURACY_SOURCE)
from .fractions import Frac, mean, median, p95, pool
from .loader import Loaded, Run, parse_bool


class OracleLeak(AssertionError):
    pass


class ArchitectureMix(AssertionError):
    pass


def assert_no_oracle(runs: list[Run], where: str) -> None:
    bad = [r.key for r in runs if r.is_oracle]
    if bad:
        raise OracleLeak(f"{where}: oracle (REF) runs reached an agent-condition table: {bad}")


def assert_single_architecture(runs: list[Run], where: str) -> None:
    """independence_gap means different things in S1-S3 and S4 (pitfall 3)."""
    sc = {r.scenario for r in runs}
    if sc & GATED_SCENARIOS and "S4" in sc:
        raise ArchitectureMix(f"{where}: independence_gap aggregated across S1-S3 and S4: {sorted(sc)}")


@dataclass
class Table:
    name: str
    columns: list[str] = field(default_factory=list)
    rows: list[dict[str, Any]] = field(default_factory=list)
    na: dict[tuple[str, str], str] = field(default_factory=dict)        # (row_label, col) -> reason
    provenance: list[tuple[str, str, str]] = field(default_factory=list)  # (row_label, col, "num/den" or detail)
    included: dict[str, list[str]] = field(default_factory=dict)        # row_label -> run ids
    excluded: dict[str, list[tuple[str, str]]] = field(default_factory=dict)  # row_label -> [(run, reason)]
    notes: list[str] = field(default_factory=list)

    def add_row(self, label: str, cells: dict[str, Any]) -> None:
        for c in cells:
            if c not in self.columns:
                self.columns.append(c)
        self.rows.append({"row": label, **cells})

    def set_na(self, label: str, col: str, reason: str) -> None:
        self.na[(label, col)] = reason


# ---------------------------------------------------------------------------
# aggregation helpers (all over an already-filtered list of included runs)
# ---------------------------------------------------------------------------

def split_included(runs: list[Run]) -> tuple[list[Run], list[tuple[str, str]]]:
    inc = [r for r in runs if not r.excluded]
    exc = [(r.run_id, r.exclude_reason or "exclude_flag") for r in runs if r.excluded]
    return inc, exc


def pooled_frac(t: Table, label: str, col: str, runs: list[Run], frac_col: str) -> Frac:
    fr = [r.fracs.get(frac_col) for r in runs]
    blank = [r.run_id for r, f in zip(runs, fr) if f is None]
    p = pool(fr)
    t.provenance.append((label, col, f"{p} from {frac_col} over {len(runs) - len(blank)} runs" + (f"; blank on {blank}" if blank else "")))
    if p.den == 0:
        t.set_na(label, col, f"{frac_col}_blank_on_all_runs")
    return p


def count_true(t: Table, label: str, col: str, runs: list[Run], bool_col: str) -> Frac:
    vals = [parse_bool(r.results.get(bool_col)) for r in runs]
    f = Frac(sum(1 for v in vals if v), len(runs))
    t.provenance.append((label, col, f"{f} = runs with {bool_col}==True / included runs"))
    return f


def sum_int(runs: list[Run], col: str) -> int:
    return sum(r.rint(col) or 0 for r in runs)


def fault_rates(t: Table, label: str, runs: list[Run], prefix: str = "fault") -> tuple[Frac, Frac]:
    tp, fp, fn = sum_int(runs, f"{prefix}_TP"), sum_int(runs, f"{prefix}_FP"), sum_int(runs, f"{prefix}_FN")
    rec, prec = Frac(tp, tp + fn), Frac(tp, tp + fp)
    t.provenance.append((label, f"{prefix}_recall", f"{rec} = sum {prefix}_TP / (sum TP + sum FN)"))
    t.provenance.append((label, f"{prefix}_precision", f"{prec} = sum {prefix}_TP / (sum TP + sum FP)"))
    return rec, prec


def param_accuracy(t: Table, label: str, runs: list[Run]) -> tuple[Optional[Frac], str]:
    sources = {PARAM_ACCURACY_SOURCE.get(r.scenario, "") for r in runs}
    if len(sources) != 1:
        raise AssertionError(f"{label}: parameter-accuracy source is not unique across the row: {sources}")
    src = sources.pop()
    f = pooled_frac(t, label, "param_accuracy", runs, src)
    return f, src


def false_alarm_rate(t: Table, label: str, runs: list[Run]) -> Frac:
    conf = [r for r in runs if (r.results.get("expected_final_verdict") or "").strip() == "conformant"]
    f = Frac(sum_int(conf, "verdict_FP"), len(conf))
    t.provenance.append((label, "false_alarm_rate", f"{f} = sum verdict_FP over runs with expected_final_verdict==conformant (any class) / those runs"))
    return f


def observation_cells(t: Table, label: str, runs: list[Run], suffix: str = "") -> dict[str, Any]:
    """Engagement, detection and false-fire fraction are three separate metrics (pitfall 4)."""
    withobs = [r for r in runs if r.has_observation_row]
    usable = [r for r in withobs if not r.obs_excluded]
    dropped = [(r.run_id, r.obs_exclude_reason) for r in withobs if r.obs_excluded]
    eng = Frac(sum(1 for r in usable if (r.oint("n_looks") or 0) > 0 or (r.oint("n_observation_monitors_armed") or 0) > 0), len(usable))
    det = Frac(sum(r.oint("n_detected") or 0 for r in usable), sum(r.oint("n_detectable") or 0 for r in usable))
    ff = Frac(sum(r.oint("obs_monitor_false_fires") or 0 for r in usable), sum(r.oint("n_observation_fires") or 0 for r in usable))
    t.provenance.append((label, "obs_engagement" + suffix, f"{eng} = runs with obs n_looks>0 or n_observation_monitors_armed>0 / runs with a usable observation row"))
    t.provenance.append((label, "obs_detection" + suffix, f"{det} = sum obs n_detected / sum obs n_detectable"))
    t.provenance.append((label, "obs_false_fire_fraction" + suffix, f"{ff} = sum obs_monitor_false_fires / sum obs n_observation_fires"))
    if dropped:
        t.excluded.setdefault(label, []).extend((rid, "observation_row_excluded:" + why) for rid, why in dropped)
    cells = {"n_obs_rows" + suffix: len(usable), "obs_engagement" + suffix: eng,
             "obs_detection" + suffix: det, "obs_false_fire_fraction" + suffix: ff}
    if ff.den == 0:
        t.set_na(label, "obs_false_fire_fraction" + suffix, "no_observation_fires")
    if det.den == 0:
        t.set_na(label, "obs_detection" + suffix, "no_detectable_episodes")
    return cells


def _vals(runs: list[Run], col: str) -> list[float]:
    return [float(r.derived[col]) for r in runs if r.derived.get(col) is not None]


def gate_cells(t: Table, label: str, runs: list[Run]) -> dict[str, Any]:
    cells: dict[str, Any] = {}
    if all(r.scenario not in GATED_SCENARIOS for r in runs):
        for c in ("gate_overhead_fraction", "gate_overhead_fraction_ex_timeout", "gate_wait_median_s"):
            cells[c] = None
            t.set_na(label, c, "no_gates_in_scenario")
    else:
        both = [r for r in runs if r.derived.get("gate_wait_total_s") is not None and r.derived.get("run_wall_s") is not None]
        w = [float(r.derived["gate_wait_total_s"]) for r in both]
        ww = [float(r.derived.get("gate_wait_total_ex_timeout_s") or 0.0) for r in both]
        rw = [float(r.derived["run_wall_s"]) for r in both]
        if sum(rw) > 0:
            cells["gate_overhead_fraction"] = sum(w) / sum(rw)
            cells["gate_overhead_fraction_ex_timeout"] = sum(ww) / sum(rw)
            t.provenance.append((label, "gate_overhead_fraction", f"sum gate_wait_total_s {sum(w):.1f} / sum run_wall_s {sum(rw):.1f} over {len(rw)} runs"))
            t.provenance.append((label, "gate_overhead_fraction_ex_timeout", f"sum gate_wait_total_ex_timeout_s {sum(ww):.1f} / sum run_wall_s {sum(rw):.1f}"))
        else:
            cells["gate_overhead_fraction"] = None
            t.set_na(label, "gate_overhead_fraction", "no_run_with_both_gate_wait_and_run_wall")
        allw = [x for r in runs for x in _per_gate_waits(r)]
        cells["gate_wait_median_s"] = median(allw)
        cells["gate_wait_p95_s"] = p95(allw)
        cells["n_gates_pooled"] = len(allw)
        cells["n_gate_timeouts"] = sum(int(r.derived.get("n_gate_timeouts") or 0) for r in runs)
        t.provenance.append((label, "gate_wait_median_s", f"median over {len(allw)} non-timeout gate waits pooled across runs"))
    rwall = _vals(runs, "run_wall_s")
    cells["run_wall_median_s"] = median(rwall)
    cells["run_wall_mean_s"] = mean(rwall)
    cells["n_runs_with_run_wall"] = len(rwall)
    if not rwall:
        t.set_na(label, "run_wall_median_s", "run_wall_s_unavailable_on_all_runs")
    return cells


def _per_gate_waits(r: Run) -> list[float]:
    """Per-gate waits (timeouts excluded) that gates.py stored; pooled across runs."""
    return [float(x) for x in (r.derived.get("_gate_waits") or [])]


def serialization_cells(t: Table, label: str, runs: list[Run]) -> dict[str, Any]:
    stamped = sum(int(r.derived.get("n_fires_stamped") or 0) for r in runs)
    in_any = sum(int(r.derived.get("n_fires_during_motion") or 0) for r in runs)
    in_m = sum(int(r.derived.get("n_fires_during_motion_server") or 0) for r in runs)
    in_g = sum(int(r.derived.get("n_fires_during_gripper_server") or 0) for r in runs)
    lat = [x for r in runs for x in (r.derived.get("_fire_latencies") or [])]
    cells = {
        "n_fires_log": sum(int(r.derived.get("log_n_fires") or 0) for r in runs),
        "n_fires_delivered": sum(int(r.derived.get("n_fires_delivered") or 0) for r in runs),
        "n_fires_undelivered": sum(int(r.derived.get("n_fires_undelivered") or 0) for r in runs),
        "frac_fires_during_motion": Frac(in_any, stamped),
        "frac_fires_during_motion_server": Frac(in_m, stamped),
        "frac_fires_during_gripper_server": Frac(in_g, stamped),
        "fire_delivery_median_s": median(lat), "fire_delivery_p95_s": p95(lat),
        "delivery_mechanism": ";".join(sorted({str(r.derived.get("delivery_mechanism")) for r in runs})),
    }
    t.provenance.append((label, "frac_fires_during_motion", f"{in_any}/{stamped} fires inside a motion-or-gripper call interval (pooled over runs)"))
    t.provenance.append((label, "fire_delivery_median_s", f"median over {len(lat)} delivered fires pooled across runs"))
    if stamped == 0:
        for c in ("frac_fires_during_motion", "frac_fires_during_motion_server", "frac_fires_during_gripper_server"):
            t.set_na(label, c, "no_fires")
    if not lat:
        for c in ("fire_delivery_median_s", "fire_delivery_p95_s"):
            t.set_na(label, c, "no_delivered_fires")
    return cells


def token_cells(t: Table, label: str, runs: list[Run]) -> dict[str, Any]:
    cells: dict[str, Any] = {}
    for role in ("executor", "watchdog"):
        mapped = [r for r in runs if r.derived.get(f"tokens_in_{role}") is not None]
        for col in ("tokens_in", "tokens_out", "cache_read_tokens", "cache_creation_tokens"):
            cells[f"{col}_{role}_sum"] = sum(int(r.derived[f"{col}_{role}"]) for r in mapped) if mapped else None
            cells[f"{col}_{role}_median_per_run"] = median([float(r.derived[f"{col}_{role}"]) for r in mapped]) if mapped else None
            if not mapped:
                reason = "no_watchdog_role" if role == "watchdog" and all(r.scenario == "S4" for r in runs) else "no_transcript_mapped"
                t.set_na(label, f"{col}_{role}_sum", reason)
                t.set_na(label, f"{col}_{role}_median_per_run", reason)
        cells[f"n_runs_with_transcript_{role}"] = len(mapped)
        t.provenance.append((label, f"tokens_{role}", f"summed over {len(mapped)}/{len(runs)} runs with a uniquely mapped {role} transcript"))
    return cells


def noncomparable_latency_cells(t: Table, label: str, runs: list[Run]) -> dict[str, Any]:
    """Carried through under explicit, non-comparable names (2d)."""
    cells: dict[str, Any] = {}
    if all(r.scenario in GATED_SCENARIOS for r in runs):
        cells["S1S3_fire_to_next_verdict_median_s_median"] = median([float(r.results["fire_to_next_verdict_median_s"]) for r in runs if _isnum(r.results.get("fire_to_next_verdict_median_s"))])
        cells["S4_fire_to_adjudication_median_s_median"] = None
        t.set_na(label, "S4_fire_to_adjudication_median_s_median", "not_S4")
    elif all(r.scenario == "S4" for r in runs):
        cells["S1S3_fire_to_next_verdict_median_s_median"] = None
        t.set_na(label, "S1S3_fire_to_next_verdict_median_s_median", "not_S1S3")
        cells["S4_fire_to_adjudication_median_s_median"] = median([float(r.results["fire_to_adjudication_median_s"]) for r in runs if _isnum(r.results.get("fire_to_adjudication_median_s"))])
        for c in ("interleaved_monitor_blocks", "monitor_calls_in_run", "monitor_tool_wall_sec"):
            cells[f"S4only_{c}_median"] = median([float(r.results[c]) for r in runs if _isnum(r.results.get(c))])
    return cells


def independence_gap_cell(t: Table, label: str, runs: list[Run]) -> Frac:
    assert_single_architecture(runs, label)
    f = count_true(t, label, "independence_gap", runs, "independence_gap")
    return f


def _isnum(x: Any) -> bool:
    try:
        return x is not None and str(x).strip() not in ("", "None", "n/a") and not math.isnan(float(x))
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# table builders
# ---------------------------------------------------------------------------

def _architecture_row(t: Table, label: str, runs: list[Run], with_raw: bool = True) -> dict[str, Any]:
    cells: dict[str, Any] = {"n": len(runs)}
    cells["arming_coverage"] = pooled_frac(t, label, "arming_coverage", runs, "arming_coverage")
    cells["grip_coverage"] = pooled_frac(t, label, "grip_coverage", runs, "grip_coverage")
    pa, src = param_accuracy(t, label, runs)
    cells["param_accuracy"] = pa
    cells["param_accuracy_source"] = src
    cells["verdict_correct"] = count_true(t, label, "verdict_correct", runs, "verdict_correct")
    cells["fault_recall"], cells["fault_precision"] = fault_rates(t, label, runs, "fault")
    if with_raw:
        cells["scorer_raw_fault_recall"], cells["scorer_raw_fault_precision"] = fault_rates(t, label, runs, "scorer_raw_fault")
    cells.update(observation_cells(t, label, runs))
    cells["false_alarm_rate"] = false_alarm_rate(t, label, runs)
    cells.update(gate_cells(t, label, runs))
    cells["independence_gap"] = independence_gap_cell(t, label, runs)
    cells["independence_gap_meaning"] = ("executor_claims_success_watchdog_disagrees" if all(r.scenario in GATED_SCENARIOS for r in runs)
                                        else "executor_self_contradiction_S4")
    return cells


def build_table1(loaded: Loaded) -> Table:
    t = Table("table1_baseline")
    for b in BASELINE_BATCHES:
        runs = loaded.by_batch(b)
        assert_no_oracle(runs, "table1")
        inc, exc = split_included(runs)
        t.included[b] = [r.run_id for r in inc]
        t.excluded.setdefault(b, []).extend(exc)
        t.add_row(b, _architecture_row(t, b, inc))
    for (label, col), reason in list(t.na.items()):
        for row in t.rows:
            if row["row"] == label and col in t.columns:
                row.setdefault(col, None)
    return t


def failure_mode(r: Run) -> str:
    tp = r.rint("fault_TP") or 0
    rec = r.rbool("watchdog_recovered")
    if tp >= 1 and rec is True:
        return "detected_misadjudicated"
    if tp == 0:
        return "undetected"
    return "other"


def fisher_one_sided_greater(a: int, b: int, c: int, d: int) -> float:
    """P(X >= a) for the 2x2 table [[a, b], [c, d]] under the hypergeometric null.
    Rows = groups (S1-S3 pooled, S4); columns = (caught, not caught)."""
    n1, n2, k = a + b, c + d, a + c
    N = n1 + n2
    denom = math.comb(N, k)
    p = 0.0
    for x in range(a, min(n1, k) + 1):
        p += math.comb(n1, x) * math.comb(n2, k - x) / denom
    return p


def build_table2(loaded: Loaded) -> Table:
    t = Table("table2_independence")
    caught_by: dict[str, Frac] = {}
    for b in BASELINE_BATCHES:
        runs = [r for r in loaded.by_batch(b) if r.cls == 4]
        assert_no_oracle(runs, "table2")
        inc, exc = split_included(runs)
        t.included[b] = [r.run_id for r in inc]
        t.excluded.setdefault(b, []).extend(exc)
        caught = Frac(sum(1 for r in inc if (r.rint("fault_TP") or 0) >= 1), len(inc))
        caught_raw = Frac(sum(1 for r in inc if (r.rint("scorer_raw_fault_TP") or 0) >= 1), len(inc))
        vc = count_true(t, b, "verdict_correct", inc, "verdict_correct")
        failing = [r for r in inc if not ((r.rint("fault_TP") or 0) >= 1 and parse_bool(r.results.get("verdict_correct")))]
        modes = [(r.run_id, failure_mode(r)) for r in failing]
        others = [m for m in modes if m[1] == "other"]
        caught_by[b] = caught
        t.provenance.append((b, "step_level_caught", f"{caught} = runs with fault_TP>=1 / class-4 runs"))
        t.add_row(b, {
            "n": len(inc), "step_level_caught": caught, "step_level_caught_scorer_raw": caught_raw,
            "verdict_correct": vc,
            "n_detected_misadjudicated": sum(1 for m in modes if m[1] == "detected_misadjudicated"),
            "n_undetected": sum(1 for m in modes if m[1] == "undetected"),
            "n_other": len(others),
            "failing_runs": ";".join(f"{rid}:{m}" for rid, m in modes),
        })
        if others:
            t.notes.append(f"{b}: failure mode 'other' on {others}")
    a = sum(caught_by[b].num for b in ("S1", "S2", "S3"))
    bb = sum(caught_by[b].den for b in ("S1", "S2", "S3")) - a
    c = caught_by["S4"].num
    dd = caught_by["S4"].den - c
    p = fisher_one_sided_greater(a, bb, c, dd)
    t.notes.append(f"Fisher exact one-sided (S1-S3 pooled caught {a}/{a + bb} > S4 caught {c}/{c + dd}): p = {p:.4f}")
    t.add_row("fisher", {"n": a + bb + c + dd, "fisher_table": f"[[{a},{bb}],[{c},{dd}]]", "fisher_p_one_sided": p})
    return t


def build_table3(loaded: Loaded) -> Table:
    t = Table("table3_capability")
    for b in CAPABILITY_BATCHES:
        all_runs = loaded.by_batch(b)
        assert_no_oracle(all_runs, "table3")
        all_inc, _ = split_included(all_runs)
        runs = [r for r in all_inc if r.cls in (1, 2)]
        exc = [(r.run_id, r.exclude_reason or "exclude_flag") for r in all_runs if r.excluded and r.cls in (1, 2)]
        t.included[b] = [r.run_id for r in runs]
        t.excluded.setdefault(b, []).extend(exc)
        label = b
        cells = {"model_executor": runs[0].model_executor if runs else None,
                 "effort_executor": runs[0].effort_executor if runs else None,
                 "model_watchdog": runs[0].model_watchdog if runs else None,
                 "effort_watchdog": runs[0].effort_watchdog if runs else None,
                 "classes": "1,2"}
        cells.update(_architecture_row(t, label, runs))
        cells.update(observation_cells(t, label, all_inc, suffix="_wholebatch"))
        cells.update(serialization_cells(t, label, runs))
        cells.update(noncomparable_latency_cells(t, label, runs))
        cells.update(token_cells(t, label, runs))
        t.add_row(label, cells)
    return t


def build_table_cost(loaded: Loaded) -> Table:
    t = Table("table_cost")
    for b in CAMPAIGN_BATCHES:
        runs = loaded.by_batch(b)
        assert_no_oracle(runs, "table_cost")
        inc, exc = split_included(runs)
        t.included[b] = [r.run_id for r in inc]
        t.excluded.setdefault(b, []).extend(exc)
        cells: dict[str, Any] = {"n": len(inc), "scenario": inc[0].scenario if inc else None}
        for col, name in (("total_task_time_s", "total_task_time_s"), ("executor_working_s", "executor_working_s"),
                          ("watchdog_blocking_s", "watchdog_blocking_cost_s"), ("watchdog_adjudication_s", "watchdog_postrun_adjudication_s"),
                          ("run_wall_stepping_s", "run_wall_stepping_s"), ("watchdog_postrun_tail_s", "watchdog_postrun_tail_s")):
            v = _vals(inc, col)
            cells[f"{name}_median"] = median(v)
            cells[f"{name}_mean"] = mean(v)
            cells[f"{name}_n"] = len(v)
            if not v:
                reasons = {r.na_reasons.get(col, "") for r in inc if r.derived.get(col) is None}
                t.set_na(b, f"{name}_median", ";".join(sorted(x for x in reasons if x)) or "unavailable")
        cells.update(gate_cells(t, b, inc))
        cells.update(serialization_cells(t, b, inc))
        t.add_row(b, cells)
    t.notes.append("watchdog_blocking_cost_s is the executor's blocked time at gates only; the WatchDog's concurrent "
                   "work costs no task time and is not represented. watchdog_postrun_adjudication_s is time_to_verdict_s "
                   "(S1-S3). The two are separate costs and are never summed. For S4 blocking is 0 by construction.")
    return t


def build_table_observation(loaded: Loaded) -> Table:
    """The only table REF may enter."""
    t = Table("table_observation")
    for b in list(CAMPAIGN_BATCHES) + [x for x in loaded.batches if loaded.batches[x].is_oracle]:
        runs = loaded.by_batch(b)
        inc, exc = split_included(runs)
        t.included[b] = [r.run_id for r in inc]
        t.excluded.setdefault(b, []).extend(exc)
        cells: dict[str, Any] = {"is_oracle": any(r.is_oracle for r in runs), "n_runs": len(inc)}
        cells.update(observation_cells(t, b, inc, suffix="_wholebatch"))
        t.add_row(b, cells)
    return t
