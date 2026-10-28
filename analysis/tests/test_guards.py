"""Structural guards: REF can never enter an agent-condition table; independence_gap is never
pooled across S1-S3 and S4; Fisher exact matches a hand value; failure-mode classification."""
import pytest

from analysis.fractions import Frac
from analysis.loader import Run
from analysis.tables import (ArchitectureMix, OracleLeak, assert_no_oracle, assert_single_architecture,
                             failure_mode, fisher_one_sided_greater, independence_gap_cell, Table)


def _run(batch, run_id, scenario, is_oracle=False, **results):
    return Run(batch_id=batch, run_id=run_id, session_id="20260101T000000Z", scenario=scenario, cls=1,
               role="watchdog", model_executor=None, effort_executor=None, model_watchdog=None,
               effort_watchdog=None, is_oracle=is_oracle, results={k: str(v) for k, v in results.items()},
               obs=None, fracs={}, excluded=False, exclude_reason="", obs_excluded=False,
               obs_exclude_reason="", corrections=[])


def test_oracle_cannot_enter_agent_tables():
    with pytest.raises(OracleLeak):
        assert_no_oracle([_run("REF", "x", "S3", is_oracle=True)], "table1")
    assert_no_oracle([_run("S3", "x", "S3")], "table1")


def test_independence_gap_never_pooled_across_architectures():
    mixed = [_run("S3", "a", "S3", independence_gap="True"), _run("S4", "b", "S4", independence_gap="False")]
    with pytest.raises(ArchitectureMix):
        assert_single_architecture(mixed, "t")
    with pytest.raises(ArchitectureMix):
        independence_gap_cell(Table("t"), "t", mixed)
    assert independence_gap_cell(Table("t"), "t", mixed[:1]) == Frac(1, 1)


def test_fisher_one_sided_hand_value():
    # [[9,0],[1,2]]: P(X>=9) with n1=9, n2=3, k=10, N=12 -> C(9,9)C(3,1)/C(12,10) = 3/66
    assert abs(fisher_one_sided_greater(9, 0, 1, 2) - 3 / 66) < 1e-12
    assert abs(fisher_one_sided_greater(0, 5, 0, 5) - 1.0) < 1e-12


def test_failure_mode_classification():
    assert failure_mode(_run("S1", "a", "S1", fault_TP=2, watchdog_recovered="True")) == "detected_misadjudicated"
    assert failure_mode(_run("S1", "a", "S1", fault_TP=0, watchdog_recovered="False")) == "undetected"
    assert failure_mode(_run("S1", "a", "S1", fault_TP=1, watchdog_recovered="False")) == "other"
