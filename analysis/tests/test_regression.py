"""PROVISIONAL regression expectations (hand-computed from the corrected CSVs, see
analysis/expectations.py).  Not authoritative: a failure means one side has a bug and must
be reported, not tuned away.  Runs the full read-only pipeline once per session."""
from pathlib import Path

import pytest

from analysis import expectations as X
from analysis.cli import build_everything
from analysis.fractions import Frac

REPO = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.skipif(not (REPO / "paper_results" / "batches" / "S1").exists(), reason="batches not present")


@pytest.fixture(scope="module")
def built():
    return build_everything(REPO)


def _row(t, label):
    return next(r for r in t.rows if r["row"] == label)


@pytest.mark.parametrize("batch", list(X.BASELINE))
@pytest.mark.parametrize("key", list(X.T1_KEYS))
def test_table1_provisional(built, batch, key):
    got = _row(built["tables"]["table1"], batch)[X.T1_KEYS[key]]
    exp = X.BASELINE[batch][key]
    assert isinstance(got, Frac) and (got.num, got.den) == (exp.num, exp.den), f"{batch}/{key}: expected {exp}, got {got}"


@pytest.mark.parametrize("batch", list(X.BASELINE))
def test_table2_c4_caught_provisional(built, batch):
    got = _row(built["tables"]["table2"], batch)["step_level_caught"]
    exp = X.BASELINE[batch]["c4_caught"]
    assert (got.num, got.den) == (exp.num, exp.den), f"{batch}: expected {exp}, got {got}"


@pytest.mark.parametrize("batch", list(X.CAPABILITY))
def test_table3_provisional(built, batch):
    row = _row(built["tables"]["table3"], batch)
    exp = X.CAPABILITY[batch]
    assert row["n"] == exp["n"], f"{batch}: n expected {exp['n']}, got {row['n']}"
    for key, col in X.T3_KEYS.items():
        got = row[col]
        e = exp[key]
        assert (got.num, got.den) == (e.num, e.den), f"{batch}/{key}: expected {e}, got {got}"


def test_run_count_and_join(built):
    loaded = built["loaded"]
    agent_runs = [r for r in loaded.runs if not r.is_oracle]
    assert len(agent_runs) == 108
    assert len({r.key for r in loaded.runs}) == len(loaded.runs)
    assert all(r.session_id for r in loaded.runs)
