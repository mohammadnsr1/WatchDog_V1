"""Pitfall 1: pooled ratio != mean of per-run ratios, proven on real S3 data so nobody
"simplifies" the pooling later."""
from pathlib import Path

import pytest

from analysis.fractions import Frac, mean_of_ratios, parse_frac, pool
from analysis.loader import load_batch, load_runs

REPO = Path(__file__).resolve().parents[2]
BATCHES = REPO / "paper_results" / "batches"


def test_parse_frac():
    assert parse_frac("115/116") == Frac(115, 116)
    assert parse_frac(" 0/20 ") == Frac(0, 20)
    assert parse_frac("n/a") is None
    assert parse_frac("") is None
    with pytest.raises(ValueError):
        parse_frac("0.97")


def test_pool_is_sum_of_num_and_den():
    assert pool([Frac(1, 2), Frac(3, 4), None]) == Frac(4, 6)


@pytest.mark.skipif(not (BATCHES / "S3").exists(), reason="S3 batch not present")
def test_pooled_differs_from_mean_of_ratios_on_real_S3():
    batch = load_batch(BATCHES, "S3")
    runs, _ = load_runs(batch)
    fr = [r.fracs["arming_coverage"] for r in runs if not r.excluded]
    pooled = pool(fr).ratio
    mean_r = mean_of_ratios(fr)
    assert pooled is not None and mean_r is not None
    # Denominators range from ~20 to ~174; the two estimators must not coincide.
    assert abs(pooled - mean_r) > 1e-3, (pooled, mean_r)
