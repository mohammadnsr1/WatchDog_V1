"""Fraction cells ("115/116") and pooling.

Pooling is ALWAYS sum-of-numerators over sum-of-denominators (pitfall 1).
``mean_of_ratios`` exists only so the test suite can prove the two differ.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True)
class Frac:
    num: int
    den: int

    @property
    def ratio(self) -> Optional[float]:
        return None if self.den == 0 else self.num / self.den

    def __str__(self) -> str:
        return f"{self.num}/{self.den}"

    def pct(self) -> str:
        r = self.ratio
        return "NA" if r is None else f"{100.0 * r:.1f}"


_MISSING = {"", "n/a", "na", "none", "null", "-"}


def parse_frac(cell: Optional[str]) -> Optional[Frac]:
    """'115/116' -> Frac(115, 116). Blank / 'n/a' / 'None' -> None. Anything else raises."""
    if cell is None:
        return None
    s = str(cell).strip()
    if s.lower() in _MISSING:
        return None
    if "/" not in s:
        raise ValueError(f"not a fraction cell: {cell!r}")
    a, b = s.split("/", 1)
    return Frac(int(a.strip()), int(b.strip()))


def pool(fracs: Iterable[Optional[Frac]]) -> Frac:
    """Sum numerators and denominators. None entries contribute nothing."""
    n = d = 0
    for f in fracs:
        if f is None:
            continue
        n += f.num
        d += f.den
    return Frac(n, d)


def mean_of_ratios(fracs: Iterable[Optional[Frac]]) -> Optional[float]:
    """The WRONG way to pool. Kept only for the regression test that proves it differs."""
    rs = [f.ratio for f in fracs if f is not None and f.ratio is not None]
    return None if not rs else sum(rs) / len(rs)


def median(xs: Iterable[float]) -> Optional[float]:
    v = sorted(x for x in xs if x is not None)
    if not v:
        return None
    n = len(v)
    return v[n // 2] if n % 2 else (v[n // 2 - 1] + v[n // 2]) / 2.0


def p95(xs: Iterable[float]) -> Optional[float]:
    """Nearest-rank p95 (ceil), the convention the scorers use."""
    v = sorted(x for x in xs if x is not None)
    if not v:
        return None
    return v[max(0, math.ceil(0.95 * len(v)) - 1)]


def mean(xs: Iterable[float]) -> Optional[float]:
    v = [x for x in xs if x is not None]
    return None if not v else sum(v) / len(v)
