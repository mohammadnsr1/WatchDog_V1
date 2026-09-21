"""The registry's object/frame split and its ACM clique must stay consistent.

Three separate bugs from 2026-09-21 motivate these, and each is cheap to
reintroduce by hand-editing model_registry.yaml:

1. The ACM list was two disjoint per-assembly cliques on the stated premise
   that the fmb1 and fmb2 sets "never share a table". Stage inspection showed
   all ten prims present at once, so 25 cross-set pairs were missing and an
   fmb1 part resting against an fmb2 part was an unmodelled start-state
   collision. A pair list maintained by hand cannot be trusted to stay
   complete, so assert completeness instead.

2. `enabled` was overloaded as "has collision geometry", which left every
   placement/hole/reorient frame disabled and therefore invisible to
   get_object_poses. It now means "the stage publishes this", with `kind`
   carrying the has-a-body question.

3. Because the recorder sampled entries regardless of `enabled`, four
   authored-but-unpublished entries made signal_ready(scope="run")
   permanently unsatisfiable, and every executor gate paid its full 90s
   timeout -- six minutes on a one-part run.
"""

from __future__ import annotations

import itertools
from pathlib import Path

import pytest
import yaml


def _registry_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "model_registry.yaml"


@pytest.fixture(scope="module")
def registry() -> dict:
    return yaml.safe_load(_registry_path().read_text())


def _entries(registry: dict) -> dict[str, dict]:
    return {k: v for k, v in registry["objects"].items() if isinstance(v, dict)}


def _kind(entry: dict) -> str:
    return str(entry.get("kind", "object")).strip().lower()


def _enabled_bodies(registry: dict) -> set[str]:
    return {
        oid
        for oid, e in _entries(registry).items()
        if e.get("enabled") and _kind(e) == "object"
    }


def test_every_entry_declares_a_valid_kind(registry):
    for oid, entry in _entries(registry).items():
        assert _kind(entry) in ("object", "frame"), (
            f"{oid}: kind must be 'object' or 'frame', got {entry.get('kind')!r}"
        )


def test_acm_is_the_complete_clique_over_enabled_bodies(registry):
    """Every enabled body can touch every other one, so every pair must be listed."""
    bodies = _enabled_bodies(registry)
    listed = {frozenset(pair) for pair in registry["acm_allowed_object_pairs"]}
    expected = {frozenset(p) for p in itertools.combinations(sorted(bodies), 2)}

    missing = expected - listed
    assert not missing, (
        "these enabled bodies share a table but have no ACM pair, so their "
        "resting contact is an unmodelled start-state collision: "
        f"{sorted(tuple(sorted(p)) for p in missing)}"
    )


def test_acm_never_names_a_frame_or_a_disabled_entry(registry):
    """An ACM pair must name a body MoveIt actually has in the planning scene."""
    bodies = _enabled_bodies(registry)
    entries = _entries(registry)

    for pair in registry["acm_allowed_object_pairs"]:
        for name in pair:
            assert name in entries, f"ACM pair {pair} names unknown entry {name!r}"
            assert _kind(entries[name]) == "object", (
                f"ACM pair {pair} names {name!r}, which is kind: frame -- a frame "
                "is never added to the planning scene, so there is no collision "
                "body for this pair to refer to"
            )
            assert name in bodies, (
                f"ACM pair {pair} names {name!r}, which is disabled and therefore "
                "not in the planning scene"
            )


def test_no_frame_is_graspable(registry):
    """A frame is a pose-only landmark; there is nothing there to pick up."""
    for oid, entry in _entries(registry).items():
        if _kind(entry) == "frame":
            assert not entry.get("graspable", False), (
                f"{oid} is kind: frame but graspable: true -- the agent would "
                "plan a pick against a landmark with no body"
            )


def test_recorder_samples_exactly_the_enabled_entries(registry):
    """What the recorder samples is what run-scope readiness waits to warm up.

    signal_ready(scope="run") requires a buffered pose for every entry the
    recorder tracks, so sampling anything the stage does not publish makes run
    readiness unreachable and costs every gate its full timeout. Disabled
    entries must therefore be excluded, and enabled ones of BOTH kinds
    included -- the placement frames are the watchdog's seating-check target.
    """
    from robot_backend.watchdog.recorder import _sampled_object_frames

    sampled = set(_sampled_object_frames(registry))
    enabled = {oid for oid, e in _entries(registry).items() if e.get("enabled")}
    disabled = {oid for oid, e in _entries(registry).items() if not e.get("enabled")}

    assert sampled == enabled, (
        f"recorder samples {sorted(sampled ^ enabled)} inconsistently with the "
        "registry's enabled flags"
    )
    assert not (sampled & disabled), (
        f"recorder samples disabled entries {sorted(sampled & disabled)}; these "
        "can never warm up, so run-scope readiness could never be signalled"
    )


def test_enabled_frames_are_sampled_for_the_seating_check(registry):
    """Frames are evidence even though they are not bodies."""
    from robot_backend.watchdog.recorder import _sampled_object_frames

    sampled = _sampled_object_frames(registry)
    enabled_frames = {
        oid
        for oid, e in _entries(registry).items()
        if e.get("enabled") and _kind(e) == "frame"
    }

    assert enabled_frames, "expected the stage's placement/reorient frames to be enabled"
    for oid in enabled_frames:
        assert oid in sampled, (
            f"{oid} is an enabled frame but is not in the watchdog's evidence "
            "buffer, so the seating check has no independent target"
        )
        assert sampled[oid]["kind"] == "frame"
