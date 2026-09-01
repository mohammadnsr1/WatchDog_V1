#!/usr/bin/env python3
"""Prove a compiled plan carries EXACTLY the executor task's values.

executor_task_to_plan.py is a translation, so the one thing that must never
happen is a value drifting or being silently defaulted between the two files:
a plan that dry-runs clean while carrying a different grasp width or offset
than the instruction proves nothing about the instruction.

This checks every transferred field for exact equality (floats compared
bit-exactly, not with a tolerance -- these are authored constants, not
measurements, so any difference at all is a translation bug), and separately
lists values the adapter INVENTED because the task does not carry them.

Exit 0 = every transferred value identical. Exit 1 = drift found.

    python3 verify_plan_matches_task.py                     # default task
    python3 verify_plan_matches_task.py --task X --plan Y
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

import executor_task_to_plan as adapter

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_TASK = REPO_ROOT / "agents" / "tasks" / "fmb_assembly.executor.yaml"

SETUP_ACTIONS = {"sync_scene", "set_permitted_contacts", "move_named_safe_pose"}


def exact(a, b) -> bool:
    """Bit-exact for numbers, recursive for dict/list. No float tolerance."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(exact(a[k], b[k]) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(exact(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return float(a) == float(b)
    return a == b


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default=str(DEFAULT_TASK))
    ap.add_argument("--plan", default=None,
                    help="plan to check; default = compile the task fresh")
    args = ap.parse_args()

    task = yaml.safe_load(Path(args.task).read_text())
    if args.plan:
        plan = yaml.safe_load(Path(args.plan).read_text())
    else:
        plan = adapter.compile_task(task, None, None, None, include_setup=True)

    steps = task.get("steps") or []
    plan_steps = [s for s in plan["steps"] if s["action"] not in SETUP_ACTIONS]
    by_prefix: dict[str, list[dict]] = {}
    for s in plan_steps:
        prefix = s["step_id"].rsplit("__", 1)[0]
        by_prefix.setdefault(prefix, []).append(s)

    mismatches: list[str] = []
    invented: list[str] = []
    checked = 0

    def cmp(sid: str, field: str, want, got, where: str):
        nonlocal checked
        checked += 1
        if not exact(want, got):
            mismatches.append(
                f"  {sid}\n      field   : {field}\n"
                f"      task    : {want!r}\n      plan[{where}]: {got!r}")

    print(f"task : {args.task}")
    print(f"plan : {'<compiled fresh>' if not args.plan else args.plan}")
    print(f"{len(steps)} executor steps -> {len(plan_steps)} plan steps "
          f"(+{len(plan['steps']) - len(plan_steps)} setup)\n")

    for i, step in enumerate(steps):
        sid = step.get("step_id", f"step_{i}")
        kind = adapter.classify(step)
        group = by_prefix.get(sid)
        if not group:
            mismatches.append(f"  {sid}: MISSING from plan entirely")
            continue
        actions = {s["action"]: s for s in group}

        # --- gripper width: every plan step that carries one must equal
        #     the task's grasp_width_m. A wrong width moves the TCP target.
        for ps in group:
            if "gripper_width_m" in ps:
                cmp(sid, "grasp_width_m -> gripper_width_m",
                    step["grasp_width_m"], ps["gripper_width_m"], ps["step_id"])

        # --- grasp_center_offset: position AND orientation, exactly
        for ps in group:
            if "grasp_center_offset" in ps:
                cmp(sid, "grasp_center_offset", step["grasp_center_offset"],
                    ps["grasp_center_offset"], ps["step_id"])

        if kind == "pick":
            ap_ = actions.get("approach_and_preposition")
            if ap_ is None:
                mismatches.append(f"  {sid}: pick has no approach_and_preposition")
            elif "approach_distance_m" in step:
                cmp(sid, "approach_distance_m", step["approach_distance_m"],
                    ap_["approach_distance_m"], ap_["step_id"])
            else:
                invented.append(
                    f"  {sid}: approach_distance_m absent from task; adapter "
                    f"substituted {ap_['approach_distance_m']!r}")

            if "lift_height_m" in step:
                lift = next((s for s in group
                             if s.get("frame") == "delta_from_last"), None)
                if lift is None:
                    mismatches.append(f"  {sid}: lift_height_m present but no lift step")
                else:
                    cmp(sid, "lift_height_m -> delta.z",
                        step["lift_height_m"], lift["delta"]["z"], lift["step_id"])
                    # A lift must be purely vertical: any x/y would drag the part.
                    cmp(sid, "lift delta.x must be 0", 0.0, lift["delta"]["x"],
                        lift["step_id"])
                    cmp(sid, "lift delta.y must be 0", 0.0, lift["delta"]["y"],
                        lift["step_id"])
        else:
            mv = actions.get("move_grasp_center_to_pose")
            if mv is None:
                mismatches.append(f"  {sid}: {kind} has no move step")
            else:
                cmp(sid, "target_frame -> target_id",
                    step["target_frame"], mv["target_id"], mv["step_id"])
                if "target_offset" in step:
                    cmp(sid, "target_offset", step["target_offset"],
                        mv.get("target_offset"), mv["step_id"])
                elif "target_offset" in mv:
                    mismatches.append(
                        f"  {sid}: task has no target_offset but plan carries "
                        f"{mv['target_offset']!r}")
                if "motion" in step:
                    cmp(sid, "motion", step["motion"], mv.get("motion"),
                        mv["step_id"])
                elif "motion" in mv:
                    mismatches.append(
                        f"  {sid}: task has no motion: but plan carries "
                        f"{mv['motion']!r}")

            if kind in ("place", "reorient"):
                if "release" not in actions:
                    mismatches.append(f"  {sid}: {kind} has no release")
                rt = actions.get("retreat")
                if kind == "reorient" and "retreat_distance_m" not in step:
                    # Asserted, not merely unchecked: a retreat reappearing here
                    # is a regression, since the next step re-approaches this
                    # same part at this same frame.
                    checked += 1
                    if rt is not None:
                        mismatches.append(
                            f"  {sid}: reorient must have NO retreat, but plan "
                            f"carries {rt['step_id']} distance_m="
                            f"{rt['distance_m']!r}")
                elif rt is None:
                    mismatches.append(f"  {sid}: {kind} has no retreat")
                elif "retreat_distance_m" in step:
                    cmp(sid, "retreat_distance_m -> distance_m",
                        step["retreat_distance_m"], rt["distance_m"],
                        rt["step_id"])
                else:
                    invented.append(
                        f"  {sid}: retreat distance_m={rt['distance_m']!r} "
                        f"(adapter default; add retreat_distance_m to the "
                        f"step to control it from the task)")

        # --- object_id must match on every plan step that names one
        for ps in group:
            if "object_id" in ps:
                cmp(sid, "object_id", step["object_id"], ps["object_id"],
                    ps["step_id"])

    print(f"{checked} field comparisons\n")
    if mismatches:
        print(f"VALUE DRIFT ({len(mismatches)}):")
        for m in mismatches:
            print(m)
    else:
        print("NO VALUE DRIFT -- every transferred field is bit-identical.")

    if invented:
        print(f"\nADAPTER-SUPPLIED VALUES ({len(invented)}) -- not in the task, so "
              f"tuning them in the YAML will NOT change the plan:")
        for m in invented:
            print(m)

    # Any task field never consulted by the adapter is a silent no-op for the
    # replay: worth naming so nobody tunes it and expects an effect.
    consumed = {"step_id", "object_id", "grasp_width_m", "grasp_center_offset",
                "approach_distance_m", "lift_height_m", "target_frame",
                "target_offset", "motion", "retreat_distance_m"}
    ignored: dict[str, set[str]] = {}
    for step in steps:
        for k in step:
            if k not in consumed:
                ignored.setdefault(k, set()).add(step.get("step_id", "?"))
    if ignored:
        print("\nTASK FIELDS THE ADAPTER IGNORES (prose/notes -- no effect on replay):")
        for k, where in sorted(ignored.items()):
            print(f"  {k}  ({len(where)} step(s))")

    return 1 if mismatches else 0


if __name__ == "__main__":
    sys.exit(main())
