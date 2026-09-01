#!/usr/bin/env python3
"""Compile an executor task YAML into a plan_assembly_step.py plan.

WHY AN ADAPTER AND NOT A NEW RUNNER
-----------------------------------
plan_assembly_step.py is already the "finalize the instruction without the
agent" tool: dry-run by default, --execute to drive the cell, and it shares
grasp_center_from_object_and_offset / tcp_target_from_grasp_center with
motion_mcp_server, so geometry proven there means the same thing to the
executor. What it cannot do is read agents/tasks/*.executor.yaml, because the
two schemas differ:

    executor task step            plan step
    ------------------            ---------
    action_hint (prose)     ->    action (explicit, one per tool call)
    grasp_width_m           ->    gripper_width_m
    target_frame            ->    frame: target_object + target_id
    target_offset           ->    target_offset (same meaning, frame's own axes)
    lift_height_m           ->    frame: delta_from_last + delta.z
    approach_distance_m     ->    approach_distance_m (unchanged)
    motion                  ->    motion (unchanged)

So this only translates. All execution, geometry, confirmation prompts and
safety live in plan_assembly_step.py.

STEP CLASSIFICATION
-------------------
Classified from which fields a step CARRIES, never from action_hint -- that is
prose written for the agent and cannot be parsed reliably:

  pick            approach_distance_m + lift_height_m, no target_frame
                  -> approach_and_preposition, grasp, clear_octomap, lift
  place_approach  target_frame + target_offset
                  -> move_grasp_center_to_pose (target_object, with offset)
  reorient        target_frame ending in _reorient, no target_offset
                  -> move_grasp_center_to_pose (target_object), release
                     NO retreat: the very next step re-approaches this same
                     part at this same frame, and approach_and_preposition
                     already stages the TCP off the part by
                     approach_distance_m. A retreat in between is wasted
                     motion and one more plan that can fail, for no clearance
                     the re-approach does not establish itself.
  place           target_frame, no target_offset
                  -> move_grasp_center_to_pose (target_object), release, retreat
                     Retreat KEPT here: a seated part is final, nothing
                     re-approaches it, and the jaws must clear it before the
                     arm moves on to the next part.

clear_octomap is emitted after every grasp deliberately: the picked part's own
voxels survive the attach, and the next motion then fails
START_STATE_IN_COLLISION against the space the attached part now occupies.

USAGE -- you normally do NOT run this directly
----------------------------------------------
plan_assembly_step.py --task calls this in memory, so the one-command form is:

    python3 plan_assembly_step.py --task ../../../../agents/tasks/fmb_assembly.executor.yaml
    python3 plan_assembly_step.py --task <task> --execute
    python3 plan_assembly_step.py --task <task> --task-only pick_hex_blue,place_hex_blue

Note the asymmetry, which is the one thing to keep straight:
    --task  takes an EXECUTOR TASK yaml  (agents/tasks/*.executor.yaml)
    --plan  takes an ALREADY-COMPILED plan
Passing a task to --plan fails: a task's steps carry action_hint where a plan
needs action, and grasp_width_m where a plan needs gripper_width_m.

Run this standalone only to inspect or archive the compiled plan:

    python3 executor_task_to_plan.py --print
    python3 executor_task_to_plan.py -o /tmp/fmb.plan.yaml
    python3 verify_plan_matches_task.py --plan /tmp/fmb.plan.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_TASK = REPO_ROOT / "agents" / "tasks" / "fmb_assembly.executor.yaml"

# Fallback only. Signed, along the TCP's own local Z -- same convention as
# approach_distance_m and as motion_mcp_server.retreat_tcp; negative backs away
# for the top-down grasps every step in this task uses.
#
# The task YAML carries no retreat distance today, so every place step gets
# this value and tuning the YAML cannot change it -- which is exactly the kind
# of silent adapter-supplied constant that makes a replay stop representing the
# instruction. A step may override it with retreat_distance_m, and
# verify_plan_matches_task.py reports every step still falling back to here.
DEFAULT_RETREAT_M = -0.10
SAFE_POSE_NAME = "ready"


# A destination frame whose name ends in this is a reorient STAGING pose, not a
# final seat. The task's own vocabulary makes the distinction (u_green_reorient
# / inverted_u_brown_reorient vs u_green_placement / hex_blue_hole -- see
# instruction_spec's frame list), which is why the suffix is the discriminator
# rather than the step_id prefix: the destination is what decides whether
# anything comes back for this part.
REORIENT_FRAME_SUFFIX = "_reorient"


def classify(step: dict) -> str:
    if "target_frame" not in step:
        return "pick"
    if "target_offset" in step:
        return "place_approach"
    if str(step["target_frame"]).endswith(REORIENT_FRAME_SUFFIX):
        return "reorient"
    return "place"


def compile_step(step: dict, idx: int) -> list[dict]:
    """Translate one executor step into the plan actions it implies."""
    sid = step.get("step_id", f"step_{idx}")
    oid = step["object_id"]
    width = step["grasp_width_m"]
    offset = step.get("grasp_center_offset")
    if offset is None:
        raise ValueError(
            f"step {sid!r}: no grasp_center_offset. Every step in this task "
            f"carries one; a missing offset would silently grasp at the "
            f"object's own origin."
        )
    motion = step.get("motion") or {}
    kind = classify(step)
    out: list[dict] = []

    if kind == "pick":
        out.append({
            "step_id": f"{sid}__approach",
            "action": "approach_and_preposition",
            "object_id": oid,
            "gripper_width_m": width,
            "approach_distance_m": step.get("approach_distance_m", -0.10),
            "grasp_center_offset": offset,
        })
        out.append({
            "step_id": f"{sid}__grasp",
            "action": "grasp",
            "object_id": oid,
            "gripper_width_m": width,
        })
        # The part's own voxels outlive the attach; without this the lift can
        # fail START_STATE_IN_COLLISION against the attached part's new space.
        out.append({"step_id": f"{sid}__clear_octomap", "action": "clear_octomap"})
        height = step.get("lift_height_m")
        if height is not None:
            # delta_from_last is a WORLD-frame translation, which is what makes
            # this a lift rather than a slide along the part's own axes -- see
            # the lift_note on these steps and motion_mcp_server.lift.
            out.append({
                "step_id": f"{sid}__lift",
                "action": "move_grasp_center_to_pose",
                "frame": "delta_from_last",
                "delta": {"x": 0.0, "y": 0.0, "z": float(height)},
                "object_id": oid,
                "gripper_width_m": width,
            })
        return out

    # Both placement kinds resolve the destination from a LIVE TF frame, so a
    # nudged fixture does not invalidate the plan.
    move = {
        "step_id": f"{sid}__move",
        "action": "move_grasp_center_to_pose",
        "frame": "target_object",
        "target_id": step["target_frame"],
        "object_id": oid,
        "gripper_width_m": width,
        "grasp_center_offset": offset,
    }
    if kind == "place_approach":
        # target_offset is applied in the DESTINATION FRAME'S own axes by
        # plan_assembly_step, matching the executor tool -- these frames' local
        # +Z is not up, so a world-frame add would stage the approach sideways.
        move["target_offset"] = step["target_offset"]
    if motion:
        move["motion"] = motion
    out.append(move)

    if kind in ("place", "reorient"):
        out.append({"step_id": f"{sid}__release", "action": "release",
                    "object_id": oid})
    if kind == "place":
        # Reorient steps deliberately get no retreat -- see the step taxonomy
        # in the module docstring. A step can still force one by carrying
        # retreat_distance_m explicitly.
        out.append({"step_id": f"{sid}__retreat", "action": "retreat",
                    "distance_m": float(step.get("retreat_distance_m",
                                                 DEFAULT_RETREAT_M))})
    elif kind == "reorient" and "retreat_distance_m" in step:
        out.append({"step_id": f"{sid}__retreat", "action": "retreat",
                    "distance_m": float(step["retreat_distance_m"])})
    return out


def compile_task(task: dict, only: list[str] | None,
                 from_step: str | None, to_step: str | None,
                 include_setup: bool) -> dict:
    steps = task.get("steps") or []
    if not steps:
        raise ValueError("task has no steps: block")
    ids = [s.get("step_id", f"step_{i}") for i, s in enumerate(steps)]

    keep = list(range(len(steps)))
    if only:
        unknown = [o for o in only if o not in ids]
        if unknown:
            raise ValueError(f"unknown step_id(s) {unknown}; available: {ids}")
        keep = [i for i in keep if ids[i] in only]
    if from_step:
        if from_step not in ids:
            raise ValueError(f"--from {from_step!r} not in {ids}")
        keep = [i for i in keep if i >= ids.index(from_step)]
    if to_step:
        if to_step not in ids:
            raise ValueError(f"--to {to_step!r} not in {ids}")
        keep = [i for i in keep if i <= ids.index(to_step)]

    plan_steps: list[dict] = []
    if include_setup:
        # instruction_spec: sync the scene before any motion, and move to the
        # safe joint pose at the start of the task.
        object_ids = sorted({steps[i]["object_id"] for i in keep} | {"base2"})
        plan_steps.append({"step_id": "sync_scene", "action": "sync_scene",
                           "object_ids": object_ids})
        plan_steps.append({"step_id": "permitted_contacts",
                           "action": "set_permitted_contacts"})
        plan_steps.append({"step_id": "safe_pose", "action": "move_named_safe_pose",
                           "name": SAFE_POSE_NAME})

    for i in keep:
        plan_steps.extend(compile_step(steps[i], i))

    return {
        "_generated_by": "executor_task_to_plan.py -- do not hand-edit; "
                         "change the executor task YAML and regenerate",
        "_source_task": task.get("name", "<unnamed>"),
        "steps": plan_steps,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default=str(DEFAULT_TASK))
    ap.add_argument("-o", "--out", default=None, help="write the plan here")
    ap.add_argument("--print", dest="to_stdout", action="store_true")
    ap.add_argument("--only", default=None, help="comma-separated executor step_ids")
    ap.add_argument("--from", dest="from_step", default=None)
    ap.add_argument("--to", dest="to_step", default=None)
    ap.add_argument("--no-setup", action="store_true",
                    help="omit sync_scene / permitted_contacts / safe_pose")
    args = ap.parse_args()

    task = yaml.safe_load(Path(args.task).read_text())
    only = [x.strip() for x in args.only.split(",")] if args.only else None
    plan = compile_task(task, only, args.from_step, args.to_step,
                        include_setup=not args.no_setup)

    text = yaml.dump(plan, sort_keys=False, default_flow_style=False)
    if args.to_stdout or not args.out:
        print(text)
    if args.out:
        Path(args.out).write_text(text)
        print(f"wrote {len(plan['steps'])} plan steps to {args.out}", file=sys.stderr)
        print(f"next: python3 plan_assembly_step.py --plan {args.out}", file=sys.stderr)

    # A compile-time summary, so a mis-classified step is visible before any run.
    counts: dict[str, int] = {}
    for s in plan["steps"]:
        counts[s["action"]] = counts.get(s["action"], 0) + 1
    print("action counts: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())),
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
