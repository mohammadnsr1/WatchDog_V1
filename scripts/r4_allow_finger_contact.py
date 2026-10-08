#!/usr/bin/env python3
"""Operator-side companion to Class 2 variant r4 (geometric wrong object).

r4 aims pick_u_brown's grasp at u_orange. grasp_skill then attaches u_brown
(the step's object_id) in MoveIt while the fingers are physically closed on
u_orange, which MoveIt still holds as a free WORLD object -- so every lift is
refused with START_STATE_IN_COLLISION (u_orange <-> rg2 finger). This script
allows contact between u_orange and the RG2 gripper links in MoveIt's Allowed
Collision Matrix, so the lift plans and the wrong part is carried, as the
fault intends. u_orange stays in the planning scene (the executor's view is
unchanged); only that collision CHECK is switched off.

Run it in a ROS-sourced terminal after the bringup is up and BEFORE the
executor starts. It keeps the entries in place (re-applying if anything
rebuilds the ACM) until pick_u_brown ends or the run ends, then exits.
Every apply is logged to <session>/operator_acm_override.jsonl.

  source /opt/ros/jazzy/setup.bash
  python3 scripts/r4_allow_finger_contact.py
"""
from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from moveit_msgs.msg import (AllowedCollisionEntry, AllowedCollisionMatrix,
                             PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

OBJECT = "u_orange"
# gripper_skills.RG2_TOUCH_LINKS -- the links a normal grasp's attach allows.
LINKS = [
    "onrobot_rg2_base_link",
    "onrobot_rg2_left_outer_knuckle",
    "onrobot_rg2_left_inner_knuckle",
    "onrobot_rg2_left_inner_finger",
    "onrobot_rg2_right_outer_knuckle",
    "onrobot_rg2_right_inner_knuckle",
    "onrobot_rg2_right_inner_finger",
    "tcp",
]
STOP_STEP = "pick_u_brown"
POLL_S = 1.0


def session_dir() -> Path:
    raw = (Path.home() / ".watchdog" / "current_session").read_text().strip()
    sd = Path(raw)
    if not sd.is_dir():
        sys.exit(f"current_session names {sd}, which is not a directory")
    return sd


def call(node, client, request, timeout=10.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"service {client.srv_name} unavailable")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise RuntimeError(f"service {client.srv_name} timed out")
    return future.result()


def fetch_acm(node, get_client) -> AllowedCollisionMatrix:
    req = GetPlanningScene.Request()
    req.components = PlanningSceneComponents()
    req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    return call(node, get_client, req).scene.allowed_collision_matrix


def missing_pairs(acm: AllowedCollisionMatrix) -> list[str]:
    names = list(acm.entry_names)
    if OBJECT not in names:
        return list(LINKS)
    i = names.index(OBJECT)
    out = []
    for link in LINKS:
        if link not in names or not acm.entry_values[i].enabled[names.index(link)]:
            out.append(link)
    return out


def apply(node, get_client, apply_client) -> dict:
    """Same reconstruction as ros_moveit_backend.scene.set_permitted_contacts."""
    acm = fetch_acm(node, get_client)
    names = list(acm.entry_names)
    values = [list(e.enabled) for e in acm.entry_values]

    def ensure(name):
        if name in names:
            return
        for row in values:
            row.append(False)
        names.append(name)
        values.append([False] * len(names))

    ensure(OBJECT)
    for link in LINKS:
        ensure(link)
    i = names.index(OBJECT)
    for link in LINKS:
        j = names.index(link)
        values[i][j] = values[j][i] = True

    new = AllowedCollisionMatrix()
    new.entry_names = names
    new.entry_values = [AllowedCollisionEntry(enabled=row) for row in values]
    new.default_entry_names = list(acm.default_entry_names)
    new.default_entry_values = list(acm.default_entry_values)
    req = ApplyPlanningScene.Request()
    req.scene = PlanningScene()
    req.scene.is_diff = True
    req.scene.allowed_collision_matrix = new
    return {"success": bool(call(node, apply_client, req).success)}


def run_over(sd: Path) -> str | None:
    path = sd / "events_scene.jsonl"
    if not path.exists():
        return None
    for line in path.read_text().splitlines():
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get("event_type") == "run_ended":
            return "run_ended"
        if (e.get("event_type") == "end_step"
                and (e.get("payload") or {}).get("step_id") == STOP_STEP):
            return f"end_step {STOP_STEP}"
    return None


def main() -> None:
    sd = session_dir()
    log = sd / "operator_acm_override.jsonl"
    print(f"[r4_acm] session {sd}")
    rclpy.init()
    node = rclpy.create_node("operator_r4_acm_override")
    get_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    try:
        while True:
            done = run_over(sd)
            if done:
                print(f"[r4_acm] {done} seen -- leaving the ACM as is and exiting")
                break
            missing = missing_pairs(fetch_acm(node, get_client))
            if missing:
                result = apply(node, get_client, apply_client)
                rec = {"wall_time_iso": datetime.now(timezone.utc).isoformat(),
                       "action": "allow_contact", "object": OBJECT,
                       "links": LINKS, "was_missing": missing, **result}
                with open(log, "a") as fh:
                    fh.write(json.dumps(rec) + "\n")
                print(f"[r4_acm] applied ({len(missing)} missing) -> {result}")
            time.sleep(POLL_S)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
