"""Scene maintenance and recovery skills.

These skills exist to repair inconsistent planning scene state — the condition
that arises when a pick fails mid-sequence, an agent crashes, or the robot
loses track of which objects are attached vs. world-tracked.

All three skills are intentionally free of motion planning and gripper calls.
They only interact with the scene backend (MoveIt collision scene + octomap).

Ownership model reminder:
  WORLD_TRACKED  — object is a CollisionObject in MoveIt's world scene.
  ATTACHED       — object is an AttachedCollisionObject on a robot link.
  Neither state  — object is absent from MoveIt (needs a sync to re-appear).

Recovery contract:
  reset_object_state       — single-object reset to WORLD_TRACKED, idempotent.
  recover_scene            — reset every tracked object + clear the octomap.
  get_scene_summary        — read-only audit of MoveIt's current beliefs.
  set_permitted_contacts   — apply registry-declared acm_allowed_object_pairs
                             to MoveIt's Allowed Collision Matrix, once per
                             session (not a recovery primitive, but lives
                             here alongside the other scene-backend-only
                             skills).
"""

from __future__ import annotations

from typing import Any

from robot_backend.events.decorator import emit_skill_events
from robot_backend.interfaces.robot_backend import RobotBackend


@emit_skill_events
def reset_object_state(
    backend: RobotBackend,
    *,
    object_id: str,
) -> dict[str, Any]:
    """Reset one object to WORLD_TRACKED state regardless of its current state.

    Algorithm (option A — explicit state query before detach):
    1. Call get_objects() to read the current planning scene.
    2. If the object appears in attached_objects: call detach_object().
    3. Call sync_collision_objects([object_id]) to restore the object's
       TF-based geometry in the world scene.

    Idempotent: safe to call when the object is already WORLD_TRACKED (step 2
    is skipped and step 3 is a no-op if TF pose hasn't changed).

    Returns
    -------
    ok             — True if the final sync_collision_objects succeeded.
    was_attached   — True if a detach was needed before the sync.
    """
    steps: list[dict[str, Any]] = []

    # --- 1. Query current scene state ---
    objects_result = backend.scene.get_objects()
    steps.append({"step": "get_objects", "result": objects_result})

    # Backend normalises IDs to lowercase+stripped; mirror that here.
    normalized_id = object_id.strip().lower()
    attached_ids = {
        obj.get("id", "").strip().lower()
        for obj in objects_result.get("attached_objects", [])
    }
    was_attached = normalized_id in attached_ids

    # --- 2. Detach if currently attached ---
    if was_attached:
        detach_result = backend.scene.detach_object(object_id)
        steps.append({"step": "detach_object", "result": detach_result})

    # --- 3. Restore to WORLD_TRACKED via TF ---
    sync_result = backend.scene.sync_collision_objects([object_id])
    steps.append({"step": "sync_collision_objects", "result": sync_result})

    return {
        "ok": sync_result.get("ok", False),
        "command": "reset_object_state",
        "object_id": object_id,
        "was_attached": was_attached,
        "steps": steps,
    }


@emit_skill_events
def recover_scene(
    backend: RobotBackend,
    *,
    registry: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Nuclear recovery: reset all tracked objects then clear the octomap.

    For each enabled object in the registry, calls reset_object_state
    (query scene → detach if attached → sync). After all objects are reset,
    clears the octomap to flush stale perception-derived occupancy voxels.

    Use this after a catastrophic failure where the scene state is completely
    unknown (e.g. agent crash mid-pick, manual e-stop, or a series of
    back-to-back Item 4 grasp failures).

    Parameters
    ----------
    registry:
        Loaded model_registry.yaml content (top-level dict with "objects" key).
        When None or empty, no objects are reset (only octomap is cleared).
    """
    steps: list[dict[str, Any]] = []
    reset_results: list[dict[str, Any]] = []

    # Collect enabled object IDs from registry
    object_ids: list[str] = []
    if registry:
        for oid, entry in registry.get("objects", {}).items():
            if isinstance(entry, dict) and entry.get("enabled", True):
                object_ids.append(oid)

    # Reset each object (each call emits its own skill_start/skill_end events)
    for oid in object_ids:
        result = reset_object_state(backend, object_id=oid)
        reset_results.append(result)
        steps.append({"step": f"reset_object_state[{oid}]", "result": result})

    # Clear perception-derived occupancy
    octomap_result = backend.scene.clear_octomap()
    steps.append({"step": "clear_octomap", "result": octomap_result})

    all_resets_ok = all(r.get("ok", False) for r in reset_results) if reset_results else True
    all_ok = all_resets_ok and octomap_result.get("ok", False)

    return {
        "ok": all_ok,
        "command": "recover_scene",
        "objects_reset": len(reset_results),
        "reset_results": reset_results,
        "clear_octomap_result": octomap_result,
        "steps": steps,
    }


@emit_skill_events
def get_scene_summary(
    backend: RobotBackend,
) -> dict[str, Any]:
    """Return a concise read-only audit of MoveIt's current planning scene.

    Does not modify any state. Use this to verify scene beliefs without
    triggering a sync, to cross-check against WatchDog observations, or
    to diagnose stale attachment state after a failed pick sequence.

    Returns
    -------
    world_collision_objects  — objects currently tracked in the world scene.
    attached_objects         — objects currently attached to the robot.
    world_object_count       — convenience count of world objects.
    attached_object_count    — convenience count of attached objects.
    octomap_voxel_count      — always None; not exposed by the current backend.
    """
    result = backend.scene.get_objects()

    world_objects: list[dict[str, Any]] = result.get("world_collision_objects", [])
    attached_objects: list[dict[str, Any]] = result.get("attached_objects", [])

    return {
        "ok": result.get("ok", False),
        "command": "get_scene_summary",
        "world_collision_objects": world_objects,
        "attached_objects": attached_objects,
        "world_object_count": len(world_objects),
        "attached_object_count": len(attached_objects),
        "octomap_voxel_count": None,
    }


@emit_skill_events
def set_permitted_contacts(
    backend: RobotBackend,
    *,
    registry: dict[str, Any],
) -> dict[str, Any]:
    """Apply the registry's declared ACM-allowed object pairs to MoveIt's ACM.

    Reads registry["acm_allowed_object_pairs"] (defaulting to an empty list
    when absent), validates each pair references two known registry object
    IDs, and applies them via backend.scene.set_permitted_contacts(). A
    typo'd object_id raises immediately rather than silently being a no-op.

    This is a planner tractability setting (disables collision *checking*
    for the pair so intended contact doesn't leave the planning scene
    permanently in collision) -- it carries no task-level permission and
    says nothing about which objects the robot may touch. See the field's
    own comment in model_registry.yaml.
    """
    known_object_ids = set(registry.get("objects", {}).keys())
    raw_pairs = registry.get("acm_allowed_object_pairs") or []

    pairs: list[tuple[str, str]] = []
    for raw_pair in raw_pairs:
        if not isinstance(raw_pair, (list, tuple)) or len(raw_pair) != 2:
            raise ValueError(
                f"acm_allowed_object_pairs entry must be a 2-element list, got {raw_pair!r}."
            )
        a, b = raw_pair
        for object_id in (a, b):
            if object_id not in known_object_ids:
                raise ValueError(
                    f"acm_allowed_object_pairs references unknown object_id {object_id!r}; "
                    f"known objects are {sorted(known_object_ids)}."
                )
        pairs.append((a, b))

    result = backend.scene.set_permitted_contacts(pairs=pairs)

    return {
        "ok": result.get("ok", False),
        "command": "set_permitted_contacts",
        "pairs": [[a, b] for a, b in pairs],
        "result": result,
    }
