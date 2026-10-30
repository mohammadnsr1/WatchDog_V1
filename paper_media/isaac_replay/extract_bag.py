"""Step 1: rosbag -> replay_state.npz (runs in the paper_media venv, needs rosbags + numpy).

    python -m paper_media.isaac_replay.extract_bag <session_dir> --fps 30 [--start S] [--end S]
        [--out <session_dir>/replay/replay_state.npz] [--config replay_config.yaml] [--check]

Time: every tick's wall time is the bag's message LOG time (the recorder's
receive time, wall clock ns since the epoch). header.stamp is simulation time
and is never read. --start / --end are seconds from the first message.

Joints: every name seen on /joint_states, linearly interpolated, edges held.
Poses: /tf and /tf_static composed to the world frame at every tick.
Translation is interpolated linearly, rotation with slerp, edges held.

Quaternion order in the .npz: **wxyz** (USD Gf.Quatd order). ROS messages are
xyzw; the conversion happens once, at load.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
REGISTRY = REPO_ROOT / "ros2_ws" / "src" / "robot_backend" / "config" / "model_registry.yaml"
GAP_S = 0.5


# ---------------------------------------------------------------------------
# quaternion helpers, wxyz, vectorised over the leading axis
# ---------------------------------------------------------------------------

def q_normalize(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q, axis=-1, keepdims=True)


def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def q_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vectors v [...,3] by unit quaternions q [...,4] (wxyz)."""
    w = q[..., :1]
    u = q[..., 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def q_slerp(q0: np.ndarray, q1: np.ndarray, f: np.ndarray) -> np.ndarray:
    dot = np.sum(q0 * q1, axis=-1)
    q1 = np.where(dot[..., None] < 0, -q1, q1)
    dot = np.abs(dot)
    f = f[..., None]
    close = dot > 0.9995
    theta = np.arccos(np.clip(dot, -1.0, 1.0))[..., None]
    sin_t = np.sin(theta)
    safe = np.where(sin_t == 0, 1.0, sin_t)
    s0 = np.sin((1 - f) * theta) / safe
    s1 = np.sin(f * theta) / safe
    out = np.where(close[..., None], (1 - f) * q0 + f * q1, s0 * q0 + s1 * q1)
    return q_normalize(out)


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------

def _typestore():
    from rosbags.typesys import Stores, get_typestore
    store = getattr(Stores, "ROS2_JAZZY", None) or Stores.ROS2_HUMBLE
    return get_typestore(store)


def read_bag(bag_dir: Path) -> dict:
    """Raw samples, keyed by log time (ns). Nothing is interpolated here."""
    from rosbags.highlevel import AnyReader

    joints: dict[str, tuple[list[int], list[float]]] = defaultdict(lambda: ([], []))
    joint_msgs = 0
    edges: dict[tuple[str, str], tuple[list[int], list, list]] = defaultdict(lambda: ([], [], []))
    static: dict[str, tuple[str, tuple, tuple]] = {}
    first_ns = last_ns = None
    topics = {"/joint_states", "/tf", "/tf_static"}
    with AnyReader([bag_dir], default_typestore=_typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in topics]
        present = {c.topic for c in conns}
        for conn, log_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            first_ns = log_ns if first_ns is None else min(first_ns, log_ns)
            last_ns = log_ns if last_ns is None else max(last_ns, log_ns)
            if conn.topic == "/joint_states":
                joint_msgs += 1
                for i, name in enumerate(msg.name):
                    if i < len(msg.position):
                        joints[name][0].append(log_ns)
                        joints[name][1].append(float(msg.position[i]))
            else:
                for tr in msg.transforms:
                    parent = tr.header.frame_id.lstrip("/")
                    child = tr.child_frame_id.lstrip("/")
                    t = tr.transform.translation
                    r = tr.transform.rotation
                    trans = (t.x, t.y, t.z)
                    quat = (r.w, r.x, r.y, r.z)          # ROS xyzw -> wxyz
                    if conn.topic == "/tf_static":
                        static[child] = (parent, trans, quat)
                    else:
                        e = edges[(parent, child)]
                        e[0].append(log_ns)
                        e[1].append(trans)
                        e[2].append(quat)
    return {"joints": dict(joints), "joint_msgs": joint_msgs, "edges": dict(edges), "static": static,
            "first_ns": first_ns, "last_ns": last_ns, "topics_present": sorted(present)}


# ---------------------------------------------------------------------------
# interpolation onto the grid
# ---------------------------------------------------------------------------

def interp_scalar(ts_ns: np.ndarray, vals: np.ndarray, grid_ns: np.ndarray) -> np.ndarray:
    order = np.argsort(ts_ns, kind="stable")
    return np.interp(grid_ns.astype(np.float64), ts_ns[order].astype(np.float64), vals[order])


def interp_edge(ts_ns: np.ndarray, trans: np.ndarray, quat: np.ndarray, grid_ns: np.ndarray):
    order = np.argsort(ts_ns, kind="stable")
    ts, trans, quat = ts_ns[order].astype(np.float64), trans[order], q_normalize(quat[order])
    g = grid_ns.astype(np.float64)
    pos = np.stack([np.interp(g, ts, trans[:, k]) for k in range(3)], axis=-1)
    if len(ts) == 1:
        return pos, np.repeat(quat[:1], len(g), axis=0)
    i = np.clip(np.searchsorted(ts, g, side="right") - 1, 0, len(ts) - 2)
    span = ts[i + 1] - ts[i]
    f = np.clip(np.where(span > 0, (g - ts[i]) / np.where(span > 0, span, 1), 0.0), 0.0, 1.0)
    rot = q_slerp(quat[i], quat[i + 1], f)
    return pos, rot


def build_parents(raw: dict) -> tuple[dict[str, str], list[str]]:
    """child -> parent (majority parent if a child was ever re-parented), and warnings."""
    warns = []
    seen: dict[str, Counter] = defaultdict(Counter)
    for (p, c), e in raw["edges"].items():
        seen[c][p] += len(e[0])
    for c, (p, _, _) in raw["static"].items():
        seen[c][p] += 10 ** 9  # a static parent wins
    parents = {}
    for c, cnt in seen.items():
        parents[c] = cnt.most_common(1)[0][0]
        if len(cnt) > 1:
            warns.append(f"frame {c!r} has several parents {dict(cnt)}; using {parents[c]!r}")
    return parents, warns


def root_of(frame: str, parents: dict[str, str]) -> tuple[str, list[str]]:
    chain = [frame]
    cur = frame
    for _ in range(128):
        if cur not in parents:
            return cur, chain
        cur = parents[cur]
        chain.append(cur)
    raise ValueError(f"TF cycle through {frame!r}")


def compose_world(raw: dict, grid_ns: np.ndarray, frames: list[str], world_aliases: set[str]):
    """World pose of every requested frame at every tick: (pos [T,F,3], quat [T,F,4] wxyz, kept, dropped)."""
    parents, warns = build_parents(raw)
    T = len(grid_ns)
    cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}

    def link(child: str):
        p = parents[child]
        if child in raw["static"] and raw["static"][child][0] == p:
            _, tr, q = raw["static"][child]
            return np.tile(np.array(tr, float), (T, 1)), np.tile(np.array(q, float), (T, 1))
        ts, tr, q = raw["edges"][(p, child)]
        return interp_edge(np.array(ts), np.array(tr, float), np.array(q, float), grid_ns)

    def world(frame: str):
        if frame in cache:
            return cache[frame]
        if frame not in parents:  # a root
            if frame in world_aliases:
                res = (np.zeros((T, 3)), np.tile(np.array([1.0, 0, 0, 0]), (T, 1)))
                cache[frame] = res
                return res
            raise KeyError(frame)
        p_pos, p_rot = world(parents[frame])
        l_pos, l_rot = link(frame)
        res = (p_pos + q_rotate(p_rot, l_pos), q_normalize(q_mul(p_rot, l_rot)))
        cache[frame] = res
        return res

    kept, dropped, pos, rot = [], {}, [], []
    for f in frames:
        try:
            a, b = world(f)
        except KeyError as exc:
            dropped[f] = f"root {exc.args[0]!r} is not a world frame alias {sorted(world_aliases)}"
            continue
        kept.append(f)
        pos.append(a)
        rot.append(b)
    if not kept:
        return np.zeros((T, 0, 3)), np.zeros((T, 0, 4)), kept, dropped, warns
    return np.stack(pos, axis=1), np.stack(rot, axis=1), kept, dropped, warns


# ---------------------------------------------------------------------------
# config + registry
# ---------------------------------------------------------------------------

def load_config(path: Optional[Path]) -> dict:
    if not path:
        return {}
    import yaml
    return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}


def registry_parts() -> list[tuple[str, str]]:
    """(object_id, tf_frame) for every enabled kind: object entry in the registry."""
    import yaml
    reg = yaml.safe_load(REGISTRY.read_text(encoding="utf-8")) or {}
    out = []
    for oid, e in (reg.get("objects") or {}).items():
        if isinstance(e, dict) and e.get("kind") == "object" and e.get("enabled"):
            out.append((oid, str(e.get("tf_frame") or oid)))
    return out


def iso_ns(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat()


def gaps(ts_ns: list[int]) -> tuple[int, float]:
    if len(ts_ns) < 2:
        return 0, 0.0
    d = np.diff(np.sort(np.asarray(ts_ns, dtype=np.int64))) / 1e9
    big = d[d > GAP_S]
    return int(big.size), float(big.max()) if big.size else 0.0


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def check_report(raw: dict, world_aliases: set[str]) -> int:
    print(f"bag span       {iso_ns(raw['first_ns'])} .. {iso_ns(raw['last_ns'])} "
          f"({(raw['last_ns'] - raw['first_ns']) / 1e9:.1f} s), log time")
    print(f"topics         {raw['topics_present']}")
    print(f"\n/joint_states  {raw['joint_msgs']} messages")
    for name, (ts, _) in sorted(raw["joints"].items()):
        n, mx = gaps(ts)
        print(f"  {name:<34} {len(ts):>7} samples" + (f"   {n} gaps > {GAP_S} s (max {mx:.2f} s)" if n else ""))
    parents, warns = build_parents(raw)
    print(f"\nTF frames      {len(parents)} with a parent; world aliases {sorted(world_aliases)}")
    for child in sorted(parents):
        root, _ = root_of(child, parents)
        if child in raw["static"]:
            cov = "static"
            extra = ""
        else:
            ts = raw["edges"][(parents[child], child)][0]
            cov = f"{len(ts):>6} msgs {(min(ts) - raw['first_ns']) / 1e9:7.1f}..{(max(ts) - raw['first_ns']) / 1e9:7.1f} s"
            n, mx = gaps(ts)
            extra = f"   {n} gaps > {GAP_S} s (max {mx:.2f} s)" if n else ""
        flag = "" if root in world_aliases else f"   ROOT {root!r} NOT A WORLD ALIAS"
        print(f"  {child:<34} parent {parents[child]:<22} {cov}{extra}{flag}")
    for w in warns:
        print(f"  warning: {w}")
    print("\nFMB parts in model_registry.yaml vs TF:")
    missing = 0
    for oid, tf in registry_parts():
        if tf in parents:
            root, _ = root_of(tf, parents)
            print(f"  ok       {oid:<20} tf {tf!r} (root {root!r})")
        else:
            missing += 1
            print(f"  WARNING  {oid:<20} tf {tf!r} never appears in /tf or /tf_static: this part would NOT move in the replay")
    print("\nNot in TF by design: injected intruders and visitors (/World/WatchdogIntruders is outside the "
          "TF publisher's target list), so the replay does not show them.")
    return 1 if missing else 0


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="extract_bag")
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--start", type=float, default=None, help="seconds from the first bag message")
    ap.add_argument("--end", type=float, default=None, help="seconds from the first bag message")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--config", type=Path, default=None, help="replay config (world_frame, world_aliases, track_frames)")
    ap.add_argument("--check", action="store_true", help="print bag contents and exit, write nothing")
    a = ap.parse_args(argv)

    bag = a.session_dir / "bag"
    if not (bag / "metadata.yaml").exists():
        print(f"error: no rosbag2 directory at {bag}", file=sys.stderr)
        return 2
    cfg = load_config(a.config)
    world_frame = str(cfg.get("world_frame", "world"))
    world_aliases = {world_frame, *[str(x) for x in (cfg.get("world_aliases") or [])]}

    print(f"reading {bag} ...", file=sys.stderr)
    raw = read_bag(bag)
    if raw["first_ns"] is None:
        print("error: bag has none of /joint_states, /tf, /tf_static", file=sys.stderr)
        return 2
    if a.check:
        return check_report(raw, world_aliases)

    t0 = raw["first_ns"]
    start = a.start or 0.0
    end = a.end if a.end is not None else (raw["last_ns"] - t0) / 1e9
    if end <= start:
        print(f"error: empty window [{start}, {end}]", file=sys.stderr)
        return 2
    n = int(np.floor((end - start) * a.fps)) + 1
    t_rel = start + np.arange(n) / a.fps
    grid_ns = t0 + np.round(t_rel * 1e9).astype(np.int64)

    joint_names = sorted(raw["joints"])
    joint_pos = np.zeros((n, len(joint_names)))
    for j, name in enumerate(joint_names):
        ts, vals = raw["joints"][name]
        joint_pos[:, j] = interp_scalar(np.array(ts, dtype=np.int64), np.array(vals), grid_ns)

    parents, _ = build_parents(raw)
    track = cfg.get("track_frames")
    frames = sorted(parents) if not track or track == "all" else [str(f) for f in track]
    part_frames = list((cfg.get("part_map") or {}).keys())
    frames = sorted(set(frames) | {f for f in part_frames if f in parents})
    frame_pos, frame_quat, kept, dropped, warns = compose_world(raw, grid_ns, frames, world_aliases)
    for w in warns:
        print(f"warning: {w}", file=sys.stderr)
    for f, why in dropped.items():
        print(f"warning: frame {f!r} dropped: {why}", file=sys.stderr)
    for f in part_frames:
        if f not in kept:
            print(f"warning: part_map frame {f!r} has no world pose in this bag; that part will not move", file=sys.stderr)

    out = a.out or (a.session_dir / "replay" / "replay_state.npz")
    out.parent.mkdir(parents=True, exist_ok=True)
    meta = {"session_dir": str(a.session_dir.resolve()), "fps": a.fps, "start_s": start, "end_s": end,
            "first_log_ns": int(t0), "world_frame": world_frame, "world_aliases": sorted(world_aliases),
            "quaternion_order": "wxyz", "time_source": "rosbag log time (receive wall clock)",
            "dropped_frames": dropped, "generated_utc": datetime.now(timezone.utc).isoformat()}
    np.savez_compressed(
        out,
        t_rel=t_rel, log_time_ns=grid_ns,
        wall_time_iso=np.array([iso_ns(int(x)) for x in grid_ns]),
        joint_names=np.array(joint_names), joint_pos=joint_pos,
        frame_names=np.array(kept), frame_pos=frame_pos, frame_quat=frame_quat,
        meta=np.array(json.dumps(meta)),
    )
    print(f"wrote {out}: {n} ticks at {a.fps:g} fps, {len(joint_names)} joints, {len(kept)} frames "
          f"({iso_ns(int(grid_ns[0]))} .. {iso_ns(int(grid_ns[-1]))})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
