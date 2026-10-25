#!/usr/bin/env python3
"""Scripted WatchDog that runs the reference scene monitors through the real
watchdog MCP server, for an ORACLE run: no LLM, same tools, same record.

It plays the watchdog role exactly as the agent protocol requires (run-scope
ready, per-step and per-phase gate release) so the executor runs normally,
and it handles observation fires the way the duty asks: look, then
record_observation with the look as evidence, then re-arm. Everything lands
on the same session timeline as an agent run, so observation_scorer.py scores
it unchanged -- that is the point: the reference row is produced by the same
pipeline as the agent rows.

What it deliberately does NOT do: record any conformance verdict. It is a
scene-observation oracle, not a conformance one; watchdog_scorer output for
this session is meaningless and must not be registered as a conformance row.

Usage (cell already up, executor session about to start or started):
  cd ~/watchdog/WatchDog_V1
  unset VIRTUAL_ENV
  ros2_ws/.venv/bin/python3 reference_watchdog/run_reference_watchdog.py \
      --config watchdog_2_s3.mcp.json
  ros2_ws/.venv/bin/python3 reference_watchdog/run_reference_watchdog.py \
      --config watchdog_2_s3.mcp.json --smoke     # connect, dry-run, exit

The MCP server is spawned from the SAME command and env the Claude Code
session would use (read from the mcp.json), so the scenario, task and
registry are identical to an agent run. The venv must be ros2_ws/.venv
(mcp 1.x, FastMCP) -- see watchdog.mcp.json's _venv_comment.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import re
import sys
import time
from datetime import timedelta
from pathlib import Path
from typing import Any

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

sys.path.insert(0, str(Path(__file__).resolve().parent))
import monitors as ref  # noqa: E402

LOG_NAME = "reference_watchdog_log.jsonl"
MATCH_RADIUS_M = 0.5        # a removal this close to an open observation resolves it
SAME_OBJECT_RADIUS_M = 0.3  # a new/moving fire this close to an open observation updates it
SAME_EVENT_WINDOW_S = 30.0  # repeated object_moved fires on one object within this = one event
MOVING_MATCH_RADIUS_M = 1.0 # a moving body seen again within this of its last report = same body
RESOLVE_GRACE_S = 3.0       # a 'removed' near a moving body resolves it only if it is not seen again
SETTLE_S = 5.0              # after an object_moved fire, let the physics settle before re-arming
SAME_OBJECT_HEIGHT_M = 0.08 # two sightings with heights further apart than this are different bodies
WAIT_TIMEOUT_SEC = 180.0    # same as the agent's default
MAX_CONSECUTIVE_TIMEOUTS = 2  # same as watchdog_agent_s3.yaml
RUN_READY_RETRY_SEC = 2.0
RUN_READY_MAX_SEC = 120.0


def _finite(v: Any) -> float | None:
    """A finite float, or None for anything else (missing, null, nan, text)."""
    return float(v) if isinstance(v, (int, float)) and math.isfinite(v) else None


def _num(v: Any, nd: int = 2) -> str:
    return f"{v:.{nd}f}" if isinstance(v, (int, float)) and math.isfinite(v) else "?"


def _session_dir() -> Path | None:
    try:
        p = Path(Path("~/.watchdog/current_session").expanduser().read_text().strip())
        return p if p.is_dir() else None
    except Exception:
        return None


class Log:
    def __init__(self, session_dir: Path | None):
        self.path = (session_dir / LOG_NAME) if session_dir else None
        self.fh = open(self.path, "a") if self.path else None

    def __call__(self, kind: str, **fields: Any) -> None:
        rec = {"t": time.time(), "kind": kind, **fields}
        line = json.dumps(rec, default=str)
        print(line, flush=True)
        if self.fh:
            self.fh.write(line + "\n")
            self.fh.flush()


def _parse_result(result: Any) -> dict[str, Any]:
    """FastMCP returns a dict as one TextContent of JSON; look() returns
    [text, image...]. Return the first JSON text part as a dict."""
    for part in getattr(result, "content", []) or []:
        text = getattr(part, "text", None)
        if text is None:
            continue
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            continue
        if isinstance(parsed, dict):
            return parsed
    return {"ok": False, "error": "no JSON text part in tool result", "raw": str(result)[:500]}


class ReferenceWatchdog:
    def __init__(self, session: ClientSession, log: Log):
        self.s = session
        self.log = log
        self.specs = {spec()["monitor_id"]: spec() for spec in ref.ALL_SPECS}
        self.armed: dict[str, str] = {}          # requested id -> server monitor_id
        self.open_obs: list[dict[str, Any]] = []  # {observation_id, x, y, kind}
        self.recent_moves: dict[str, dict[str, Any]] = {}  # object_id -> last object_moved record
        self.pending_resolve: list[dict[str, Any]] = []   # {obs, due, args} awaiting the grace period
        self.pending_arm: dict[str, float] = {}           # requested_id -> monotonic time to re-arm
        self.fires = 0
        self.observations = 0

    async def call(self, name: str, **args: Any) -> dict[str, Any]:
        result = await self.s.call_tool(name, args)
        parsed = _parse_result(result)
        if not parsed.get("ok", True):
            self.log("tool_error", tool=name, error=parsed.get("error"), args=list(args))
        return parsed

    # -- protocol ----------------------------------------------------------

    async def ready_run(self) -> None:
        t0 = time.monotonic()
        while True:
            r = await self.call("signal_ready", scope="run")
            if r.get("ok"):
                self.log("run_ready", waited_sec=round(time.monotonic() - t0, 1))
                return
            if time.monotonic() - t0 > RUN_READY_MAX_SEC:
                raise SystemExit("recorder buffers never warmed; is the cell up?")
            await asyncio.sleep(RUN_READY_RETRY_SEC)

    async def arm(self, requested_id: str) -> None:
        spec = dict(self.specs[requested_id])
        r = await self.call("arm_custom_monitor", **spec)
        mid = r.get("monitor_id") or (r.get("result") or {}).get("monitor_id") or requested_id
        if r.get("ok"):
            self.armed[requested_id] = mid
            self.log("armed", requested_id=requested_id, monitor_id=mid,
                     first_call_result=r.get("first_call_result"),
                     rate_hz_clamped=r.get("rate_hz_clamped"))
        else:
            self.log("arm_failed", requested_id=requested_id, error=r.get("error"))

    async def arm_all(self) -> None:
        # The scene watch is armed at the FIRST begin_step, not at run ready:
        # the executor is then blocked at the gate with the arm at home, so
        # the reference is captured with the arm out of the way instead of
        # folded over the bench at startup (the live 2026-10-23 false fire
        # came from structure uncovered by the move to home).
        self.scene_watch_pending = True
        for rid in self.specs:
            if rid != "ref_scene_watch":
                await self.arm(rid)

    def _requested_id_for(self, monitor_id: str) -> str | None:
        for rid, mid in self.armed.items():
            if mid == monitor_id or monitor_id.startswith(rid):
                return rid
        return None

    async def look(self, reason: str) -> dict[str, Any]:
        r = await self.call("look", reason=reason, views=["rgb"], max_width=640)
        return r

    async def on_fire(self, fire: dict[str, Any]) -> None:
        """Never let one fire kill the run: any failure is logged and the
        monitor is re-armed."""
        self.fires += 1
        mid = fire.get("monitor_id", "")
        rid = self._requested_id_for(mid)
        try:
            await self._handle_fire(fire, mid, rid)
        except Exception as exc:  # noqa: BLE001 - the run must go on
            import traceback
            self.log("fire_handler_error", monitor_id=mid, error=repr(exc),
                     traceback=traceback.format_exc()[-2000:])
        if rid == "ref_undisturbed_watch":
            self.pending_arm[rid] = time.monotonic() + SETTLE_S
        elif rid is not None:
            await self.arm(rid)

    async def _handle_fire(self, fire: dict[str, Any], mid: str, rid: str | None) -> None:
        raw = fire.get("detail") or "{}"
        try:
            # the sandbox has no json module; a non-finite float is spelled nan/inf
            detail = json.loads(re.sub(r":\s*-?(nan|inf)\b", ":null", raw))
        except json.JSONDecodeError:
            detail = {"kind": "unparsed", "raw": raw}
        kind = detail.get("kind", "unknown")
        self.log("fire", monitor_id=mid, requested_id=rid, fire_kind=kind, detail=detail,
                 trigger_wall_ns=fire.get("trigger_wall_ns"))

        x, y, z = (_finite(detail.get(k)) for k in ("x", "y", "z"))
        where = f"world ({_num(x)}, {_num(y)}, {_num(z)})" if x is not None else "an unknown position"
        reason = f"reference monitor {rid or mid} fired: {kind} at {where}; confirming visually"
        lk = await self.look(reason)
        look_ids = [lk["look_id"]] if lk.get("look_id") else []
        first_seen = detail.get("since_wall_ns") or fire.get("trigger_wall_ns")
        if x is not None:
            location: dict[str, Any] | None = {"frame": "world", "x": x, "y": y, "z": z}
        elif look_ids and _finite(detail.get("u")) is not None:
            location = {"pixel": {"u": detail["u"], "v": detail["v"]}, "look_id": look_ids[0]}
        else:
            location = None

        if kind == "removed":
            prior = self._nearest_open(x, y, h=_finite(detail.get("height_m")))
            if prior is not None:
                args = dict(
                    summary=(f"The unlisted object previously reported at ({_num(prior['x'])}, "
                             f"{_num(prior['y'])}) is no longer there (depth returned to the "
                             f"reference surface at ({_num(x)}, {_num(y)}))."),
                    category=prior["category"], location=location,
                    first_seen_wall_ns=prior["first_seen"], evidence_look_ids=look_ids,
                    evidence_monitor_ids=[mid], identity=prior.get("identity"),
                    confidence="high", requirement_impact="none",
                    observation_id=prior["observation_id"], status="resolved",
                )
                # a body that was moving may simply have moved on: resolve only
                # if it is not seen again within the grace period
                self.pending_resolve = [p for p in self.pending_resolve if p["obs"] is not prior]
                self.pending_resolve.append({"obs": prior, "due": time.monotonic() + RESOLVE_GRACE_S, "args": args})
                self.log("resolve_pending", observation_id=prior["observation_id"])
            else:
                r = await self.call(
                    "record_observation",
                    summary=(f"Something that was on the bench at ({_num(x)}, {_num(y)}) at the "
                             f"reference capture is gone; no registered object was there."),
                    category="object_missing", location=location,
                    first_seen_wall_ns=first_seen, evidence_look_ids=look_ids,
                    evidence_monitor_ids=[mid], identity=None, confidence="medium",
                    requirement_impact="none", status="new",
                )
                self.observations += 1
                self.log("observation_new", fire_kind=kind, result=r)
        elif kind == "object_moved":
            oid = detail.get("object_id")
            summary = (f"Registered object {oid} moved {detail.get('delta_m')} m from "
                       f"({detail.get('from_x')}, {detail.get('from_y')}) to ({_num(x)}, {_num(y)}) "
                       f"while the gripper was not holding it and the TCP was far from it: an "
                       f"external displacement, not a robot action.")
            prior = self._recent_object(oid)
            common = dict(category="object_moved", location=location, evidence_look_ids=look_ids,
                          evidence_monitor_ids=[mid], identity=oid, confidence="high",
                          requirement_impact="unknown",
                          impact_detail="A governed part moved by an external cause; conformance is judged separately.")
            if prior is not None:
                r = await self.call("record_observation", summary=summary + " (continued motion)",
                                    first_seen_wall_ns=prior["first_seen"], status="update",
                                    observation_id=prior["observation_id"], **common)
                prior.update(x=x, y=y, t=time.monotonic())
                self.log("observation_update", fire_kind=kind, observation_id=prior["observation_id"], ok=r.get("ok"))
            else:
                r = await self.call("record_observation", summary=summary,
                                    first_seen_wall_ns=first_seen, status="new", **common)
                self.observations += 1
                obs_id = r.get("observation_id")
                if obs_id:
                    self.recent_moves[oid] = {"observation_id": obs_id, "first_seen": first_seen,
                                              "x": x, "y": y, "t": time.monotonic()}
                self.log("observation_new", fire_kind=kind, result=r)
        else:  # new_static / moving: an unlisted body in the workspace
            h = _finite(detail.get("height_m"))
            side = _finite(detail.get("side_m"))
            moving = kind == "moving"
            identity = (f"unidentified {'moving ' if moving else ''}object, about "
                        f"{_num(side)} m across and {_num(h)} m tall" if h is not None
                        else "unidentified object")
            summary = (f"An object not on any listing {'entered the workspace and is moving' if moving else 'appeared on the bench'} "
                       f"at world ({_num(x)}, {_num(y)}); depth shows it {_num(h)} m above the bench, "
                       f"about {_num(side)} m across. No registered object is at that position.")
            common = dict(category="unlisted_object", location=location, evidence_look_ids=look_ids,
                          evidence_monitor_ids=[mid], identity=identity,
                          confidence="high" if not moving else "medium", requirement_impact="none")
            prior = self._nearest_open(x, y, radius=SAME_OBJECT_RADIUS_M, h=h)
            if prior is None and x is not None:
                for pend in self.pending_resolve:   # a moving body seen again: same body
                    o = pend["obs"]
                    if (math.hypot(o["x"] - x, o["y"] - y) < MOVING_MATCH_RADIUS_M
                            and (h is None or o.get("h") is None or abs(h - o["h"]) <= SAME_OBJECT_HEIGHT_M)):
                        prior = o
                        break
            if prior is not None:
                self.pending_resolve = [p for p in self.pending_resolve if p["obs"] is not prior]
                # the same body seen again (moving then at rest, or still moving): one record
                r = await self.call("record_observation", summary=summary + " (same object, updated position)",
                                    first_seen_wall_ns=prior["first_seen"], status="update",
                                    observation_id=prior["observation_id"], **common)
                prior.update(x=x, y=y, kind=kind, identity=identity, h=h if h is not None else prior.get("h"))
                self.log("observation_update", fire_kind=kind, observation_id=prior["observation_id"], ok=r.get("ok"))
            else:
                r = await self.call("record_observation", summary=summary,
                                    first_seen_wall_ns=first_seen, status="new", **common)
                self.observations += 1
                obs_id = r.get("observation_id")
                if obs_id and x is not None:
                    self.open_obs.append({"observation_id": obs_id, "x": x, "y": y, "kind": kind,
                                          "category": "unlisted_object", "identity": identity,
                                          "first_seen": first_seen, "h": h})
                self.log("observation_new", fire_kind=kind, result=r)

    def _recent_object(self, oid: str | None) -> dict[str, Any] | None:
        rec = self.recent_moves.get(oid) if oid else None
        if rec and time.monotonic() - rec["t"] < SAME_EVENT_WINDOW_S:
            return rec
        return None

    def _nearest_open(self, x: Any, y: Any, radius: float = MATCH_RADIUS_M,
                      h: float | None = None) -> dict[str, Any] | None:
        """The open observation this fire most plausibly belongs to: within
        radius (a body last seen moving may be anywhere within its reach), and
        of similar height when both heights are known -- a 4.5 cm cube never
        absorbs a 19 cm forearm passing 27 cm from it (live 2026-10-25)."""
        if x is None or y is None:
            return None
        best, best_d = None, None
        for o in self.open_obs:
            limit = max(radius, MOVING_MATCH_RADIUS_M) if o.get("kind") == "moving" else radius
            oh = o.get("h")
            if h is not None and oh is not None and abs(h - oh) > SAME_OBJECT_HEIGHT_M:
                continue
            d = math.hypot(o["x"] - x, o["y"] - y)
            if d < limit and (best_d is None or d < best_d):
                best, best_d = o, d
        return best

    async def on_event(self, ev: dict[str, Any]) -> bool:
        """Handle one agent-facing event; True when the run has ended."""
        et = ev.get("event_type")
        payload = ((ev.get("envelope") or {}).get("payload")) or {}
        if et == "begin_step":
            step = payload.get("step_id")
            if getattr(self, "scene_watch_pending", False):
                self.scene_watch_pending = False
                await self.arm("ref_scene_watch")
            r = await self.call("signal_ready", scope="step", step_id=step)
            self.log("step_released", step_id=step, objects=payload.get("object_ids"), ok=r.get("ok"))
        elif et == "phase":
            step, phase = payload.get("step_id"), payload.get("phase")
            r = await self.call("signal_ready", scope="phase", step_id=step, phase=phase)
            self.log("phase_released", step_id=step, phase=phase, ok=r.get("ok"))
        elif et == "monitor_disarmed":
            reason = str(payload.get("reason") or "")
            mid = payload.get("monitor_id", "")
            if reason.startswith("authored_"):
                rid = self._requested_id_for(mid)
                self.log("monitor_died", monitor_id=mid, reason=reason, detail=payload.get("detail"))
                if rid is not None:
                    await self.arm(rid)
        elif et == "run_ended":
            self.log("run_ended", status=payload.get("status"), reason=payload.get("reason"))
            return True
        return False

    async def _flush_pending(self) -> None:
        now = time.monotonic()
        for pend in [p for p in self.pending_resolve if p["due"] <= now]:
            self.pending_resolve.remove(pend)
            r = await self.call("record_observation", **pend["args"])
            if pend["obs"] in self.open_obs:
                self.open_obs.remove(pend["obs"])
            self.log("observation_resolved", observation_id=pend["obs"]["observation_id"], result=r)
        for rid in [k for k, due in self.pending_arm.items() if due <= now]:
            del self.pending_arm[rid]
            await self.arm(rid)

    def _next_wait(self) -> float:
        dues = [p["due"] for p in self.pending_resolve] + list(self.pending_arm.values())
        if not dues:
            return WAIT_TIMEOUT_SEC
        return max(0.2, min(WAIT_TIMEOUT_SEC, min(dues) - time.monotonic()))

    async def loop(self) -> None:
        timeouts = 0
        while True:
            await self._flush_pending()
            short = self._next_wait()
            r = await self.call("wait_for_event", timeout_sec=short)
            wake = r.get("wake")
            if wake == "monitor_fire":
                for fire in [r.get("fire")] + list(r.get("additional_fires") or []):
                    if fire:
                        await self.on_fire(fire)
                timeouts = 0
                continue
            if wake == "timeout" or wake is None:
                if short < WAIT_TIMEOUT_SEC:
                    continue  # a short wait for pending bookkeeping, not an executor silence
                timeouts += 1
                self.log("wait_timeout", consecutive=timeouts, waited_sec=r.get("waited_sec"),
                         finalized=r.get("finalized"))
                if r.get("finalized") or timeouts >= MAX_CONSECUTIVE_TIMEOUTS:
                    return
                continue
            timeouts = 0
            ended = False
            for ev in [r.get("event")] + list(r.get("additional_events") or []):
                if ev and await self.on_event(ev):
                    ended = True
            if ended:
                await self._flush_pending()
                return


def _server_params(config_path: Path, session_dir: Path | None) -> StdioServerParameters:
    cfg = json.loads(config_path.read_text())
    servers = cfg["mcpServers"]
    if len(servers) != 1:
        raise SystemExit(f"{config_path} must name exactly one server (watchdog); found {list(servers)}")
    (name, srv), = servers.items()
    env = dict(os.environ)
    env.pop("VIRTUAL_ENV", None)
    env.update(srv.get("env", {}))
    if session_dir is not None:
        # Only for --smoke against a scratch directory: a scored run must keep
        # resolving the bringup-written pointer, never an override by hand.
        env["WATCHDOG_SESSION_DIR"] = str(session_dir)
    return StdioServerParameters(command=srv["command"], args=srv.get("args", []), env=env)


async def _main(args: argparse.Namespace) -> int:
    override = args.session_dir if args.smoke else None
    if args.session_dir and not args.smoke:
        raise SystemExit("--session-dir is only honoured with --smoke; a real run uses "
                         "the bringup-written ~/.watchdog/current_session pointer")
    session_dir = override or _session_dir()
    if override:
        override.mkdir(parents=True, exist_ok=True)
    log = Log(session_dir)
    log("start", config=str(args.config), session_dir=str(session_dir), smoke=args.smoke,
        monitors=[s()["monitor_id"] for s in ref.ALL_SPECS])
    params = _server_params(args.config, override)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write, read_timeout_seconds=timedelta(seconds=700)) as session:
            await session.initialize()
            tools = {t.name for t in (await session.list_tools()).tools}
            needed = {"signal_ready", "arm_custom_monitor", "dry_run_custom_monitor",
                      "wait_for_event", "look", "record_observation"}
            missing = needed - tools
            if missing:
                raise SystemExit(f"watchdog server lacks tools {sorted(missing)}")
            wd = ReferenceWatchdog(session, log)
            if args.smoke:
                for rid, spec in wd.specs.items():
                    dry = {k: spec[k] for k in ("code", "params", "object_id", "description",
                                                "rate_hz", "purpose")}
                    r = await wd.call("dry_run_custom_monitor", ticks=3, **dry)
                    log("dry_run", requested_id=rid, ok=r.get("ok"), error=r.get("error"),
                        returned=r.get("returned", r.get("first_call_result")),
                        usage=r.get("usage"), channels_read=r.get("channels_read"))
                log("smoke_done")
                return 0
            await wd.ready_run()
            await wd.arm_all()
            await wd.look("Reference baseline of the workspace before the first step, for later comparison.")
            await wd.loop()
            log("done", fires=wd.fires, observations=wd.observations,
                open_observations=[o["observation_id"] for o in wd.open_obs])
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=Path("watchdog_2_s3.mcp.json"),
                    help="watchdog mcp.json whose single server to spawn (default: S3, assembly 2)")
    ap.add_argument("--smoke", action="store_true",
                    help="connect, dry-run both monitors through the server, exit (no run)")
    ap.add_argument("--session-dir", type=Path, default=None,
                    help="--smoke only: scratch session directory for the server, so a smoke "
                         "never writes into a real session")
    args = ap.parse_args()
    if not args.config.is_file():
        raise SystemExit(f"no such config: {args.config}")
    return asyncio.run(_main(args))


if __name__ == "__main__":
    sys.exit(main())
