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
WAIT_TIMEOUT_SEC = 180.0    # same as the agent's default
MAX_CONSECUTIVE_TIMEOUTS = 2  # same as watchdog_agent_s3.yaml
RUN_READY_RETRY_SEC = 2.0
RUN_READY_MAX_SEC = 120.0


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
        for rid in self.specs:
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
        self.fires += 1
        mid = fire.get("monitor_id", "")
        rid = self._requested_id_for(mid)
        try:
            detail = json.loads(fire.get("detail") or "{}")
        except json.JSONDecodeError:
            detail = {"kind": "unparsed", "raw": fire.get("detail")}
        kind = detail.get("kind", "unknown")
        self.log("fire", monitor_id=mid, requested_id=rid, kind=kind, detail=detail,
                 trigger_wall_ns=fire.get("trigger_wall_ns"))

        x, y, z = detail.get("x"), detail.get("y"), detail.get("z")
        where = (f"world ({x:.2f}, {y:.2f}, {z:.2f})" if isinstance(x, (int, float)) else "unknown")
        reason = f"reference monitor {rid or mid} fired: {kind} at {where}; confirming visually"
        lk = await self.look(reason)
        look_ids = [lk["look_id"]] if lk.get("look_id") else []
        first_seen = detail.get("since_wall_ns") or fire.get("trigger_wall_ns")

        if kind == "removed":
            prior = self._nearest_open(x, y)
            if prior is not None:
                r = await self.call(
                    "record_observation",
                    summary=(f"The unlisted object previously reported at ({prior['x']:.2f}, "
                             f"{prior['y']:.2f}) is no longer there (depth returned to the "
                             f"reference surface at ({x:.2f}, {y:.2f}))."),
                    category=prior["category"], location={"frame": "world", "x": x, "y": y, "z": z},
                    first_seen_wall_ns=prior["first_seen"], evidence_look_ids=look_ids,
                    evidence_monitor_ids=[mid], identity=prior.get("identity"),
                    confidence="high", requirement_impact="none",
                    observation_id=prior["observation_id"], status="resolved",
                )
                self.open_obs.remove(prior)
                self.log("observation_resolved", observation_id=prior["observation_id"], result=r)
            else:
                r = await self.call(
                    "record_observation",
                    summary=(f"Something that was on the bench at ({x:.2f}, {y:.2f}) at the "
                             f"reference capture is gone; no registered object was there."),
                    category="object_missing", location={"frame": "world", "x": x, "y": y, "z": z},
                    first_seen_wall_ns=first_seen, evidence_look_ids=look_ids,
                    evidence_monitor_ids=[mid], identity=None, confidence="medium",
                    requirement_impact="none", status="new",
                )
                self.observations += 1
                self.log("observation_new", kind=kind, result=r)
        elif kind == "object_moved":
            oid = detail.get("object_id")
            r = await self.call(
                "record_observation",
                summary=(f"Registered object {oid} moved {detail.get('delta_m')} m from "
                         f"({detail.get('from_x')}, {detail.get('from_y')}) to ({x}, {y}) while "
                         f"the gripper was not holding it and the TCP was far from it: an "
                         f"external displacement, not a robot action."),
                category="object_moved", location={"frame": "world", "x": x, "y": y, "z": z},
                first_seen_wall_ns=first_seen, evidence_look_ids=look_ids,
                evidence_monitor_ids=[mid], identity=oid, confidence="high",
                requirement_impact="unknown",
                impact_detail="A governed part moved by an external cause; conformance is judged separately.",
                status="new",
            )
            self.observations += 1
            self.log("observation_new", kind=kind, result=r)
        else:  # new_static / moving: an unlisted body in the workspace
            h = detail.get("height_m")
            side = detail.get("side_m")
            moving = kind == "moving"
            identity = (f"unidentified {'moving ' if moving else ''}object, about "
                        f"{side:.2f} m across and {h:.2f} m tall" if isinstance(h, (int, float))
                        else "unidentified object")
            r = await self.call(
                "record_observation",
                summary=(f"An object not on any listing {'entered the workspace and is moving' if moving else 'appeared on the bench'} "
                         f"at world ({x:.2f}, {y:.2f}); depth shows it {h:.2f} m above the bench, "
                         f"about {side:.2f} m across. No registered object is at that position."),
                category="unlisted_object", location={"frame": "world", "x": x, "y": y, "z": z},
                first_seen_wall_ns=first_seen, evidence_look_ids=look_ids,
                evidence_monitor_ids=[mid], identity=identity,
                confidence="high" if not moving else "medium",
                requirement_impact="none", status="new",
            )
            self.observations += 1
            oid = r.get("observation_id") or (r.get("observation") or {}).get("observation_id")
            if oid and isinstance(x, (int, float)):
                self.open_obs.append({"observation_id": oid, "x": x, "y": y, "kind": kind,
                                      "category": "unlisted_object", "identity": identity,
                                      "first_seen": first_seen})
            self.log("observation_new", kind=kind, result=r)

        if rid is not None:
            await self.arm(rid)

    def _nearest_open(self, x: Any, y: Any) -> dict[str, Any] | None:
        if not isinstance(x, (int, float)):
            return None
        best, best_d = None, MATCH_RADIUS_M
        for o in self.open_obs:
            d = math.hypot(o["x"] - x, o["y"] - y)
            if d < best_d:
                best, best_d = o, d
        return best

    async def on_event(self, ev: dict[str, Any]) -> bool:
        """Handle one agent-facing event; True when the run has ended."""
        et = ev.get("event_type")
        payload = ((ev.get("envelope") or {}).get("payload")) or {}
        if et == "begin_step":
            step = payload.get("step_id")
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

    async def loop(self) -> None:
        timeouts = 0
        while True:
            r = await self.call("wait_for_event", timeout_sec=WAIT_TIMEOUT_SEC)
            wake = r.get("wake")
            if wake == "monitor_fire":
                for fire in [r.get("fire")] + list(r.get("additional_fires") or []):
                    if fire:
                        await self.on_fire(fire)
                timeouts = 0
                continue
            if wake == "timeout" or wake is None:
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
