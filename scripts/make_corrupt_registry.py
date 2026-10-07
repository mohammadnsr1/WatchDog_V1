#!/usr/bin/env python3
"""make_corrupt_registry.py — build the corrupted registry for a Class 4 run.

USAGE:
  python3 scripts/make_corrupt_registry.py OUT.yaml \
      --set objects.u_brown.spec.limits.max_clamp_effort=400 \
      --set objects.u_green.spec.tolerances.disturbance_tolerance_m=0.2
  EXECUTOR_REGISTRY_PATH=$(pwd)/OUT.yaml ./run_executor.sh 2

Copies the INSTALL-space registry (the file a clean run loads), applies the
--set edits, and writes OUT.yaml. Each edit is a dotted path from the document
root; the field must already exist (a typo must fail loudly, not add a key no
code reads). Values parse as YAML scalars (numbers stay numbers).

asset_root is forced to the source registry's own directory (absolute):
relative mesh paths resolve against the registry FILE's directory, so a copy
written anywhere else would otherwise break every mesh load in the executor's
scene server.

The edits are recorded in a YAML comment header and a sibling OUT.yaml.edits.json
(old/new per field), and the file's sha256 is printed — that hash goes in
batch.yaml's run_registry_sha256 for the session. The run itself also records
it (registry_provenance_executor.json, written by the loader when the
override is active).

This writes a NEW file only. It never touches the install-space registry: the
cell and, in S1–S3, the WatchDog must keep loading the clean one — that is
the whole point of Class 4.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
INSTALL_REGISTRY = (REPO_ROOT / "ros2_ws" / "install" / "robot_backend" / "share" /
                    "robot_backend" / "config" / "model_registry.yaml")


def apply_edit(doc: dict, dotted: str, raw_value: str) -> tuple[object, object]:
    """Set doc[dotted path] = parsed value. Returns (old, new). Fails loudly."""
    parts = dotted.split(".")
    node = doc
    for part in parts[:-1]:
        if not isinstance(node, dict) or part not in node:
            raise KeyError(
                f"--set {dotted!r}: path component {part!r} does not exist in "
                f"the registry. Edits may only change fields that are already "
                f"there."
            )
        node = node[part]
    leaf = parts[-1]
    if not isinstance(node, dict) or leaf not in node:
        raise KeyError(
            f"--set {dotted!r}: field {leaf!r} does not exist in the registry."
        )
    old = node[leaf]
    new = yaml.safe_load(raw_value)
    node[leaf] = new
    return old, new


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="path to write the corrupted registry to")
    ap.add_argument("--source", default=str(INSTALL_REGISTRY),
                    help="clean registry to start from (default: install space)")
    ap.add_argument("--set", dest="edits", action="append", required=True,
                    metavar="DOTTED.PATH=VALUE",
                    help="field to corrupt, e.g. "
                         "objects.u_brown.spec.limits.max_clamp_effort=400 "
                         "(repeatable; at least one required)")
    a = ap.parse_args()

    source = Path(a.source).resolve()
    doc = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        sys.exit(f"ERROR: {source} is not a mapping")

    # Relative mesh paths resolve against the registry file's own directory;
    # pin them to the CLEAN registry's directory so the copy loads identically.
    doc["asset_root"] = str(source.parent)

    applied = []
    for edit in a.edits:
        if "=" not in edit:
            sys.exit(f"ERROR: --set needs DOTTED.PATH=VALUE, got {edit!r}")
        dotted, raw_value = edit.split("=", 1)
        try:
            old, new = apply_edit(doc, dotted.strip(), raw_value.strip())
        except KeyError as exc:
            sys.exit(f"ERROR: {exc}")
        if old == new:
            sys.exit(f"ERROR: --set {dotted!r}: new value equals the clean value "
                     f"({old!r}) — that is not a corruption and the run would "
                     f"score as if nothing was injected.")
        applied.append({"field": dotted.strip(), "clean_value": old,
                        "corrupted_value": new})

    out = Path(a.out).resolve()
    if out == source:
        sys.exit("ERROR: refusing to overwrite the clean registry in place.")
    header = (
        f"# CORRUPTED registry for a Class 4 run — generated "
        f"{datetime.now(timezone.utc).isoformat()}\n"
        f"# source: {source}\n"
        + "".join(f"# corrupted: {e['field']}  {e['clean_value']!r} -> "
                  f"{e['corrupted_value']!r}\n" for e in applied)
        + "# NEVER install this file; use it via EXECUTOR_REGISTRY_PATH only.\n"
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(header + yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    (out.parent / (out.name + ".edits.json")).write_text(
        json.dumps({"source": str(source), "edits": applied}, indent=2),
        encoding="utf-8")

    sha = hashlib.sha256(out.read_bytes()).hexdigest()
    clean_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    print(f"wrote {out}")
    print(f"corrupted sha256: {sha}   (-> batch.yaml run_registry_sha256)")
    print(f"clean    sha256: {clean_sha}")
    print(f"run with: EXECUTOR_REGISTRY_PATH={out} ./run_executor.sh ...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
