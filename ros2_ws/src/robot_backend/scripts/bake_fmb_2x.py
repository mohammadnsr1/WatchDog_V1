#!/usr/bin/env python3
"""Bake a uniform vertex-level scale into FMB collision assets (USD + OBJ).

Requires a Python with `pxr` on the path, e.g.:
    ros2_ws/.venv/bin/python ros2_ws/src/robot_backend/scripts/bake_fmb_2x.py

For each part's single collision-tagged UsdGeom.Mesh prim:
    scaled_points_m = raw_usd_points * stage.metersPerUnit * scale_factor

The mesh prim's own local xformOp:scale is a redundant artifact of the
Collected_fmb_assemblies export (it duplicates metersPerUnit) and is never
composed into the transform -- see the fmb_u_green.obj provenance audit.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, Vt

REPO_ROOT = Path(__file__).resolve().parents[4]
SRC_USD_DIR = REPO_ROOT / "assets/usd/Collected_fmb_assemblies/Collected_fmb2"
OBJ_OUT_DIR = REPO_ROOT / "ros2_ws/src/robot_backend/config/assets"
USD_OUT_DIR = SRC_USD_DIR

DEFAULT_PARTS = ["base2", "hex_blue", "hex_red", "inverted_u_brown", "u_green"]
REL_SRC_PREFIX = "assets/usd/Collected_fmb_assemblies/Collected_fmb2"


class PartFailed(Exception):
    pass


@dataclass
class PartReport:
    part: str
    src_usd: str
    meters_per_unit: float
    n_verts: int
    n_tris: int
    aabb_1x: tuple
    aabb_2x: tuple
    ancestors_identity: bool
    points_match: bool
    aabb_ratio_ok: bool
    status: str = "OK"
    detail: str = ""


def _is_close_vec(v, target, tol=1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(v, target))


def find_collision_mesh(stage: Usd.Stage) -> Usd.Prim:
    candidates = [
        p
        for p in stage.Traverse()
        if p.IsA(UsdGeom.Mesh) and "PhysicsCollisionAPI" in p.GetAppliedSchemas()
    ]
    if len(candidates) != 1:
        raise PartFailed(
            f"expected exactly 1 collision mesh prim, found {len(candidates)}: "
            f"{[str(p.GetPath()) for p in candidates]}"
        )
    return candidates[0]


def assert_ancestors_identity(mesh_prim: Usd.Prim) -> None:
    p = mesh_prim.GetParent()
    root_path = mesh_prim.GetStage().GetPseudoRoot().GetPath()
    while p and p.GetPath() != root_path:
        xf = UsdGeom.Xformable(p)
        for op in xf.GetOrderedXformOps():
            name = op.GetOpName()
            val = op.Get()
            if val is None:
                continue
            if "translate" in name and not _is_close_vec(val, (0, 0, 0)):
                raise PartFailed(f"non-identity translate on {p.GetPath()}: {val}")
            if "rotate" in name and not _is_close_vec(val, (0, 0, 0)):
                raise PartFailed(f"non-identity rotate on {p.GetPath()}: {val}")
            if "scale" in name and not _is_close_vec(val, (1, 1, 1)):
                raise PartFailed(f"non-identity scale on {p.GetPath()}: {val}")
        p = p.GetParent()


def get_mesh_local_scale_op(mesh_prim: Usd.Prim) -> Usd.Attribute:
    xf = UsdGeom.Xformable(mesh_prim)
    for op in xf.GetOrderedXformOps():
        if "scale" in op.GetOpName():
            return op.GetAttr()
    raise PartFailed(f"{mesh_prim.GetPath()} has no local xformOp:scale to normalize")


def aabb(points) -> tuple:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    lo = (min(xs), min(ys), min(zs))
    hi = (max(xs), max(ys), max(zs))
    return lo, hi


def aabb_size(bb) -> tuple:
    lo, hi = bb
    return tuple(h - l for l, h in zip(lo, hi))


def format_num(v: float) -> str:
    s = f"{v:.9f}".rstrip("0")
    if s.endswith("."):
        s += "0"
    return s


def write_obj(path: Path, part: str, src_usd_rel: str, points, tri_indices) -> None:
    lines = [
        f"# FMB {part} collision mesh extracted from",
        f"# {src_usd_rel},",
        "# scaled x2 at vertex level, units meters.",
    ]
    for p in points:
        lines.append(f"v {format_num(p[0])} {format_num(p[1])} {format_num(p[2])}")
    for i0, i1, i2 in tri_indices:
        lines.append(f"f {i0 + 1} {i1 + 1} {i2 + 1}")
    path.write_text("\n".join(lines) + "\n")


def read_obj_vertices(path: Path):
    verts = []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append(tuple(float(x) for x in line.split()[1:4]))
    return verts


def triangles_from_counts_indices(counts, indices):
    tris = []
    cursor = 0
    for c in counts:
        if c != 3:
            raise PartFailed(f"non-triangle face with {c} vertices encountered")
        tris.append(tuple(indices[cursor : cursor + 3]))
        cursor += c
    return tris


def process_part(part: str, scale_factor: float) -> PartReport:
    src_path = SRC_USD_DIR / f"{part}.usd"
    src_rel = f"{REL_SRC_PREFIX}/{part}.usd"
    src_stage = Usd.Stage.Open(str(src_path))
    mpu = UsdGeom.GetStageMetersPerUnit(src_stage)
    mesh_prim = find_collision_mesh(src_stage)

    assert_ancestors_identity(mesh_prim)

    local_scale_attr = get_mesh_local_scale_op(mesh_prim)
    local_scale = local_scale_attr.Get()
    if not _is_close_vec(local_scale, (mpu, mpu, mpu), tol=1e-6):
        raise PartFailed(
            f"mesh local xformOp:scale {local_scale} does not match the expected "
            f"redundant (metersPerUnit,metersPerUnit,metersPerUnit)=({mpu},{mpu},{mpu}); "
            "refusing to silently ignore an unexpected local scale"
        )

    mesh = UsdGeom.Mesh(mesh_prim)
    raw_points = mesh.GetPointsAttr().Get()
    fv_counts = mesh.GetFaceVertexCountsAttr().Get()
    fv_indices = mesh.GetFaceVertexIndicesAttr().Get()
    tris = triangles_from_counts_indices(fv_counts, fv_indices)

    points_1x = [tuple(c * mpu for c in p) for p in raw_points]
    points_2x = [tuple(c * mpu * scale_factor for c in p) for p in raw_points]

    aabb_1x = aabb(points_1x)
    aabb_2x = aabb(points_2x)
    size_1x = aabb_size(aabb_1x)
    size_2x = aabb_size(aabb_2x)
    ratio_ok = all(
        abs(s2 - s1 * scale_factor) <= 1e-6 for s1, s2 in zip(size_1x, size_2x)
    )

    # --- USD output: full copy of the source layer, then rewrite points +
    # normalize the scale representation so no factor is double-applied. ---
    usd_out_path = USD_OUT_DIR / f"{part}_2x.usd"
    shutil.copy2(src_path, usd_out_path)
    out_stage = Usd.Stage.Open(str(usd_out_path))
    out_mesh_prim = out_stage.GetPrimAtPath(mesh_prim.GetPath())
    out_mesh = UsdGeom.Mesh(out_mesh_prim)

    points_vt = Vt.Vec3fArray([Gf.Vec3f(*p) for p in points_2x])
    out_mesh.GetPointsAttr().Set(points_vt)

    out_scale_attr = get_mesh_local_scale_op(out_mesh_prim)
    out_scale_attr.Set(Gf.Vec3f(1.0, 1.0, 1.0))

    UsdGeom.SetStageMetersPerUnit(out_stage, 1.0)

    ext_lo, ext_hi = aabb_2x
    out_mesh.GetExtentAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(*ext_lo), Gf.Vec3f(*ext_hi)])
    )

    out_stage.GetRootLayer().Save()

    # --- OBJ output: identical scaled point set. ---
    obj_out_path = OBJ_OUT_DIR / f"{part}_2x.obj"
    write_obj(obj_out_path, part, src_rel, points_2x, tris)

    # --- Cross-check: reload both artifacts from disk, compare. ---
    check_stage = Usd.Stage.Open(str(usd_out_path))
    check_mesh = UsdGeom.Mesh(check_stage.GetPrimAtPath(mesh_prim.GetPath()))
    usd_points_readback = [tuple(p) for p in check_mesh.GetPointsAttr().Get()]
    obj_points_readback = read_obj_vertices(obj_out_path)

    points_match = len(usd_points_readback) == len(obj_points_readback) and all(
        _is_close_vec(a, b, tol=1e-5)
        for a, b in zip(usd_points_readback, obj_points_readback)
    )
    usd_aabb_check = aabb(usd_points_readback)
    obj_aabb_check = aabb(obj_points_readback)
    aabb_match = _is_close_vec(usd_aabb_check[0], obj_aabb_check[0], 1e-5) and _is_close_vec(
        usd_aabb_check[1], obj_aabb_check[1], 1e-5
    )
    if not (points_match and aabb_match):
        raise PartFailed("OBJ vertices and saved-USD mesh points diverged after write")

    return PartReport(
        part=part,
        src_usd=src_rel,
        meters_per_unit=mpu,
        n_verts=len(raw_points),
        n_tris=len(tris),
        aabb_1x=size_1x,
        aabb_2x=size_2x,
        ancestors_identity=True,
        points_match=points_match,
        aabb_ratio_ok=ratio_ok,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--parts", nargs="+", default=DEFAULT_PARTS)
    ap.add_argument("--scale", type=float, default=2.0)
    args = ap.parse_args()

    reports: list[PartReport] = []
    failures = 0
    for part in args.parts:
        try:
            reports.append(process_part(part, args.scale))
        except PartFailed as exc:
            failures += 1
            print(f"STOP [{part}]: {exc}", file=sys.stderr)
            reports.append(
                PartReport(
                    part=part,
                    src_usd=f"{REL_SRC_PREFIX}/{part}.usd",
                    meters_per_unit=float("nan"),
                    n_verts=0,
                    n_tris=0,
                    aabb_1x=(0, 0, 0),
                    aabb_2x=(0, 0, 0),
                    ancestors_identity=False,
                    points_match=False,
                    aabb_ratio_ok=False,
                    status="FAILED",
                    detail=str(exc),
                )
            )

    header = (
        f"{'part':<18}{'mpu':<8}{'verts':<7}{'tris':<6}"
        f"{'1x AABB (m)':<26}{'2x AABB (m)':<26}{'ancestors':<11}{'obj==usd':<10}{'status'}"
    )
    print(header)
    print("-" * len(header))
    for r in reports:
        a1 = "x".join(f"{v:.4f}" for v in r.aabb_1x)
        a2 = "x".join(f"{v:.4f}" for v in r.aabb_2x)
        print(
            f"{r.part:<18}{r.meters_per_unit:<8.4f}{r.n_verts:<7}{r.n_tris:<6}"
            f"{a1:<26}{a2:<26}{str(r.ancestors_identity):<11}{str(r.points_match):<10}"
            f"{r.status}{(' - ' + r.detail) if r.detail else ''}"
        )

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
