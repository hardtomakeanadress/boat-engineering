#!/usr/bin/env python3
"""Part audit for the boat STL - the gap/intersection slicer the parts go
through before anyone trusts the model:

- DISCONNECTED: a part whose true surface distance to EVERY other part is
  more than --gap-mm (real millimetres) is floating.  Designed clearances
  are model-scale nudges (0.2 mm = ~5 mm real at 1:24); the risers once sat
  20 mm off the hull and every zoom-in showed it - this tool catches that
  class automatically.
- POKES THROUGH: interior parts (floor, benches, risers, posts) whose
  sampled surface lands INSIDE the hull laminate (the closed shell solid),
  i.e. through the inner skin, in more than 1% of its samples.
- OVERLAPS: pairs of parts whose volumes intersect - tips tuck into rails
  and legs BY DESIGN, so these are REPORTED with names, never a fail.

Distances are exact point-to-triangle (centroid KD-tree candidate
prefetch, nearest-first with early exit); containment is chunked
ray-cast Moeller-Trumbore - numpy + scipy only.

Usage: python3 part_audit.py boat.stl [--gap-mm 5.0] [--json out.json]
Exit code 0 = clean, 1 = disconnected parts / pokes / broken parts.
"""
import argparse
import json
import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_stl as R
import export_stl as E


def mesh_volume(tris):
    v = np.asarray(tris, float)
    return abs(float(np.einsum('ij,ij->i', v[:, 0],
                               np.cross(v[:, 1], v[:, 2])).sum()) / 6.0)


def closest_pt_on_tri(p, a, b, c):
    """Vectorized closest point on triangle (Ericson regions): p, a, b, c
    all (N,3) for ONE triangle's worth of points."""
    ab, ac = b - a, c - a
    d1 = ((p - a) * ab).sum(1)
    d2 = ((p - a) * ac).sum(1)
    d3 = ((p - b) * ab).sum(1)
    d4 = ((p - b) * ac).sum(1)
    d5 = ((p - c) * ab).sum(1)
    d6 = ((p - c) * ac).sum(1)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    den = va + vb + vc
    den = np.where(np.abs(den) < 1e-30, 1e-30, den)
    v = vb / den
    w = vc / den
    out = a + v[:, None] * ab + w[:, None] * ac
    m = (d1 <= 0) & (d2 <= 0)
    out[m] = a[m]
    m = (d3 >= 0) & (d4 <= d3)
    out[m] = b[m]
    m = (d6 >= 0) & (d5 <= d6)
    out[m] = c[m]
    m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
    den2 = d1[m] - d3[m]
    vv = d1[m] / np.where(den2 == 0, 1e-30, den2)
    out[m] = a[m] + vv[:, None] * ab[m]
    m = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
    den2 = (d4 - d3)[m] + (d5 - d6)[m]
    ww = (d4 - d3)[m] / np.where(den2 == 0, 1e-30, den2)
    out[m] = b[m] + ww[:, None] * (c - b)[m]
    m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
    den2 = d2[m] - d6[m]
    vv = d2[m] / np.where(den2 == 0, 1e-30, den2)
    out[m] = a[m] + vv[:, None] * ac[m]
    return out


def surface_gap(pts, tri, k=24, tree=None):
    """Min distance from points to a triangle-set surface: exact
    point-to-triangle over the k nearest triangles per point."""
    if len(tri) == 0:
        return 1e18
    if tree is None:
        tree = cKDTree(tri.mean(axis=1))
    kk = min(k, len(tri))
    _d, idx = tree.query(pts, k=kk)
    idx = np.atleast_2d(idx)
    best = np.full(len(pts), 1e18)
    for col in range(kk):
        t = tri[idx[:, col]]
        q = closest_pt_on_tri(pts, t[:, 0], t[:, 1], t[:, 2])
        best = np.minimum(best, np.linalg.norm(q - pts, axis=1))
    return float(best.min())


def points_inside(pts, tris):
    """Ray-cast containment (Moeller-Trumbore, +z rays), chunked over the
    points so the broadcasts stay fast: odd crossings = inside."""
    t = np.asarray(tris, float)
    v0, e1, e2 = t[:, 0], t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]
    d3 = np.array([0.0, 0.0, 1.0])
    h = np.cross(d3, e2)
    a = np.einsum('ij,ij->i', e1, h)
    ok = np.abs(a) > 1e-12
    f = np.where(ok, 1.0 / np.where(ok, a, 1.0), 0.0)
    d_b = np.broadcast_to(d3, e1.shape)
    out = np.zeros(len(pts), bool)
    for c0 in range(0, len(pts), 64):
        chunk = pts[c0:c0 + 64]
        s = chunk[:, None, :] - v0[None, :, :]          # (P,T,3)
        q = np.cross(s, e1[None, :, :])
        u = np.einsum('ptj,tj->pt', s, h) * f
        v = np.einsum('ptj,tj->pt', q, d_b) * f
        tt = np.einsum('ptj,tj->pt', q, e2) * f
        hit = (ok[None, :] & (u >= 0) & (v >= 0) & (u + v <= 1)
               & (tt > 1e-9))
        out[c0:c0 + len(chunk)] = (hit.sum(axis=1) % 2) == 1
    return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stl")
    ap.add_argument("--gap-mm", type=float, default=5.0,
                    help="real-mm gap above which a part counts as floating")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    if not (math.isfinite(args.gap_mm) and args.gap_mm > 0):
        raise SystemExit("--gap-mm must be a finite positive value")

    tris = R.parse_stl(args.stl)
    groups = R.bodies(tris)
    base = os.path.dirname(os.path.abspath(args.stl))
    hj = os.path.join(base, "hull.json")
    names = [f"body {i}" for i in range(len(groups))]
    scale = 24.0
    if os.path.exists(hj):
        import hull as H
        hh = H.load_hull(hj)
        # overall span: the gunwale caps sit exactly on the end walls, so
        # the hull span = the full span minus those two cap thicknesses
        xspan = float(np.ptp(np.asarray(tris).reshape(-1, 3)[:, 0])
                      - 2.0 * 0.02 * 1000.0 / 24.0)
        if xspan > 1e-9:
            scale = hh["stations"][-1]["x"] * 1000.0 / xspan
        try:
            _, nm = R.classify(tris, groups, hj)
            names = ["hull shell"] + list(nm)
        except Exception:
            pass

    part_tris = [np.asarray(tris[g], float) for g in groups]
    volumes = [mesh_volume(pt) for pt in part_tris]
    n = len(part_tris)
    print(f"part audit: {args.stl}  {n} parts, 1:{scale:.1f}")

    broken = []
    for i, pt in enumerate(part_tris):
        # welds_ok: parts sit with their faces EXACTLY on what they touch,
        # so a body may legitimately contain welded contact edges (4 uses)
        oe, sd = E.edge_report([(a, b, c, None) for a, b, c in pt],
                               welds_ok=True)
        if oe or sd:
            broken.append((i, oe, sd))
    for i, oe, sd in broken:
        print(f"  BROKEN PART: {names[i]} ({oe} open edges, {sd} winding)")

    rng = np.random.default_rng(7)

    def _sample(pt, cap=250):   # enough for face contacts; keeps ~40 s
        nn = min(cap, max(50, len(pt) * 20))
        r1 = rng.random((len(pt), nn, 1))
        r2 = rng.random((len(pt), nn, 1))
        m_ = r1 + r2 > 1
        r1[m_] = 1 - r1[m_]
        r2[m_] = 1 - r2[m_]
        a = pt[:, None, 0, :]
        b = pt[:, None, 1, :]
        c = pt[:, None, 2, :]
        return (a + r1 * (b - a) + r2 * (c - a)).reshape(-1, 3)

    pts = [_sample(pt) if len(pt) else np.zeros((1, 3)) for pt in part_tris]
    trees = [cKDTree(pt.mean(axis=1)) if len(pt) else None for pt in part_tris]
    aabb = [(pt.min(axis=0), pt.max(axis=0)) if len(pt) else
            (np.zeros(3), np.zeros(3)) for pt in part_tris]

    def _aabb_gap(i, j):
        a, b = aabb[i], aabb[j]
        dd = np.maximum(np.maximum(a[0] - b[1], b[0] - a[1]), 0.0)
        return float(np.linalg.norm(dd))

    gaps = np.full(n, 1e18)
    near = ["" for _ in range(n)]
    for i in range(1, n):                 # the hull shell is the reference
        cand = sorted((_aabb_gap(i, j), j) for j in range(n)
                      if j != i and trees[j] is not None)
        for ag, j in cand:
            if ag >= gaps[i]:
                break                     # nearest-first: cannot win anymore
            d = surface_gap(pts[i], part_tris[j], tree=trees[j])
            if d < gaps[i]:
                gaps[i], near[i] = d, names[j]
    gaps[0] = 0.0
    near[0] = "-"
    gap_real_mm = gaps * scale
    floated = [i for i in range(1, n) if gap_real_mm[i] > args.gap_mm]

    hull_tris = part_tris[0]
    # touch band: bonded parts sit with their faces EXACTLY on the hull
    # surface (no gap, no interpenetration), and a point exactly on the
    # surface flips ray parity randomly - only samples clearly INSIDE the
    # laminate count as pokes
    TOUCH_BAND = 0.25           # model mm = 6 mm real: the GLUE_SINK burial
    #   (0.125) plus the measured ceiling of curved-hull artifacts on
    #   narrowing stems (row chords vs plan curvature, <=0.235 on the demo
    #   launch); genuine wall penetrations from the defect era were 0.54+,
    #   so the band still separates them cleanly
    inside_prefixes = ("floor", "bench", "riser", "post")
    pokes = []
    for i in range(1, n):
        if not names[i].startswith(inside_prefixes) or not len(part_tris[i]):
            continue
        # inside the CLOSED shell solid = inside the laminate = a real poke
        # (the cockpit is OUTSIDE that solid, so attached parts read clean);
        # samples within TOUCH_BAND of the surface are touching faces, not
        # pokes - ray parity on a boundary is random
        mask = points_inside(pts[i], hull_tris)
        if mask.mean() <= 0.01:
            continue
        ins = pts[i][mask]
        kk = min(16, len(hull_tris))
        _d, idx = trees[0].query(ins, k=kk)
        dep = np.full(len(ins), 1e18)
        for col in range(kk):
            ht = hull_tris[idx[:, col]]
            q = closest_pt_on_tri(ins, ht[:, 0], ht[:, 1], ht[:, 2])
            dep = np.minimum(dep, np.linalg.norm(q - ins, axis=1))
        frac = float((dep > TOUCH_BAND).sum()) / len(pts[i])
        if frac > 0.01:
            pokes.append((names[i], frac))
        # (an "outside the shell" protrusion check cannot distinguish the
        #  cockpit - which is outside the laminate - from the open water:
        #  the laminate-poke test above is the meaningful defect)

    overlaps = []
    for i in range(1, n):
        for j in range(i + 1, n):
            a, b = aabb[i], aabb[j]
            if np.any(a[1] < b[0]) or np.any(b[1] < a[0]):
                continue
            if not len(part_tris[i]) or not len(part_tris[j]):
                continue
            cnt = int(points_inside(pts[i], part_tris[j]).sum())
            if cnt >= max(8, 0.02 * len(pts[i])):
                vol_cm3 = (cnt / len(pts[i]) * volumes[i]
                           * scale ** 3 / 1000.0)
                overlaps.append((names[i], names[j], round(vol_cm3, 1)))

    print("\nnearest gap per part (real mm):")
    for i in range(n):
        flag = "  <<< DISCONNECTED" if gap_real_mm[i] > args.gap_mm else ""
        print(f"  {names[i]:16s} {gap_real_mm[i]:7.1f}  (nearest: "
              f"{near[i]}){flag}")
    if overlaps:
        print("\noverlapping pairs (designed tucks + anything suspicious):")
        for a, b, v in sorted(overlaps, key=lambda t: -t[2])[:14]:
            print(f"  {a} <-> {b}: ~{v} cm3 real")

    ok = not floated and not pokes and not broken
    if floated:
        print(f"\nFAIL: {len(floated)} disconnected part(s) over "
              f"{args.gap_mm} mm: " + ", ".join(names[i] for i in floated))
    for nm, fr in pokes:
        print(f"FAIL: {nm} sits inside the hull laminate - pokes through "
              f"the wall ({fr:.0%} of its samples)")
    if args.json:
        json.dump({"parts": n, "ok": ok,
                   "gaps_mm": {names[i]: round(float(gap_real_mm[i]), 1)
                               for i in range(n)},
                   "disconnected": [names[i] for i in floated],
                   "pokes": [nm for nm, _ in pokes],
                   "overlaps": overlaps}, open(args.json, "w"), indent=1)
    print("\nVERDICT:", "PASS (every part attached, no hull pokes)"
          if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
