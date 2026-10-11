#!/usr/bin/env python3
"""Export a watertight, scaled STL of the OPEN boat (3D printing / viewing).

Usage:
  python3 export_stl.py hull.json -o boat.stl [--scale 24] [--samples 48]

The model is a hollow hull shell - outer skin, inner skin, gunwale rim and
end walls - with the cross benches inside; the top is open like a real boat
(it used to export the closed decked volume, which read as a "brick"). Wall
thickness is chosen for printing: ~1.2 mm at model scale regardless of the
scale denominator.

--scale 24 means 1:24 (model mm = real mm / 24). Default 1:24. Binary STL in
millimetres at MODEL scale, Z up, outward normals. A watertight + winding
check runs and the export is refused if it fails.

Print notes: 5.2 m at 1:24 -> 217 mm long; split longer hulls in the slicer
(at a bulkhead) if the bed is smaller. Give the hull enough stations
(param_hull --stations 15+) for smooth prints.
"""

import argparse
import math
import struct
import sys
from collections import Counter

import numpy as np

from hull import load_hull
from view3d import (build_mesh, box_quads, y_at_z,
                    hull_with_rails, hull_inner_with_rails)


def _reject_if_collapsed(ring, where):
    """An inward offset that crosses the centreline means the stem is
    narrower than the laminate: the inner hull self-intersects there.
    Reject with an actionable error instead of shipping broken geometry."""
    for (y, z) in ring:
        if y < -1e-6:
            raise ValueError(
                f"hull too narrow at {where}: the inner offset crosses the "
                f"centreline by {-y * 1000:.1f} mm - widen the beam or raise "
                "the bow/transom factor")


def offset_section(pts, t):
    """Offset a station polyline (keel -> side -> deck edge -> CL top)
    INWARD by wall thickness t, in the section plane. The keel point stays
    on the centreline (y=0); the closing edge (CL top -> keel) is taken into
    account for the corner normals. Returns the same point count."""
    loop = [(p[0], p[1]) for p in pts]
    n = len(loop)

    def inward_normal(p, q):
        dy, dz = q[0] - p[0], q[1] - p[1]
        L = math.hypot(dy, dz) or 1.0
        return (-dz / L, dy / L)          # CCW loop -> left of travel = inward

    out = []
    for i in range(n - 1):                # keel .. deck edge (CL top dropped)
        y, z = loop[i]
        if abs(y) < 1e-9:                 # centreline keel: go straight up
            out.append([0.0, round(z + t, 6)])
            continue
        if i == n - 2:                    # deck edge: offset straight inboard,
            out.append([round(y - t, 6), round(z, 6)])   # so the top edge is
            continue                      # VERTICAL with a real thickness edge
        n1 = inward_normal(loop[i - 1], loop[i])
        n2 = inward_normal(loop[i], loop[i + 1])
        ny, nz = n1[0] + n2[0], n1[1] + n2[1]
        L2 = ny * ny + nz * nz
        if L2 < 0.36:               # near-reversal: keep the edge normal
            ny, nz, L2 = n2[0], n2[1], 1.0
        # mitered offset: t along the bisector = (n1+n2) * 2t / |n1+n2|^2;
        # uniform wall t at every edge, corners mitered (limit above)
        out.append([round(y + ny * 2.0 * t / L2, 6),
                    round(z + nz * 2.0 * t / L2, 6)])
    out.append(list(loop[-1]))            # CL top point unchanged
    return out


def _slant_box_quads(c0, c1, sx, sy):
    """Closed box between two centres (canopy leg / brace), horizontal
    cross-section at each end."""
    def rect(c):
        x, y, z = c
        return [(x - sx / 2, y - sy / 2, z), (x + sx / 2, y - sy / 2, z),
                (x + sx / 2, y + sy / 2, z), (x - sx / 2, y + sy / 2, z)]
    b, t = rect(c0), rect(c1)
    return [[b[0], b[1], b[2], b[3]], [t[0], t[3], t[2], t[1]]] + \
           [[b[i], t[i], t[j], b[j]] for i, j in ((0, 1), (1, 2), (2, 3), (3, 0))]


def _tris_from_quads(quads):
    tris = []
    for q in quads:
        tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
    return tris


def _signed_volume(tris):
    vol = 0.0
    for a, b, d in tris:
        a, b, d = (np.asarray(p, float) for p in (a, b, d))
        vol += float(np.dot(a, np.cross(b, d))) / 6.0
    return vol


def _point_in_cavity(inner_sts, x, y, z, tol=0.006):
    """True if (x, y, z) lies inside the inner hull cavity (metres).
    Interpolates the inner section at x and checks the half-width at z."""
    xs = [st["x"] for st in inner_sts]
    if x < xs[0] - tol or x > xs[-1] + tol:
        return False
    i = 0
    while i < len(xs) - 2 and x > xs[i + 1]:
        i += 1
    x0, x1 = xs[i], xs[i + 1]
    t = 0.0 if x1 == x0 else max(0.0, min(1.0, (x - x0) / (x1 - x0)))
    p0, p1 = _pair_profiles(inner_sts[i]["pts"], inner_sts[i + 1]["pts"])
    pts = [(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
           for a, b in zip(p0, p1)]
    zk, ze = pts[0][1], pts[-2][1]
    if z < zk - tol or z > ze + tol:
        return False
    return abs(y) <= y_at_z(pts, min(z, ze)) + tol


def _pair_profiles(p0, p1):
    """Interpolate two station profiles feature-wise.  Raw profiles can
    differ in point count (11 vs 12 at a bow): zip() then truncates and the
    interpolated ring is wrong (the foredeck fit check failed on it).
    Resample both onto a common count first."""
    if len(p0) == len(p1):
        return p0, p1
    import view3d as _v
    n = max(len(p0), len(p1))
    return (_v.resample([(q[0], q[1]) for q in p0], n),
            _v.resample([(q[0], q[1]) for q in p1], n))


def _inner_half(inner_sts, x, z):
    """Half-width of the inner hull at (x, z), stations interpolated."""
    xs = [st["x"] for st in inner_sts]
    x = min(max(x, xs[0]), xs[-1])
    i = 0
    while i < len(xs) - 2 and x > xs[i + 1]:
        i += 1
    x0, x1 = xs[i], xs[i + 1]
    t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
    p0, p1 = _pair_profiles(inner_sts[i]["pts"], inner_sts[i + 1]["pts"])
    pts = [(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
           for a, b in zip(p0, p1)]
    return y_at_z(pts, z)


def _taper_slab(xa, xb, ya, yb, z0, z1):
    """Closed solid slab xa..xb with half-widths ya..yb, z0..z1 - so parts
    follow the hull taper instead of poking through it."""
    t0 = [(xa, -ya, z1), (xa, ya, z1)]
    t1 = [(xb, -yb, z1), (xb, yb, z1)]
    b0 = [(xa, -ya, z0), (xa, ya, z0)]
    b1 = [(xb, -yb, z0), (xb, yb, z0)]
    return [(t0[0], t1[0], t1[1], t0[1]),        # top
            (b0[0], b0[1], b1[1], b1[0]),        # bottom
            (t0[0], b0[0], b1[0], t1[0]),        # -y rim
            (t0[1], t1[1], b1[1], b0[1]),        # +y rim
            (t0[0], t0[1], b0[1], b0[0]),        # cap xa
            (t1[0], b1[0], b1[1], t1[1])]        # cap xb


def _slope_slab(xa, xb, pa, pb, z0, z1):
    """Closed slab between xa and xb whose SIDE faces follow the hull's
    flare: at each end the ring is (inner-bot, outer-bot, outer-top,
    inner-top) in signed y."""
    A = [(xa, pa[0], z0), (xa, pa[1], z0), (xa, pa[2], z1), (xa, pa[3], z1)]
    B = [(xb, pb[0], z0), (xb, pb[1], z0), (xb, pb[2], z1), (xb, pb[3], z1)]
    return [(A[0], A[1], B[1], B[0]),      # bottom
            (A[3], B[3], B[2], A[2]),      # top
            (A[0], B[0], B[3], A[3]),      # -y side (tilted)
            (A[1], A[2], B[2], B[1]),      # +y side (tilted)
            (A[0], A[3], A[2], A[1]),      # xa cap
            (B[0], B[1], B[2], B[3])]      # xb cap


def _wall_rows(inner_sts, x, z0, z1):
    """The INNER hull profile at station x between heights z0..z1, as rows
    (y, z) bottom to top, splitting at every wall kink (the mitered offset
    bends inboard near the sheer).  A straight chord across the kink cuts
    into the laminate - the bench/riser faces must FOLLOW this profile."""
    xs = [st["x"] for st in inner_sts]
    x = min(max(x, xs[0]), xs[-1])
    i = 0
    while i < len(xs) - 2 and x > xs[i + 1]:
        i += 1
    x0, x1 = xs[i], xs[i + 1]
    t = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
    p0, p1 = _pair_profiles(inner_sts[i]["pts"], inner_sts[i + 1]["pts"])
    pts = [(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
           for a, b in zip(p0, p1)]
    side = [(p[0], p[1]) for p in pts[:-1]]      # keel -> sheer, no CL
    rows = []
    for (ya, za), (yb, zb) in zip(side, side[1:]):
        dz = zb - za
        if abs(dz) < 1e-12:
            continue
        for z in (za, zb):
            if z0 - 1e-9 <= z <= z1 + 1e-9:
                rows.append((ya + (yb - ya) * (z - za) / dz, z))
    for z in (z0, z1):
        for (ya, za), (yb, zb) in zip(side, side[1:]):
            dz = zb - za
            if abs(dz) < 1e-12:
                continue
            if min(za, zb) <= z <= max(za, zb):
                rows.append((ya + (yb - ya) * (z - za) / dz, z))
                break
    rows = [r for r in rows if z0 - 1e-9 <= r[1] <= z1 + 1e-9]
    ded = []
    for r in sorted(rows, key=lambda q: q[1]):
        if not ded or abs(r[1] - ded[-1][1]) > 1e-9:
            ded.append(r)
    return ded


def _wall_loft(cols_f, cols_g):
    """Closed slab lofted through MULTIPLE x columns: cols = list of
    (x, rows) with rows = (y, z) signed along the wall.  Endpoint-only
    slabs chord across the plan curvature between stations (up to 15 mm
    into the wall on curved hulls, measured by the codex audit); this
    passes through every crossed station instead."""
    zs = sorted({round(z, 9) for cols in (cols_f, cols_g)
                 for (_x, rows) in cols for (_y, z) in rows})

    def regrid(rows):
        out = []
        for z in zs:
            if z <= rows[0][1]:
                out.append((rows[0][0], z))
            elif z >= rows[-1][1]:
                out.append((rows[-1][0], z))
            else:
                for (ya, za), (yb, zb) in zip(rows, rows[1:]):
                    if za - 1e-12 <= z <= zb + 1e-12:
                        t = (z - za) / (zb - za) if zb != za else 0.0
                        out.append((ya + t * (yb - ya), z))
                        break
        return out

    F = [(x, regrid(rows)) for (x, rows) in cols_f]
    G = [(x, regrid(rows)) for (x, rows) in cols_g]
    nrow = len(zs)
    qs = []
    for (x0, r0), (x1, r1) in zip(F, F[1:]):        # face F
        for j in range(nrow - 1):
            qs.append([(x0, r0[j][0], r0[j][1]), (x1, r1[j][0], r1[j][1]),
                       (x1, r1[j + 1][0], r1[j + 1][1]),
                       (x0, r0[j + 1][0], r0[j + 1][1])])
    for (x0, r0), (x1, r1) in zip(G, G[1:]):        # face G (opposite)
        for j in range(nrow - 1):
            qs.append([(x0, r0[j][0], r0[j][1]),
                       (x0, r0[j + 1][0], r0[j + 1][1]),
                       (x1, r1[j + 1][0], r1[j + 1][1]),
                       (x1, r1[j][0], r1[j][1])])
    x0, r0 = F[0]
    x1, r1 = F[-1]
    s0, s1 = G[0][1], G[-1][1]
    for j in range(nrow - 1):                        # x-end cap ribbons
        qs.append([(x0, r0[j][0], r0[j][1]),
                   (x0, r0[j + 1][0], r0[j + 1][1]),
                   (x0, s0[j + 1][0], s0[j + 1][1]),
                   (x0, s0[j][0], s0[j][1])])
        qs.append([(x1, r1[j][0], r1[j][1]), (x1, s1[j][0], s1[j][1]),
                   (x1, s1[j + 1][0], s1[j + 1][1]),
                   (x1, r1[j + 1][0], r1[j + 1][1])])
    for k in range(len(F) - 1):                      # bottom + top faces
        xa, ra = F[k]
        xb, rb = F[k + 1]
        sa, sb = G[k][1], G[k + 1][1]
        for j in (0, nrow - 1):
            # opposite faces of the slab need opposite windings
            if j == 0:
                qs.append([(xa, ra[0][0], ra[0][1]), (xa, sa[0][0], sa[0][1]),
                           (xb, sb[0][0], sb[0][1]), (xb, rb[0][0], rb[0][1])])
            else:
                qs.append([(xa, ra[-1][0], ra[-1][1]),
                           (xb, rb[-1][0], rb[-1][1]),
                           (xb, sb[-1][0], sb[-1][1]),
                           (xa, sa[-1][0], sa[-1][1])])
    return qs


def _deck_quads(h_in, i0, i1, thick):
    """Closed tapered end-deck plate between stations i0..i1, sitting just
    under the inner deck edge, buried GLUE_SINK into the inner hull. Own
    closed solid - caller righted by volume."""
    import designer as _des
    top, bot = [], []
    for i in range(i0, i1 + 1):
        pts = h_in["stations"][i]["pts"]
        z_t = pts[-2][1] - 0.001      # flush at the sheer: ends read CLOSED
        y_w = max(0.004, y_at_z(pts, z_t - thick) + _des.GLUE_SINK)
        #     the deck NARROWS with a pointy stem (a 20 mm floor overhung it)
        x = h_in["stations"][i]["x"]
        top.append([(x, -y_w, z_t), (x, y_w, z_t)])
        bot.append([(x, -y_w, z_t - thick), (x, y_w, z_t - thick)])
    quads = []
    for i in range(len(top) - 1):
        t0, t1, b0, b1 = top[i], top[i + 1], bot[i], bot[i + 1]
        quads.append((t0[0], t1[0], t1[1], t0[1]))              # top (up)
        quads.append((b0[0], b0[1], b1[1], b1[0]))              # bottom (down)
        quads.append((t0[0], b0[0], b1[0], t1[0]))              # -y rim
        quads.append((t0[1], t1[1], b1[1], b0[1]))              # +y rim
    quads.append((top[0][0], top[0][1], bot[0][1], bot[0][0]))  # end caps
    quads.append((top[-1][0], bot[-1][0], bot[-1][1], top[-1][1]))
    return quads


def hull_triangles(h, n_per_half=48, wall=0.025, ply=0.006, roof_n=0,
                   seat_h=None, roof_h=None, people=None):
    """Hollow open-boat shell with consistent outward winding (unit: m).
    Outer skin + reversed inner skin + gunwale rim + end walls close into a
    watertight solid of wall thickness `wall`; benches are added as closed
    boxes sized to the inner cavity. Degenerates dropped, each box and the
    shell checked/righted by signed volume."""
    import designer as _des
    # the gunwale rails are FUSED into the hull: the outer profile carries
    # the outwale band, the inner profile the inwale band - one continuous
    # surface, so there are no contact faces, no coplanar pairs, no slits
    # and the cap reads as a closed butt joint
    Vo, fo = build_mesh(hull_with_rails(h), n_per_half=n_per_half,
                        open_top=True)
    h_in = hull_inner_with_rails(h, wall)
    Vi, fi = build_mesh(h_in, n_per_half=n_per_half, open_top=True)
    ns, n = Vo.shape[0], Vo.shape[1]
    quads = []
    cap_tris = []
    quads += [tuple(q) for q in fo[:-2]]                     # outer skin
    quads += [(q[0], q[3], q[2], q[1]) for q in fi[:-2]]     # inner skin
    for i in range(ns - 1):                                  # gunwale rim
        quads.append((Vi[i, 0], Vo[i, 0], Vo[i + 1, 0], Vi[i + 1, 0]))
        quads.append((Vi[i + 1, n - 1], Vo[i + 1, n - 1],
                      Vo[i, n - 1], Vi[i, n - 1]))
    for i, xdir in ((0, 1.0), (ns - 1, -1.0)):               # transom + bow walls
        for j in range(n - 1):
            q = (Vo[i, j], Vi[i, j], Vi[i, j + 1], Vo[i, j + 1])
            quads.append(q if i == 0 else (q[0], q[3], q[2], q[1]))
        # SOLID END PLATE: the outer section extruded `wall` inboard - a
        # closed slab reaching the sheer.  Without it the hull is an open
        # trough: everything below the deck was see-through end to end.
        x0 = float(Vo[i, 0, 0])
        # nudged 0.2 mm off the shell surface so the slab is an INDEPENDENT
        # closed solid - sharing vertices with the shell would fuse edges.
        # The gunwale band is FUSED into the plate: the outer face juts out
        # by GUNW_T over the band (the cap, closing over the merged side
        # rails like a butt joint) and the inner face juts in by GUNW_T
        # (the inboard board) - no separate end boards, no steps
        _zs = float(np.max(Vo[i, :, 2]))
        _zb = _zs - _des.GUNW_H
        A, B = [], []
        out = -xdir          # OUTWARD = away from the hull at this end
        for j in range(n):
            band = Vo[i, j, 2] >= _zb - 1e-9
            # the plate's outer face sits 0.2 mm real OUTSIDE the end ring
            # (an independent solid, sub-print-layer proud so it fuses) and
            # juts out by GUNW_T over the rail band. Inboard-nudging it used
            # to leave the hull's end-ring edges proud of the plate face -
            # they rendered as ragged patches THROUGH the plate in viewers.
            ax = x0 + out * (0.0002 + (_des.GUNW_T if band else 0.0))
            A.append(np.array([ax, Vo[i, j, 1], Vo[i, j, 2]], float))
            # the inboard face comes off x0 INDEPENDENTLY: deriving it from
            # `ax` cancelled the band and the inboard board vanished
            B.append(np.array([x0 + xdir * (wall
                                            + (_des.GUNW_T if band else 0.0)),
                               Vo[i, j, 1], Vo[i, j, 2]], float))
        pt = []
        # triangulate the plate faces as a structured STRIP between mirror
        # ring pairs - a fan from a centre folds into bow-ties at the
        # notched rail corners ("two triangles" flashing on the end walls)
        half = n // 2
        for j in range(half):
            j2 = j + 1
            k, k2 = n - 1 - j, n - 2 - j
            if j2 == k2:                 # the keel pair: one triangle each
                pt.append((A[j], A[j2], A[k]))
                pt.append((B[j], B[k], B[j2]))
                continue
            pt.append((A[j], A[j2], A[k2]))
            pt.append((A[j], A[k2], A[k]))
            pt.append((B[j], B[k2], B[j2]))
            pt.append((B[j], B[k], B[k2]))
        for j in range(n):                           # side ribbon: the FULL
            j2 = (j + 1) % n                         # ring, not just a half
            tb = (A[j], B[j], B[j2], A[j2])
            pt.append((tb[0], tb[1], tb[2]))
            pt.append((tb[0], tb[2], tb[3]))
        if _signed_volume(pt) < 0:                   # right each plate
            pt = [(a, d, b) for (a, b, d) in pt]
        cap_tris += pt
    tris = _tris_from_quads(quads) + cap_tris
    if _signed_volume(tris) < 0:
        tris = [(a, d, b) for (a, b, d) in tris]

    # benches + floor: closed boxes clamped into the cavity, righted by volume
    # panel material: the plywood thickness, but never below the printable
    # minimum at model scale
    mat = max(ply, wall)
    problems = []
    try:
        import designer as _des
        lay = _des.layout(h, {"roof_n": roof_n,
                              **({"seat_h": seat_h} if seat_h else {}),
                              **({"roof_h": roof_h} if roof_h else {}),
                              **({"people": people} if people else {})})

        def _check(quads, what):
            for q in quads:
                for (px, py, pz) in q:
                    if not _point_in_cavity(h_in["stations"], px, py, pz):
                        problems.append(f"{what} pokes through the hull at "
                                        f"({px:.2f}, {py:.2f}, {pz:.2f})")
                        return

        def _check_span(xa_, xb_, ya_, yb_, zlvl, what):
            """Tapered slabs are straight between their ends: check them at
            every INTERMEDIATE station too, not just the slab corners."""
            for st in h_in["stations"]:
                x = st["x"]
                if xa_ + 1e-9 < x < xb_ - 1e-9:
                    tt = (x - xa_) / (xb_ - xa_)
                    w_slab = ya_ + tt * (yb_ - ya_)
                    if w_slab > _inner_half(h_in["stations"], x, zlvl) + 5e-4:
                        problems.append(
                            f"{what} pokes through at station x={x:.2f} "
                            f"(slab {w_slab:.3f} > inner "
                            f"{_inner_half(h_in['stations'], x, zlvl):.3f})")
                        return

        def _check_span2(xa_, xb_, ya_b_, yb_b_, ya_t_, yb_t_, z0_, z1_, what):
            """Bevel-following slabs: both the bottom and top widths are
            straight between the ends - check each level separately."""
            _check_span(xa_, xb_, ya_b_, yb_b_, z0_, f"{what} bottom")
            _check_span(xa_, xb_, ya_t_, yb_t_, z1_, f"{what} top")

        z_sole, z_seat = lay["z_sole"], lay["z_seat"]
        boxes = []

        def _rowsV(x, z0, z1):
            """Wall rows (y, z) taken from Vi - the SAMPLED inner rings the
            printed inner skin is actually built from.  Measuring against
            the raw station ring drifts up to 5 mm at the mitered kink
            (resampling cuts it), which read as 'bench inside the wall'."""
            xsv = [st["x"] for st in h_in["stations"]]
            x = min(max(x, xsv[0]), xsv[-1])
            i = 0
            while i < len(xsv) - 2 and x > xsv[i + 1]:
                i += 1
            t = ((x - xsv[i]) / (xsv[i + 1] - xsv[i])
                 if xsv[i + 1] != xsv[i] else 0.0)
            ring = Vi[i] * (1 - t) + Vi[i + 1] * t
            sub = [p for p in ring if p[1] >= -1e-9]      # one side, ring order
            if len(sub) < 2:
                return []
            pl = [(p[1], p[2]) for p in sub] + [(0.0, sub[-1][2])]
            return _wall_rows([{"x": x - 1.0, "pts": pl},
                               {"x": x, "pts": pl}], x, z0, z1)

        # no sole plate: the hull's inner bottom IS the floor (asked for)
        for bi, bn in enumerate(lay["benches"], 1):
            xa = bn["xc"] - bn["w"] / 2.0
            xb = bn["xc"] + bn["w"] / 2.0
            z_bot = z_seat - mat
            z_top = z_seat                # the visible edge: flush at the top
            # the ends are BEVELLED: each corner follows the hull side at
            # both faces and EVERY crossed station follow the inner wall
            # profile: endpoint-only slabs chord across the plan curvature
            # (up to 15 mm into the wall - the codex audit measured it)
            def _colsV(xa_, xb_, z0, z1):
                xin = [sx["x"] for sx in h_in["stations"]]
                xs_use = ([xa_]
                          + [x for x in xin if xa_ + 1e-9 < x < xb_ - 1e-9]
                          + [xb_])
                return [(x, [(y + _des.GLUE_SINK, z)
                             for (y, z) in _rowsV(x, z0, z1)])
                        for x in xs_use]
            bench_cols = _colsV(xa, xb, z_bot, z_top)
            bq = _wall_loft(bench_cols,
                            [(x, [(-y, z) for (y, z) in rows])
                             for (x, rows) in bench_cols])
            _check(bq, f"bench {bi}")
            boxes.append(bq)
            for sgn in (1, -1):      # risers: cleats hugging the wall rows
                rxa = xa - _des.RISER_EXT
                rxb = xb + _des.RISER_EXT
                rt_top = z_bot + _des.GLUE_SINK   # riser TOP buried into
                rb_bot = rt_top - _des.RISER_T    # the bench underside
                base = _colsV(rxa, rxb, rb_bot, rt_top)
                fa = [(x, [(sgn * y, z) for (y, z) in rows])
                      for (x, rows) in base]
                ga = [(x, [(sgn * (y - _des.RISER_T), z)
                           for (y, z) in rows]) for (x, rows) in base]
                rq = _wall_loft(fa, ga)
                _check(rq, f"bench {bi} riser")
                boxes.append(rq)
            if 2 * bench_cols[0][1][-1][0] > _des.POST_SPAN:
                # centre post foot on the INNER bottom under its own
                # footprint: sample the inner-keel curve at both ends of
                # the footprint and the middle, take the highest (rocker
                # rises toward the bow; z_sole is a GLOBAL floor and can
                # sit below the local bottom - that poked through)
                def _keel_z(xq):
                    sts_i = h_in["stations"]
                    for i in range(len(sts_i) - 1):
                        a, bb2 = sts_i[i], sts_i[i + 1]
                        if a["x"] <= xq <= bb2["x"]:
                            t = ((xq - a["x"]) / (bb2["x"] - a["x"])
                                 if bb2["x"] != a["x"] else 0.0)
                            return (a["pts"][0][1]
                                    + t * (bb2["pts"][0][1] - a["pts"][0][1]))
                    return sts_i[-1]["pts"][0][1]
                zb2 = max(_keel_z(bn["xc"] - _des.POST_SZ),
                          _keel_z(bn["xc"]),
                          _keel_z(bn["xc"] + _des.POST_SZ))
                # foot sits ON the inner bottom: a GLUE_SINK burial would
                # dip under the inner skin (laminate IS the hull part) and
                # read as a poke; 0.2 mm real proud touches at print scale
                zb2 += 0.0002
                if z_bot - zb2 > 0.05:      # post runs to the bench underside
                    pq = box_quads(bn["xc"], 0.0,
                                   (zb2 + z_bot) / 2.0,
                                   _des.POST_SZ, _des.POST_SZ,
                                   z_bot - zb2)
                    _check(pq, f"bench {bi} post")
                    boxes.append(pq)
        for bq in boxes:
            bt = _tris_from_quads(bq)
            if _signed_volume(bt) < 0:
                bt = [(a, d, b) for (a, b, d) in bt]
            tris += bt
        # foredeck only: the stern stays OPEN for the outboard (the transom
        # plate still closes the hull; no cover over the motor clamp area)
        xs_st = [st["x"] for st in h_in["stations"]]
        for nm, (xa, xb) in (("foredeck", lay["decks"]["fore"]),):
            idx = [i for i, x in enumerate(xs_st)
                   if xa - 1e-9 <= x <= xb + 1e-9]
            if len(idx) < 2:
                continue
            dq = _deck_quads(h_in, idx[0], idx[-1], mat)
            _check(dq, nm)
            dt = _tris_from_quads(dq)
            if _signed_volume(dt) < 0:
                dt = [(a, d, b) for (a, b, d) in dt]
            tris += dt
        # solar canopy - the same members designer.canopy_struct dimensions
        #: frame tubes, slanted legs, bolted feet, clamps and the framed
        #  450 W modules - so the STL shows the boat WITH its solar roof
        # (construction errors below are re-raised as ValueError: the outer
        # handler keeps benches/decks optional, a requested ROOF is not)
        r = lay.get("roof")
        if r:
          try:
            cs = _des.canopy_struct(h, r)
            cq = []
            for pn in cs["panels"]:           # 450 W module slabs
                cq.append(box_quads(pn["x0"] + cs["pan_x"] / 2.0,
                                    pn["y0"] + cs["pan_y"] / 2.0,
                                    cs["z_pan0"] + cs["pan_t"] / 2.0,
                                    cs["pan_x"], cs["pan_y"], cs["pan_t"]))
            for rl in cs["rails"]:            # perimeter frame, tube boxes
                cq.append(box_quads((rl["x0"] + rl["x1"]) / 2.0, rl["y"],
                                    (rl["z0"] + rl["z1"]) / 2.0,
                                    rl["x1"] - rl["x0"], rl["sy"], rl["sz"]))
            for en in cs["ends"]:   # 1 mm short of the side rails, else the
                cq.append(box_quads(  # shared corner edge fuses the solids
                    en["x"], (en["y0"] + en["y1"]) / 2.0 - 0.0,
                    (en["z0"] + en["z1"]) / 2.0,
                    en["sx"], en["y1"] - en["y0"] - 0.002,
                    en["z1"] - en["z0"]))
            for cr in cs["cross"]:            # crossbars under panel edges
                cq.append(box_quads(cr["x"] + cr["sx"] / 2.0,
                                    (cr["y0"] + cr["y1"]) / 2.0,
                                    (cr["z0"] + cr["z1"]) / 2.0,
                                    cr["sx"], cr["y1"] - cr["y0"], cr["sz"]))
            for xl in cs["xlegs"]:            # X supports (two per side)
                cq.append(_slant_box_quads((xl["x0"], xl["y0"], xl["z0"]),
                                           (xl["x1"], xl["y1"], xl["z1"]),
                                           xl["sx"], xl["sy"]))
            for ft in cs["feet"]:             # foot plate sitting on the sheer
                cq.append(box_quads(ft["x"], ft["y"], ft["z"] + ft["sz"] / 2.0,
                                    ft["sx"], ft["sy"], ft["sz"]))
            for cl in cs["clamps"]:           # clamps bridging the air gap
                cq.append(box_quads(cl["x"], cl["y"],
                                    (cl["z0"] + cl["z1"]) / 2.0,
                                    cl["sx"], cl["sy"], cl["z1"] - cl["z0"]))
            for q in cq:
                ct = _tris_from_quads(q)
                if _signed_volume(ct) < 0:
                    ct = [(a, d, b) for (a, b, d) in ct]
                tris += ct
          except Exception as e:
              raise ValueError(f"canopy build failed (roof_n={roof_n}): "
                               f"{type(e).__name__}: {e}")
    except ValueError:
        raise
    except Exception as exc:   # benches are a bonus; the shell alone is a
        import sys, traceback  # valid model - but NEVER silently: a swallowed
        print(f"WARNING: interior fit failed: {exc!r}", file=sys.stderr)
        traceback.print_exc()
    if problems:
        raise ValueError("part fit check failed: " + "; ".join(problems[:4]))
    # the gunwale (side rails AND the transom/bow caps + inboard boards) is
    # FUSED into the hull profile and the end plates - one continuous
    # surface, no separate boards, no contact faces, no steps or slits

    out = []
    for t in tris:
        a, b, d = (np.asarray(p, float) for p in t)
        nrm = np.cross(b - a, d - a)
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            # keep zero-area faces: they carry the mesh topology (dropping
            # them opens holes); slicers ignore them
            out.append((a, b, d, np.array([0.0, 0.0, 1.0])))
            continue
        out.append((a, b, d, nrm / ln))
    vol = 0.0
    for a, b, d, _n in out:
        vol += float(np.dot(a, np.cross(b, d))) / 6.0
    if vol < 0:  # consistent but inverted: flip everything once
        out = [(a, d, b, -n) for (a, b, d, n) in out]
    return out


def edge_report(tris, welds_ok=False):
    """(open_edges, same_direction_pairs): (0, 0) means watertight with a
    consistent winding - every edge traversed once in each direction.
    welds_ok=True allows WELDED CONTACTS: parts sit with their faces
    EXACTLY on the surface they touch (no gap, no interpenetration), so
    where the two triangulations coincide an edge carries 4 uses (two per
    direction) - that is two face-to-face closed solids, not a broken
    mesh.  Real holes (1 or 3 uses) still fail."""
    und, dirn = Counter(), Counter()
    for a, b, d, _n in tris:
        for p, q in ((a, b), (b, d), (d, a)):
            kp, kq = tuple(np.round(p, 6)), tuple(np.round(q, 6))
            und[tuple(sorted((kp, kq)))] += 1
            dirn[(kp, kq)] += 1

    def _welded(ekey):
        return (und[ekey] == 4 and dirn[ekey] == 2
                and dirn[(ekey[1], ekey[0])] == 2)
    open_edges = sum(1 for ekey, v in und.items()
                     if v != 2 and not (welds_ok and _welded(ekey)))
    same_dir = sum(1 for (kp, kq), v in dirn.items()
                   if v > 1 and not (welds_ok
                                     and _welded(tuple(sorted((kp, kq))))))
    return open_edges, same_dir


def write_stl(path, tris, scale):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(tris)))
        for a, b, d, n in tris:
            f.write(struct.pack('<3f', *n))
            for p in (a, b, d):
                f.write(struct.pack('<3f', *(p * 1000.0 / scale)))
            f.write(struct.pack('<H', 0))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("-o", "--out", default=None, help="output .stl")
    ap.add_argument("--scale", type=float, default=24.0,
                    help="scale denominator: 24 = 1:24 (default)")
    ap.add_argument("--samples", type=int, default=48,
                    help="points per half-section ring (print smoothness)")
    ap.add_argument("--ply-mm", type=int, default=6,
                    help="plywood panel thickness [mm] (real boat; the "
                         "model keeps 1.2 mm at model scale as the print "
                         "minimum)")
    ap.add_argument("--roof-n", type=int, default=0,
                    help="number of 450 W solar panels in the canopy "
                         "(0 = no roof; the STL then includes frame, X supports "
                         "and panels)")
    ap.add_argument("--seat-h", type=float, default=None,
                    help="bench height above the floor [m] (page seat_h)")
    ap.add_argument("--people", type=int, default=None,
                    help="people aboard: the bench count follows it")
    ap.add_argument("--roof-h", type=float, default=None,
                    help="roof clear height over the deck edge [m] (page roof_h)")
    args = ap.parse_args()

    if not (math.isfinite(args.scale) and args.scale > 0):
        sys.exit("ERROR: --scale must be a positive finite number")
    args.samples = int(max(8, min(200, args.samples)))
    h = load_hull(args.hull)
    wall = 1.2 * args.scale / 1000.0        # ~1.2 mm wall at model scale
    ply = int(max(3, min(18, args.ply_mm))) / 1000.0
    tris = hull_triangles(h, n_per_half=args.samples, wall=wall, ply=ply,
                          roof_n=max(0, min(8, args.roof_n)),
                          seat_h=args.seat_h, roof_h=args.roof_h,
                          people=args.people)
    # drop exactly-zero-area triangles (horizontal runs and closure ribbons
    # collapse some mirror-strip quads to lines); if that would open the
    # mesh, keep them - a print tolerates a degenerate, not a hole
    _kept = [t for t in tris
             if float(np.linalg.norm(np.cross(np.subtract(t[1], t[0]),
                                              np.subtract(t[2], t[0]))))
             > 1e-12]
    _oe_f, _sd_f = edge_report(_kept, welds_ok=True)
    if _oe_f == 0 and _sd_f == 0:
        tris = _kept
    else:
        print(f"note: kept {len(tris) - len(_kept)} degenerate triangles "
              "(dropping them would open the mesh)", file=__import__("sys").stderr)
    open_edges, same_dir = edge_report(tris, welds_ok=True)
    if open_edges or same_dir:
        print(f"ERROR: mesh failed validation ({open_edges} open edges, "
              f"{same_dir} inconsistent-winding edges) - refusing to write a "
              "broken STL; check the offsets")
        sys.exit(1)
    verts = np.vstack([[t[0], t[1], t[2]] for t in tris])
    size_mm = (verts.max(axis=0) - verts.min(axis=0)) * 1000.0 / args.scale
    out = args.out or (h.get("name", "hull").replace(" ", "_")
                       + f"_1-{args.scale:g}.stl")
    write_stl(out, tris, args.scale)
    print(f"wrote {out}: {len(tris)} triangles, watertight with consistent "
          f"winding (hollow shell, wall {wall * 1000 / args.scale:.1f} mm "
          "at model scale, open top, benches included)")
    print(f"model size (mm): {size_mm[0]:.1f} x {size_mm[1]:.1f} x {size_mm[2]:.1f}"
          f"  (scale 1:{args.scale:g})")


if __name__ == "__main__":
    main()
