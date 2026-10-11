#!/usr/bin/env python3
"""3D hull views from an offsets table (visualization, not a dimensioned sheet).

Usage:
  python3 view3d.py hull.json -o hull3d.png [--draft 0.25] [--title "..."]

Renders the open boat - hull sides and bottom, floor, cross benches (thwarts,
aft one = helm), and the solar canopy on its end supports - shaded from six
viewpoints, plus an optional translucent waterline plane at the design
draft. (STL export uses the closed watertight mesh instead.)
Sections are arc-length resampled before meshing, so input point counts may
differ between stations. Requires numpy + matplotlib.

For the dimensioned drawing use linesplan.py + engineering-drafting instead;
this script answers "what does the shape actually look like from every side".
"""

import argparse
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from hull import load_hull

VIEWS = [
    ("bow quarter", 22, 55),
    ("stern quarter", 22, 135),
    ("side (starboard)", 6, 90),
    ("top", 88, 90),
    ("bow on", 8, 0),
    ("stern on", 8, 180),
]


def resample(pts, n):
    """Resample a polyline [(y, z), ...] into n points, preserving every
    original vertex (chines stay sharp); extra points are spaced by length."""
    n = max(n, len(pts))
    d = [0.0]
    for i in range(1, len(pts)):
        d.append(d[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                    pts[i][1] - pts[i - 1][1]))
    total = d[-1]
    if total <= 0:
        return [pts[0]] * n
    segs = len(pts) - 1
    m = [max(1, round((d[i + 1] - d[i]) * (n - 1) / total)) for i in range(segs)]
    while sum(m) > n - 1:
        j = max(range(segs), key=lambda i: m[i] if m[i] > 1 else -1)
        if m[j] <= 1:
            break
        m[j] -= 1
    while sum(m) < n - 1:
        j = max(range(segs), key=lambda i: d[i + 1] - d[i])
        m[j] += 1
    out = [pts[0]]
    for i in range(segs):
        for k in range(1, m[i] + 1):
            t = k / m[i]
            out.append((pts[i][0] + t * (pts[i + 1][0] - pts[i][0]),
                        pts[i][1] + t * (pts[i + 1][1] - pts[i][1])))
    return out


def build_mesh(h, n_per_half=28, open_top=False):
    """Vertex grid + quads. V[i, j] = (x, y, z) around station i.
    All stations use the SAME per-segment sample allocation, so matching
    features (keel, chines, deck edge) stay at matching ring indices.
    open_top=True renders an open boat shell (no deck seal) for viewing;
    the default closes the deck (watertight, for STL export)."""
    sts = h["stations"]
    bases = [st["pts"][:-1] if open_top else st["pts"] for st in sts]
    counts = {len(b) for b in bases}
    rings = []
    if len(counts) == 1:
        npts = counts.pop()
        n_t = max(n_per_half, npts)
        segs = npts - 1
        lengths = [0.0] * segs
        for b in bases:
            for i in range(segs):
                lengths[i] += math.hypot(b[i + 1][0] - b[i][0],
                                         b[i + 1][1] - b[i][1])
        lengths = [v / len(bases) for v in lengths]
        total = sum(lengths) or 1.0
        m = [max(1, round(v * (n_t - 1) / total)) for v in lengths]
        while sum(m) > n_t - 1:
            j = max(range(segs), key=lambda i: m[i] if m[i] > 1 else -1)
            if m[j] <= 1:
                break
            m[j] -= 1
        while sum(m) < n_t - 1:
            j = max(range(segs), key=lambda i: lengths[i])
            m[j] += 1

        def sample(p):
            out = [p[0]]
            for i in range(segs):
                for k in range(1, m[i] + 1):
                    t = k / m[i]
                    out.append((p[i][0] + t * (p[i + 1][0] - p[i][0]),
                                p[i][1] + t * (p[i + 1][1] - p[i][1])))
            return out

        for b in bases:
            pts_r = sample(b)
            if open_top:
                left = [(-y, z) for (y, z) in reversed(pts_r)]
                rings.append(left[:-1] + list(pts_r))
            else:
                ring = list(pts_r) + [(-y, z) for (y, z) in reversed(pts_r[1:-1])]
                rings.append(ring)
    else:  # differing point counts: fall back to per-station resampling
        n_t = max(n_per_half, max(len(b) for b in bases))
        for b in bases:
            pts_r = resample(b, n_t)
            if open_top:
                left = [(-y, z) for (y, z) in reversed(pts_r)]
                rings.append(left[:-1] + list(pts_r))
            else:
                rings.append(list(pts_r)
                             + [(-y, z) for (y, z) in reversed(pts_r[1:-1])])
    n = len(rings[0])
    V = np.zeros((len(sts), n, 3))
    for i, (st, ring) in enumerate(zip(sts, rings)):
        for j, (y, z) in enumerate(ring):
            V[i, j] = (st["x"], y, z)
    quads = []
    jmax = n if not open_top else n - 1
    for i in range(len(sts) - 1):
        for j in range(jmax):
            j2 = (j + 1) % n
            quads.append([V[i, j], V[i, j2], V[i + 1, j2], V[i + 1, j]])
    quads.append([V[0, j] for j in range(n)])      # transom
    quads.append([V[-1, j] for j in range(n)])     # bow
    return V, quads


def y_at_z(pts, z):
    """Hull half-width at height z, interpolated along the side polyline
    (pts run keel -> deck edge -> CL; the CL point is ignored)."""
    side = [(p[0], p[1]) for p in pts[:-1]]
    if z <= side[0][1]:
        return side[0][0]
    for (y0, z0), (y1, z1) in zip(side, side[1:]):
        if z <= z1:
            if z1 - z0 <= 1e-12:
                return max(y0, y1)
            t = (z - z0) / (z1 - z0)
            return y0 + t * (y1 - y0)
    return side[-1][0]


def rail_band_pts(side, t, hh, sign):
    """Augment a station's side polyline (keel -> deck edge, y >= 0) with
    the gunwale band as PART OF THE PROFILE: below z_sheer-hh unchanged;
    then a jog out/in of t; then the board face up to the sheer.  Merging
    the rail into the shell makes it one continuous surface - no separate
    boards, no contact faces, no coplanar pairs, no hairline slits, and
    the cap reads as a closed butt joint."""
    y_s, z_s = side[-1]
    z_step = z_s - hh

    def y_at(z):
        for (y0, z0), (y1, z1) in zip(side, side[1:]):
            if z0 - 1e-12 <= z <= z1 + 1e-12:
                if z1 - z0 <= 1e-12:
                    continue
                f = (z - z0) / (z1 - z0)
                return y0 + f * (y1 - y0)
        return side[0][0]
    out = [(y, z) for (y, z) in side if z < z_step - 1e-12]
    y_w = y_at(z_step)
    # a pointy stem has no room for the full board: the band TAPERS to the
    # stem's own width and closes there (both rails follow the same angle)
    tt = max(0.0, min(t, y_w - 0.005))
    out.append((y_w, z_step))               # wall at the band's bottom
    out.append((y_w + sign * tt, z_step))   # jog: board's bottom edge
    out.append((y_s + sign * tt, z_s))      # board face to the sheer
    return out


def hull_with_rails(h, t=None, hh=None):
    """The hull dict with the outwale band merged into every station's
    profile (the 3D view and the STL then build the rail as part of the
    hull body)."""
    if t is None or hh is None:
        import designer as _d
        t = _d.GUNW_T if t is None else t
        hh = _d.GUNW_H if hh is None else hh
    sts = []
    for st in h["stations"]:
        pts = st["pts"]
        side = [(p[0], p[1]) for p in pts[:-1]]     # keel -> sheer
        cl = pts[-1]
        aug = rail_band_pts(side, t, hh, +1.0)
        sts.append({"x": st["x"],
                    "pts": aug + [(0.0, cl[1])]})   # CL sentinel back
    out = dict(h)
    out["stations"] = sts
    return out


def hull_inner_with_rails(h, wall, t=None, hh=None):
    """h_in equivalent with the INWALE band merged: offset the raw rings
    inward by wall as before, then add the inwale band on the inner face."""
    if t is None or hh is None:
        import designer as _d
        t = _d.GUNW_T if t is None else t
        hh = _d.GUNW_H if hh is None else hh
    import export_stl as _E
    sts = []
    for st in h["stations"]:
        off = _E.offset_section(st["pts"], wall)
        _E._reject_if_collapsed([(q[0], q[1]) for q in off],
                                f"station x={st['x']:.2f} m")
        side = [(p[0], p[1]) for p in off[:-1]]
        cl = off[-1]
        aug = rail_band_pts(side, t, hh, -1.0)
        sts.append({"x": st["x"],
                    "pts": aug + [(0.0, cl[1])]})
    out = dict(h)
    out["stations"] = sts
    return out


def box_quads(cx, cy, cz, sx, sy, sz):
    """Axis-aligned box as 6 quads (centre + size)."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    return [[c[i] for i in q] for q in
            ((0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0))]


def prism_quads(x, y0, z0, y1, z1, sx, sy):
    """Slanted prism (canopy leg): rectangle at (x, y0, z0) up to (x, y1, z1)."""
    b = [(x - sx / 2, y0 - sy / 2, z0), (x + sx / 2, y0 - sy / 2, z0),
         (x + sx / 2, y0 + sy / 2, z0), (x - sx / 2, y0 + sy / 2, z0)]
    t = [(x - sx / 2, y1 - sy / 2, z1), (x + sx / 2, y1 - sy / 2, z1),
         (x + sx / 2, y1 + sy / 2, z1), (x - sx / 2, y1 + sy / 2, z1)]
    q = [[b[i], b[(i + 1) % 4], t[(i + 1) % 4], t[i]] for i in range(4)]
    q.append(list(t))
    q.append(list(reversed(b)))    # closed bottom (the STL twin has one)
    return q


def tube_quads(p0, p1, sx, sy):
    """Closed box between two centres, tilted in any direction (canopy
    braces): horizontal cross-section at each end."""
    def rect(c):
        x, y, z = c
        return [(x - sx / 2, y - sy / 2, z), (x + sx / 2, y - sy / 2, z),
                (x + sx / 2, y + sy / 2, z), (x - sx / 2, y + sy / 2, z)]
    b, t = rect(p0), rect(p1)
    return [[b[0], b[1], b[2], b[3]], [t[0], t[3], t[2], t[1]]] + \
           [[b[i], t[i], t[j], b[j]] for i, j in ((0, 1), (1, 2), (2, 3), (3, 0))]


def shade(quads, base=(0.62, 0.69, 0.78), light=(0.35, -0.55, 0.75)):
    L = np.asarray(light, float)
    L /= np.linalg.norm(L)
    cols = []
    for q in quads:
        q = np.asarray(q, float)
        nrm = np.cross(q[1] - q[0], q[2] - q[0])
        ln = np.linalg.norm(nrm)
        lam = abs(float(np.dot(nrm / ln, L))) if ln > 1e-12 else 0.5
        f = 0.45 + 0.55 * lam
        cols.append((base[0] * f, base[1] * f, base[2] * f, 1.0))
    return cols


def draw_panel(ax, V, quads, cols, label, elev, azim, draft, lo, hi):
    ax.add_collection3d(Poly3DCollection(
        quads, facecolors=cols, edgecolors=(0.25, 0.28, 0.32, 0.16),
        linewidths=0.25))
    if draft is not None:
        x0 = float(V[:, :, 0].min()) - 0.2
        x1 = float(V[:, :, 0].max()) + 0.2
        y0 = float(V[:, :, 1].min()) - 0.2
        y1 = float(V[:, :, 1].max()) + 0.2
        plane = [[(x0, y0, draft), (x1, y0, draft),
                  (x1, y1, draft), (x0, y1, draft)]]
        ax.add_collection3d(Poly3DCollection(
            plane, facecolors=(0.35, 0.6, 0.95, 0.20),
            edgecolors=(0.2, 0.4, 0.8, 0.6), linewidths=0.8))
    c = (lo + hi) / 2
    ext = np.maximum(hi - lo, 1e-6)
    ax.set_xlim(c[0] - ext[0] / 2, c[0] + ext[0] / 2)
    ax.set_ylim(c[1] - ext[1] / 2, c[1] + ext[1] / 2)
    ax.set_zlim(c[2] - ext[2] / 2, c[2] + ext[2] / 2)
    try:
        ax.set_box_aspect(tuple(ext), zoom=0.8)
    except TypeError:
        ax.set_box_aspect(tuple(ext))
    ax.view_init(elev=elev, azim=azim)
    ax.set_title(label, fontsize=10)
    ax.set_axis_off()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("-o", "--out", required=True, help="output basename (.png/.pdf)")
    ap.add_argument("--draft", type=float, default=None,
                    help="draw a translucent waterline plane at this draft [m]")
    ap.add_argument("--seat-h", type=float, default=None,
                    help="bench height above the floor [m] (page seat_h)")
    ap.add_argument("--roof-h", type=float, default=None,
                    help="roof clear height over the deck edge [m] (page roof_h)")
    ap.add_argument("--roof-n", type=int, default=0,
                    help="number of standard 450 W solar panels on the "
                         "canopy (0 = no roof)")
    ap.add_argument("--ply-mm", type=int, default=6,
                    help="plywood panel thickness [mm] (plans/notes)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    h = load_hull(args.hull)
    V, quads = build_mesh(hull_with_rails(h), open_top=True)
    cols = shade(quads)
    lo = V.reshape(-1, 3).min(axis=0)
    hi = V.reshape(-1, 3).max(axis=0)

    # interior so it reads as an open boat: floor + cross benches
    # (thwarts, the way small boats are actually fitted; aft bench = helm)
    sts = h["stations"]
    try:
        import designer as _des
        lay = _des.layout(h, {"roof_n": args.roof_n or 0,
                              **({"seat_h": args.seat_h} if args.seat_h
                                 else {}),
                              **({"roof_h": args.roof_h} if args.roof_h
                                 else {})})
    except Exception:
        lay = {"benches": [], "roof": None}
    z_sole = lay.get("z_sole")
    z_seat = lay.get("z_seat")
    if z_sole is None:
        z_sole = min(s["pts"][0][1] for s in sts) + 0.05
    if z_seat is None:      # layout unavailable: last-resort ballpark
        try:
            z_seat = z_sole + _des.SEAT_H
        except NameError:
            z_seat = z_sole + 0.45
    sole_q, bench_q = [], []      # no sole plate: inner bottom = floor
    def st_half(x, z):
        """Hull half-width at (x, z), linearly between stations - the
        benches and risers follow the FLARE with it (a plain box leaves a
        wedge gap on a flared bilge)."""
        if x <= sts[0]["x"]:
            return y_at_z(sts[0]["pts"], z)
        for i in range(len(sts) - 1):
            if x <= sts[i + 1]["x"]:
                t = (x - sts[i]["x"]) / (sts[i + 1]["x"] - sts[i]["x"])
                return ((1 - t) * y_at_z(sts[i]["pts"], z)
                        + t * y_at_z(sts[i + 1]["pts"], z))
        return y_at_z(sts[-1]["pts"], z)

    from export_stl import _slope_slab as _slab
    eps = -0.003        # benches sink a hair into the wall (fused print)

    for bn in lay.get("benches", []):
        xc, hb = bn["xc"], bn["half"]
        xa_, xb_ = xc - bn["w"] / 2.0, xc + bn["w"] / 2.0
        th = _des.RISER_TOP           # preview bench thickness (= 0.04)
        z_bot = z_seat - th
        ya_b = max(0.02, st_half(xa_, z_bot) - eps)
        yb_b = max(0.02, st_half(xb_, z_bot) - eps)
        ya_t = max(0.02, st_half(xa_, z_seat) - eps)
        yb_t = max(0.02, st_half(xb_, z_seat) - eps)
        bench_q += _slab(xa_, xb_, (-ya_b, ya_b, ya_t, -ya_t),
                         (-yb_b, yb_b, yb_t, -yb_t), z_bot, z_seat)
        for sgn in (1, -1):     # risers (cleats) glued + filleted/taped to
            rxa = xa_ - _des.RISER_EXT            # the hull sides, bevelled
            rxb = xb_ + _des.RISER_EXT            # to the flare like the STL
            rb_bot = z_bot - _des.RISER_T
            ra_b = st_half(rxa, rb_bot) - _des.RISER_CLEAR
            rb_b = st_half(rxb, rb_bot) - _des.RISER_CLEAR
            ra_t = st_half(rxa, z_bot) - _des.RISER_CLEAR
            rb_t = st_half(rxb, z_bot) - _des.RISER_CLEAR
            if sgn > 0:
                pa_ = (ra_b - _des.RISER_T, ra_b, ra_t, ra_t - _des.RISER_T)
                pb_ = (rb_b - _des.RISER_T, rb_b, rb_t, rb_t - _des.RISER_T)
            else:
                pa_ = tuple(-v for v in (ra_b - _des.RISER_T, ra_b, ra_t,
                                         ra_t - _des.RISER_T))
                pb_ = tuple(-v for v in (rb_b - _des.RISER_T, rb_b, rb_t,
                                         rb_t - _des.RISER_T))
            bench_q += _slab(rxa, rxb, pa_, pb_, rb_bot, z_bot)
        if 2 * hb > _des.POST_SPAN:     # long span: centre support post,
            # foot on the RISING keel (no sole plate any more) up to the
            # bench underside - same as the STL exporter
            zk = z_sole
            for st_i in range(len(sts) - 1):
                if sts[st_i]["x"] <= xc <= sts[st_i + 1]["x"]:
                    tt_ = ((xc - sts[st_i]["x"])
                           / (sts[st_i + 1]["x"] - sts[st_i]["x"]))
                    zk = max(zk, sts[st_i]["pts"][0][1] + tt_ *
                             (sts[st_i + 1]["pts"][0][1]
                              - sts[st_i]["pts"][0][1]))
                    break
            z_bot_ = z_seat - th
            bench_q += box_quads(xc, 0.0, (zk + z_bot_) / 2.0,
                                 _des.POST_SZ, _des.POST_SZ, z_bot_ - zk)
    for nm, rng in lay.get("decks", {}).items():  # fore + stern decks
        idx = [i for i, st in enumerate(sts)
               if rng[0] - 1e-9 <= st["x"] <= rng[1] + 1e-9]
        if len(idx) < 2:
            continue
        # the stern deck keeps the outboard clamp area clear (two plates)
        notch = _des.MOTOR_NOTCH if nm == "aft" else 0.0
        tops = []
        for i in idx:
            pts = sts[i]["pts"]
            z_e = max(pts, key=lambda q: (round(q[0], 9), q[1]))[1]
            z_t = z_e - 0.008          # flush with the gunwale
            tops.append((sts[i]["x"], max(0.02, y_at_z(pts, z_t) - 0.004),
                         z_t))
        for side in ((1,) if notch <= 1e-9 else (1, -1)):
            for (x0, y0, z0), (x1, y1, z1) in zip(tops, tops[1:]):
                yi0 = min(notch / 2.0, y0 - 0.03) if notch > 0 else -y0
                yi1 = min(notch / 2.0, y1 - 0.03) if notch > 0 else -y1
                a0, b0 = (yi0, y0) if side > 0 else (-y0, -yi0)
                a1, b1 = (yi1, y1) if side > 0 else (-y1, -yi1)
                bench_q.append([(x0, a0, z0), (x0, b0, z0),
                                (x1, b1, z1), (x1, a1, z1)])            # top
                bench_q.append([(x0, a0, z0 - 0.02), (x1, a1, z1 - 0.02),
                                (x1, b1, z1 - 0.02), (x0, b0, z0 - 0.02)])  # bottom
                bench_q.append([(x0, a0, z0), (x1, a1, z1),
                                (x1, a1, z1 - 0.02), (x0, a0, z0 - 0.02)])
                bench_q.append([(x0, b0, z0), (x0, b0, z0 - 0.02),
                                (x1, b1, z1 - 0.02), (x1, b1, z1)])
            x0_, y0_, z0_ = tops[0]
            x1_, y1_, z1_ = tops[-1]
            a0, b0 = ((min(notch / 2.0, y0_ - 0.03), y0_) if side > 0
                      else (-y0_, -min(notch / 2.0, y0_ - 0.03))) \
                if notch > 0 else (-y0_, y0_)
            a1, b1 = ((min(notch / 2.0, y1_ - 0.03), y1_) if side > 0
                      else (-y1_, -min(notch / 2.0, y1_ - 0.03))) \
                if notch > 0 else (-y1_, y1_)
            bench_q.append([(x0_, a0, z0_), (x0_, a0, z0_ - 0.02),
                            (x0_, b0, z0_ - 0.02), (x0_, b0, z0_)])   # end caps
            bench_q.append([(x1_, a1, z1_), (x1_, b1, z1_),
                            (x1_, b1, z1_ - 0.02), (x1_, a1, z1_ - 0.02)])
    # (the rails + end caps are fused into the hull profile: nothing to add)

    # solar canopy: the same members the fabrication detail sheet dimensions
    # (designer.canopy_struct) - frame tubes, bolted feet, clamped panels
    roof_q, alu_q, seam_q = [], [], []
    r = lay.get("roof")
    cs = None
    if r:
        try:
            cs = _des.canopy_struct(h, r)
        except Exception:
            cs = None
    if cs:
        for rl in cs["rails"]:
            alu_q += box_quads((rl["x0"] + rl["x1"]) / 2, rl["y"],
                               (rl["z0"] + rl["z1"]) / 2,
                               rl["x1"] - rl["x0"], rl["sy"], rl["sz"])
        for en in cs["ends"]:
            alu_q += box_quads(en["x"], (en["y0"] + en["y1"]) / 2,
                               (en["z0"] + en["z1"]) / 2, en["sx"],
                               en["y1"] - en["y0"], en["z1"] - en["z0"])
        for cx in cs["cross"]:
            alu_q += box_quads(cx["x"] + cx["sx"] / 2,
                               (cx["y0"] + cx["y1"]) / 2,
                               (cx["z0"] + cx["z1"]) / 2, cx["sx"],
                               cx["y1"] - cx["y0"], cx["z1"] - cx["z0"])
        for xl in cs["xlegs"]:      # X supports: crossing diagonals
            alu_q += tube_quads((xl["x0"], xl["y0"], xl["z0"]),
                                (xl["x1"], xl["y1"], xl["z1"]),
                                xl["sx"], xl["sy"])
        for ft in cs["feet"]:      # plate sits ON the sheer like the exporter
            alu_q += box_quads(ft["x"], ft["y"], ft["z"] + ft["sz"] / 2.0,
                               ft["sx"], ft["sy"], ft["sz"])
        for cl in cs["clamps"]:
            alu_q += box_quads(cl["x"], cl["y"], (cl["z0"] + cl["z1"]) / 2,
                               cl["sx"], cl["sy"], cl["z1"] - cl["z0"])
        for pn in cs["panels"]:            # standard 450 W modules
            xa, ya = pn["x0"], pn["y0"]
            xb_, yb_ = xa + cs["pan_x"], ya + cs["pan_y"]
            roof_q.append([(xa, ya, cs["z_pan1"]), (xb_, ya, cs["z_pan1"]),
                           (xb_, yb_, cs["z_pan1"]), (xa, yb_, cs["z_pan1"])])
            alu_q.append([(xa, ya, cs["z_pan0"]), (xa, yb_, cs["z_pan0"]),
                          (xb_, yb_, cs["z_pan0"]), (xb_, ya, cs["z_pan0"])])
            for ey in (ya, yb_):           # module frame rims
                alu_q.append([(xa, ey, cs["z_pan0"]), (xb_, ey, cs["z_pan0"]),
                              (xb_, ey, cs["z_pan1"]), (xa, ey, cs["z_pan1"])])
            for ex in (xa, xb_):
                alu_q.append([(ex, ya, cs["z_pan0"]), (ex, yb_, cs["z_pan0"]),
                              (ex, yb_, cs["z_pan1"]), (ex, ya, cs["z_pan1"])])

    sole_c = shade([np.asarray(q, float) for q in sole_q], base=(0.70, 0.73, 0.78))
    bench_c = shade([np.asarray(q, float) for q in bench_q], base=(0.55, 0.40, 0.24))
    roof_c = shade([np.asarray(q, float) for q in roof_q], base=(0.16, 0.24, 0.50))
    alu_c = shade([np.asarray(q, float) for q in alu_q], base=(0.70, 0.72, 0.74))
    seam_c = shade([np.asarray(q, float) for q in seam_q], base=(0.06, 0.07, 0.10))
    # one single collection: per-polygon depth sort needs them together,
    # otherwise interior parts paint over hull panels regardless of real depth
    quads = list(quads) + sole_q + bench_q + roof_q + alu_q + seam_q
    cols = list(cols) + sole_c + bench_c + roof_c + alu_c + seam_c
    pts_all = np.asarray([p for q in quads for p in q], float)
    rlo, rhi = pts_all.min(axis=0), pts_all.max(axis=0)

    fig = plt.figure(figsize=(13.5, 8.5))
    for i, (label, elev, azim) in enumerate(VIEWS, start=1):
        ax = fig.add_subplot(2, 3, i, projection="3d")
        draw_panel(ax, V, quads, cols, label, elev, azim, args.draft, rlo, rhi)

    name = args.title or h.get("name", "hull")
    sub = (f"LOA {hi[0] - lo[0]:.2f} m   B {hi[1] - lo[1]:.2f} m   "
           f"D {hi[2] - lo[2]:.2f} m")
    if args.draft is not None:
        sub += f"   waterline T = {args.draft:.2f} m"
    fig.suptitle(f"{name}  -  {sub}", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(args.out + ".png", dpi=args.dpi, facecolor="white")
    fig.savefig(args.out + ".pdf", facecolor="white")
    print(f"wrote {args.out}.png and {args.out}.pdf")


if __name__ == "__main__":
    main()
