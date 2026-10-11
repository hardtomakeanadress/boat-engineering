#!/usr/bin/env python3
"""Color-coded check render of an exported STL: exterior skin, interior
faces, benches, risers, floors and decks get different colours so mistakes
are obvious at a glance (interior colour on the outside = broken normals;
magenta = a part that could not be classified -> geometry changed).

Usage:
  python3 render_stl.py boat.stl -o check.png [--views 6]
"""

import argparse
import sys
import struct

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

C_EXT_PORT = (0.22, 0.42, 0.80, 1.0)    # outside, port side
C_EXT_STBD = (0.30, 0.75, 0.95, 1.0)    # outside, starboard side
C_EXT_BOT = (0.35, 0.35, 0.38, 1.0)     # outside, bottom
C_EXT_BOW = (0.30, 0.70, 0.35, 1.0)     # outside, bow face
C_EXT_STERN = (0.85, 0.55, 0.05, 1.0)   # outside, transom
C_RIM = (1.00, 0.95, 0.60, 1.0)         # gunwale rim (top edge)
C_INT_PORT = (0.95, 0.35, 0.45, 1.0)    # inside, port side
C_INT_STBD = (0.55, 0.15, 0.35, 1.0)    # inside, starboard side
C_INT_BOT = (0.75, 0.25, 0.55, 1.0)     # inside, bottom
C_INT_BOW = (0.95, 0.65, 0.75, 1.0)     # inside, bow face
C_INT_STERN = (0.60, 0.20, 0.60, 1.0)   # inside, transom face
C_FLOOR = (0.20, 0.60, 0.25, 1.0)       # floor plate
C_FORE_DECK = (0.20, 0.75, 0.75, 1.0)   # foredeck
C_AFT_DECK = (0.15, 0.50, 0.90, 1.0)    # stern deck
BENCH_COLS = [(0.80, 0.55, 0.25, 1.0),  # bench 1 (aft/helm)
              (0.65, 0.42, 0.15, 1.0),  # bench 2
              (0.90, 0.70, 0.40, 1.0),  # bench 3
              (0.55, 0.38, 0.28, 1.0)]  # bench 4 (long boats)
C_PLATE_STERN = (0.45, 0.28, 0.28, 1.0)   # solid transom plate
C_PLATE_BOW = (0.28, 0.30, 0.48, 1.0)     # solid bow plate
C_GUNW = (0.80, 0.62, 0.30, 1.0)          # gunwale rails (teak tone)
C_GUNW_T = (0.66, 0.48, 0.22, 1.0)        # gunwale transom cap
C_PANEL = (0.16, 0.24, 0.55, 1.0)         # solar panel
C_RAIL = (0.58, 0.61, 0.68, 1.0)          # canopy frame rail (aluminium)
C_RAIL_D = (0.42, 0.45, 0.52, 1.0)        # canopy frame end
C_CROSS = (0.70, 0.73, 0.80, 1.0)         # crossbar
C_LEG = (0.85, 0.52, 0.20, 1.0)           # leg (orange)
C_BRACE = (0.20, 0.60, 0.45, 1.0)         # fore/aft brace (green)
C_FOOT = (0.45, 0.30, 0.15, 1.0)          # foot plate (brown)
C_CLAMP = (0.92, 0.86, 0.25, 1.0)         # clamp (yellow)
C_ODD = (1.0, 0.0, 1.0, 1.0)            # unclassified - ALARM magenta


def _darker(c, f=0.55):
    return (c[0] * f, c[1] * f, c[2] * f, 1.0)


def parse_stl(path):
    raw = open(path, 'rb').read()
    n = struct.unpack('<I', raw[80:84])[0]
    tris = np.zeros((n, 3, 3), float)
    for i in range(n):
        off = 84 + i * 50
        v = struct.unpack('<9f', raw[off + 12:off + 48])
        tris[i] = np.array(v).reshape(3, 3)
    return tris


def bodies(tris):
    """Split triangles into connected bodies via shared vertices."""
    key = np.round(tris.reshape(-1, 3) * 1e3, 0).astype(np.int64)
    _, vidx = np.unique(key, axis=0, return_inverse=True)
    vidx = vidx.reshape(-1, 3)
    parent = list(range(vidx.max() + 1))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for t in vidx:
        for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
    groups = {}
    for i, t in enumerate(vidx):
        groups.setdefault(find(t[0]), []).append(i)
    return [np.array(g) for g in sorted(groups.values(), key=len, reverse=True)]


def classify(tris, groups, hull_json=None):
    """Per-triangle colour. Largest body = hull shell; its faces split into
    exterior/interior by normal-vs-centroid. The other bodies are matched to
    the expected parts (floor, benches, risers, decks) from the hull+layout,
    in scale-free normalised coordinates; anything that does not match a
    known part is MAGENTA - the alarm colour."""
    cols = np.zeros((len(tris), 4))
    names_list = []
    exp_size = {}   # nm -> (dx, dy) in real m: centroid alone matched a
    #               # foreign 200 mm cube to a bench; the size must fit too
    shell = groups[0]
    ctr = tris[shell].reshape(-1, 3).mean(axis=0)
    for i in shell:
        t = tris[i]
        nrm = np.cross(t[1] - t[0], t[2] - t[0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            cols[i] = C_ODD
            continue
        n = nrm / ln
        mid = t.mean(axis=0)
        if np.dot(n, mid - ctr) > 0:              # exterior skin
            if n[2] > 0.5:
                cols[i] = C_RIM
            elif n[2] < -0.5:
                cols[i] = C_EXT_BOT
            elif n[0] < -0.5:
                cols[i] = C_EXT_STERN
            elif n[0] > 0.5:
                cols[i] = C_EXT_BOW
            elif n[1] > 0:
                cols[i] = C_EXT_STBD
            else:
                cols[i] = C_EXT_PORT
        else:                                     # interior skin
            if n[2] > 0.5:
                cols[i] = C_INT_BOT
            elif n[0] > 0.5:
                cols[i] = C_INT_STERN             # inside of the transom
            elif n[0] < -0.5:
                cols[i] = C_INT_BOW
            elif n[1] > 0:
                cols[i] = C_INT_PORT              # inside of the port side
            else:
                cols[i] = C_INT_STBD
    allv = tris.reshape(-1, 3)
    span = np.maximum(allv.max(0) - allv.min(0), 1e-9)
    exp = []
    exp_can = []
    exp_size = {}   # nm -> (dx, dy) real m: centroid alone matched a foreign
    #               # 200 mm cube to a bench; the SIZE must fit too
    z_sheer = None
    try:
        import os
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import hull as H
        import designer as D
        if hull_json and os.path.exists(hull_json):
            h = H.load_hull(hull_json)
            # the bench count follows the PEOPLE slider and lives in the
            # STL, not the hull file: detect bench-sized bodies and pick
            # the people count whose layout matches - otherwise every
            # bench/riser expectation is off and they all read UNKNOWN
            n_bench_det = 0
            for gg in groups[1:]:
                pp = tris[gg].reshape(-1, 3)
                bx = (pp[:, 0].max() - pp[:, 0].min()) / span[0]
                by = (pp[:, 1].max() - pp[:, 1].min()) / span[1]
                bz = (pp[:, 2].max() - pp[:, 2].min()) / span[2]
                if 0.02 < bx < 0.14 and 0.45 < by < 1.0 and bz < 0.12:
                    n_bench_det += 1
            lay = D.layout(h, {"roof_n": 0})
            if 1 <= n_bench_det <= 12 and \
                    len(lay["benches"]) != n_bench_det:
                for _p in range(1, 13):
                    _l2 = D.layout(h, {"roof_n": 0, "people": _p})
                    if len(_l2["benches"]) == n_bench_det:
                        lay = _l2
                        break
            loa = lay["loa"]
            ed = [max(s["pts"], key=lambda q: (round(q[0], 9), q[1]))
                  for s in h["stations"]]
            beam = 2.0 * max(q[0] for q in ed)
            depth = max(q[1] for q in ed)
            # total model height in REAL metres: the STL coords are model
            # millimetres, so the matching keys are ratios - keep the
            # expected parts in the same ratio space (loa * z-extent/x-extent)
            zt = loa * span[2] / span[0]
            z_sheer = depth
            z_sole, z_seat = lay["z_sole"], lay["z_seat"]
            zk_h = [st["pts"][0][1] for st in h["stations"]]
            cov = [st["x"] for st in h["stations"]
                   if st["pts"][0][1] <= min(zk_h) + 0.025]
            x_fl = (cov[0] + cov[-1]) / 2.0 if len(cov) > 1 else 0.45 * loa
            # (no floor plate any more: the inner bottom IS
            #  the floor - expectations removed with the part)
            exp.append(("stern plate", C_PLATE_STERN, (0.0, 0.0, depth / 2.0 / zt)))
            exp.append(("bow plate", C_PLATE_BOW, (1.0, 0.0, depth / 2.0 / zt)))
            # (the end-wall caps + inboard boards are fused
            #  into the end plates: no separate cap bodies exist)
            for bi, bn in enumerate(lay["benches"]):
                bcol = BENCH_COLS[bi % len(BENCH_COLS)]
                exp.append((f"bench{bi + 1}", bcol,
                            (bn["xc"] / loa, 0.0, z_seat / zt)))
                exp_size[f"bench{bi + 1}"] = (D.SEAT_W, 2 * bn["half"])
                exp.append((f"riser{bi + 1}", _darker(bcol),
                            (bn["xc"] / loa,
                             (bn["half"] - D.RISER_T / 2) / beam,
                             (z_seat - D.RISER_TOP - D.RISER_T / 2) / zt)))
                # the riser hugs the flared wall: its swept y-extent follows
                # the flare over its 60 mm height - size gate allows for it
                exp_size[f"riser{bi + 1}"] = (D.SEAT_W + 2 * D.RISER_EXT,
                                              D.RISER_T * 3.0)
                exp.append((f"post{bi + 1}", _darker(_darker(bcol)),
                            (bn["xc"] / loa, 0.0,
                             (z_sole + z_seat) / 2.0 / zt)))
                exp_size[f"post{bi + 1}"] = (D.POST_SZ, D.POST_SZ)
            for nm, rng in (("foredeck", lay["decks"]["fore"]),):
                col = C_FORE_DECK if nm == "foredeck" else C_AFT_DECK
                exp.append((nm, col,
                            (((rng[0] + rng[1]) / 2.0) / loa, 0.0,
                             (depth - 0.05) / zt)))
            # solar canopy parts: the body count of panel-sized slabs high
            # above the sheer tells how many panels the STL was built with;
            # rebuild the same canopy and list its members as expected parts
            n_pan = 0
            for gg in groups[1:]:
                pp = tris[gg].reshape(-1, 3)
                cc = pp.mean(axis=0)
                zn = (cc[2] - allv[:, 2].min()) / span[2]
                dxy = min(pp[:, 0].max() - pp[:, 0].min(),
                          pp[:, 1].max() - pp[:, 1].min())
                if zn > 0.85 and dxy > 0.3 * span[1]:
                    n_pan += 1             # big slabs high up = the modules
            if n_pan:
                cs = D.canopy_struct(h, D.layout(h, {"roof_n": n_pan})["roof"])
                for k, rl in enumerate(cs["rails"]):
                    exp_can.append((f"canopy rail {k + 1}", C_RAIL,
                                    ((rl["x0"] + rl["x1"]) / 2 / loa,
                                     abs(rl["y"]) / beam,
                                     (rl["z1"] - rl["sz"] / 2) / zt)))
                    exp_size[f"canopy rail {k + 1}"] = (rl["x1"] - rl["x0"],
                                                        rl["sy"])
                for k, en in enumerate(cs["ends"]):
                    exp_can.append((f"canopy end {k + 1}", C_RAIL_D,
                                    (en["x"] / loa, 0.0,
                                     (en["z0"] + en["z1"]) / 2 / zt)))
                    exp_size[f"canopy end {k + 1}"] = (en["sx"],
                                                       en["y1"] - en["y0"])
                for k, cr in enumerate(cs["cross"]):
                    exp_can.append((f"crossbar {k + 1}", C_CROSS,
                                    ((cr["x"] + cr["sx"] / 2) / loa, 0.0,
                                     (cr["z1"] - cr["sz"] / 2) / zt)))
                    exp_size[f"crossbar {k + 1}"] = (cr["sx"],
                                                     cr["y1"] - cr["y0"])
                for k, xl in enumerate(cs["xlegs"]):
                    exp_can.append((f"X support {k + 1}", C_LEG,
                                    ((xl["x0"] + xl["x1"]) / 2 / loa,
                                     abs((xl["y0"] + xl["y1"]) / 2) / beam,
                                     (xl["z0"] + xl["z1"]) / 2 / zt)))
                    exp_size[f"X support {k + 1}"] = (
                        abs(xl["x1"] - xl["x0"]) + xl["sx"],
                        xl["sy"] + abs(xl["y1"] - xl["y0"]))
                for k, ft in enumerate(cs["feet"]):
                    exp_can.append((f"foot {k + 1}", C_FOOT,
                                    (ft["x"] / loa, abs(ft["y"]) / beam,
                                     (ft["z"] + ft["sz"] / 2) / zt)))
                    exp_size[f"foot {k + 1}"] = (ft["sx"], ft["sy"])
                for k, cl in enumerate(cs["clamps"]):
                    exp_can.append((f"clamp {k + 1}", C_CLAMP,
                                    (cl["x"] / loa, abs(cl["y"]) / beam,
                                     (cl["z0"] + cl["z1"]) / 2 / zt)))
                    exp_size[f"clamp {k + 1}"] = (cl["sx"], cl["sy"])
                for k, pn in enumerate(cs["panels"]):
                    exp_can.append((f"panel {k + 1}", C_PANEL,
                                    ((pn["x0"] + cs["pan_x"] / 2) / loa,
                                     abs(pn["y0"] + cs["pan_y"] / 2) / beam,
                                     (cs["z_pan0"] + cs["pan_t"] / 2) / zt)))
                    exp_size[f"panel {k + 1}"] = (cs["pan_x"], cs["pan_y"])
    except Exception:
        exp = []
        exp_can = []
        exp_size = {}
    shell_top = float(tris[groups[0]][:, :, 2].max()) if groups else 0.0
    for g in groups[1:]:
        pts = tris[g].reshape(-1, 3)
        cen = pts.mean(axis=0)
        key = ((cen[0] - allv[:, 0].min()) / span[0],
               abs(cen[1]) / span[1],
               (cen[2] - allv[:, 2].min()) / span[2])
        footprint = max(pts[:, 0].max() - pts[:, 0].min(),
                        pts[:, 1].max() - pts[:, 1].min())
        pool = exp
        if exp_can:
            # above the sheer (frame + panels) or small and at sheer level
            # (feet, clamps) - all in the STL's own model-mm units
            above = cen[2] > shell_top + 0.10 * span[2]
            small_near_sheer = (footprint < 0.15 * span[1]
                                and cen[2] > shell_top - 0.05 * span[2])
            if (above and footprint >= 0.15 * span[1]) or small_near_sheer:
                pool = exp_can
        cand = []
        for nm, col, k in pool:
            d = abs(key[0] - k[0]) + abs(key[1] - k[1]) + abs(key[2] - k[2])
            cand.append((d, col, nm))
        cand.sort(key=lambda c: c[0])
        best, bd, bname = C_ODD, 9.9, "UNKNOWN"
        bxy = (pts[:, 0].max() - pts[:, 0].min(),
               pts[:, 1].max() - pts[:, 1].min())
        for d, col, nm in cand:
            if d >= 0.35:
                break
            want = exp_size.get(nm)
            if want is not None:      # generous per-axis size band in RATIO
                #                       space (model-mm cancels with the
                #                       spans): a wrong SIZE is a wrong part,
                #                       whatever the centroid distance says
                r0 = (bxy[0] / span[0]) / max(want[0] / loa, 1e-9)
                r1 = (bxy[1] / span[1]) / max(want[1] / beam, 1e-9)
                hi = 2.2 if nm.startswith("brace") else 2.5
                if not (0.4 <= r0 <= hi and 0.4 <= r1 <= hi):
                    continue
            best, bd, bname = col, d, nm
            break
        cols[g] = best if bd < 0.35 else C_ODD
        names_list.append(bname if bd < 0.35 else "UNKNOWN")
    return cols, names_list


def rasterize(tris, cols, elev_deg, azim_deg, W=560, H=420, ss=2, zoom=1.06,
              frame=None):
    """Software z-buffer render (orthographic): per-pixel depth test, so the
    near surface always occludes the far one - no painter-algorithm lies
    (matplotlib draws interior faces over the exterior and looks see-through).
    frame=(center, radius) pins the camera so multiple renders align
    pixel-for-pixel.  Returns an HxWx3 float image."""
    W2, H2 = W * ss, H * ss
    e, a = np.radians(elev_deg), np.radians(azim_deg)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross([0.0, 0.0, 1.0], d)
    right /= np.linalg.norm(right)
    upv = np.cross(d, right)
    allv = tris.reshape(-1, 3)
    if frame is not None:
        c, rad = np.asarray(frame[0], float), float(frame[1])
    else:
        c = (allv.min(0) + allv.max(0)) / 2.0
        rad = np.linalg.norm(allv.max(0) - allv.min(0)) / 2.0 * zoom
    P = tris - c
    u = P @ right
    v = P @ upv
    dep = P @ d
    s = min(W2, H2) / (2.0 * rad)
    px = u * s + W2 / 2.0
    py = H2 / 2.0 - v * s
    img = np.ones((H2, W2, 3))
    zbuf = np.full((H2, W2), -1e12)
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm, axis=1)
    ln[ln == 0] = 1.0
    nrm = nrm / ln[:, None]
    L = np.array([0.35, -0.5, 0.8])
    L /= np.linalg.norm(L)
    shade = 0.4 + 0.6 * np.abs(nrm @ L)
    for i in range(len(tris)):
        x_, y_ = px[i], py[i]
        xmin = max(int(x_.min()) - 1, 0)
        xmax = min(int(x_.max()) + 1, W2 - 1)
        ymin = max(int(y_.min()) - 1, 0)
        ymax = min(int(y_.max()) + 1, H2 - 1)
        if xmin > xmax or ymin > ymax:
            continue
        den = (y_[1] - y_[2]) * (x_[0] - x_[2]) \
            + (x_[2] - x_[1]) * (y_[0] - y_[2])
        if abs(den) < 1e-9:
            continue
        gy, gx = np.mgrid[ymin:ymax + 1, xmin:xmax + 1]
        gx = gx + 0.5
        gy = gy + 0.5
        l0 = ((y_[1] - y_[2]) * (gx - x_[2])
              + (x_[2] - x_[1]) * (gy - y_[2])) / den
        l1 = ((y_[2] - y_[0]) * (gx - x_[2])
              + (x_[0] - x_[2]) * (gy - y_[2])) / den
        l2 = 1.0 - l0 - l1
        m = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
        if not m.any():
            continue
        zz = l0 * dep[i, 0] + l1 * dep[i, 1] + l2 * dep[i, 2]
        subz = zbuf[ymin:ymax + 1, xmin:xmax + 1]
        upd = m & (zz > subz)
        subz[upd] = zz[upd]
        img[ymin:ymax + 1, xmin:xmax + 1][upd] = np.array(cols[i][:3]) \
            * shade[i]
    return img.reshape(H, ss, W, ss, 3).mean(axis=(1, 3))


def sheet_zbuf(tris, cols, out, title="", views=None, dpi=140):
    """6-view sheet rendered with the z-buffer so occlusion is truthful."""
    views = views or [("bow quarter", 22, 55), ("stern quarter", 22, 135),
                      ("side", 6, 90), ("top", 88, 90),
                      ("bow on", 8, 0), ("stern on", 8, 180)]
    fig = plt.figure(figsize=(4.6 * 3, 3.6 * 2))
    for k, (ttl, elev, azim) in enumerate(views, 1):
        ax = fig.add_subplot(2, 3, k)
        ax.imshow(rasterize(tris, cols, elev, azim))
        ax.set_axis_off()
        ax.set_title(ttl, fontsize=10)
    if title:
        fig.suptitle(title, fontsize=8.5)
    fig.savefig(out, dpi=dpi, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def part_sheet(tris, groups, cols, names, out, dpi=110):
    """One small render per part (hull split into exterior/interior faces)
    so every piece can be checked on its own before assembly."""
    allv = tris.reshape(-1, 3)
    c = (allv.min(0) + allv.max(0)) / 2
    ext = np.maximum(allv.max(0) - allv.min(0), 1)
    ctr = tris[groups[0]].reshape(-1, 3).mean(axis=0)
    shell = groups[0]
    ext_face = np.array([np.dot(np.cross(tris[i][1] - tris[i][0],
                                         tris[i][2] - tris[i][0]),
                                tris[i].mean(axis=0) - ctr) > 0
                         for i in shell])
    items = [("hull EXTERIOR", shell[ext_face]),
             ("hull INTERIOR", shell[~ext_face])]
    for i, g in enumerate(groups[1:], 1):
        items.append((f"{names[i - 1]} (#{i})", g))
    ncols = 4
    nrows = (len(items) + ncols - 1) // ncols
    fig = plt.figure(figsize=(4.4 * ncols, 3.8 * nrows))
    for k, (ttl, idxs) in enumerate(items, 1):
        ax = fig.add_subplot(nrows, ncols, k, projection='3d')
        sub = [tris[i] for i in idxs]
        subc = [tuple(cols[i]) for i in idxs]
        ax.add_collection3d(Poly3DCollection(
            sub, facecolors=subc,
            edgecolors=(0.15, 0.15, 0.18, 0.35), linewidths=0.25))
        sv = np.array(sub).reshape(-1, 3)          # zoom to the part itself
        c2 = (sv.min(0) + sv.max(0)) / 2
        e2 = np.maximum(sv.max(0) - sv.min(0), 0.05 * ext)
        ax.set_xlim(c2[0] - e2[0] / 2, c2[0] + e2[0] / 2)
        ax.set_ylim(c2[1] - e2[1] / 2, c2[1] + e2[1] / 2)
        ax.set_zlim(c2[2] - e2[2] / 2, c2[2] + e2[2] / 2)
        ax.set_box_aspect(tuple(e2), zoom=0.95)
        ax.view_init(elev=28, azim=55)
        ax.set_axis_off()
        ax.set_title(f"{ttl}  ({len(idxs)} tris)", fontsize=9)
    fig.suptitle("individual parts - each part rendered alone", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=dpi, facecolor="white")
    plt.close(fig)
    print(f"wrote {out}  ({len(items)} parts)")


def exploded(tris, groups, cols, out, dpi=130):
    """Exploded view: the parts stacked upward in assembly order."""
    allv = tris.reshape(-1, 3)
    span = np.maximum(allv.max(0) - allv.min(0), 1e-9)
    shifted = tris.copy()
    for i, g in enumerate(groups[1:], 1):
        shifted[g, :, 2] += 0.55 * span[2] * i
    sv = shifted.reshape(-1, 3)
    ct = sv.mean(0)
    ext = np.maximum(sv.max(0) - sv.min(0), 1)
    fig = plt.figure(figsize=(12, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.add_collection3d(Poly3DCollection(
        list(shifted), facecolors=[tuple(c2) for c2 in cols],
        edgecolors=(0.15, 0.15, 0.18, 0.3), linewidths=0.15))
    ax.set_xlim(ct[0] - ext[0] / 2, ct[0] + ext[0] / 2)
    ax.set_ylim(ct[1] - ext[1] / 2, ct[1] + ext[1] / 2)
    ax.set_zlim(sv[:, 2].min() - 1, sv[:, 2].max() + 1)
    ax.set_box_aspect(tuple(ext), zoom=0.9)
    ax.view_init(elev=22, azim=50)
    ax.set_axis_off()
    ax.set_title("exploded - hull, then floor, benches, risers and decks "
                 "stacked in order")
    fig.tight_layout()
    fig.savefig(out, dpi=dpi, facecolor="white")
    plt.close(fig)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("stl")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--dpi", type=int, default=140)
    args = ap.parse_args()
    tris = parse_stl(args.stl)
    groups = bodies(tris)
    import os
    hull_json = os.path.join(os.path.dirname(os.path.abspath(args.stl)),
                             "hull.json")
    cols, names = classify(tris, groups, hull_json)
    odd = int((abs(cols - np.array(C_ODD)).sum(axis=1) < 1e-6).sum())
    base = args.out.rsplit('.', 1)[0]
    part_sheet(tris, groups, cols, names, base + "_parts.png")
    exploded(tris, groups, cols, base + "_exploded.png")
    legend = (
        f"{args.stl}   (z-buffer render - occlusion is truthful)\n"
        "OUTSIDE: port=bold blue  stbd=light blue  bottom=grey  bow=green  "
        "stern=orange  rim=pale yellow  |  INSIDE: port=red  stbd=dark red  "
        "bottom=purple  bow=pink  stern=violet\n"
        "PARTS: bench1/2/3 = wood tones, risers = darker of their bench  |  "
        "floor=green  foredeck=teal  (no stern deck: open transom)\n"
        "SOLAR CANOPY: panels=dark blue  rails=grey  ends=darker grey  "
        "crossbars=light grey  X supports=orange  feet=brown  "
        "clamps=yellow  |  "
        f"ALARM magenta = {odd} tris")
    sheet_zbuf(tris, cols, args.out, title=legend, dpi=args.dpi)
    print(f"bodies={len(groups)}  unclassified tris={odd}")
    if odd:
        sys.exit(1)     # alarms are failures when this runs as a gate


if __name__ == "__main__":
    main()
