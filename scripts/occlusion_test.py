#!/usr/bin/env python3
"""Occlusion test: put a bright marker wall BEHIND the boat (below the
sheer, so every sightline to it must cross solid hull), render from the
opposite side, and count marker pixels that fall INSIDE the boat's own
silhouette.  Any such pixel = the marker is visible THROUGH the boat =
a hole or a transparency bug.  Runs four views (bow, stern, both sides).

Usage: python3 occlusion_test.py boat.stl -o occlusion_check.png
"""

import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import render_stl as R


def box(x0, x1, y0, y1, z0, z1):
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    idx = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
           (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    tris = []
    for q in idx:
        p = [c[i] for i in q]
        tris += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return np.array(tris, float)


def model_to_real(tris, scale):
    # the STL is in model mm; hull coords in metres = mm * scale / 1000
    return tris * scale / 1000.0


def raster_ids(tris, frame, W=640, H=460, elev=8, azim=0):
    """z-buffer that also returns which triangle owns each pixel.
    `frame` = (centre, radius) shared across renders so the images align."""
    e, a = np.radians(elev), np.radians(azim)
    d = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.cross([0.0, 0.0, 1.0], d)
    right /= np.linalg.norm(right)
    upv = np.cross(d, right)
    c, rad = np.asarray(frame[0], float), float(frame[1])
    P = tris - c
    u = P @ right
    v = P @ upv
    dep = P @ d
    s = min(W, H) / (2.0 * rad)
    px = u * s + W / 2.0
    py = H / 2.0 - v * s
    zbuf = np.full((H, W), -1e12)
    idbuf = np.full((H, W), -1, int)
    for i in range(len(tris)):
        x_, y_ = px[i], py[i]
        xmin = max(int(x_.min()) - 1, 0)
        xmax = min(int(x_.max()) + 1, W - 1)
        ymin = max(int(y_.min()) - 1, 0)
        ymax = min(int(y_.max()) + 1, H - 1)
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
        subi = idbuf[ymin:ymax + 1, xmin:xmax + 1]
        upd = m & (zz > subz)
        subz[upd] = zz[upd]
        subi[upd] = i
    return idbuf


def build_markers(hh):
    """Marker walls sized from THE BOAT (via its hull.json): a gap behind
    each side, tops kept below the LOWEST sheer so every sightline from the
    opposite camera must cross solid hull.  (They used to be absolute metres
    tuned to the 5.2 m demo - on any other boat they missed the hull and the
    test passed vacuously.)"""
    import hull as H
    xs = [st["x"] for st in hh["stations"]]
    x0, x1 = xs[0], xs[-1]
    beam_h = 2.0 * max(p[0] for st in hh["stations"] for p in st["pts"])
    zs = H.min_deck_edge(hh)                 # lowest sheer: stay under it
    gap = max(0.35, 0.08 * (x1 - x0))
    thick = 0.45
    ywide = max(0.75 * beam_h, 1.5)
    xa, xb = x0 + 0.15 * (x1 - x0), x1 - 0.15 * (x1 - x0)
    # side walls must be GENUINELY visible (through the open side gap over
    # the far gunwale) or the test is blind: a fully hidden marker proves
    # nothing - n_seen stayed 0 and every verdict was vacuous
    zs_side = min(p[1] for st in hh["stations"] if xa <= st["x"] <= xb
                  for p in [max(st["pts"], key=lambda q: (round(q[0], 9), q[1]))])
    z_side = zs_side + 0.12
    return [
        # (name, colour, camera elev/azim, box behind that side, hull metres)
        ("green wall behind stern", (0.0, 0.8, 0.0), 8, 0,
         (x0 - gap - thick, x0 - gap, -ywide, ywide, 0.05, 0.92 * zs)),
        ("red wall beyond bow", (0.9, 0.05, 0.05), 8, 180,
         (x1 + gap, x1 + gap + thick, -ywide, ywide, 0.05, 0.92 * zs)),
        # camera azim 90 looks along +y: the wall must sit BEYOND the boat
        # (negative y), else it stands between camera and hull and covers it
        ("blue wall off starboard", (0.05, 0.25, 0.95), 8, 90,
         (xa, xb, -beam_h / 2 - gap - thick, -beam_h / 2 - gap,
          0.05, z_side)),
        ("yellow wall off port", (0.95, 0.8, 0.05), 8, 270,
         (xa, xb, beam_h / 2 + gap, beam_h / 2 + gap + thick,
          0.05, z_side)),
    ]
# thresholds detect the SATURATED marker colours above on the shaded
# render (marker channel bright, other channels near zero)
DETECT = {
    (0.0, 0.8, 0.0): lambda im: (im[..., 1] > 0.15) & (im[..., 0] < 0.04)
                              & (im[..., 2] < 0.04),
    (0.9, 0.05, 0.05): lambda im: (im[..., 0] > 0.30) & (im[..., 1] < 0.08)
                                 & (im[..., 2] < 0.08),
    (0.05, 0.25, 0.95): lambda im: (im[..., 2] > 0.40) & (im[..., 0] < 0.08),
    (0.95, 0.8, 0.05): lambda im: (im[..., 0] > 0.35) & (im[..., 1] > 0.30)
                                & (im[..., 2] < 0.08),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stl")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--scale", type=float, default=None,
                    help="STL scale denominator; default: derived from the "
                         "hull.json beside the STL (no silent 1:24)")
    ap.add_argument("--leak-tol", type=int, default=4,
                    help="marker pixels tolerated inside the silhouette "
                         "(anti-aliasing on marker edges); 0 = strict")
    args = ap.parse_args()

    import os
    stl = R.parse_stl(args.stl)
    groups = R.bodies(stl)
    hj = os.path.join(os.path.dirname(os.path.abspath(args.stl)), "hull.json")
    if not os.path.exists(hj):
        raise SystemExit(f"need {hj} next to the STL (the builder writes it) - "
                         "it sizes the marker walls and the hull classification")
    import hull as H
    hh = H.load_hull(hj)
    loa = hh["stations"][-1]["x"] - hh["stations"][0]["x"]
    shell = stl[groups[0]]                 # the SHELL's own x extent:
    xspan = float(np.ptp(shell.reshape(-1, 3)[:, 0]))   # numpy 2: np.ptp()
    scale = args.scale
    if scale is None:
        scale = loa * 1000.0 / xspan if xspan > 1e-9 else 24.0
        if not (2.0 < scale < 200.0):
            raise SystemExit(f"derived scale 1:{scale:.1f} is implausible - "
                             "wrong hull.json next to the STL?")
        print(f"scale derived from the hull: 1:{scale:.2f}")
    cols, names = R.classify(stl, groups, hj)
    hull = model_to_real(stl, scale)
    # "inside the boat silhouette" = the HULL's own outline.  The canopy
    # sits on legs with air around it - seeing the marker through THAT gap
    # is legitimate.  So fill the holes of the hull-only body set (biggest
    # body + everything not classified as a canopy member): a real hole in
    # the hull then still counts, the roof gap does not.
    _canopy = ("panel ", "canopy ", "crossbar", "leg ", "brace", "foot ",
               "clamp")
    _keep = [0] + [i for i in range(1, len(groups))
                   if not names[i - 1].startswith(_canopy)]
    hull_only = np.concatenate([stl[groups[g]] for g in _keep])
    MARKERS = build_markers(hh)

    fig = plt.figure(figsize=(4.8 * 4, 3.8 * 2))
    verdicts = []
    failed = False
    for k, (nm, col, elev, azim, bx) in enumerate(MARKERS):
        mk = box(*bx)
        marker_cols = [(col[0], col[1], col[2], 1.0)] * len(mk)
        all_t = np.concatenate([hull, mk])
        all_c = cols.tolist() + marker_cols
        allv = all_t.reshape(-1, 3)
        frame = ((allv.min(0) + allv.max(0)) / 2.0,
                 np.linalg.norm(allv.max(0) - allv.min(0)) / 2.0 * 1.04)
        boat_only = raster_ids(model_to_real(hull_only, scale), frame,
                               elev=elev, azim=azim)
        # the test region is the boat's FILLED outline - a z-buffer-hit mask
        # would exclude exactly the pixels where a hole lets background
        # through, hiding the very defect being tested
        from scipy.ndimage import binary_fill_holes, binary_erosion
        mask = binary_fill_holes(boat_only >= 0)
        # trim the outermost rim: marker-boundary anti-aliasing on the
        # silhouette edge is not a hole; a real hole leaks well inside
        mask = binary_erosion(mask, iterations=3)
        img = R.rasterize(all_t, all_c, elev, azim, W=640, H=460,
                          frame=frame)
        vis = DETECT[col](img) & mask
        n_leak = int(vis.sum())
        # ALSO: legally visible pixels outside the hull (sanity: marker is there)
        n_seen = int(DETECT[col](img).sum())
        if n_seen < 100:            # a blind fixture must never read PASS
            verdict = f"NO COVERAGE (only {n_seen} marker px visible)"
        elif n_leak <= args.leak_tol:
            verdict = "PASS"
        else:
            verdict = f"LEAK {n_leak}px"
        if not verdict.startswith("PASS"):
            failed = True
        verdicts.append((nm, verdict))
        ax = fig.add_subplot(2, 4, k + 1)
        ax.imshow(img)
        if n_leak:
            ys, xs = np.where(vis)
            ax.scatter(xs, ys, s=40, facecolors="none", edgecolors="red")
        ax.set_axis_off()
        ax.set_title(f"{nm}\n{verdict} (marker pixels: {n_seen})",
                     fontsize=8.5, color="red" if n_leak > 4 else "green")
    fig.suptitle(f"occlusion test - {args.stl}: marker behind the boat must "
                 "never appear inside the hull silhouette", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.out, dpi=130, facecolor="white")
    print("=== occlusion test")
    for nm, v in verdicts:
        print(f"  {nm}: {v}")
    print(f"wrote {args.out}")
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
