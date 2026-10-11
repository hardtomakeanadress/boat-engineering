#!/usr/bin/env python3
"""Emit a lines-plan drawing spec for the engineering-drafting skill.

Usage:
  python3 linesplan.py hull.json -o spec.json [--scale 1:20|auto] [--sheet A3]
      [--draft 0.32] [--wl-step 100] [--title "..."] [--offsets offsets.csv]

Views: PROFILE (sheer plan), HALF-BREADTH, BODY PLAN - all coordinates in mm
(model space 1:1, sheet drawn at the chosen scale). Dimensions are computed
from the offsets table - never hand-entered.

Then render with the engineering-drafting skill:
  python3 ~/.claude/skills/engineering-drafting/scripts/draft.py spec.json \
      --out linesplan --png

Also exports the lofting offsets table (half-breadths at each waterline) as
CSV + Markdown when --offsets is given.
"""

import argparse
import json
import os
import time

from hull import (RHO_FRESH, UNIFORM_TOL, deck_edge, keel_crossing,
                  load_hull, waterline_halfwidth)
from hydrostatics import at_draft

SHEETS = {"A4": (297, 210), "A3": (420, 297), "A2": (594, 420)}
SCALES = [10, 20, 25, 50, 100]
TITLE_BLOCK_H = 55    # paper mm the drafting sheet reserves for its title block
LABEL_FAN = 0.85      # label spacing fan: 1.0 = at the deck edge, less = lower


def build(h, opt, s):
    """Build spec for scale denominator s. Returns (spec, report)."""
    P = lambda mm: mm * s
    sheet_w, sheet_h = opt["sheet_wh"]
    W, H = sheet_w * s, sheet_h * s
    M, D, G = P(12), P(18), P(36)

    sts = h["stations"]
    xs = [st["x"] * 1000.0 for st in sts]
    zk = [st["pts"][0][1] * 1000.0 for st in sts]
    ye = [deck_edge(st["pts"])[0] * 1000.0 for st in sts]
    ze = [deck_edge(st["pts"])[1] * 1000.0 for st in sts]
    n = len(sts)
    loa = xs[-1] - xs[0]
    z_max = max(ze)
    ym_h = max(ye)
    b_mm = 2 * ym_h
    sp = xs[1] - xs[0]
    wl_step = opt["wl_step"]
    wls = [w for w in range(wl_step, int(min(ze) // wl_step) * wl_step + 1, wl_step)
           if w <= min(ze) - wl_step * 0.5]

    def hw(i, w):
        return waterline_halfwidth(sts[i]["pts"], w / 1000.0) * 1000.0

    views = []

    # ---------- PROFILE ----------
    ox = M + D
    y0p = H - M - P(20) - z_max
    X = lambda x: ox + (x - xs[0])
    Y = lambda z: y0p + z
    ent = [{"type": "polyline", "points": [[X(xs[i]), Y(zk[i])] for i in range(n)],
            "layer": "OUTLINE"},
           {"type": "polyline", "points": [[X(xs[i]), Y(ze[i])] for i in range(n)],
            "layer": "OUTLINE"}]
    for i in range(n):
        ent.append({"type": "line", "p1": [X(xs[i]), Y(zk[i])],
                    "p2": [X(xs[i]), Y(ze[i])], "layer": "STATION"})
    def wl_spans(w):
        """List of x-intervals where the hull is above this waterline.
        Entries/exits between stations are interpolated (keel or deck line
        crossing), so dry stretches in the middle are not bridged."""
        spans = []
        start = None
        for i in range(n):
            wet = zk[i] <= w <= ze[i]
            if wet and start is None:
                if i == 0:
                    start = xs[0]
                else:
                    xc = keel_crossing(xs[i - 1], zk[i - 1], xs[i], zk[i], w)
                    if xc is None:
                        xc = keel_crossing(xs[i - 1], ze[i - 1], xs[i], ze[i], w)
                    start = xc if xc is not None else xs[i]
            elif not wet and start is not None:
                xc = keel_crossing(xs[i - 1], zk[i - 1], xs[i], zk[i], w)
                if xc is None:
                    xc = keel_crossing(xs[i - 1], ze[i - 1], xs[i], ze[i], w)
                spans.append((start, xc if xc is not None else xs[i - 1]))
                start = None
        if start is not None:
            spans.append((start, xs[-1]))
        return spans

    for w in wls:
        for xa, xb in wl_spans(w):
            if xa < xb:
                ent.append({"type": "line", "p1": [X(xa), Y(w)],
                            "p2": [X(xb), Y(w)], "layer": "WATERLINE"})
    txt = [{"pos": [X(xs[i]), y0p - P(4)], "h": P(2.5), "s": str(i),
            "align": "center"} for i in range(n)]
    txt.append({"pos": [ox + loa / 2, y0p + z_max + P(9)], "h": P(4),
                "s": "PROFILE", "align": "center"})
    dims = [{"kind": "linear", "axis": "x", "p1": [X(xs[0]), y0p],
             "p2": [X(xs[-1]), y0p], "off": -P(12)},
            {"kind": "linear", "axis": "x", "p1": [X(xs[0]), y0p],
             "p2": [X(xs[1]), y0p], "off": -P(24)},
            {"kind": "linear", "axis": "y", "p1": [X(xs[-1]), y0p],
             "p2": [X(xs[-1]), Y(ze[-1])], "off": P(12)}]
    views.append({"name": "PROFILE", "entities": ent, "texts": txt, "dims": dims})
    profile_right = X(xs[-1])

    # ---------- HALF-BREADTH ----------
    yH = y0p - G - ym_h
    Yb = lambda y: yH + y
    ent = []
    for w in wls:
        for xa, xb in wl_spans(w):
            if xa >= xb:
                continue
            inx = [i for i in range(n)
                   if zk[i] <= w <= ze[i] and xa - 1e-9 <= xs[i] <= xb + 1e-9]
            pts = [[X(xa), Yb(0.0)]]
            pts += [[X(xs[i]), Yb(hw(i, w))] for i in inx]
            pts += [[X(xb), Yb(0.0)]]
            ent.append({"type": "polyline", "points": pts, "layer": "WATERLINE"})
    ent.append({"type": "polyline", "points": [[X(xs[i]), Yb(ye[i])] for i in range(n)],
                "layer": "OUTLINE"})
    for i in range(n):
        ent.append({"type": "line", "p1": [X(xs[i]), yH], "p2": [X(xs[i]), Yb(ye[i])],
                    "layer": "STATION"})
    ent.append({"type": "line", "p1": [X(xs[0]) - P(4), yH],
                "p2": [X(xs[-1]) + P(4), yH], "layer": "CENTER"})
    txt = [{"pos": [ox + loa / 2, yH + ym_h + P(9)], "h": P(4),
            "s": "HALF-BREADTH", "align": "center"}]
    dims = [{"kind": "linear", "axis": "y", "p1": [X(xs[0]), yH],
             "p2": [X(xs[0]), Yb(ym_h)], "off": -P(12)}]
    views.append({"name": "HALF-BREADTH", "entities": ent, "texts": txt, "dims": dims})

    # ---------- BODY PLAN ----------
    cl = W - M - D - ym_h
    y0b = M + P(TITLE_BLOCK_H) + P(28)   # clear of the title block
    ent = []
    for i in range(n):
        pts = sts[i]["pts"]
        full = [[cl + y * 1000.0, y0b + z * 1000.0] for (y, z) in pts]
        full += [[cl - y * 1000.0, y0b + z * 1000.0] for (y, z) in reversed(pts)]
        ent.append({"type": "polyline", "points": full, "layer": "STATION"})
    for w in wls:
        ent.append({"type": "line", "p1": [cl - ym_h, y0b + w],
                    "p2": [cl + ym_h, y0b + w], "layer": "WATERLINE"})
    ent.append({"type": "line", "p1": [cl, y0b - P(5)], "p2": [cl, y0b + z_max + P(5)],
                "layer": "CENTER"})
    ent.append({"type": "line", "p1": [cl - ym_h - P(4), y0b],
                "p2": [cl + ym_h + P(4), y0b], "layer": "OUTLINE"})
    txt = [{"pos": [cl, y0b + z_max + P(12)], "h": P(4), "s": "BODY PLAN",
            "align": "center"}]
    for i in range(n):
        # label next to its own section curve, stepping down along the fan so
        # labels do not pile up where deck-edge heights are similar
        zl = zk[i] + (ze[i] - zk[i]) * (1.0 - LABEL_FAN * i / (n - 1))
        yl = waterline_halfwidth(sts[i]["pts"], zl / 1000.0) * 1000.0
        txt.append({"pos": [cl + yl + P(4), y0b + zl],
                    "h": P(2.5), "s": str(i), "align": "left"})
    for w in wls:
        txt.append({"pos": [cl - ym_h - P(6), y0b + w], "h": P(2.5),
                    "s": f"{w:.0f}", "align": "right"})
    dims = [{"kind": "linear", "axis": "x", "p1": [cl - ym_h, y0b],
             "p2": [cl + ym_h, y0b], "off": -P(20)},
            {"kind": "linear", "axis": "y", "p1": [cl, y0b],
             "p2": [cl, y0b + z_max], "off": ym_h + P(12)}]
    # deadrise angle (bottom slope from horizontal) at the midship section.
    # Only one - the body plan overlays all stations on the same centreline,
    # so dims for several stations would land on top of each other.
    i_st = (n - 1) // 2
    pts_i = sts[i_st]["pts"]
    y1, z1 = pts_i[1]
    z0 = pts_i[0][1]
    if y1 > 1e-4 and (z1 - z0) > 1e-4:
        vxx, vyy = cl, y0b + z0 * 1000.0
        ent.append({"type": "line", "p1": [vxx, vyy],
                    "p2": [vxx + P(14), vyy], "layer": "CENTER"})
        dims.append({"kind": "angle", "vertex": [vxx, vyy],
                     "ray1": [vxx + P(12), vyy],
                     "ray2": [vxx + y1 * 1000.0, y0b + z1 * 1000.0],
                     "r": P(9)})
    views.append({"name": "BODY PLAN", "entities": ent, "texts": txt, "dims": dims})

    # ---------- notes ----------
    # same rule as the Simpson integrator: spacing vs the MEAN spacing,
    # relative tolerance (the two used to disagree and fail a clean plan)
    sp_mean = (xs[-1] - xs[0]) / (n - 1) if n > 1 else 0.0
    uniform = all(abs((xs[i + 1] - xs[i]) - sp_mean)
                  <= UNIFORM_TOL * max(1.0, abs(sp_mean)) for i in range(n - 1))
    sp = sp_mean
    notes = ["All dimensions in mm", "Do not scale from drawing",
             f"LOA {loa:.0f} B {b_mm:.0f} D {z_max:.0f}, spacing {sp:.0f} mm"
             + ("" if uniform else " (NON-UNIFORM - CHECK)"),
             f"Stations {n}, waterlines every {wl_step} mm"]
    if opt.get("hydro"):
        r = opt["hydro"]
        notes.append(f"T {r['T']:.2f} m, disp {r['disp']:.0f} kg, Cb {r['Cb']:.2f}")
        notes.append(f"KB {r['KB']:.3f} KM {r['KM']:.3f}, TPC {r['TPC']:.1f} kg/cm")
        notes.append(f"Even keel, rho={opt.get('rho', RHO_FRESH):.0f} kg/m3")

    spec = {"title": "LINES PLAN", "subtitle": h.get("name", "hull"),
            "scale": f"1:{s}", "sheet": opt["sheet"], "units": "mm",
            "date": time.strftime("%Y-%m-%d"), "drawn": "boat-engineering skill",
            "grid": False, "notes": notes, "views": views}

    # ---------- fit report ----------
    bb = [1e18, 1e18, -1e18, -1e18]

    def note(x0, y0, x1, y1):
        bb[0] = min(bb[0], x0)
        bb[1] = min(bb[1], y0)
        bb[2] = max(bb[2], x1)
        bb[3] = max(bb[3], y1)

    for v in views:
        for e in v["entities"]:
            if e["type"] == "polyline":
                for p in e["points"]:
                    note(p[0], p[1], p[0], p[1])
            elif e["type"] == "line":
                for p in (e["p1"], e["p2"]):
                    note(p[0], p[1], p[0], p[1])
        for t in v["texts"]:
            h = t.get("h", P(3))
            note(t["pos"][0] - P(3), t["pos"][1] - h, t["pos"][0] + P(3), t["pos"][1] + h)
        for d in v["dims"]:
            if d.get("kind") == "angle":
                vx, vy = d["vertex"]
                rad = d.get("r", P(12)) + P(10)
                note(vx - rad, vy - rad, vx + rad, vy + rad)
                continue
            (x1_, y1_), (x2_, y2_) = d["p1"], d["p2"]
            off = d.get("off", 0)
            ax = d.get("axis", "x")
            if ax == "x":
                yd = (y1_ + y2_) / 2 + off
                note(min(x1_, x2_) - P(3), yd - P(3), max(x1_, x2_) + P(3), yd + P(8))
            elif ax == "y":
                xd = (x1_ + x2_) / 2 + off
                note(xd - P(8), min(y1_, y2_) - P(3), xd + P(4), max(y1_, y2_) + P(3))
            else:
                pad = abs(off) + P(8)
                note(min(x1_, x2_) - pad, min(y1_, y2_) - pad,
                     max(x1_, x2_) + pad, max(y1_, y2_) + pad)
    bx0, by0, bx1, by1 = bb[0] - P(2), bb[1] - P(2), bb[2] + P(2), bb[3] + P(2)
    report = {"ok": (bx0 >= M * 0.5 and bx1 <= W - M * 0.5
                     and by0 >= M * 0.5 and by1 <= H - M * 0.5
                     and cl - ym_h - P(8) >= profile_right + P(8)
                     and y0b - P(20) - P(2) >= M + P(TITLE_BLOCK_H) + P(2)),
              "bbox": [bx0, by0, bx1, by1], "sheet_mm": [sheet_w, sheet_h]}
    return spec, report


def offsets_table(h, wl_step, path):
    sts = h["stations"]
    xs = [st["x"] * 1000.0 for st in sts]
    ze = [deck_edge(st["pts"])[1] * 1000.0 for st in sts]
    zk = [st["pts"][0][1] * 1000.0 for st in sts]
    wls = [w for w in range(0, int(min(ze) // wl_step) * wl_step + 1, wl_step)]
    header = ["z\\x"] + [f"{x:.0f}" for x in xs]
    rows = []
    for w in wls:
        vals = []
        for i, st in enumerate(sts):
            if not (zk[i] <= w <= ze[i]):
                vals.append("-")
            else:
                hw = waterline_halfwidth(st["pts"], w / 1000.0) * 1000.0
                vals.append(f"{hw:.0f}")
        rows.append([f"{w:.0f}"] + vals)
    rows.append(["keel z"] + [f"{z:.0f}" for z in zk])
    rows.append(["deck z"] + [f"{z:.0f}" for z in ze])
    rows.append(["deck y"] + [f"{y * 1000:.0f}" for y in
                              [deck_edge(st["pts"])[0] for st in sts]])
    with open(path, "w") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(",".join(r) + "\n")
    md = os.path.splitext(path)[0] + ".md"   # a dot in a DIRECTORY
    #                                          # name must not matter
    with open(md, "w") as f:
        f.write(f"# Offsets table - {h.get('name', 'hull')}\n\n")
        f.write("Half-breadths in mm at each station. '-' = the waterline is\n"
                "outside the section there (below the keel or above the deck\n"
                "edge); a true zero width (e.g. exactly at the keel) is 0.\n\n")
        f.write("| " + " | ".join(header) + " |\n")
        f.write("|" + "---|" * len(header) + "\n")
        for r in rows:
            f.write("| " + " | ".join(r) + " |\n")
    return md


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("-o", "--out", required=True, help="drawing spec JSON to write")
    ap.add_argument("--scale", default="auto", help="'auto' or e.g. 1:20")
    ap.add_argument("--sheet", default="A3", choices=sorted(SHEETS))
    ap.add_argument("--draft", type=float, default=None,
                    help="design draft [m]; adds hydrostatic particulars to the title block")
    ap.add_argument("--rho", type=float, default=RHO_FRESH)
    ap.add_argument("--wl-step", type=int, default=100,
                    help="waterline spacing [mm] (default suits ~5 m hulls)")
    
    ap.add_argument("--title", default=None)
    ap.add_argument("--offsets", default=None, help="write lofting offsets table (CSV + MD)")
    args = ap.parse_args()

    if args.wl_step < 1:
        raise SystemExit("--wl-step must be >= 1 mm")
    h = load_hull(args.hull)
    if args.title:
        h["name"] = args.title
    opt = {"sheet": args.sheet, "sheet_wh": SHEETS[args.sheet], "wl_step": args.wl_step,
           "rho": args.rho}
    if args.draft is not None:
        try:
            opt["hydro"] = at_draft(h, args.draft, args.rho)
        except Exception as e:
            print(f"WARNING: hydrostatics at draft {args.draft} failed: {e}")

    if args.scale != "auto":
        try:                       # accept "20" as well as "1:20"
            s = int(args.scale.split(":")[-1])
        except (ValueError, IndexError):
            raise SystemExit(f"--scale must be 'auto' or e.g. 1:20 "
                             f"(got {args.scale!r})")
        if s <= 0:
            raise SystemExit(f"--scale denominator must be positive (got {s})")
        spec, report = build(h, opt, s)
        if not report["ok"]:
            print(f"WARNING: content overflows {args.sheet} at {args.scale} "
                  f"(bbox {report['bbox']} vs sheet {report['sheet_mm']} mm paper "
                  f"x scale {s}) - use a coarser scale")
        chosen = s
    else:
        spec = None
        for s in SCALES:
            cand, report = build(h, opt, s)
            if report["ok"]:
                spec, chosen = cand, s
                break
        if spec is None:
            spec, chosen = cand, SCALES[-1]
            print("WARNING: nothing fits cleanly; used the coarsest scale - check output")

    with open(args.out, "w") as f:
        json.dump(spec, f, indent=1)
    print(f"wrote {args.out} - scale 1:{chosen}, sheet {args.sheet}")
    if args.offsets:
        md = offsets_table(h, args.wl_step, args.offsets)
        print(f"offsets: {args.offsets} and {md}")
    print("render: python3 <engineering-drafting skill>/scripts/draft.py "
          f"{args.out} --out linesplan --png")


if __name__ == "__main__":
    main()
