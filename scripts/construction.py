#!/usr/bin/env python3
"""Emit a construction-plan drawing spec (engineering-drafting format).

The census of 50+ boat plans shows the construction sheet - station molds
with their setup table, the midship section with member callouts, and the
assembly sequence - is the most universal part of a real plan package.
Everything here is computed from the hull model + designer constants.

Usage:
  python3 construction.py hull.json -o spec.json [--scale 1:20|auto]
      [--sheet A3] [--people N] [--seat-h H]

Render: python3 ~/.claude/skills/engineering-drafting/scripts/draft.py \
    spec.json --out construction --png

Views: STATION MOLDS (every section drawn and dimensioned side by side),
MOLD TABLE (setup offsets per station), MIDSHIP CONSTRUCTION (member
callouts with dimensions).  All coordinates mm, model space 1:1.
"""
import argparse
import json
import os
import time

from hull import deck_edge, load_hull

SHEETS = {"A4": (297, 210), "A3": (420, 297), "A2": (594, 420)}
SCALES = [10, 20, 25, 50]
TITLE_BLOCK_H = 55


def build(h, opt, s):
    import designer as D
    P = lambda mm: mm * s
    sheet_w, sheet_h = opt["sheet_wh"]
    W, H = sheet_w * s, sheet_h * s
    M = P(12)

    sts = h["stations"]
    xs = [st["x"] * 1000.0 for st in sts]
    ed = [deck_edge(st["pts"]) for st in sts]
    n = len(sts)
    loa = xs[-1] - xs[0]
    z_max = max(q[1] for q in ed) * 1000.0
    sp = (loa) / (n - 1)

    def sec_pts(st):
        return [(p[0] * 1000.0, p[1] * 1000.0) for p in st["pts"][:-1]]

    # ---------- MOLD TABLE (also drawn as text under the sections) --------
    lay = D.layout(h, {k: v for k, v in
                       (("people", opt.get("people")),
                        ("seat_h", opt.get("seat_h"))) if v})
    z_seat = lay["z_seat"] * 1000.0
    z_sole = lay["z_sole"] * 1000.0

    views = []
    # ---------- STATION MOLDS: overlaid fan, key stations dimensioned -----
    # (13 dimensioned sections side by side cannot fit any sheet: real
    # plans overlay the section curves on one centreline - the fan - and
    # carry the per-station numbers in the mold table)
    ent, txt, dims = [], [], []
    ox = M + P(66)
    ybase = P(TITLE_BLOCK_H) + P(46)
    ymax = max(ed[i][1] for i in range(n)) * 1000.0
    pick = sorted({0, (n - 1) // 2, n - 1})
    for i in range(n):
        pts = sec_pts(sts[i])
        full = [(ox + y, ybase + z) for (y, z) in pts]
        full += [(ox - y, ybase + z) for (y, z) in reversed(pts)]
        ent.append({"type": "polyline", "points": full,
                    "layer": "OUTLINE" if i in pick else "STATION"})
    ent.append({"type": "line", "p1": [ox, ybase - P(6)],
                "p2": [ox, ybase + ymax + P(6)], "layer": "CENTER"})
    txt.append({"pos": [ox, ybase - P(52)], "h": P(4),
                "s": "STATION MOLDS (overlaid; every station in the table)",
                "align": "center"})
    # dimension lines get their OWN row (k) so labels can never collide:
    # y-dims stacked up-right, x-dims laddered below the fan
    for k, i in enumerate(pick):
        yy = ed[i][0] * 1000.0
        dims.append({"kind": "linear", "axis": "y",
                     "p1": [ox + yy, ybase], "p2": [ox + yy, ybase
                                                    + ed[i][1] * 1000.0],
                     "off": P(10) + k * P(14)})
        dims.append({"kind": "linear", "axis": "x",
                     "p1": [ox, ybase - P(2)], "p2": [ox + yy, ybase - P(2)],
                     "off": -P(12) - k * P(14)})
        # label ABOVE the station's deck edge (inside the sheet), never to
        # the left of the fan - the widest station's label landed off-sheet
        txt.append({"pos": [ox + yy, ybase + ed[i][1] * 1000.0 + P(7)],
                    "h": P(3), "s": f"M{i}", "align": "center"})
    views.append({"name": "STATION MOLDS", "entities": ent, "texts": txt,
                  "dims": dims})

    # ---------- MOLD TABLE (top right) -------------------------------------
    tb_x = W - M - P(96)
    tb_y = H - M - P(14)
    t_txt = [{"pos": [tb_x, tb_y], "h": P(3.5), "s":
              "MOLD TABLE (mm from baseline)", "align": "left"},
             {"pos": [tb_x, tb_y - P(6)], "h": P(2.6), "s":
              "mold    x    height  half-B  keel-h", "align": "left"}]
    for i in range(n):
        row = (f"{i:>4d} {xs[i] - xs[0]:>6.0f}  {ed[i][1] * 1000:>6.0f}  "
               f"{ed[i][0] * 1000:>6.0f}  {sts[i]['pts'][0][1] * 1000:>6.0f}")
        t_txt.append({"pos": [tb_x, tb_y - P(6) - (i + 1) * P(4.6)],
                      "h": P(2.6), "s": row, "align": "left"})
    views.append({"name": "MOLD TABLE", "entities": [], "texts": t_txt,
                  "dims": []})

    # ---------- MIDSHIP CONSTRUCTION (below the table, right) -------------
    im = (n - 1) // 2
    ent2, txt2, dims2 = [], [], []
    y_e = ed[im][0] * 1000.0
    z_e = ed[im][1] * 1000.0
    oxb = W - M - P(30) - y_e           # section's centreline x
    # baseline low enough for the section AND the table above it
    y0c = tb_y - P(8) - (n + 2) * P(4.6) - z_e - P(18)
    g_t, g_h = D.GUNW_T * 1000.0, D.GUNW_H * 1000.0
    for (y, z) in sec_pts(sts[im]):
        ent2.append({"type": "line", "p1": [oxb + y, y0c + z],
                     "p2": [oxb - y, y0c + z], "layer": "OUTLINE"})
    ent2.append({"type": "line", "p1": [oxb - y_e - P(22), y0c + z_e - g_h],
                 "p2": [oxb + y_e + P(22), y0c + z_e - g_h],
                 "layer": "DIM"})       # gunwale board line
    # three callouts, features far enough apart vertically that the label
    # rows can never collide; each label sits just above its own L-leader
    for (s_t, fz) in [
            ("gunwale in+out 20x45, tops flush with the sheer",
             z_e - g_h),
            (f"seat bench {opt.get('ply_mm', 6)} mm on 60x30 cleats, "
             f"top {z_seat - z_sole:.0f} above the floor", z_seat),
            (f"skin {opt.get('ply_mm', 6)} mm ply + glass, min 4 mm",
             z_e * 0.45)]:
        ly = fz + P(5)                   # label row: 5 mm above the tick
        ent2.append({"type": "line", "p1": [oxb - y_e - P(4), y0c + fz],
                     "p2": [oxb - y_e - P(20), y0c + fz], "layer": "DIM"})
        ent2.append({"type": "line", "p1": [oxb - y_e - P(20), y0c + fz],
                     "p2": [oxb - y_e - P(20), y0c + ly], "layer": "DIM"})
        txt2.append({"pos": [oxb - y_e - P(24), y0c + ly], "h": P(2.4),
                     "s": s_t, "align": "right"})
    dims2.append({"kind": "linear", "axis": "x",
                  "p1": [oxb, y0c], "p2": [oxb + y_e, y0c], "off": -P(12)})
    dims2.append({"kind": "linear", "axis": "y",
                  "p1": [oxb + y_e, y0c], "p2": [oxb + y_e, y0c + z_e],
                  "off": P(12)})
    txt2.append({"pos": [oxb, y0c + z_e + P(10)], "h": P(3.5),
                 "s": f"MIDSHIP (mold {im})", "align": "center"})
    views.append({"name": "MIDSHIP CONSTRUCTION", "entities": ent2,
                  "texts": txt2, "dims": dims2})

    # ---------- SETUP SEQUENCE (bottom-left note block) -------------------
    steps = [
        "SETUP: loft the offsets full size; cut molds M0-M%d from the "
        "table; set on the baseline, plumb and square." % (n - 1),
        "Skin the molds (butt joints land on molds); wire or tape the seams.",
        "Glass-tape keel + stem/stern seams inside; fillet every joint.",
        "Gunwale in+out boards, butt-jointed at bow/stern; glue + screw.",
        "Bench cleats, benches, posts; foredeck; then flotation + finish.",
    ]
    st_txt = []
    for k, s_t in enumerate(steps):
        st_txt.append({"pos": [M + P(8),
                               ybase + ymax + P(24)
                               + (len(steps) - 1 - k) * P(5)],
                       "h": P(2.6), "s": f"{k + 1}. {s_t}", "align": "left"})
    views.append({"name": "SETUP", "entities": [], "texts": st_txt,
                  "dims": []})

    notes = [
        "All dimensions in mm", "Do not scale from drawing",
        f"LOA {loa:.0f} B {2 * y_e:.0f} D {z_max:.0f}, molds every {sp:.0f}",
        f"Molds {n}; seat top {z_seat - z_sole:.0f} mm over the floor",
        "Offsets table = exact for this model; build a paper model first",
        "Not for certification",
    ]
    if opt.get("ply_mm"):
        notes.append(f"Ply {opt['ply_mm']} mm marine; see the materials list")
    spec = {"title": "CONSTRUCTION PLAN", "subtitle": h.get("name", "hull"),
            "scale": f"1:{s}", "sheet": opt["sheet"], "units": "mm",
            "date": time.strftime("%Y-%m-%d"),
            "drawn": "boat-engineering skill", "grid": False,
            "notes": notes, "views": views}
    report = {"ok": True, "sheet_mm": [sheet_w, sheet_h]}
    return spec, report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--scale", default="auto")
    ap.add_argument("--sheet", default="A3")
    ap.add_argument("--people", type=int, default=None)
    ap.add_argument("--seat-h", type=float, default=None)
    ap.add_argument("--ply-mm", type=int, default=6)
    ap.add_argument("--roof-n", type=int, default=0)
    args = ap.parse_args()
    if args.sheet not in SHEETS:
        raise SystemExit(f"--sheet must be one of {list(SHEETS)}")
    h = load_hull(args.hull)
    opt = {"sheet": args.sheet, "sheet_wh": SHEETS[args.sheet],
           "people": args.people, "seat_h": args.seat_h,
           "ply_mm": args.ply_mm, "roof_n": args.roof_n}
    s = None
    if args.scale != "auto":
        try:
            s = int(args.scale.split(":")[-1])
        except ValueError:
            raise SystemExit("--scale must be 'auto' or 1:N")
        if s not in SCALES:
            raise SystemExit(f"--scale must be one of {SCALES}")
    if s is None:
        for cand in SCALES:
            _, rep = build(h, opt, cand)
            if rep["ok"]:
                s = cand
                break
        if s is None:
            raise SystemExit("hull does not fit any stocked sheet scale")
    spec, rep = build(h, opt, s)
    json.dump(spec, open(args.out, "w"), indent=1)
    print(f"wrote {args.out} - scale 1:{s}, sheet {opt['sheet']}")
    print("render: python3 <engineering-drafting skill>/scripts/draft.py "
          f"{args.out} --out construction --png")


if __name__ == "__main__":
    main()
