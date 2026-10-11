#!/usr/bin/env python3
"""Generate a plausible hull offsets table from a few parameters.

A starter / placeholder hull so the whole pipeline (hydrostatics, stability,
budget, lines plan) can run before real lines exist. NOT a design tool -
replace the offsets with refined lines, then re-run everything.

Usage:
  python3 param_hull.py --loa 5.2 --beam 1.9 --depth 0.78 --rocker 0.07 \
      --stations 9 --levels 8 -o hull.offsets.json

Section shape: round-bilge side curve y(z) = ye * sqrt(1 - ((ze-z)/(ze-zk))^1.5);
--bottom-f blends in a flat bottom run for jon-boat / utility sections.
Plan shape: beam factor f(t) through (0, transom-f), (0.55, 1.0), (1, bow-f).
Keel: flat aft to rocker rise forward: zk(t) = rocker * t^3.
Sheer: ze(t) = depth + sheer_rise * t^2.
"""

import argparse
import json
import math


# --- shape heuristics (named, so nothing here pretends to be physics) ----
MID_T = 0.55          # widest-point control location, fraction of loa
MIN_BEAM_F = 0.03     # min beam factor: keeps the extreme sections invertible
BILGE_EXP = 1.5       # round-bilge shape exponent (heuristic)
MIN_STATIONS, MAX_STATIONS = 3, 61   # 61: integration + plot sanity limit
MIN_LEVELS, MAX_LEVELS = 2, 30       # 30: section sampling limit
BOTTOM_F_MAX = 1.0    # 1.0 = true box: flat bottom out to the chine


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loa", type=float, default=5.2)
    ap.add_argument("--beam", type=float, default=1.9)
    ap.add_argument("--depth", type=float, default=0.78,
                    help="midship depth, baseline to deck edge [m]")
    ap.add_argument("--rocker", type=float, default=0.07, help="keel rise at bow [m]")
    ap.add_argument("--sheer-rise", type=float, default=0.08,
                    help="deck edge rise at bow above depth [m]")
    ap.add_argument("--transom-f", type=float, default=0.80)
    ap.add_argument("--mid-f", type=float, default=1.00)
    ap.add_argument("--bow-f", type=float, default=0.33)
    ap.add_argument("--bottom-f", type=float, default=0.0,
                    help="flat bottom width as a fraction of the section half-"
                         "width (0 = round/V bottom; 0.8 = jon-boat style)")
    ap.add_argument("--sheet", action="store_true",
                    help="build from flat sheets (hard chine): straight-sided "
                         "sections - flat bottom + straight topsides, no round "
                         "bilge curves")
    ap.add_argument("--stations", type=int, default=13,
                    help="stations across the length (matches the builder "
                         "default; odd interval counts use the 3/8 Simpson "
                         "rule at the tail)")
    ap.add_argument("--levels", type=int, default=8, help="points down each side")
    ap.add_argument("--name", default=None)
    ap.add_argument("-o", "--out", required=True)
    args = ap.parse_args()

    for name in ("loa", "beam", "depth", "rocker", "sheer_rise",
                 "transom_f", "mid_f", "bow_f", "bottom_f"):
        if not math.isfinite(getattr(args, name)):
            raise SystemExit(f"--{name.replace('_', '-')} must be a finite number")
    if not (MIN_STATIONS <= args.stations <= MAX_STATIONS):
        raise SystemExit(f"need --stations between {MIN_STATIONS} and {MAX_STATIONS}")
    if not (MIN_LEVELS <= args.levels <= MAX_LEVELS):
        raise SystemExit(f"need --levels between {MIN_LEVELS} and {MAX_LEVELS}")
    if not (args.loa > 0 and args.beam > 0):
        raise SystemExit("--loa and --beam must be positive")
    if args.depth <= 0:
        raise SystemExit("--depth must be positive")
    if args.rocker < 0:
        raise SystemExit("--rocker must be >= 0 (the keel rises, never sinks)")
    if args.depth <= args.rocker:
        raise SystemExit("--depth must exceed --rocker (keel would sit above the deck)")
    if not (0.0 <= args.bottom_f <= BOTTOM_F_MAX):
        raise SystemExit(f"--bottom-f must be between 0 and {BOTTOM_F_MAX}"
                         " (1.0 = degenerate flat chine)")
    n = args.stations
    loa, beam, depth = args.loa, args.beam, args.depth
    fb = args.bottom_f

    # beam factor quadratic through (0, transom_f), (0.55, mid_f), (1, bow_f)
    t1, f0, f1, f2 = MID_T, args.transom_f, args.mid_f, args.bow_f
    a = (f1 - f0 - t1 * (f2 - f0)) / (t1 * t1 - t1)
    bcoef = f2 - f0 - a

    def raw_f(t):
        return a * t * t + bcoef * t + f0

    # normalize against the EMITTED station positions so the widest station has
    # exactly the requested beam (the continuous curve may bulge a little
    # between stations; verify in the lines plan)
    fmax = max(raw_f(i / (n - 1)) for i in range(n))
    if fmax < 1e-6:
        raise SystemExit("beam factors are all zero - check --transom-f/--mid-f/--bow-f")

    def f(t):
        return max(MIN_BEAM_F, raw_f(t) / fmax)

    stations = []
    for i in range(n):
        t = i / (n - 1)
        x = round(loa * t, 6)
        ye = beam / 2.0 * f(t)
        zk = args.rocker * t ** 3
        ze = depth + args.sheer_rise * t * t
        if ze <= zk + 1e-9:
            raise SystemExit(f"station {i}: deck edge at/below the keel "
                             f"(ze={ze:.3f} <= zk={zk:.3f}) - check --depth / "
                             "--rocker / --sheer-rise")
        pts = [[0.0, round(zk, 4)]]
        if args.sheet:
            # hard chine: flat bottom run + ONE straight topside panel
            if fb > 1e-6:
                pts.append([round(fb * ye, 4), round(zk, 4)])
            pts.append([round(ye, 4), round(ze, 4)])
            pts.append([0.0, round(ze, 4)])
        else:
            if fb > 1e-6:  # flat bottom run to the chine
                pts.append([round(fb * ye, 4), round(zk, 4)])
            if fb < 1.0 - 1e-9:
                # bilge curve only while there IS a bilge: at bottom-f 1.0
                # the side is vertical and every curved sample would land
                # ON the chine (duplicate points, the old reason for the
                # artificial 0.85 cap) - go straight to the deck edge
                for j in range(1, args.levels):
                    z = zk + (ze - zk) * j / args.levels
                    curve = (1.0 - ((ze - z) / (ze - zk)) ** BILGE_EXP) ** 0.5
                    y = ye * (fb + (1.0 - fb) * curve)
                    pts.append([round(y, 4), round(z, 4)])
            pts.append([round(ye, 4), round(ze, 4)])
            pts.append([0.0, round(ze, 4)])
        stations.append({"x": x, "section": pts})

    # the ROUNDED output must still be a usable hull: rounding can collapse
    # stations/sections for extreme parameter combinations that pass the
    # scalar guards above
    if len({s["x"] for s in stations}) != n:
        raise SystemExit("rounding collapsed station x values - increase "
                         "--loa or reduce --stations")
    if any(st["section"][-2][0] <= 1e-9 for st in stations[1:-1]):
        raise SystemExit("rounding collapsed a mid section breadth - adjust "
                         "--beam/--bottom-f or reduce --levels")
    if any(st["section"][-2][1] <= st["section"][0][1] + 1e-6
           for st in stations):
        raise SystemExit("a section came out flat (deck at the keel height) - "
                         "reduce --rocker or --sheer-rise, increase --depth")
    out = {"name": args.name or f"parametric {loa} x {beam} hull",
           "units": "m", "stations": stations}
    with open(args.out, "w") as fp:
        json.dump(out, fp, indent=1)
    print(f"wrote {args.out}: {n} stations, LOA {loa} m, beam {beam} m, depth {depth} m")
    print("Next: python3 hydrostatics.py " + args.out)


if __name__ == "__main__":
    main()
