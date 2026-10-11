#!/usr/bin/env python3
"""Hydrostatics for a hull offsets table.

Usage:
  python3 hydrostatics.py hull.json --drafts 0.15:0.02:0.45 [--rho 1000] [--json out.json]
  python3 hydrostatics.py hull.json --mass 1200 [--rho 1000]     # find draft for a mass

Units: m, kg. rho = water density (fresh 1000 default, sea 1025).

Definitions used (see references/hydrostatics.md):
  V   = displaced volume            [m3]   (Simpson integration of sections)
  disp= rho * V                     [kg]
  Aw  = waterplane area             [m2]
  KB  = vertical center of buoyancy [m]    (from baseline)
  BM  = I_T / V                     [m]    transverse metacentric radius
  KM  = KB + BM                     [m]
  TPC = rho * Aw / 100              [kg/cm]
  MCT = disp * BM_L / (100 * L)     [kg*m/cm]  (approx: GML ~ BM_L)
"""

import argparse
import json
import math
import sys

from hull import (deck_edge, full_section, immersed, keel_crossing, load_hull,
                  simpson_uniform, waterline_halfwidth)

from hull import DECK_CLEARANCE, RHO_FRESH, integrate_wet, min_deck_edge

def at_draft(h, T, rho, kg=None):
    """Hydrostatic properties at waterline T (even keel). None if no
    immersion.  kg = optional VCG: MCT then uses the standard GM_L lever;
    without it MCT is the BM_L approximation (overestimates)."""
    xs = [s["x"] for s in h["stations"]]
    A, Az, hw = [], [], []
    for s in h["stations"]:
        poly = full_section(s["pts"])
        a, _yb, zb = immersed(poly, 0.0, T)
        A.append(a)
        Az.append(a * zb)
        hw.append(waterline_halfwidth(s["pts"], T))
    # waterline extent first: wet stations + keel crossings give [xa, xb];
    # EVERY integral below uses that span (hull.integrate_wet) so V, the
    # moments and Lwl describe the same waterline
    idx = [i for i, a in enumerate(A) if a > 1e-6]
    if not idx:
        return None
    i0, i1 = idx[0], idx[-1]
    zk_list = [s["pts"][0][1] for s in h["stations"]]
    xa, xb = xs[i0], xs[i1]
    if i0 > 0:
        xc = keel_crossing(xs[i0 - 1], zk_list[i0 - 1], xs[i0], zk_list[i0], T)
        xa = xc if xc is not None else xs[i0 - 1]
    if i1 < len(xs) - 1:
        xc = keel_crossing(xs[i1 + 1], zk_list[i1 + 1], xs[i1], zk_list[i1], T)
        xb = xc if xc is not None else xs[i1 + 1]
    Lwl = xb - xa
    V = integrate_wet(xs, A, xa, xb)
    if V <= 1e-9:
        return None
    Aw = 2.0 * integrate_wet(xs, hw, xa, xb)
    I_T = (2.0 / 3.0) * integrate_wet(xs, [w ** 3 for w in hw], xa, xb)
    half_Aw = integrate_wet(xs, hw, xa, xb)
    LCF = (integrate_wet(xs, [w * x for x, w in zip(xs, hw)], xa, xb) / half_Aw
           if half_Aw > 1e-9 else 0.5 * (xa + xb))
    I_L = 2.0 * integrate_wet(xs, [w * (x - LCF) ** 2 for x, w in zip(xs, hw)],
                              xa, xb)
    KB = integrate_wet(xs, Az, xa, xb) / V
    LCB = integrate_wet(xs, [a * x for a, x in zip(A, xs)], xa, xb) / V
    BM = I_T / V
    BM_L = I_L / V
    disp = rho * V
    Bwl = 2.0 * max(hw) if hw else 0.0
    Cb = V / (Lwl * Bwl * T) if Lwl > 0 and Bwl > 0 and T > 0 else 0.0
    f_min = min_deck_edge(h) - T
    return {
        "T": T, "V": V, "disp": disp, "Aw": Aw, "Cb": Cb,
        "KB": KB, "BM": BM, "KM": KB + BM, "BM_L": BM_L,
        "LCB": LCB, "LCF": LCF, "Lwl": Lwl, "Bwl": Bwl,
        "TPC": rho * Aw / 100.0,
        # trim lever is LWL, not the station span (raked ends); 0 while
        # the hull is not yet afloat along any length (solver probes).
        # Standard MCT uses GM_L = KB + BM_L - KG; without a KG argument
        # only the BM_L approximation is available (overestimates).
        "MCT": (disp * (KB + BM_L - kg if kg is not None else BM_L)
                / (100.0 * Lwl)) if Lwl > 1e-9 else 0.0,
        "MCT_uses_GM_L": kg is not None,
        "freeboard_min": f_min,
    }


def find_draft(h, mass, rho):
    """Draft (even keel) at which displacement equals mass. Open-boat cap:
    the deck edge must stay above water. Raises if the mass cannot be floated
    or the solver cannot converge."""
    if not (math.isfinite(mass) and mass > 0):
        raise ValueError(f"mass must be finite and positive (got {mass!r})")
    if not (math.isfinite(rho) and rho > 0):
        raise ValueError(f"rho must be finite and positive (got {rho!r})")
    cap = min_deck_edge(h) - DECK_CLEARANCE
    top = at_draft(h, cap, rho)
    if top is None or top["disp"] < mass:
        max_mass = top["disp"] if top else 0.0
        raise ValueError(
            f"{mass:.0f} kg exceeds displacement at deck-edge immersion "
            f"({max_mass:.0f} kg) - reduce load or increase hull volume")
    floor = at_draft(h, 1e-4, rho)
    if floor is not None and floor["disp"] > mass:
        raise ValueError(
            f"target mass {mass:.4f} kg is below the solver floor "
            f"(draft 0.0001 m already displaces {floor['disp']:.4f} kg)")
    lo, hi = 1e-4, cap
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        r = at_draft(h, mid, rho)
        if r is None or r["disp"] < mass:
            lo = mid
        else:
            hi = mid
    T = 0.5 * (lo + hi)
    res = at_draft(h, T, rho)
    if res is None or abs(res["disp"] - mass) > max(0.005 * mass, 1e-6):
        got = res["disp"] if res else 0.0
        raise ValueError(
            f"draft solver did not converge: at T={T:.4f} m displacement is "
            f"{got:.3f} kg vs target {mass:.3f} kg")
    return T


def parse_drafts(spec):
    if ":" in spec:
        a, b, c = (float(x) for x in spec.split(":"))
        if not all(math.isfinite(v) for v in (a, b, c)):
            raise ValueError("range needs finite values")
        if b == 0:
            raise ValueError("range step must be nonzero")
        if c == a:
            return [round(a, 4)]
        if (c - a) * b < 0:
            raise ValueError("range step points away from the end value")
        if abs((c - a) / b) > 1000:
            raise ValueError("range would make more than 1000 samples")
        # sample by MULTIPLICATION: adding a tiny step to a large value can
        # fail to advance in float (an infinite loop); indexing cannot
        n = int(round((c - a) / b))
        out = [round(a + i * b, 6) for i in range(n + 1)]
        if abs(out[-1] - c) > 1e-6:   # the END value is always included, even
            out.append(round(c, 6))   # when the step does not divide the range
        return out
    vals = [float(x) for x in spec.split(",")]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("list needs finite values")
    return vals


def fmt_row(r):
    return ("| {T:.2f} | {V:.3f} | {disp:.0f} | {Aw:.2f} | {Cb:.2f} | {KB:.3f} | "
            "{BM:.3f} | {KM:.3f} | {TPC:.1f} | {LCB:.2f} | {freeboard_min:.2f} |").format(**r)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("--drafts", default=None, help="start:step:end or comma list of drafts [m]")
    ap.add_argument("--mass", type=float, default=None,
                    help="find the draft for this displacement mass [kg]")
    ap.add_argument("--rho", type=float, default=RHO_FRESH, help="water density [kg/m3]")
    ap.add_argument("--kg", type=float, default=None,
                    help="optional VCG: MCT then uses the GM_L lever "
                         "(standard); without it MCT is the BM_L approximation")
    ap.add_argument("--json", default=None, help="write results as JSON")
    args = ap.parse_args()

    if not (math.isfinite(args.rho) and args.rho > 0):
        print("ERROR: --rho must be a finite positive density")
        sys.exit(1)
    h = load_hull(args.hull)
    print(f"# Hydrostatics - {h.get('name', args.hull)}  (rho={args.rho:.0f} kg/m3)")
    print()

    if args.mass is not None:
        try:
            T = find_draft(h, args.mass, args.rho)
        except ValueError as e:
            print(f"ERROR: {e}")
            sys.exit(1)
        r = at_draft(h, T, args.rho, kg=args.kg)
        print(f"Displacement {args.mass:.0f} kg -> draft T = {T:.3f} m")
        print(f"Freeboard (min, at deck edge): {r['freeboard_min']:.3f} m")
        print(f"Cb = {r['Cb']:.2f}, KB = {r['KB']:.3f} m, KM = {r['KM']:.3f} m, "
              f"Aw = {r['Aw']:.2f} m2, TPC = {r['TPC']:.1f} kg/cm, "
              f"MCT = {r['MCT']:.0f} kg*m/cm"
              + (" (GM_L)" if r["MCT_uses_GM_L"] else
                 " (BM_L approx - pass --kg for GM_L)"))
        if args.json:
            with open(args.json, "w") as f:
                json.dump(r, f, indent=2)
        return

    if args.drafts:
        drafts = parse_drafts(args.drafts)
    else:
        cap = min(deck_edge(s["pts"])[1] for s in h["stations"])
        drafts = [round(cap * f, 3) for f in (0.15, 0.3, 0.45, 0.6, 0.75)]
        print(f"(no --drafts given; using {drafts})")
        print()

    print("| T m | V m3 | disp kg | Aw m2 | Cb | KB m | BM m | KM m | TPC kg/cm | LCB m | f_min m |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    rows = []
    for T in drafts:
        r = at_draft(h, T, args.rho, kg=args.kg)
        if r:
            rows.append(r)
            print(fmt_row(r))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=2)
    print()
    print("f_min = minimum freeboard to the deck edge (water enters here first).")
    print("LCB measured from station 0; positive = forward of x=0.")


if __name__ == "__main__":
    main()
