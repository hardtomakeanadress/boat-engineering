#!/usr/bin/env python3
"""Transverse stability: GM and the GZ curve (righting arms at large heel).

Usage:
  python3 stability.py hull.json --kg 0.42 --mass 1100 [--rho 1000]
      [--angles 0:5:90] [--json out.json]

Method (see references/stability.md):
- Displaced volume must stay constant as the boat heels; for each heel angle
  the waterline height is solved (heave only, no trim) so that
  V(theta) = mass / rho.
- KN(theta) = ybar*cos(theta) + zbar*sin(theta), where (ybar, zbar) is the
  centroid of the displaced volume in the body frame (y from centerline,
  z from baseline).
- GZ(theta) = KN(theta) - KG*sin(theta).

Limits: even keel, no trim, no dynamic effects. Deck-edge immersion is
reported - beyond it an open boat is taking water and the GZ curve is only
formal.
"""

import argparse
import json
import math
import sys

from hull import deck_edge, full_section, immersed, load_hull, simpson_uniform
from hydrostatics import at_draft, find_draft

from hull import RHO_FRESH
import hydrostatics

BRACKET_PAD = 0.5      # waterline search bracket beyond the hull extent [m]

def volume_at(h, theta, c):
    """Displaced volume, body-frame centroid, and min deck-edge margin at
    heel theta with waterline constant c."""
    xs = [s["x"] for s in h["stations"]]
    A, Ay, Az = [], [], []
    margins = []
    ct, st = math.cos(theta), math.sin(theta)
    for s in h["stations"]:
        poly = full_section(s["pts"])
        a, yb, zb = immersed(poly, theta, c)
        A.append(a)
        Ay.append(a * yb)
        Az.append(a * zb)
        ye, ze = deck_edge(s["pts"])
        m_sb = ze * ct - ye * st - c   # starboard deck edge
        m_pt = ze * ct + ye * st - c   # port deck edge
        margins.append(min(m_sb, m_pt))
    V = simpson_uniform(xs, A)
    if V <= 1e-12:
        return 0.0, 0.0, 0.0, min(margins)
    return (V, simpson_uniform(xs, Ay) / V, simpson_uniform(xs, Az) / V,
            min(margins))


def solve_waterline(h, theta, V_target, c_lo, c_hi):
    v_lo = volume_at(h, theta, c_lo)[0]
    v_hi = volume_at(h, theta, c_hi)[0]
    if not (v_lo <= V_target <= v_hi):
        raise ValueError(f"waterline bracket invalid: volume range "
                         f"{v_lo:.3f}..{v_hi:.3f} m3 does not contain target "
                         f"{V_target:.3f} m3")
    lo, hi = c_lo, c_hi
    for _ in range(90):
        mid = 0.5 * (lo + hi)
        V, _y, _z, _m = volume_at(h, theta, mid)
        if V < V_target:
            lo = mid
        else:
            hi = mid
    c = 0.5 * (lo + hi)
    return c, volume_at(h, theta, c)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("--kg", type=float, required=True, help="VCG (KG) above baseline [m]")
    ap.add_argument("--mass", type=float, required=True, help="loading displacement [kg]")
    ap.add_argument("--rho", type=float, default=RHO_FRESH)
    ap.add_argument("--angles", default="0:5:90", help="start:step:end degrees")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    if not (math.isfinite(args.kg) and args.kg >= 0):
        print("ERROR: --kg must be a finite VCG >= 0 m")
        sys.exit(1)
    if not (math.isfinite(args.mass) and args.mass > 0):
        print("ERROR: --mass must be a finite positive displacement")
        sys.exit(1)
    if not (math.isfinite(args.rho) and args.rho > 0):
        print("ERROR: --rho must be a finite positive density")
        sys.exit(1)
    h = load_hull(args.hull)
    try:                 # shared range parser: the END angle is always
        #                  # included (it used to be dropped when the step
        #                  # did not divide the range - the areas then
        #                  # described a different curve than requested)
        angles = sorted(math.radians(v) for v in
                        hydrostatics.parse_drafts(args.angles))
    except ValueError as e:
        print(f"ERROR: --angles {e}")
        sys.exit(1)

    try:
        T = find_draft(h, args.mass, args.rho)
    except ValueError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    r0 = at_draft(h, T, args.rho)
    KM, KB, BM = r0["KM"], r0["KB"], r0["BM"]
    GM = KM - args.kg

    print(f"# Stability - {h.get('name', args.hull)}")
    print(f"Loading mass {args.mass:.0f} kg, KG = {args.kg:.3f} m, "
          f"draft T = {T:.3f} m (even keel)")
    print(f"KB = {KB:.3f} m, BM = {BM:.3f} m, KM = {KM:.3f} m, "
          f"GM = KM - KG = {GM:.3f} m")
    if GM <= 0:
        print("WARNING: GM <= 0 - the boat does not float upright. Lower the")
        print("center of gravity or add beam/volume before anything else.")
    print()

    pts_all = [(y, z) for s in h["stations"] for (y, z) in s["pts"]]
    pts_all += [(-y, z) for (y, z) in pts_all]  # mirrored side: bracket must cover full sections
    V_target = args.mass / args.rho

    rows = []
    deck_angle = None
    for th in angles:
        ct, st = math.cos(th), math.sin(th)
        proj = [z * ct - y * st for (y, z) in pts_all]
        c_lo, c_hi = min(proj) - BRACKET_PAD, max(proj) + BRACKET_PAD
        if volume_at(h, th, c_hi)[0] < V_target:
            print(f"WARNING: at {math.degrees(th):.0f} deg heel the hull cannot "
                  "displace the loading mass - increase hull volume.")
            break
        c, (V, yb, zb, marg) = solve_waterline(h, th, V_target, c_lo, c_hi)
        if abs(V - V_target) / V_target > 0.005:
            print(f"WARNING: waterline solve did not converge at "
                  f"{math.degrees(th):.0f} deg (residual "
                  f"{abs(V - V_target) / V_target:.1%}) - results unreliable.")
            break
        KN = yb * math.cos(th) + zb * math.sin(th)
        GZ = KN - args.kg * math.sin(th)
        # keep the PREVIOUS sample before appending: checking rows[-1] after
        # the append read the flooded row itself and the interpolation never
        # ran (40 deg reported where the crossing is 33 deg)
        prev_marg, prev_deg = ((rows[-1]["deck_margin"], rows[-1]["theta_deg"])
                               if rows else (None, None))
        rows.append({"theta_deg": round(math.degrees(th), 1),
                     "GZ": round(GZ, 4), "KN": round(KN, 4),
                     "deck_margin": round(marg, 4),
                     "waterline_z": round(c, 4)})
        if marg <= 0 and deck_angle is None:
            # refine to the actual CROSSING: the first flooded SAMPLE is an
            # overestimate by up to one step (linear interp of the margin)
            if prev_marg is not None and prev_marg > 0:
                t = prev_marg / (prev_marg - marg)
                deck_angle = prev_deg + t * (math.degrees(th) - prev_deg)
            else:
                deck_angle = math.degrees(th)

    print("| heel deg | GZ m | KN m | deck edge margin m |")
    print("|---|---|---|---|")
    for r in rows:
        note = "  <-- deck edge awash (open boat takes water)" if r["deck_margin"] <= 0 else ""
        print(f"| {r['theta_deg']:.1f} | {r['GZ']:.3f} | {r['KN']:.3f} | "
              f"{r['deck_margin']:.3f}{note} |")
    print()

    # summary criteria values (acceptance numbers depend on the chosen standard,
    # see references/standards-capacity.md)
    def gz_at(deg):
        """GZ at an arbitrary angle, by linear interpolation over the samples."""
        for i in range(len(rows) - 1):
            d0, d1 = rows[i]["theta_deg"], rows[i + 1]["theta_deg"]
            if d0 <= deg <= d1:
                if d1 == d0:
                    return rows[i]["GZ"]
                t = (deg - d0) / (d1 - d0)
                return rows[i]["GZ"] + t * (rows[i + 1]["GZ"] - rows[i]["GZ"])
        return None

    def area_under(deg_from, deg_to):
        """Integral of GZ over [deg_from, deg_to]; None unless the samples
        cover the whole interval (endpoints are interpolated)."""
        if (not rows or rows[0]["theta_deg"] > deg_from + 1e-9
                or rows[-1]["theta_deg"] < deg_to - 1e-9):
            return None
        xs_ = ([deg_from]
               + [r["theta_deg"] for r in rows if deg_from < r["theta_deg"] < deg_to]
               + [deg_to])
        gz = [gz_at(d) for d in xs_]
        s = 0.0
        for i in range(len(xs_) - 1):
            s += (0.5 * (math.radians(xs_[i + 1]) - math.radians(xs_[i]))
                  * (gz[i] + gz[i + 1]))
        return s

    a30 = area_under(0, 30)
    a40 = area_under(0, 40)
    max_row = max(rows, key=lambda r: r["GZ"]) if rows else None
    vanish = None
    if rows and rows[0]["GZ"] > -1e-6:
        for i in range(1, len(rows)):
            if rows[i - 1]["GZ"] > 1e-9 >= rows[i]["GZ"]:
                d0, d1 = rows[i - 1], rows[i]
                t = d0["GZ"] / (d0["GZ"] - d1["GZ"])
                vanish = d0["theta_deg"] + t * (d1["theta_deg"] - d0["theta_deg"])
                break

    if a30 is not None:
        print(f"Area under GZ curve 0-30 deg: {a30:.4f} m*rad")
    else:
        print("Area 0-30 deg: n/a (heel samples do not cover 0-30 deg)")
    if a40 is not None:
        print(f"Area under GZ curve 0-40 deg: {a40:.4f} m*rad")
    else:
        print("Area 0-40 deg: n/a (heel samples do not cover 0-40 deg)")
    if max_row:
        print(f"Max GZ: {max_row['GZ']:.3f} m at {max_row['theta_deg']:.0f} deg")
    starts_upright = bool(rows) and abs(rows[0]["theta_deg"]) < 1e-9
    if starts_upright and rows[0]["GZ"] < -1e-6:
        print("NOTE: GZ is already negative upright - no range of stability.")
    elif vanish is not None:
        print(f"GZ crosses zero at: {vanish:.1f} deg")
    if deck_angle is not None:
        print(f"Deck edge immersed at: {deck_angle:.0f} deg "
              f"(open boat: this is the practical limit)")
    print()
    print("Compare these against the acceptance criteria of your chosen")
    print("standard (references/standards-capacity.md). Assumptions: even keel,")
    print("no trim, static, hull as modeled - not a certified stability booklet.")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"GM": GM, "KB": KB, "BM": BM, "KM": KM, "T": T,
                       "KG": args.kg, "mass": args.mass, "curve": rows,
                       "deck_immersion_deg": deck_angle}, f, indent=2)


if __name__ == "__main__":
    main()
