#!/usr/bin/env python3
"""Weight and loading budget with equilibrium draft, KG and capacity check.

Usage:
  python3 budget.py hull.json budget.json [--rho 1000] [--json out.json]

budget.json:
{
  "name": "demo launch, day trip load",
  "person_kg": 75,
  "items": [
    {"name": "Hull structure", "kg": 240, "count": 1, "x": 2.6, "z": 0.35,
     "class": "lightship"},
    {"name": "Crew", "kg": 75, "count": 4, "x": 2.9, "z": 0.65, "class": "payload"}
  ]
}
x = longitudinal position from the station-0 datum [m]
z = height above the baseline (keel) [m]
class: "lightship" or "payload" (optional, default payload)
"""

import argparse
import json
import math
import sys

from hull import deck_edge, load_hull
from hydrostatics import at_draft, find_draft

from hull import (DECK_CLEARANCE, FB_MIN, FREEBOARD_CAP,
                   GM_MIN, PERSON_KG, RHO_FRESH, min_deck_edge)

TRIM_WARN_F = 0.02    # longitudinal imbalance warning, fraction of LOA


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("budget")
    ap.add_argument("--rho", type=float, default=RHO_FRESH)
    ap.add_argument("--freeboard", type=float, default=FREEBOARD_CAP,
                    help="freeboard kept at the capacity limit [m]; 0 = absolute "
                         "physical cap (water at the deck edge)")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    def fail(msg):
        print(f"ERROR: {msg}")
        sys.exit(1)

    h = load_hull(args.hull)
    with open(args.budget) as f:
        b = json.load(f)
    person_kg = float(b.get("person_kg", PERSON_KG))  # ISO test weight
    if not (math.isfinite(person_kg) and person_kg > 0):
        fail(f"person_kg must be finite and positive (got {person_kg!r})")

    total = mx = mz = 0.0
    by_class = {}
    rows = []
    for it in b["items"]:
        name = it.get("name", "?")
        kg = float(it["kg"])
        if not (math.isfinite(kg) and kg >= 0):
            fail(f"item {name!r}: kg must be finite and >= 0")
        n_raw = float(it.get("count", 1))
        if not (math.isfinite(n_raw) and n_raw >= 1 and n_raw == int(n_raw)):
            fail(f"item {name!r}: count must be a whole number >= 1")
        n = int(n_raw)
        x, z = float(it["x"]), float(it["z"])
        if not (math.isfinite(x) and math.isfinite(z)):
            fail(f"item {name!r}: x and z must be finite")
        m = kg * n
        total += m
        mx += m * x
        mz += m * z
        k = it.get("class", "payload")
        by_class[k] = by_class.get(k, 0.0) + m
        rows.append((name, n, kg, m, x, z))
    if total <= 0:
        fail("empty budget (total mass is zero)")

    LCG = mx / total
    KG = mz / total
    print(f"# Weight budget - {b.get('name', args.budget)}")
    print()
    print("| item | n | kg ea | kg | x m | z m |")
    print("|---|---|---|---|---|---|")
    for name, n, ke, m, x, z in rows:
        print(f"| {name} | {n} | {ke:.0f} | {m:.0f} | {x:.2f} | {z:.2f} |")
    print(f"| **TOTAL** |  |  | **{total:.0f}** | **{LCG:.2f}** | **{KG:.2f}** |")
    print()
    for k, v in sorted(by_class.items()):
        print(f"{k}: {v:.0f} kg")
    print(f"LCG = {LCG:.3f} m from datum | KG = {KG:.3f} m above baseline")
    print()

    try:
        T = find_draft(h, total, args.rho)
    except ValueError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
    r = at_draft(h, T, args.rho)
    GM = r["KM"] - KG
    print(f"At {total:.0f} kg: draft T = {T:.3f} m, freeboard (min, deck edge) = "
          f"{r['freeboard_min']:.3f} m")
    dlong = LCG - r["LCB"]
    print(f"LCG = {LCG:.3f} m vs LCB = {r['LCB']:.3f} m -> longitudinal imbalance "
          f"{dlong:+.3f} m ({total * dlong:+.0f} kg*m)")
    L_hull = h["stations"][-1]["x"] - h["stations"][0]["x"]
    if abs(dlong) > TRIM_WARN_F * L_hull:   # 2%: past this the even-keel
        #                                     # numbers visibly lie
        print(f"WARNING: weight is {abs(dlong):.2f} m off the centre of buoyancy - "
              f"the boat will trim by the {'bow' if dlong > 0 else 'stern'}. "
              "Results assume even keel; move weight or add trim (v2).")
    print(f"KB = {r['KB']:.3f} m, BM = {r['BM']:.3f} m, KM = {r['KM']:.3f} m, "
          f"GM = KM - KG = {GM:.3f} m")
    print(f"TPC = {r['TPC']:.1f} kg/cm, Cb = {r['Cb']:.2f}")
    if GM <= 0:
        print("WARNING: GM <= 0 - not stable upright. Lower weight / raise KM.")
    elif GM < GM_MIN:
        print(f"NOTE: GM < {GM_MIN} m - tender; verify against your standard's "
              "criteria (references/standards-capacity.md).")
    if r["freeboard_min"] < FB_MIN:
        print(f"NOTE: freeboard under {FB_MIN:.2f} m - check downflooding "
              "requirements.")
    print()

    # capacity limit: freeboard margin kept at the deck edge (open boat);
    # --freeboard 0 gives the absolute physical cap (water at the gunwale)
    if not (math.isfinite(args.rho) and args.rho > 0):
        fail("--rho must be a finite positive density")
    if not (math.isfinite(args.freeboard) and args.freeboard >= 0):
        fail("--freeboard must be a finite value >= 0")
    capT = min_deck_edge(h) - max(args.freeboard, DECK_CLEARANCE)
    capr = at_draft(h, capT, args.rho) if capT > 0 else None
    capd = capr["disp"] if capr else None
    persons = None
    light = by_class.get("lightship")
    if capd is None:
        print(f"Capacity: n/a at {args.freeboard:.2f} m freeboard - the hull is "
              "too shallow for this margin; reduce --freeboard.")
    elif light is not None:
        max_payload = capd - light
        if max_payload < 0:
            print(f"OVERLOAD: lightship {light:.0f} kg already exceeds the "
                  f"{capd:.0f} kg this hull floats with {args.freeboard:.2f} m "
                  "freeboard kept - no payload left")
            persons = 0
        else:
            # snap roundoff: exact-boundary payloads (e.g. 3400 kg) land a
            # float hair short and would lose a whole person
            persons = int(math.floor(max_payload / person_kg + 1e-9))
            print(f"Lightship {light:.0f} kg -> max payload with "
                  f"{args.freeboard:.2f} m freeboard kept: {max_payload:.0f} kg")
            print(f"= {persons} persons at {person_kg:.0f} kg (gear counts "
                  "against this too)")
        print("This is a buoyancy limit only - small-craft standards additionally "
              "cap persons/freeboard by rule (references/standards-capacity.md).")
    else:
        print(f"Capacity cap (freeboard {args.freeboard:.2f} m kept): {capd:.0f} kg "
              f"({capd - total:.0f} kg above current load). "
              f"Tag lightship items with \"class\": \"lightship\" for a payload figure.")

    if args.json:
        with open(args.json, "w") as f:
            json.dump({"items": rows, "total": total, "LCG": LCG, "KG": KG,
                       "draft": T, "GM": GM, "hydro": r, "cap_disp": capd,
                       "by_class": by_class, "persons": persons}, f, indent=2)


if __name__ == "__main__":
    main()
