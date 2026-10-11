#!/usr/bin/env python3
"""Bill of materials: compute the actual quantities to buy for the boat.

The census of 50+ real boat plans says a materials list (ply schedule with
sheet counts, timber, tape, epoxy, fasteners, hardware) appears in most
complete plan sets - this generates it from the hull model and the design
constants, never hand-entered.

Usage:
  python3 materials.py hull.json [--people N] [--roof-n N] [--ply-mm M]
      [--sheet 2440x1220] [--json out.json] [-o bom.md]

Prints a markdown table; -o also writes it; --json dumps machine-readable.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import designer as D
import hull as H
from hull import deck_edge


def _girth_m(pts):
    """Developed length of the section polyline from keel to deck edge."""
    side = [(p[0], p[1]) for p in pts[:-1]]
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(side, side[1:]))


def build(h, people=2, roof_n=0, ply_mm=6, sheet_mm=(2440, 1220),
          gear_kg=300.0, flat_bottom=False):
    sts = h["stations"]
    xs = [st["x"] for st in sts]
    loa = xs[-1] - xs[0]
    beam = 2.0 * max(deck_edge(st["pts"])[0] for st in sts)
    depth = max(deck_edge(st["pts"])[1] for st in sts)
    ply = ply_mm / 1000.0
    lay = D.layout(h, {"roof_n": roof_n, "people": people})
    cs = D.canopy_struct(h, lay["roof"]) if lay.get("roof") else None

    # ---- hull skin area: girth integrated (trapezoid) along the stations,
    # plus 15% for curvature/offcuts (sheet goods never nest perfectly).
    # Station pts are HALF sections (centreline -> deck edge), so the
    # developed girth integrates one side: double it for both sides.
    girths = [2.0 * _girth_m(st["pts"]) for st in sts]
    area = float(np.trapezoid(girths, xs))
    area_ply = area * 1.15

    # ---- decks, end plates, benches, risers, posts
    fore = lay["decks"]["fore"]
    deck_a = (fore[1] - fore[0]) * beam * 0.75      # tapered deck plate
    plate_a = 2.0 * (beam * depth * 0.85)           # transom + bow plates
    bench_a = sum(b["w"] * 2.0 * b["half"] for b in lay["benches"])
    riser_a = sum(2 * (b["w"] + 2 * D.RISER_EXT) * D.RISER_T
                  for b in lay["benches"])
    post_a = sum((D.POST_SZ ** 2) for b in lay["benches"]
                 if 2 * b["half"] > D.POST_SPAN)
    ply_a = area_ply + deck_a + plate_a + bench_a + riser_a + post_a
    n_sheets = math.ceil(ply_a / (sheet_mm[0] * sheet_mm[1] / 1e6 * 0.92))

    # ---- timber (solid wood: gunwale boards + risers/cleats as battens)
    g_len = 2.0 * (loa - 2.0 * 0.03)                # side rail length
    rail_20x45 = 2.0 * g_len * 2.0 + 2.0 * beam     # in+out per side + ends
    batten_60x30 = sum(2.0 * (b["w"] + 2 * D.RISER_EXT)
                       for b in lay["benches"])     # risers as cleats

    # ---- glass + epoxy: fillet/tape at the seams + 2 sealing coats
    tape_m = 2.0 * g_len + 2.0 * depth * 2.0        # gunwale + stem/stern
    coat_a = ply_a * 2.0                            # both faces, 2 coats
    epoxy_kg = coat_a * 0.6                         # documented 0.6 kg/m2
    tape_w = 0.05                                   # 50 mm tape
    tape_m2 = tape_m * tape_w
    epoxy_kg += tape_m2 * 1.2                       # wet-out + fillet resin

    # ---- hardware: computed from the design, not guessed
    motor = "24 V outboard, 3 kW class (19.7-29.5 kg from datasheets)"
    batt_ah = 100 if people <= 3 else 200
    battery = (f"LiFePO4 24 V {batt_ah} Ah "
               f"({'22-29' if batt_ah == 100 else '40-46'} kg from datasheets)")
    fasten = []
    fasten.append((f"bench/riser screws 5x50 SS: "
                   f"{len(lay['benches']) * 2 * 6} pcs"))
    fasten.append("gunwale: M8 SS through-bolts + nuts, 2 per X-support foot"
                  if cs else "gunwale: glue + 5x50 screws, 6 per side rail")
    if cs:
        fasten.append(f"X-support foot plates: {len(cs['feet'])} pcs "
                      "120x80x6 + backing plates 160x100")
        fasten.append(f"panel clamps: {len(cs['clamps'])} sets")
        fasten.append(f"solar: {len(cs['panels'])} x 450 W panels, "
                      "frame tube 40x40x4 aluminium")

    # flotation + capacity: the small-boat plan essentials (census)
    support = float(people) * 85.0 + max(0.0, gear_kg)
    foam_l = int(-(-support / 0.95))         # 0.95 kg/L net foam buoyancy
    hp, fac = D.max_hp(loa, deck_edge(sts[0]["pts"])[0],
                       flat_bottom=flat_bottom)
    rows = [
        ("Plywood", f"{ply_mm} mm marine/exterior (sheet "
                    f"{sheet_mm[0]:.0f}x{sheet_mm[1]:.0f})",
         f"{n_sheets} sheets"),
        ("  - hull skin", f"developed girth {sum(girths)/len(girths):.2f} m "
                          f"avg x {loa:.2f} m", f"{area_ply:.1f} m2 incl. 15%"),
        ("  - decks/plates/benches", "foredeck, transom+bow, seats, risers",
         f"{ply_a - area_ply:.1f} m2"),
        ("Gunwale timber", "20x45 inwale+outwale, both sides + end caps",
         f"{rail_20x45:.1f} lin.m (~{rail_20x45 * 0.020 * 0.045 * 420:.1f} kg "
         f"at 420 kg/m3)"),
        ("Cleats/battens", "60x30 risers under benches",
         f"{batten_60x30:.1f} lin.m"),
        ("Glass tape", "50 mm biaxial, gunwale + stem/stern seams",
         f"{tape_m:.1f} lin.m ({tape_m2:.2f} m2)"),
        ("Epoxy + hardener", "2 sealing coats both faces + tape wet-out",
         f"~{epoxy_kg:.1f} kg mixed"),
        ("Paint", "2 top coats, above the waterline", "~1.2 L per coat"),
        ("Propulsion", motor, "by design"),
        ("Battery", battery, "by design"),
        ("Wiring", "50 mm2 tinned copper + 125 A fuse at battery", "4-6 m"),
        ("Flotation", "closed-cell foam blocks or pour foam, under the "
                      "benches + bow locker (USCG-style swamped support)",
         f"~{foam_l} L ({support:.0f} kg kept afloat)"),
        ("Capacity plate", "USCG 33 CFR 183.53 method: factor "
                           f"{fac} (L x transom width, ft)",
         f"max {hp:g} hp petrol; {people} persons / see the stats for kg"),
    ]
    return {"rows": rows, "fasteners": fasten, "ply_a": ply_a,
            "n_sheets": n_sheets, "epoxy_kg": epoxy_kg, "tape_m": tape_m,
            "rail_m": rail_20x45, "loa": loa, "beam": beam}


def markdown(h, res):
    L = [f"# Materials list - {h.get('name', 'hull')}",
         "", f"LOA {res['loa']:.2f} m, beam {res['beam']:.2f} m "
         "(computed from the model; buy with the usual 10% margin)", "",
         "| item | what | quantity |", "|---|---|---|"]
    for a, b, cq in res["rows"]:
        L.append(f"| {a} | {b} | {cq} |")
    if res["fasteners"]:
        L += ["", "**Fasteners & hardware**", ""]
        L += [f"- {f}" for f in res["fasteners"]]
    L += ["", "Quantities are computed from the hull model and the design",
          "constants; sheet counts assume 92% sheet yield."]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("hull")
    ap.add_argument("--people", type=int, default=2)
    ap.add_argument("--roof-n", type=int, default=0)
    ap.add_argument("--ply-mm", type=int, default=6)
    ap.add_argument("--gear", type=float, default=300.0,
                    help="boat + gear mass [kg] (the flotation basis)")
    ap.add_argument("--flat-bottom", action="store_true",
                    help="flat-bottom hard-chine hull (power table step)")
    ap.add_argument("--sheet", default="2440x1220")
    ap.add_argument("-o", "--out")
    ap.add_argument("--json")
    args = ap.parse_args()
    sw, sh = (float(v) for v in args.sheet.lower().split("x"))
    h = H.load_hull(args.hull)
    res = build(h, people=max(1, args.people), roof_n=max(0, args.roof_n),
                ply_mm=max(3, min(18, args.ply_mm)), sheet_mm=(sw, sh),
                gear_kg=max(0.0, args.gear),
                flat_bottom=bool(args.flat_bottom))
    md = markdown(h, res)
    print(md)
    if args.out:
        with open(args.out, "w") as f:
            f.write(md + "\n")
        print(f"\nwrote {args.out}")
    if args.json:
        json.dump({"n_sheets": res["n_sheets"], "ply_a": res["ply_a"],
                   "epoxy_kg": res["epoxy_kg"], "tape_m": res["tape_m"]},
                  open(args.json, "w"), indent=1)


if __name__ == "__main__":
    main()
