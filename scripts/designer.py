#!/usr/bin/env python3
"""Easy boat builder: local web UI with sliders over the whole pipeline.

Usage:
  python3 designer.py [--port 8765] [--outdir ~/boat]
  then open http://127.0.0.1:8765/

Slider changes regenerate the hull (param_hull), re-render a 3D preview
(view3d) and recompute live numbers (hydrostatics, in-process). Buttons
export the full package into the output directory:
  - dimensioned lines plan (PDF + DXF + offsets CSV) via linesplan + drafting
  - watertight scaled STL for 3D printing (export_stl)

Only stdlib + the packages already used by the skill. Binds 127.0.0.1 only.
"""

import argparse
import json
import logging
import math
import numpy as np
import os
import subprocess
import sys
import threading
import time
import traceback
import warnings
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

warnings.filterwarnings("ignore", message="overflow encountered")

LOG = None


def setup_log(outdir):
    global LOG
    from logging.handlers import RotatingFileHandler
    LOG = logging.getLogger("builder")
    LOG.setLevel(logging.INFO)
    LOG.handlers.clear()
    # small, bounded log: errors and rare events only (~200 KB, 1 rotation)
    fh = RotatingFileHandler(os.path.join(outdir, "builder.log"),
                             maxBytes=200_000, backupCount=1)
    fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
    LOG.addHandler(fh)


def log(msg):
    if LOG:
        LOG.info(msg)
    print(msg, flush=True)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import hull          # noqa: E402
import hydrostatics  # noqa: E402

DRAFT_PY = os.path.normpath(os.path.join(
    HERE, "..", "..", "engineering-drafting", "scripts", "draft.py"))


def _full_pts(h):
    pts = [(y, z) for s in h["stations"] for (y, z) in s["pts"]]
    return pts + [(-y, z) for (y, z) in pts]


def _gz_at(h, theta, mass, kg, pts):
    """Exact GZ (m) and deck-edge margin at one heel angle (radians), from the
    tested stability core. None if the solve is not possible."""
    import stability as st
    V_target = mass / 1000.0
    ct, sn = math.cos(theta), math.sin(theta)
    proj = [z * ct - y * sn for (y, z) in pts]
    c_lo, c_hi = min(proj) - 0.5, max(proj) + 0.5
    if st.volume_at(h, theta, c_hi)[0] < V_target:
        return None
    try:
        _c, (_V, yb, zb, marg) = st.solve_waterline(h, theta, V_target, c_lo, c_hi)
    except ValueError:
        return None
    return yb * ct + zb * sn - kg * sn, marg


def _gz_rows(h, mass, kg, pts, ang_step=4, ang_max=72):
    """GZ curve samples (deg, GZ, deck-edge margin) from the tested stability core."""
    rows = []
    for a in range(0, ang_max + 1, ang_step):
        r = _gz_at(h, math.radians(a), mass, kg, pts)
        if r is None:
            break
        rows.append((float(a), r[0], r[1]))
    return rows


USCG_HP_TABLE = ((35, 3.0), (39, 5.0), (42, 7.5), (45, 10.0), (52, 15.0))


def max_hp(loa_m, transom_half_beam_m, flat_bottom=False, remote=False):
    """USCG-style rated maximum power, 33 CFR 183.53: Factor = LOA(ft) x
    max transom width (ft), nearest integer.  Factor <= 52 uses the table
    (flat-bottom hard-chine boats drop one increment); above that the
    formulas apply, rounded UP to the next multiple of 5."""
    f = round((loa_m / 0.3048) * (2.0 * transom_half_beam_m / 0.3048))
    if f <= 52:
        hp = 3.0
        for lim, v in USCG_HP_TABLE:
            if f <= lim:
                hp = v
                break
        if flat_bottom:
            hp = max(3.0, hp - 2.5)
        return hp, f
    if remote:
        hp = 2.0 * f - 90.0
    elif flat_bottom:
        hp = 0.5 * f - 15.0
    else:
        hp = 0.8 * f - 25.0
    import math as _m
    hp = max(0.0, _m.ceil(hp / 5.0) * 5.0)
    return hp, f


def persons_estimate(h, mass, loa, bwl, depth, n_sel, kg=None,
                     tpc_kgcm=None, z_crew=None):
    """ISO-style offset-load estimate: all crew (85 kg each) seated on ONE
    side at 0.30*BWL from the centreline, low VCG (0.5*depth). Heel from the
    GZ curve; limits = Cat C offset-load formula phi(O) = 11.5 + (24-LH)^3/520
    and >= 0.10 m residual freeboard. An estimate, not a certified rating.
    kg = the ship's combined KG; when omitted it falls back to the blanket
    0.5*depth convention (the builder passes the real KG incl. the roof).
    tpc_kgcm = tonnes-per-cm: with it, each extra person's mass is added to
    the displacement (freeboard sinks by dm/TPC) instead of free crew."""
    if kg is None:
        kg = KG_HULL_F * depth
    y_p = HEEL_LEVER_F * bwl
    phi_limit = 11.5 + (24.0 - loa) ** 3 / 520.0
    pts = _full_pts(h)
    rows = _gz_rows(h, mass, kg, pts)
    if len(rows) < 3:
        return {"error": "no GZ data (hull cannot float this load)"}

    cache = {}

    def gz(deg):
        key = round(deg, 3)
        if key not in cache:
            cache[key] = _gz_at(h, math.radians(deg), mass, kg, pts)
        return cache[key]

    for (a, gg, mm) in rows:   # seed the cache with the coarse samples
        cache[round(float(a), 3)] = (gg, mm)

    gz_cache = {}

    def gz_n(deg, n):
        """GZ for the CANDIDATE load case (n aboard): each crew count gets
        its own displacement and KG - reusing the base curve made the max
        crew depend on which count was selected (4/3/2/2 for the same
        boat).  z_crew (crew VCG) keeps the moments right when given."""
        key = (n, round(deg, 3))
        if key not in gz_cache:
            mass_n = mass + (n - n_sel) * PERSON_KG
            if z_crew is not None and mass_n > 0:
                moment = mass * kg + (n - n_sel) * PERSON_KG * z_crew
                kg_n = max(0.05, moment / mass_n)
            else:
                kg_n = kg
            gz_cache[key] = _gz_at(h, math.radians(deg), mass_n, kg_n, pts)
        return gz_cache[key]

    def heel_for(n):
        mass_n = mass + (n - n_sel) * PERSON_KG
        w = (n * PERSON_KG * y_p) / mass_n   # heeling arm at zero heel

        def f(deg):
            r = gz_n(deg, n)
            if r is None:
                return None
            return r[0] - w * math.cos(math.radians(deg))

        f0 = f(0.0)
        if f0 is None:
            return None, None
        if f0 >= 0:
            r = gz_n(0.0, n)
            return 0.0, r[1]
        lo = 0.0
        for a in range(HEEL_STEP, HEEL_MAX + 1, HEEL_STEP):
            fa = f(float(a))
            if fa is None:
                return None, None
            if fa >= 0:
                hi = float(a)
                for _ in range(6):
                    mid = 0.5 * (lo + hi)
                    fm = f(mid)
                    if fm is None:
                        return None, None
                    if fm < 0:
                        lo = mid
                    else:
                        hi = mid
                th = 0.5 * (lo + hi)
                r = gz_n(th, n)
                return (None, None) if r is None else (th, r[1])
            lo = float(a)
        return None, None

    max_n = 0
    for n in range(1, 31):
        th, marg = heel_for(n)
        dT = 0.0    # the candidate's own displacement already includes the
        #             added crew (gz_n) - no separate TPC correction
        if th is None or th > phi_limit or marg - dT < FB_MIN:
            break
        max_n = n
    flood = None            # heel where the DECK EDGE goes under: the real
    prev_a = prev_m = None  # limit in a wave, unlike the paperwork test
    for (a_, _g, m_) in rows:
        if m_ <= 0 and prev_a is not None and prev_m is not None:
            t = prev_m / (prev_m - m_)
            flood = prev_a + t * (a_ - prev_a)   # rows are ALREADY degrees
            break
        if m_ > 0:
            prev_a, prev_m = a_, m_
        elif prev_a is None:
            break               # flooded from the start
    out = {"max_persons": max_n, "heel_limit_deg": round(phi_limit, 1),
           "flood_deg": None if flood is None else round(flood, 0)}
    if n_sel:
        th, marg = heel_for(max(int(n_sel), 1))
        if th is None:
            reason = ("no balance below 72 deg: this load plus the roof is "
                      "too tender - crew on one side would capsize it")
        elif th > phi_limit:
            reason = f"heel {th:.0f} deg over the {phi_limit:.0f} deg limit"
        elif marg < FB_MIN:
            reason = f"freeboard left {marg:.2f} m under {FB_MIN:.2f} m"
        else:
            reason = "ok"
        out["sel"] = {"n": int(n_sel),
                      "heel_deg": None if th is None else round(th, 1),
                      "margin_m": None if th is None else round(marg, 2),
                      "reason": reason,
                      "passes": None if th is None
                      else bool(th <= phi_limit and marg >= FB_MIN)}
    return out

def layout(h, p):
    """Seating + solar roof arrangement (v1). Seating = CROSS benches
    (thwarts) spanning the boat, the way small open boats are actually
    fitted (0.75 m of bench per seated person, ISO 14946). The AFT bench
    is the HELM seat: with a tiller electric outboard (motor on the
    transom) you steer from there, tiller in reach. Roof canopy centred
    over the middle (gunwale to gunwale, `roof_h` clear height over the
    deck edge: sitting headroom by default, walking when raised). The
    canopy is driven by the
    NUMBER of standard 450 W panels requested: rows (1-2) from the boat
    width, columns from the length; the roof length follows the array + a
    small frame margin."""
    xs = [s["x"] for s in h["stations"]]
    loa = xs[-1] - xs[0]
    ed = [max(s["pts"], key=lambda q: (round(q[0], 9), q[1]))
          for s in h["stations"]]
    ye = [q[0] for q in ed]
    ze = [q[1] for q in ed]
    beam = 2.0 * max(ye)

    def ixv(vals, x):
        if x <= xs[0]:
            return vals[0]
        for i in range(len(xs) - 1):
            if x <= xs[i + 1]:
                t = (x - xs[i]) / (xs[i + 1] - xs[i])
                return vals[i] + t * (vals[i + 1] - vals[i])
        return vals[-1]

    # cross benches: aft one = helm (nearest the tiller/motor), the rest
    # evenly spaced forward; plank 0.32 m fore-aft, spanning wall-to-wall
    # AT SEAT HEIGHT (a thwart bolts to the hull sides + a centre post)
    def y_z(pts, z):
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

    z_sole = min(s["pts"][0][1] for s in h["stations"]) + SOLE_OFF
    # seat height over the floor: the seat_h slider (default 0.38, lower
    # than the ergonomic 0.45 - lower bench = lower crew CG = more stable);
    # on shallow hulls it must come down so the bench stays under the sheer
    seat_h = float(p.get("seat_h", SEAT_H) or SEAT_H)
    z_seat = max(min(z_sole + seat_h, SEAT_F * min(ze)), z_sole + SEAT_MIN)
    yz_seat = [y_z(s["pts"], z_seat) for s in h["stations"]]
    x_helm = max(0.5, BENCH_AFT_F * loa)
    x_end = BENCH_FORE_F * loa
    # ROWING STATION (plan census): oars cross the gunwale at/near the
    # widest beam, and the rowing thwart sits AT that station amidships -
    # real plans never make the transom bench the only seat, you cannot
    # row from there (no oar spread, no trim). Widest station within the
    # middle half so boxy hulls (max beam at the transom) still row well.
    mid_lo, mid_hi = xs[0] + 0.35 * loa, xs[0] + 0.65 * loa
    _cand = [i for i in range(len(xs)) if mid_lo <= xs[i] <= mid_hi]
    x_oar = (xs[max(_cand, key=lambda i: ye[i])] if _cand
             else xs[0] + 0.5 * loa)
    # bench COUNT follows the people you seat and what one bench seats on
    # THIS hull: a canoe with one aboard gets ONE thwart, not three
    try:
        n_people = max(0, int(float(p.get("people", 2) or 0)))
    except (TypeError, ValueError):
        n_people = 2
    half_mid = ixv(yz_seat, 0.5 * (x_helm + x_end))
    s_est = max(1, int((2.0 * half_mid) // PERS_W))     # seats per bench
    n_rows = max(1, min(4, -(-max(1, n_people) // s_est))) if n_people > 0 \
        else 1
    # SOLO: one thwart, at the rowing station (not the transom: rowing +
    # trim). Elsewhere: helm bench aft (tiller) + rowing thwart at the
    # oarlocks + any further benches spread toward the bow.
    if n_rows == 1:
        xcs = [x_oar]
    else:
        xcs = [x_helm, x_oar]
        xcs += [x_oar + k * (x_end - x_oar) / (n_rows - 2)
                for k in range(1, n_rows - 1)]      # n_rows >= 3
    benches = []
    for xc in xcs:
        half = max(BENCH_HALF_MIN, ixv(yz_seat, xc) - 0.01)
        benches.append({"xc": xc, "w": SEAT_W, "half": half,
                        "seated": int((2.0 * half) // PERS_W),   # a narrow bench seats nobody
                        "helm": abs(xc - x_helm) < 1e-9})
    roof = None
    roof_note = None
    try:
        n_req = int(float(p.get("roof_n", 0) or 0))
    except (TypeError, ValueError):
        n_req = 0
    if n_req > 0:
        w = beam - ROOF_INSET            # canopy spans cap to cap: the two
                              # gunwale boards + 10 mm clear each side. Two
                              # rows of 450 W need 2.29 m, so boats wider
                              # than ~2.35 m get 2 across

        def _fit(p_x, p_y):
            """Panels p_x (along the boat) x p_y (across) at PAN_GAP pitch:
            (rows_that_fit, cols_that_fit, n_fit, roof length) or None.
            The roof may run almost the whole boat (50 mm ends) - NO 75%
            cap: on a 4 m boat a nose-to-tail pair of 1.76 m panels fits."""
            pit_x, pit_y = p_x + PAN_GAP, p_y + PAN_GAP
            rf = int((w + PAN_GAP) // pit_y)
            if rf < 1:
                return None
            cf = max(1, int((loa - 2 * END_M - 2 * ROOF_M + PAN_GAP) // pit_x))
            ru = min(rf, n_req)
            cc = min(-(-n_req // ru), cf)
            return {"rows_fit": rf, "cols_fit": cf, "rows": ru, "cols": cc,
                    "n_fit": min(n_req, ru * cc),
                    "len": (cc - 1) * pit_x + p_x + 2 * ROOF_M,
                    "pit_x": pit_x, "pit_y": pit_y, "p_x": p_x, "p_y": p_y}

        fl = _fit(PAN_L, PAN_W)          # normal: long side along the boat
        fp = _fit(PAN_W, PAN_L)          # rotated: long side across the boat
        best, orient = fl, "land"
        if fp and (fl is None or fp["n_fit"] > fl["n_fit"]):
            best, orient = fp, "port"    # rotate only when it fits MORE
        if best is None:
            roof_note = ("boat too narrow for a 450 W panel (needs "
                         f">= {PAN_W + ROOF_INSET:.2f} m beam)")
        else:
            length, n_fit = best["len"], best["n_fit"]
            # a short roof sits centred; a long one slides toward the STERN
            # end - both bounds keep END_M at bow and stern (length is
            # guaranteed <= loa - 2*END_M by cols_fit)
            x0 = min(max(END_M, (loa - length) / 2.0), loa - length - END_M)
            zr = ze[len(ze) // 2] + float(p.get("roof_h", CLEAR_H)
                                           or CLEAR_H)
            roof = {"x0": x0, "x1": x0 + length, "w": w, "z": zr,
                    "area": length * w, "n_req": n_req, "n_fit": n_fit,
                    "rows": best["rows"], "cols": best["cols"],
                    "orient": orient, "fits": n_fit == n_req}
            if not roof["fits"]:
                # concrete recommendation: how much longer / wider?
                cols_need = -(-n_req // best["rows_fit"])
                loa_need = math.ceil(
                    (cols_need * best["pit_x"] + 2 * ROOF_M
                     - PAN_GAP + 2 * END_M) * 10) / 10.0
                rows_need = -(-n_req // best["cols_fit"])
                beam_need = math.ceil(
                    (rows_need * best["pit_y"] - PAN_GAP + ROOF_INSET) * 20) / 20.0
                rot = " (panels rotated, long side across)" \
                    if orient == "port" else ""
                roof_note = (
                    f"solar roof: {n_fit} of {n_req} panels fit{rot} - a "
                    f"450 W panel is 1.76 x 1.13 m. To fit all {n_req}: "
                    f"make the boat about {loa_need:.1f} m long (now "
                    f"{loa:.1f} m) or {beam_need:.2f} m wide (now "
                    f"{beam:.2f} m)")
    # closed ends: foredeck over the bow + a short stern deck (common on
    # dinghies/skiffs - storage/flotation below; keeps the ends from being
    # open voids). Ranges in hull x; each must span at least two stations.
    fore_x0 = loa - min(DECK_FORE_MAX, DECK_FORE_F * loa)
    aft_x1 = min(DECK_AFT_MAX, DECK_AFT_F * loa)
    if len(xs) > 1:
        fore_x0 = min(fore_x0, xs[-2] - 0.01)
        aft_x1 = max(aft_x1, xs[1] + 0.01)
    decks = {"fore": [max(fore_x0, 0.5 * loa), loa]}
    # NO stern deck: the aft end stays open for the outboard (the transom
    # plate closes the hull) - 'aft' used to cover the motor clamp area
    return {"loa": loa, "benches": benches, "roof": roof,
            "roof_note": roof_note, "z_sole": z_sole, "z_seat": z_seat,
            "decks": decks, "x_oarlock": x_oar}


def canopy_struct(h, r):
    """Solar canopy members - ONE definition used by both the fabrication
    detail sheet and the 3D view. Follows common small-boat practice:
    light aluminium frame tube (40x30x4 perimeter, 40x20x2 cross members),
    4 legs taken down to the GUNWALE CAP in opposed pairs (bimini-style),
    through-bolted with foot plates and backing plates under the inwale,
    fore/aft braces at ~45-60 deg to the gunwale against racking, panels
    clamped to the frame with a 25 mm air gap.
    All values in metres, hull frame. z counts from the hull baseline."""
    xs = [s["x"] for s in h["stations"]]
    ed = [max(s["pts"], key=lambda q: (round(q[0], 9), q[1]))
          for s in h["stations"]]
    ye = [q[0] for q in ed]
    ze = [q[1] for q in ed]

    def ix(vals, x):
        if x <= xs[0]:
            return vals[0]
        for i in range(len(xs) - 1):
            if x <= xs[i + 1]:
                t = (x - xs[i]) / (xs[i + 1] - xs[i])
                return vals[i] + t * (vals[i + 1] - vals[i])
        return vals[-1]

    x0, x1 = r["x0"], r["x1"]
    hw = r["w"] / 2.0
    zu = r["z"]                       # frame underside = deck edge + roof_h
    z_rail_top = zu + RAIL_SZ
    z_pan0 = z_rail_top + GAP
    z_pan1 = z_pan0 + PAN_T
    # panel grid decided in layout() from the boat: rows across (1-2) by
    # width, columns by length; the module dims stay the standard 450 W size
    npx = max(int(r.get("cols", 0)), 0)
    npy = max(int(r.get("rows", 0)), 0)
    # build the panels that DO fit even when more were requested; r["fits"]
    # only drives the "N of M fit" note, not what gets modelled
    fit = npx > 0 and npy > 0
    orient = r.get("orient", "land")
    # panels run p_x along the boat, p_y across; "port" = long side across
    p_x, p_y = (PAN_L, PAN_W) if orient == "land" else (PAN_W, PAN_L)
    pit_x, pit_y = p_x + PAN_GAP, p_y + PAN_GAP
    arr_l = (npx - 1) * pit_x + p_x                # module pitch, not edge
    arr_w = (npy - 1) * pit_y + p_y                # to edge - like a real array
    ax0 = x0 + ((x1 - x0) - arr_l) / 2.0           # array centred on the roof
    ay0 = -arr_w / 2.0
    panels = ([{"x0": ax0 + (k // npy) * pit_x,
                "y0": ay0 + (k % npy) * pit_y}
               for k in range(int(r.get("n_fit", 0)))] if fit else [])
    rails, ends, cross, feet, clamps = [], [], [], [], []
    for sg in (1.0, -1.0):            # perimeter sides (fore-aft)
        rails.append({"y": sg * (hw - RAIL_SY / 2), "x0": x0, "x1": x1,
                      "sy": RAIL_SY, "sz": RAIL_SZ, "z0": zu, "z1": z_rail_top})
    for xe in (x0 + RAIL_SY / 2, x1 - RAIL_SY / 2):   # perimeter ends
        ends.append({"x": xe, "y0": -(hw - RAIL_SY), "y1": hw - RAIL_SY,
                     "sx": RAIL_SY, "z0": zu, "z1": z_rail_top})
    xb = ([ax0 + i * pit_x for i in range(npx)]
          + [ax0 + (npx - 1) * pit_x + p_x]) if fit else []
    for xv in xb:                     # crossbars under every panel edge line
        cross.append({"x": xv - CR_SX / 2,
                      "y0": -(hw - RAIL_SY / 2),   # buried INTO the rails:
                      "y1": hw - RAIL_SY / 2,      # ends flush on their
                      "sx": CR_SX, "sz": CR_SZ,
                      "z0": z_rail_top - CR_SZ,
                      "z1": z_rail_top - GLUE_SINK})
    # X supports: on each side TWO diagonals crossing mid-span - foot at one
    # roof end to the frame corner at the OTHER end.  Pure truss: every
    # member in compression/tension, no verticals, no separate braces.
    # Feet take push AND pull -> through-bolts + backing plates both needed.
    xlegs = []
    yt_x = hw - RAIL_SY / 2          # frame rail centreline (top attach y)
    for xp in (x0, x1):
        yb = ix(ye, xp) - GUNW_T     # foot centre ON the gunwale cap
        # the sheer rises toward the bow and the plate is FLAT: its bottom
        # must sit at the LOWEST deck edge over its own x span, else the
        # up-sheer corner floats above the cap (visible at any zoom)
        zb = min(ix(ze, xp - FOOT_PL[0] / 2.0), ix(ze, xp + FOOT_PL[0] / 2.0),
                 ix(ze, xp))
        xo = x1 if xp == x0 else x0  # the OTHER roof end (the crossing)
        for sg in (1.0, -1.0):
            feet.append({"x": xp, "y": sg * yb,
                         "z": zb - GLUE_SINK,
                         "sx": FOOT_PL[0], "sy": FOOT_PL[1],
                         "sz": FOOT_PL[2]})
            y_top = yt_x + (XLEG_SX / 4.0 if xp == x0 else -XLEG_SX / 4.0)
            # the two diagonals per side land on OPPOSITE half-faces of the
            # rail: landing both on the centreline put their side faces in
            # one plane (coplanar overlap pairs at the crossing)
            xlegs.append({"x0": xp, "y0": sg * yb,
                          "z0": zb - GLUE_SINK,    # buried in the foot plate
                          "x1": xo, "y1": sg * y_top,
                          "z1": zu + GLUE_SINK,    # buried in the rail
                          "sx": XLEG_SX, "sy": XLEG_SX})
    yb_lines = ([ay0 + j * pit_y for j in range(npy)]
                + [ay0 + (npy - 1) * pit_y + p_y])
    for xv in xb:                     # clamps where panel edges meet crossbars
        for yv in yb_lines:
            clamps.append({"x": xv, "y": yv,
                           "z0": z_rail_top - GLUE_SINK,   # buried into the
                           "z1": z_pan0 + GLUE_SINK,       # rail + panel
                           "sx": CLAMP_SX, "sy": CLAMP_SY})
    # --- weight + VCG of the whole canopy system (feeds the builder's KG):
    # aluminium 2700 kg/m3 with the real tube sections; framed 450 W panels
    # at 22 kg each.  CG = mass-weighted mean height.
    al = AL_RHO
    def _kgm(sx, sy, t):        # hollow rect tube, kg/m, from the section
        return (sx * sy - (sx - 2 * t) * (sy - 2 * t)) * AL_RHO
    kgm_rail = _kgm(RAIL_SY, RAIL_SZ, TUBE_T_RAIL)
    kgm_cross = _kgm(CR_SX, CR_SZ, TUBE_T_CROSS)
    kgm_xleg = _kgm(XLEG_SX, XLEG_SX, TUBE_T_XLEG)   # 40x40x4 X diagonals
    span = 2.0 * (hw - RAIL_SY)
    xleg_lens = [math.hypot(b["x1"] - b["x0"], b["y1"] - b["y0"],
                            b["z1"] - b["z0"]) for b in xlegs]
    m_rail = kgm_rail * (2 * (x1 - x0) + 2 * span)
    m_cross = kgm_cross * len(cross) * span
    m_xlegs = kgm_xleg * sum(xleg_lens)
    m_feet = len(feet) * (FOOT_PL[0] * FOOT_PL[1] * FOOT_PL[2]
                          + BACK_PL[0] * BACK_PL[1] * BACK_PL[2]) * al
    m_clamps = len(clamps) * CLAMP_KG
    m_panels = len(panels) * PAN_KG
    z_xlegs = (sum(l * (lg["z0"] + lg["z1"]) / 2.0
                   for l, lg in zip(xleg_lens, xlegs)) / sum(xleg_lens)
               if m_xlegs else zu)
    items = [(m_rail, zu + RAIL_SZ / 2),
             (m_cross, z_rail_top - CR_SZ / 2),
             (m_xlegs, z_xlegs),
             (m_feet, sum(f["z"] for f in feet) / len(feet)),
             (m_clamps, (z_rail_top + z_pan0) / 2.0),
             (m_panels, z_pan0 + PAN_T / 2.0)]
    mass_kg = sum(m for m, _ in items)
    z_cg = sum(m * z for m, z in items) / mass_kg
    return {"x0": x0, "x1": x1, "hw": hw, "zu": zu, "gap": GAP,
            "z_pan0": z_pan0, "z_pan1": z_pan1, "pan_t": PAN_T,
            "pan_l": PAN_L, "pan_w": PAN_W, "pan_x": p_x, "pan_y": p_y,
            "orient": orient, "panels": panels, "fit": fit,
            "npx": npx, "npy": npy, "clear": zu - ix(ze, (x0 + x1) / 2.0),
            "mass_kg": mass_kg, "z_cg": z_cg,
            "kwp": len(panels) * PAN_KWP,
            "kwh_day": len(panels) * PAN_KWP * PSH_JULY * PV_DERATE,
            "rails": rails, "ends": ends, "cross": cross, "xlegs": xlegs,
            "xleg_sx": XLEG_SX, "feet": feet, "clamps": clamps}


def quick_preview(h, draft, path, lay=None):
    """Fast live preview: engineering style 2D views - side, top and front
    (outline) - drawn from the same curves the plans use. draft=None means
    the load does not float: geometry is shown without a waterline."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from hull import keel_crossing, waterline_halfwidth

    n = len(h["stations"])
    xs = [s["x"] for s in h["stations"]]
    zk = [s["pts"][0][1] for s in h["stations"]]
    edge = [max(s["pts"], key=lambda p: (round(p[0], 9), p[1]))
            for s in h["stations"]]
    ze = [p[1] for p in edge]
    ye = [p[0] for p in edge]
    wl = draft if draft is not None else 0.0
    hw = [waterline_halfwidth(s["pts"], wl) for s in h["stations"]]

    def wl_spans(w):
        spans, start = [], None
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

    def ye_at(x):
        for i in range(n - 1):
            if xs[i] <= x <= xs[i + 1]:
                t = (x - xs[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] != xs[i] else 0.0
                return ye[i] + t * (ye[i + 1] - ye[i])
        return ye[-1]

    def ze_at(x):
        for i in range(n - 1):
            if xs[i] <= x <= xs[i + 1]:
                t = (x - xs[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] != xs[i] else 0.0
                return ze[i] + t * (ze[i + 1] - ze[i])
        return ze[-1]

    fig = plt.figure(figsize=(12.8, 4.4), dpi=100)
    try:
        ax = fig.add_axes([0.045, 0.13, 0.26, 0.74])
        ax.fill_between(xs, zk, ze, color="#b9c6d4", alpha=0.95)
        ax.plot(xs, ze, "k-", lw=1.4, label="deck edge")
        ax.plot(xs, zk, "k-", lw=1.4, label="bottom")
        if draft is not None:
            for xa, xb in wl_spans(draft):
                ax.plot([xa, xb], [draft, draft], "b-", lw=1.3)
            ax.plot([], [], "b-", lw=1.3, label=f"waterline @ {draft:.2f} m")
        if lay and lay.get("roof"):
            r = lay["roof"]
            ax.plot([r["x0"], r["x1"]], [r["z"], r["z"]], color="#3a5f8a",
                    lw=3.0)
            ax.plot([r["x0"], r["x0"]], [ze_at(r["x0"]), r["z"]],
                    color="#3a5f8a", lw=1.5)
            ax.plot([r["x1"], r["x1"]], [ze_at(r["x1"]), r["z"]],
                    color="#3a5f8a", lw=1.5)
            ax.plot([], [], color="#3a5f8a", lw=2.5, label="solar roof")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.tick_params(labelsize=7)
        ax.set_title("Side view", fontsize=10)
        ax.legend(fontsize=7, loc="upper left")
        ax.text(0.97, 0.94, "bow →", transform=ax.transAxes, fontsize=8,
                color="#556", ha="right")

        ax2 = fig.add_axes([0.37, 0.13, 0.26, 0.74])
        ax2.fill_between(xs, ye, [-y for y in ye], color="#b9c6d4", alpha=0.55)
        ax2.plot(xs, ye, "k-", lw=1.4, label="deck edge")
        ax2.plot(xs, [-y for y in ye], "k-", lw=1.4)
        if lay and lay.get("benches"):
            for bn in lay["benches"]:
                hb = bn["half"]
                xa, xb2 = bn["xc"] - bn["w"] / 2.0, bn["xc"] + bn["w"] / 2.0
                ax2.fill([xa, xb2, xb2, xa], [hb, hb, -hb, -hb],
                         color="#a98a63", edgecolor="#6b5638", lw=1.0,
                         alpha=0.9, zorder=4)
                for si in range(1, bn["seated"]):
                    ys = -hb + si * 2.0 * hb / bn["seated"]
                    ax2.plot([xa, xb2], [ys, ys], color="#6b5638", lw=0.7,
                             zorder=5)
            ax2.plot([], [], color="#a98a63", lw=6, alpha=0.9,
                     label="cross benches (aft = helm)")
        if lay and lay.get("decks"):
            for rng in lay["decks"].values():      # closed ends: decks
                xd = [x for x in xs if rng[0] - 1e-9 <= x <= rng[1] + 1e-9]
                if len(xd) >= 2:
                    yd = [max(0.02, ye_at(x) - 0.02) for x in xd]
                    poly = (list(zip(xd, yd))
                            + [(x, -y) for (x, y) in zip(xd[::-1], yd[::-1])])
                    ax2.fill([q[0] for q in poly], [q[1] for q in poly],
                             color="#c8bfae", alpha=0.9, zorder=4)
            ax2.plot([], [], color="#c8bfae", lw=6, alpha=0.9,
                     label="fore + stern deck")
        # gunwale rails capping the sheer
        ax2.plot(xs, [y + 0.03 for y in ye], color="#7a5c33", lw=1.8,
                 zorder=6)
        ax2.plot(xs, [-y - 0.03 for y in ye], color="#7a5c33", lw=1.8,
                 zorder=6)
        ax2.plot([xs[0], xs[0]], [-ye[0] - 0.03, ye[0] + 0.03],
                 color="#7a5c33", lw=1.8, zorder=6)
        ax2.plot([], [], color="#7a5c33", lw=2, label="gunwale")
        if lay and lay.get("roof"):
            r = lay["roof"]
            hw2 = r["w"] / 2.0
            ax2.fill([r["x0"], r["x1"], r["x1"], r["x0"]],
                     [hw2, hw2, -hw2, -hw2], facecolor="#cfe3ff",
                     edgecolor="#4a6fa5", alpha=0.45, linewidth=1.2, zorder=5)
            try:
                cs_r = canopy_struct(h, r)
            except Exception:
                cs_r = None
            if cs_r:
                for pn in cs_r["panels"]:      # standard 450 W modules
                    px0, py0 = pn["x0"], pn["y0"]
                    ax2.fill([px0, px0 + cs_r["pan_x"],
                              px0 + cs_r["pan_x"], px0],
                             [py0, py0, py0 + cs_r["pan_y"],
                              py0 + cs_r["pan_y"]],
                             facecolor="#24406e", edgecolor="k",
                             alpha=0.85, linewidth=0.8, zorder=6)
            ax2.plot([], [], color="#4a6fa5", lw=2, label="solar roof")
        if draft is not None:
            for xa, xb in wl_spans(draft):
                seg = ([(xa, 0.0)]
                       + [(xs[i], hw[i]) for i in range(n)
                          if xa - 1e-9 <= xs[i] <= xb + 1e-9]
                       + [(xb, 0.0)])
                ax2.plot([p[0] for p in seg], [p[1] for p in seg], "b-", lw=1.3)
                ax2.plot([p[0] for p in seg], [-p[1] for p in seg], "b-", lw=1.3)
            ax2.plot([], [], "b-", lw=1.3, label=f"waterline @ {draft:.2f} m")
        ax2.set_aspect("equal")
        ax2.grid(alpha=0.3)
        ax2.tick_params(labelsize=7)
        ax2.set_title("Top view", fontsize=10)
        ax2.legend(fontsize=7, loc="lower right")

        ax3 = fig.add_axes([0.70, 0.13, 0.26, 0.74])
        zmax = max(ze)
        zs = [zmax * k / 60.0 for k in range(61)]
        env = [max((waterline_halfwidth(st["pts"], z) for st in h["stations"]),
                   default=0.0) for z in zs]
        ax3.fill_betweenx(zs, env, [-w for w in env], color="#b9c6d4", alpha=0.9)
        ax3.plot(env, zs, "k-", lw=1.4)
        ax3.plot([-w for w in env], zs, "k-", lw=1.4)
        ax3.set_title("Front view (outline)", fontsize=10)
        if draft is not None:
            ax3.axhline(draft, color="b", lw=1.3,
                        label=f"waterline @ {draft:.2f} m")
        ax3.set_aspect("equal")
        ax3.grid(alpha=0.3)
        ax3.tick_params(labelsize=7)
        ax3.legend(fontsize=7, loc="upper right")

        fig.savefig(path, facecolor="white")
    finally:
        plt.close(fig)


DEFAULTS = {
    "loa": 5.2, "beam": 1.9, "depth": 0.78, "sheer_rise": 0.08, "rocker": 0.07,
    "transom_f": 0.80, "mid_f": 1.00, "bow_f": 0.33, "bottom_f": 0.0,
    "stations": 13, "levels": 8, "gear_kg": 300, "people": 2, "roof_n": 1,
    "ply_mm": 6, "sheet": 0, "stl_roof": 1, "roof_h": 1.00, "seat_h": 0.38,
}

BOUNDS = {
    "loa": (3.0, 12.0), "beam": (0.8, 4.0), "depth": (0.2, 2.0),
    "sheer_rise": (0.0, 0.5), "rocker": (0.0, 0.5), "bottom_f": (0.0, 1.0),
    "bow_f": (0.1, 1.1), "transom_f": (0.3, 1.1), "mid_f": (0.6, 1.3),
    "gear_kg": (50.0, 5000.0), "roof_n": (0.0, 8.0), "ply_mm": (3.0, 18.0),
    "sheet": (0.0, 1.0), "stl_roof": (0.0, 1.0), "roof_h": (0.60, 1.80),
    "seat_h": (0.25, 0.52),
}

# solar canopy constants: standard 450 W module + frame margin around the array
PAN_L, PAN_W, PAN_T, PAN_KG = 1.762, 1.134, 0.030, 24.5   # m, kg
# panel mass from real 1762x1134x30 mm 450 W datasheets: Dualsun
# FLASH 450 = 24.5 kg, Sunpal SP450M-48H = 25 kg, Trina Vertex S+
# TSM-NEG9R.28 = 21 kg (verified online 2026-10)
PAN_GAP = 0.020                      # mounting air gap between modules
PAN_PITCH_X = PAN_L + PAN_GAP
PAN_PITCH_Y = PAN_W + PAN_GAP
ROOF_M = 0.10                        # frame ends beyond the outer panel
# --- one place for every design dimension the boat parts share; the
# --- renderers and prints import these instead of repeating numbers
GUNW_T = 0.020        # gunwale cap board thickness (inwale = outwale) [m]
GUNW_H = 0.045        # gunwale cap height [m]
END_M = 0.05          # roof keeps >= this from the bow/stern [m]
CLEAR_H = 1.00        # default canopy clear height over the deck edge [m];
                      # the roof_h slider overrides it per boat (1.00 =
                      # comfortable SITTING for a 1.8 m person, 1.80 = walk
                      # upright).  Nobody stands in a 4 m jon, so the low
                      # default keeps windage + CG down.
SEAT_H = 0.38         # default seat height over the floor [m] - lower
                      # than the ergonomic 0.45: crew CG drops with the
                      # bench, and in a 4 m jon stability wins; the seat_h
                      # slider overrides per boat
SEAT_F = 0.80         # seat max as a fraction of the sheer height
SEAT_MIN = 0.12       # seat minimum above the sole [m]
SEAT_W = 0.32         # seat plank fore-aft width [m]
PERS_W = 0.75         # shoulder width per seated person [m]
GLUE_SINK = 0.003     # mating faces bury this far (real m) INSIDE the
                      # surface they bond to - 1/8 of a print layer at 1:24.
                      # Faces exactly ON each other are COPLANAR and every
                      # viewer + slicer renders that as holes/z-fighting;
                      # buried faces read flush and the slicer unions them
RISER_CLEAR = -GLUE_SINK   # riser sinks into the laminate wall
                      # (it used to sit 20 mm off the wall - floating
                      # blocks in every exported model)
POST_SPAN = 1.6       # bench span that needs a centre post [m]
FOOT_PL = (0.12, 0.08, 0.006)     # canopy foot plate (x, y, t) [m]
BACK_PL = (0.16, 0.10, 0.006)     # backing plate under the inwale [m]
XLEG_SX, TUBE_T_XLEG = 0.040, 0.004   # X support tube 40x40x4 [m]
RISER_T = 0.06        # bench riser cleat 60x60 [m]
RISER_TOP = 0.04      # riser top sits this far below the seat top [m]
RISER_EXT = 0.03      # riser sticks this far past each bench end [m]
POST_SZ = 0.10        # bench centre post 100x100 [m]
PERSON_KG = 85.0      # crew mass allowance [kg] (ISO 14946-ish)
KG_HULL_F = 0.5       # hull+gear VCG as a fraction of depth (estimate)
HEEL_LEVER_F = 0.30   # crew heel lever as a fraction of BWL (ISO-style)
FB_MIN = 0.10         # ISO offset-load residual freeboard limit [m]
FREEBOARD_CAP = 0.15  # freeboard at which the capacity cap is read [m]
FRESH_RHO = 1000.0    # fresh water density [kg/m3]
FROUDE_K = 2.43       # displacement hull speed constant [kn/m^0.5]
KN_KMH = 1.852        # knots -> km/h
KW_PER_T_HULL = 3.7     # VETUS E-Line (24 V, 3 kW class) at hull speed,
                        # from the published consumption tables [kW/t]
# those E-Line figures came off mid-fullness displacement hulls (Cb~0.48).
# Wave-making resistance at the hull-speed hump (Fr~0.3-0.4) rises steeply
# with fullness - "it pays to reduce the prismatic coefficient as far as
# possible there" (Ship Design: Resistance & Propulsion Vol.1); a fuller
# bow "shoulders the water" and costs power for the SAME hull speed.
# Power therefore scales with (Cb/CB_REF)^2 as a documented estimate.
CB_REF = 0.48            # block coefficient of the reference E-Line data
FORM_EXP = 2.0           # wave+form resistance vs fullness (hump regime)
PSH_JULY = 4.5        # peak sun hours July (Cluj ~46 deg N)
PV_DERATE = 0.85      # PV system derate: wiring, heat, MPPT
RAIL_SY, RAIL_SZ = 0.030, 0.040   # 40x30x4 perimeter tube (40 tall) [m]
CR_SX, CR_SZ = 0.040, 0.020       # 40x20x2 crossbar [m]
LEG_SX, LEG_SY = 0.040, 0.030     # 40x30x3 leg tube [m]
GAP = 0.025                       # panel air gap over the frame [m]
AL_RHO = 2700.0       # aluminium density [kg/m3]
CLAMP_KG = 0.05       # an M6 mid-clamp with its bolts [kg]
CLAMP_SX, CLAMP_SY = 0.05, 0.03   # clamp body [m]
SOLE_OFF = 0.05       # floor sole above the inner keel [m]
ROOF_INSET = 2 * GUNW_T + 0.02    # hull beam minus this = canopy width [m]
CREW_UP = 0.30        # seated crew VCG above the seat surface [m]
HEEL_MAX, HEEL_STEP = 72, 4   # heel search: no balance below HEEL_MAX deg
                              # counts as over the limit (solver grid)
PAN_KWP = 0.45        # one 450 W module [kWp]
CRUISE_F = 0.65       # comfortable cruise as a fraction of hull speed
# Cruise power follows the SAME cubic scaling as the hull-speed anchor
# (resistance ~ v^3): 3.7 * 0.65^3 ~ 1.02 kW/t.  The old 0.64 kW/t figure
# came from a VETUS table labelled GENTLE SAILING, not cruise - using it
# as cruise understated the draw by ~40% (codex audit 2026-10-11)
KW_PER_T_CRUISE = KW_PER_T_HULL * CRUISE_F ** 3
TUBE_T_RAIL, TUBE_T_CROSS, TUBE_T_LEG = 0.004, 0.002, 0.003  # wall [m]
BENCH_AFT_F = 0.12    # aft (helm) bench centre, fraction of loa
BENCH_FORE_F = 0.80   # forward bench centre, fraction of loa
BENCH_4_SPLIT = 6.0   # loa above which 4 benches instead of 3 [m]
BENCH_HALF_MIN = 0.15  # minimum bench half-length [m]
DECK_FORE_F = 0.15    # foredeck length as a fraction of loa
DECK_FORE_MAX = 0.60  # ... but never longer than this [m]
DECK_AFT_F = 0.08     # stern deck length as a fraction of loa
DECK_AFT_MAX = 0.30   # ... but never longer than this [m]


def sanitize(params):
    """Validate and normalize a request: finite numbers, sane bounds, integral
    counts. Raises ValueError with a clear message on anything bad."""
    p = dict(DEFAULTS)
    for k in DEFAULTS:
        if k not in params:
            continue
        try:
            v = float(params[k])
        except (TypeError, ValueError):
            raise ValueError(f"bad value for {k}: {params[k]!r}")
        if not math.isfinite(v):
            raise ValueError(f"bad value for {k}: must be a finite number")
        p[k] = v
    for k, (lo, hi) in BOUNDS.items():
        if not (lo <= p[k] <= hi):
            raise ValueError(f"{k}={p[k]:g} out of range [{lo:g}, {hi:g}]")
    if p["depth"] <= p["rocker"] + 0.05:
        raise ValueError("depth must be at least 0.05 m more than the bottom rise")
    p["stations"] = int(max(5, min(41, p["stations"])))
    p["levels"] = int(max(4, min(24, p["levels"])))
    p["people"] = int(max(0, min(30, p["people"])))
    p["roof_n"] = int(max(0, min(8, p["roof_n"])))
    p["ply_mm"] = int(max(3, min(18, p["ply_mm"])))
    p["sheet"] = 1 if p["sheet"] >= 0.5 else 0
    p["stl_roof"] = 1 if p.get("stl_roof", 1) >= 0.5 else 0
    return p

STATE = {"outdir": None, "draft": 0.25, "params": dict(DEFAULTS)}
LOCK = threading.Lock()


def run(cmd, timeout=180):
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def hull_path():
    return os.path.join(STATE["outdir"], "hull.json")


def plans_path(*parts):
    """Drawing outputs live in <outdir>/plans - they are the deliverable
    the user keeps, and they must never mix with the machine files
    (hull.json, boat.stl, logs) or be overwritten by mistake."""
    d = os.path.join(STATE["outdir"], "plans")
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, *parts)


def fnum(params, key):
    return f'{float(params[key]):g}'


def _hist_path():
    return os.path.join(STATE["outdir"], "param_history.json")


def hist_load():
    with LOCK:      # the writer rewrites the file - read under the same lock
        try:
            items = json.load(open(_hist_path()))
            return hist_load_fixed(items) if isinstance(items, list) else []
        except (OSError, ValueError):
            return []


def hist_load_fixed(items):
    """Legacy history entries lack the newer keys (roof_h/seat_h/stl_roof):
    fill them from DEFAULTS so restoring an old boat re-applies defaults
    instead of silently inheriting whatever the current boat uses."""
    out = []
    for it in items:
        if isinstance(it, dict) and isinstance(it.get("params"), dict):
            it = dict(it, params={**DEFAULTS, **it["params"]})
        out.append(it)
    return out


def hist_snapshot(prev, label):
    """Keep the last few boats around so a preset click can never destroy
    the user's boat.  Newest first, capped, consecutive duplicates skipped."""
    if not isinstance(prev, dict) or not prev:
        return
    try:
        with LOCK:
            try:
                items = json.load(open(_hist_path()))
            except (OSError, ValueError):
                items = []
            if not isinstance(items, list):
                items = []
            if items and isinstance(items[0].get("params"), dict) \
                    and all(items[0]["params"].get(k) == v
                            for k, v in prev.items()):
                return
            items.insert(0, {"when": time.strftime("%Y-%m-%d %H:%M"),
                             "label": label, "params": prev})
            json.dump(items[:10], open(_hist_path(), "w"), indent=1)
    except OSError as e:
        log(f"history save failed: {e}")


def generate(params):
    p = sanitize(params or {})
    prev = STATE.get("params")
    if prev:
        # a bigger jump (preset click, restored state, API call) - not a
        # slider drag - keeps the previous boat recoverable
        diff = sum(1 for k in p if prev.get(k) != p.get(k))
        if diff >= 3:
            hist_snapshot(prev, f"auto {'%gx%g' % (prev.get('loa', 0),
                                                   prev.get('beam', 0))}")
    def _persist():
        """Commit the new parameters only after the build succeeded: a
        failed generation must leave the previous boat's state intact."""
        STATE["params"] = p
        try:  # remember where the user left off (reloaded on next start)
            with open(os.path.join(STATE["outdir"], "last_params.json"),
                      "w") as fh:
                json.dump(p, fh, indent=1)
        except OSError as e:
            log(f"could not save last_params.json: {e}")
    with LOCK:
        res = run(["python3", os.path.join(HERE, "param_hull.py"),
                   "--loa", fnum(p, "loa"), "--beam", fnum(p, "beam"),
                   "--depth", fnum(p, "depth"), "--rocker", fnum(p, "rocker"),
                   "--sheer-rise", fnum(p, "sheer_rise"),
                   "--transom-f", fnum(p, "transom_f"),
                   "--mid-f", fnum(p, "mid_f"), "--bow-f", fnum(p, "bow_f"),
                   "--bottom-f", fnum(p, "bottom_f"),
                   "--stations", str(int(p["stations"])),
                   "--levels", str(int(p["levels"])), "-o", hull_path()]
                   + (["--sheet"] if p["sheet"] >= 0.5 else []))
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}

        h = hull.load_hull(hull_path())
        _persist()      # the hull (and only a good one) is on disk now
        gear = float(p["gear_kg"])
        crew = int(p["people"]) * PERSON_KG
        lay = layout(h, p)
        cs = canopy_struct(h, lay["roof"]) if lay.get("roof") else None
        roof_kg = cs["mass_kg"] if cs else 0.0
        total = gear + crew + roof_kg
        # combined KG: hull + gear + crew keep the low 0.5*depth convention;
        # the canopy sits at its own computed CG (~1.9 m) and raises the KG
        kg_hull = KG_HULL_F * p["depth"]
        # crew sits on the benches: its VCG is the seat + a seated-body
        # allowance, NOT the hull's half-depth convention
        kg_crew = lay["z_seat"] + CREW_UP
        kg_load = ((max(total - roof_kg - crew, 0.0) * kg_hull
                    + crew * kg_crew
                    + (roof_kg * cs["z_cg"] if cs else 0.0)) / total)
        edge = min(hull.deck_edge(s["pts"])[1] for s in h["stations"])
        try:
            T = hydrostatics.find_draft(h, total, FRESH_RHO)
            r = hydrostatics.at_draft(h, T, FRESH_RHO)
            capT = edge - FREEBOARD_CAP
            cap = (hydrostatics.at_draft(h, capT, FRESH_RHO)
                   if capT > 0 else None)
            numbers = {
                "draft_m": round(T, 3),
                "freeboard_m": round(r["freeboard_min"], 3),
                "displacement_kg": round(r["disp"]),
                "lwl_m": round(r["Lwl"], 2),
                "bwl_m": round(r["Bwl"], 2),
                "cb": round(r["Cb"], 2),
                "tpc_kg": round(r["TPC"]),
                "form": round((max(r["Cb"], 0.2) / CB_REF) ** FORM_EXP, 2),
                "kw_cruise": round(total / 1000.0 * KW_PER_T_CRUISE
                                   * (max(r["Cb"], 0.2) / CB_REF) ** FORM_EXP, 1),
                "kw_t_cruise": KW_PER_T_CRUISE, "kw_t_hull": KW_PER_T_HULL,
                "kw_hull": round(total / 1000.0 * KW_PER_T_HULL
                                 * (max(r["Cb"], 0.2) / CB_REF) ** FORM_EXP, 1),
                # 2 decimals: at 1 dp, real responses (<2%) rounded to the
                # same value and the number looked frozen
                "hull_kn": round(FROUDE_K * math.sqrt(max(r["Lwl"], 0.1)), 2),
                "hull_kmh": round(FROUDE_K * math.sqrt(max(r["Lwl"], 0.1))
                                  * KN_KMH, 2),
                "cruise_kn": round(CRUISE_F * FROUDE_K
                                   * math.sqrt(max(r["Lwl"], 0.1)), 2),
                "cruise_kmh": round(CRUISE_F * FROUDE_K
                                    * math.sqrt(max(r["Lwl"], 0.1)) * KN_KMH, 2),
                "km_m": round(r["KM"], 3),
                "kg_m": round(kg_load, 3),
                "gm_m": round(r["KM"] - kg_load, 3),
                "roof_cg": round(cs["z_cg"], 2) if cs else None,
                "capacity_kg": round(cap["disp"]) if cap else None,
                "loa": round(p["loa"], 2),
                "gear_kg": round(gear), "crew_kg": round(crew),
                "roof_kg": round(roof_kg), "total_kg": round(total),
                "seated": sum(bb["seated"] for bb in lay["benches"]),
                "bench_n": len(lay["benches"]),
                "seat_h": round(lay["z_seat"] - lay["z_sole"], 2),
                "seat_cm": round(lay["z_seat"] * 100),   # bench top over the
                #                                          lowest point of the hull

                "ply_mm": int(p["ply_mm"]),
                "roof_area": round(lay["roof"]["area"], 1) if lay.get("roof") else 0,
                "roof_pan": len(cs["panels"]) if cs else 0,
                "roof_fit": (lay["roof"]["fits"] if lay.get("roof")
                             else None),   # request satisfied, not just
                "roof_note": lay.get("roof_note"),
                "roof_kwp": round(cs["kwp"], 2) if cs else 0,
                "roof_kwh": round(cs["kwh_day"], 1) if cs else 0,
                "roof_clear_floor": (round(cs["zu"] - lay["z_sole"], 2)
                                     if cs else None),
                "roof_clear_seat": (round(cs["zu"] - lay["z_seat"], 2)
                                    if cs else None),
            }
            # capacity + flotation (small-boat plan essentials)
            _flat = p["bottom_f"] >= 0.999
            _half_tr = hull.deck_edge(h["stations"][0]["pts"])[0]
            _hp, _fac = max_hp(p["loa"], _half_tr, flat_bottom=_flat,
                               remote=False)
            numbers["max_hp"] = _hp
            numbers["hp_factor"] = _fac
            _support = float(p["gear_kg"]) + max(1, int(p["people"])) * PERSON_KG
            numbers["flotation_l"] = int(-(-_support / 0.95))  # 0.95 kg/L net
            numbers["flotation_kg"] = int(_support)
            try:
                numbers["persons"] = persons_estimate(
                    h, total, p["loa"], r["Bwl"], p["depth"],
                    int(p.get("people", 2)), kg=kg_load, tpc_kgcm=r["TPC"],
                    z_crew=kg_crew)     # candidate loads keep crew moments
            except Exception as e:
                numbers["persons"] = {"error": f"{type(e).__name__}: {e}"}
            STATE["draft"] = T
        except ValueError as e:
            numbers = {"error": str(e)}
            STATE["draft"] = None   # no valid loading condition: no waterline

        png = os.path.join(STATE["outdir"], "preview_quick.png")
        try:
            quick_preview(h, STATE["draft"], png + ".tmp.png", lay=lay)
            os.replace(png + ".tmp.png", png)   # atomic publish, no torn reads
            ok, err = True, ""
        except Exception as e:
            ok, err = False, f"{type(e).__name__}: {e}"
            log(f"  preview failed: {err}")
        STATE["params"] = p
        return {"ok": ok, "numbers": numbers, "params": p,
                "image": "/files/preview_quick.png",
                "error": err}


def make_plan():
    if STATE.get("draft") is None:
        return {"ok": False, "error": "no valid loading condition - the boat "
                "cannot float this load; reduce the load first"}
    with LOCK:
        spec = plans_path("linesplan.spec.json")
        res = run(["python3", os.path.join(HERE, "linesplan.py"), hull_path(),
                   "-o", spec, "--draft", f"{STATE['draft']:.3f}",
                   "--offsets", plans_path("offsets.csv")])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        res2 = run(["python3", DRAFT_PY, spec, "--out",
                    plans_path("linesplan"), "--png"])
        if res2.returncode != 0:
            return {"ok": False, "error": (res2.stderr or res2.stdout)[-300:]}
        return {"ok": True, "files": [
            "/files/plans/linesplan.pdf", "/files/plans/linesplan.dxf",
            "/files/plans/linesplan.png", "/files/plans/offsets.csv"]}


def make_construction():
    """Construction plan: station molds, mold table, midship, setup steps."""
    with LOCK:
        p = STATE.get("params") or {}
        spec = plans_path("construction.spec.json")
        res = run(["python3", os.path.join(HERE, "construction.py"),
                   hull_path(), "-o", spec,
                   "--people", str(int(p.get("people", 2) or 2)),
                   "--seat-h", str(float(p.get("seat_h", 0.38) or 0.38)),
                   "--ply-mm", str(int(p.get("ply_mm", 6) or 6)),
                   "--roof-n", str(int(p.get("roof_n", 0) or 0))])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        res2 = run(["python3", DRAFT_PY, spec, "--out",
                    plans_path("construction"), "--png"])
        if res2.returncode != 0:
            return {"ok": False, "error": (res2.stderr or res2.stdout)[-300:]}
        return {"ok": True, "files": ["/files/plans/construction.pdf",
                                      "/files/plans/construction.png"]}


def make_materials():
    """Bill of materials: everything computed from the model."""
    with LOCK:
        p = STATE.get("params") or {}
        out = plans_path("materials.md")
        cmd = ["python3", os.path.join(HERE, "materials.py"), hull_path(),
               "-o", out,
               "--people", str(int(p.get("people", 2) or 2)),
               "--gear", str(float(p.get("gear_kg", 300) or 300)),
               "--ply-mm", str(int(p.get("ply_mm", 6) or 6)),
               "--roof-n", str(int(p.get("roof_n", 0) or 0))]
        if float(p.get("bottom_f", 0) or 0) >= 0.999:
            cmd.append("--flat-bottom")
        res = run(cmd)
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        md = open(out).read()
        return {"ok": True, "files": ["/files/plans/materials.md"],
                "report": md}


def make_views():
    if STATE.get("draft") is None:
        return {"ok": False, "error": "no valid loading condition - the boat "
                "cannot float this load; reduce the load first"}
    with LOCK:
        p = STATE.get("params") or {}
        res = run(["python3", os.path.join(HERE, "view3d.py"), hull_path(),
                   "-o", plans_path("preview3d"),
                   "--draft", f"{STATE['draft']:.3f}",
                   "--roof-n", str(int(p.get("roof_n", 0) or 0)),
                   "--ply-mm", str(int(p.get("ply_mm", 6) or 6)),
                   "--seat-h", str(float(p.get("seat_h", 0.38) or 0.38)),
                   "--roof-h", str(float(p.get("roof_h", 1.0) or 1.0)),
                   "--people", str(int(p.get("people", 2) or 2))])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        return {"ok": True, "files": ["/files/plans/preview3d.png",
                                      "/files/plans/preview3d.pdf"]}


def make_layout():
    """General-arrangement sheet: seating plan + solar roof, dims + notes,
    exported through the engineering-drafting skill (PDF/DXF/PNG)."""
    with LOCK:
        h = hull.load_hull(hull_path())
        p = STATE.get("params", {})
        lay = layout(h, p)
        xs = [s["x"] for s in h["stations"]]
        ed = [max(s["pts"], key=lambda q: (round(q[0], 9), q[1]))
              for s in h["stations"]]
        xm = [x * 1000 for x in xs]
        ye = [q[0] * 1000 for q in ed]
        ze = [q[1] * 1000 for q in ed]
        zk = [s["pts"][0][1] * 1000 for s in h["stations"]]
        n = len(h["stations"])
        s = 25
        P = lambda mm: mm * s
        M = P(12)
        W, H = 420 * s, 297 * s
        ox = M + P(18)
        X = lambda x: ox + x

        def interp(vals, x):
            for i in range(n - 1):
                if xm[i] <= x <= xm[i + 1]:
                    t = ((x - xm[i]) / (xm[i + 1] - xm[i])
                         if xm[i + 1] != xm[i] else 0.0)
                    return vals[i] + t * (vals[i + 1] - vals[i])
            return vals[-1]

        yct = H - M - P(30) - max(ye)
        y0s = M + P(25)
        ent_t, dims_t, txt_t = [], [], []
        for sign in (1, -1):
            ent_t.append({"type": "polyline", "layer": "OUTLINE",
                          "points": [[X(xm[i]), yct + sign * ye[i]]
                                     for i in range(n)]})
        for bn in lay["benches"]:              # cross benches (thwarts)
            xc_, hb = bn["xc"] * 1000, bn["half"] * 1000
            xa_, xb2 = X(xc_ - SEAT_W * 500), X(xc_ + SEAT_W * 500)
            ent_t.append({"type": "polyline", "closed": True,
                          "layer": "STATION",
                          "points": [[xa_, yct - hb], [xb2, yct - hb],
                                     [xb2, yct + hb], [xa_, yct + hb]]})
            for si in range(1, bn["seated"]):
                ys = yct - hb + si * 2.0 * hb / bn["seated"]
                ent_t.append({"type": "line", "layer": "STATION",
                              "p1": [xa_, ys], "p2": [xb2, ys]})
        # oarlock pair at the ROWING STATION, both gunwales (census: the
        # oars cross at max beam and the rowing thwart sits there)
        xo_mm = float(lay.get("x_oarlock") or 0.0) * 1000.0
        if xo_mm:
            yo_mm = interp(ye, xo_mm)
            for sy in (1, -1):
                ent_t.append({"type": "polyline", "closed": True,
                              "layer": "CENTER",
                              "points": [[X(xo_mm - 40), yct + sy * (yo_mm + 40)],
                                         [X(xo_mm + 40), yct + sy * (yo_mm + 40)],
                                         [X(xo_mm + 40), yct + sy * (yo_mm - 80)],
                                         [X(xo_mm - 40), yct + sy * (yo_mm - 80)]]})
        # ---- propulsion kit: WHERE IT GOES (outboard on the open transom,
        # batteries LOW under the helm bench).  Two small callouts only -
        # the details live in the title-block notes (no floating clutter).
        ent_t.append({"type": "polyline", "closed": True, "layer": "OUTLINE",
                      "points": [[X(-330), yct - 130], [X(-60), yct - 130],
                                 [X(-60), yct + 130], [X(-330), yct + 130]]})
        helm = next((b for b in lay["benches"] if b["helm"]), None)
        if helm:
            bx = X(helm["xc"] * 1000)
            ent_t.append({"type": "polyline", "closed": True, "layer": "CENTER",
                          "points": [[bx - 250, yct - 110], [bx + 250, yct - 110],
                                     [bx + 250, yct + 110], [bx - 250, yct + 110]]})
            ent_t.append({"type": "polyline", "layer": "CENTER",
                          "points": [[bx + 250, yct - 110], [X(-60), yct - 110]]})
        # equipment key BELOW the plan (floating labels collided on short
        # boats where everything crowds the stern)
        ymax_mm = 1000.0 * max(max(q[0] for q in st["pts"])
                               for st in h["stations"])
        ky = yct - ymax_mm - P(42)   # clear BELOW the dimensions
        for j, s__ in enumerate((
                "rect behind transom = OUTBOARD (clamps the open transom)",
                "box at helm bench = 2 x LiFePO4 100 Ah, LOW under the bench",
                "line = 50 mm2 cable: switch + 125 A fuse at battery -> motor",
                "boxes at the sheer = OARLOCK pair at the rowing station")):
            txt_t.append({"pos": [ox, ky - j * P(7)], "h": P(4), "s": s__,
                          "align": "left", "layer": "TEXT"})
        r = lay.get("roof")
        cs_l = canopy_struct(h, r) if r else None
        if r:
            rx0, rx1 = X(r["x0"] * 1000), X(r["x1"] * 1000)
            hw = r["w"] * 1000 / 2.0
            ent_t.append({"type": "polyline", "closed": True,
                          "layer": "WATERLINE",
                          "points": [[rx0, yct + hw], [rx1, yct + hw],
                                     [rx1, yct - hw], [rx0, yct - hw]]})
            for pn in cs_l["panels"]:      # standard 450 W modules
                px = X(pn["x0"] * 1000)
                py = yct + pn["y0"] * 1000
                ent_t.append({"type": "polyline", "closed": True,
                              "layer": "STATION",
                              "points": [[px, py],
                                         [px + cs_l["pan_x"] * 1000, py],
                                         [px + cs_l["pan_x"] * 1000,
                                          py + cs_l["pan_y"] * 1000],
                                         [px, py + cs_l["pan_y"] * 1000]]})
            dims_t.append({"kind": "linear", "axis": "x",
                           "p1": [rx0, yct - hw], "p2": [rx1, yct - hw],
                           "off": -P(24)})
            dims_t.append({"kind": "linear", "axis": "y",
                           "p1": [rx1, yct - hw], "p2": [rx1, yct + hw],
                           "off": P(14)})
        dims_t.append({"kind": "linear", "axis": "x",
                       "p1": [X(0), yct - max(ye)],
                       "p2": [X(lay["benches"][0]["xc"] * 1000),
                              yct - max(ye)], "off": -P(10)})
        txt_t.append({"pos": [X((xm[0] + xm[-1]) / 2), yct + max(ye) + P(12)],
                      "h": P(4), "s": "TOP (plan) - benches, roof, seat positions",
                      "align": "center"})
        ent_s, dims_s, txt_s = [], [], []
        ent_s.append({"type": "polyline", "layer": "OUTLINE",
                      "points": [[X(xm[i]), y0s + zk[i]] for i in range(n)]})
        ent_s.append({"type": "polyline", "layer": "OUTLINE",
                      "points": [[X(xm[i]), y0s + ze[i]] for i in range(n)]})
        top_s = max(ze)
        if r:
            rz = y0s + r["z"] * 1000
            top_s = r["z"] * 1000
            ent_s.append({"type": "line", "layer": "CENTER",
                          "p1": [X(r["x0"] * 1000), rz],
                          "p2": [X(r["x1"] * 1000), rz]})
            for xe in (r["x0"] * 1000, r["x1"] * 1000):
                ent_s.append({"type": "line", "layer": "STATION",
                              "p1": [X(xe), y0s + interp(ze, xe)],
                              "p2": [X(xe), rz]})
            xmid = (r["x0"] + r["x1"]) / 2.0 * 1000
            dims_s.append({"kind": "linear", "axis": "y",
                           "p1": [X(xmid), y0s + interp(ze, xmid)],
                           "p2": [X(xmid), rz], "off": P(12)})
        txt_s.append({"pos": [X((xm[0] + xm[-1]) / 2), y0s + top_s + P(12)],
                      "h": P(4), "s": "SIDE - roof + supports", "align": "center"})
        notes = []
        if r:
            notes.append(f"Solar roof: {len(cs_l['panels'])} x 450 W = "
                         f"{cs_l['kwp']:.2f} kWp ~ {cs_l['kwh_day']:.1f} "
                         "kWh/day (July, flat)")
            if not cs_l["fit"]:
                notes.append("NO 450 W panel fits this roof - make the roof "
                             "longer or the boat wider")
        _helm = any(bb["helm"] for bb in lay["benches"])
        notes.append(f"{len(lay['benches'])} cross bench(es) on side risers "
                     "(glued + fillet/taped)" +
                     (", aft = helm (tiller)" if _helm else
                      ", at the rowing station") +
                     f" -> {sum(bb['seated'] for bb in lay['benches'])} "
                     "seated (ISO 14946)")
        if lay.get("x_oarlock"):
            notes.append(f"Oarlock pair {lay['x_oarlock']:.2f} m from the "
                         "transom (row from that thwart; tiller extension "
                         "when motoring solo)")
        notes.append("Panels flat-mounted, ~25 mm air gap (Z-brackets)")
        notes.append("Foredeck fore; transom open for motor")
        notes.append("Equipment: 24 V 3 kW outboard (open transom)")
        notes.append("Batteries: 2 x 100 Ah LiFePO4, low under helm")
        notes.append("125 A fuse at battery; 50 mm2 to the motor")
        notes.append("Gunwale: inwale + outwale 20 x 45 mm, glued "
                     "(thickened epoxy) + screwed through the topside")
        notes.append(f"Plywood {int((STATE.get('params') or {}).get('ply_mm', 6))}"
                     " mm: hull, benches, decks, floor")
        spec = {"title": "GENERAL ARRANGEMENT",
                "subtitle": "seating plan + solar roof",
                "scale": f"1:{s}", "sheet": "A3", "units": "mm",
                "date": time.strftime("%Y-%m-%d"), "drawn": "boat builder",
                "notes": notes,
                "views": [{"name": "", "entities": ent_t, "texts": txt_t,
                           "dims": dims_t},
                          {"name": "", "entities": ent_s, "texts": txt_s,
                           "dims": dims_s}]}
        spath = plans_path("arrangement.spec.json")
        with open(spath, "w") as f:
            json.dump(spec, f)
        res = run(["python3", DRAFT_PY, spath, "--out",
                   plans_path("arrangement"), "--png"])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        return {"ok": True, "files": ["/files/plans/arrangement.pdf",
                                      "/files/plans/arrangement.dxf",
                                      "/files/plans/arrangement.png"]}


def make_roof_plan():
    """Solar-canopy fabrication detail: sheet 1 = dimensioned plan + side
    (1:20), sheet 2 = transverse half-section through a leg at the gunwale
    (1:5) with the bolted-foot detail, plus a parts list. Built from the
    same canopy_struct() members the 3D view uses."""
    if not os.path.exists(hull_path()):
        return {"ok": False, "error": "build the boat first"}
    with LOCK:
        h = hull.load_hull(hull_path())
        p = {**DEFAULTS, **(STATE.get("params") or {})}
        r = layout(h, p).get("roof")
        if not r:
            return {"ok": False, "error": "set the Solar panels slider above 0 first"}
        cs = canopy_struct(h, r)
        xs = [s["x"] for s in h["stations"]]
        ed = [max(s["pts"], key=lambda q: (round(q[0], 9), q[1]))
              for s in h["stations"]]
        yem = [q[0] * 1000 for q in ed]
        zem = [q[1] * 1000 for q in ed]
        xm = [x * 1000 for x in xs]
        x0, x1 = cs["x0"] * 1000, cs["x1"] * 1000
        hw = cs["hw"] * 1000
        zu, zr_top = cs["zu"] * 1000, (cs["zu"] + 0.04) * 1000
        zp0, zp1 = cs["z_pan0"] * 1000, cs["z_pan1"] * 1000

        def interp(vals, x):
            for i in range(len(xs) - 1):
                if xm[i] <= x <= xm[i + 1]:
                    t = ((x - xm[i]) / (xm[i + 1] - xm[i])
                         if xm[i + 1] != xm[i] else 0.0)
                    return vals[i] + t * (vals[i + 1] - vals[i])
            return vals[-1]

        def rect(a, b, c, d, layer):
            return {"type": "polyline", "closed": True, "layer": layer,
                    "points": [[a, b], [c, b], [c, d], [a, d]]}

        # ---------------- sheet 1: plan + side, 1:20 ----------------
        s = 20
        P = lambda mm: mm * s
        M = P(12)
        W, H = 420 * s, 297 * s
        ox = M + P(18)
        X = lambda x: ox + x
        yct = H - M - P(30) - max(yem)
        y0s = M + P(25)
        ent_t, dims_t, txt_t = [], [], []
        for sign in (1, -1):
            ent_t.append({"type": "polyline", "layer": "OUTLINE",
                          "points": [[X(xm[i]), yct + sign * yem[i]]
                                     for i in range(len(xs))]})
        ent_t.append({"type": "line", "layer": "CENTER",
                      "p1": [X(xm[0] - 100), yct], "p2": [X(xm[-1] + 100), yct]})
        for rl in cs["rails"]:
            sy2 = rl["sy"] * 500
            ent_t.append(rect(X(x0), yct + rl["y"] * 1000 - sy2,
                              X(x1), yct + rl["y"] * 1000 + sy2, "CENTER"))
        for e in cs["ends"]:
            ent_t.append(rect(X(e["x"] * 1000 - e["sx"] * 500),
                              yct + e["y0"] * 1000,
                              X(e["x"] * 1000 + e["sx"] * 500),
                              yct + e["y1"] * 1000, "CENTER"))
        for c in cs["cross"]:
            ent_t.append(rect(X(c["x"] * 1000), yct + c["y0"] * 1000,
                              X(c["x"] * 1000 + c["sx"] * 1000),
                              yct + c["y1"] * 1000, "CENTER"))
        for xl in cs["xlegs"]:      # plan projection of the X diagonals
            ent_t.append({"type": "line", "layer": "STATION",
                          "p1": [X(xl["x0"] * 1000), yct + xl["y0"] * 1000],
                          "p2": [X(xl["x1"] * 1000), yct + xl["y1"] * 1000]})
        for ft in cs["feet"]:
            ent_t.append(rect(X(ft["x"] * 1000 - ft["sx"] * 500),
                              yct + ft["y"] * 1000 - ft["sy"] * 500,
                              X(ft["x"] * 1000 + ft["sx"] * 500),
                              yct + ft["y"] * 1000 + ft["sy"] * 500, "STATION"))
        for pn in cs["panels"]:            # standard 450 W modules
            ent_t.append(rect(X(pn["x0"] * 1000), yct + pn["y0"] * 1000,
                              X((pn["x0"] + cs["pan_x"]) * 1000),
                              yct + (pn["y0"] + cs["pan_y"]) * 1000,
                              "WATERLINE"))
        if cs["panels"]:
            pn = cs["panels"][0]
            dims_t.append({"kind": "linear", "axis": "x",
                           "p1": [X(pn["x0"] * 1000), yct - hw],
                           "p2": [X((pn["x0"] + cs["pan_x"]) * 1000), yct - hw],
                           "off": -P(10)})
        dims_t.append({"kind": "linear", "axis": "x",
                       "p1": [X(x0), yct - hw], "p2": [X(x1), yct - hw],
                       "off": -P(24)})
        dims_t.append({"kind": "linear", "axis": "y",
                       "p1": [X(x1), yct - hw], "p2": [X(x1), yct + hw],
                       "off": P(16)})
        txt_t.append({"pos": [X((xm[0] + xm[-1]) / 2), yct + max(yem) + P(12)],
                      "h": P(4),
                      "s": "TOP (plan) - frame, panels, feet on the gunwale, braces",
                      "align": "center"})
        ent_s, dims_s, txt_s = [], [], []
        ent_s.append({"type": "polyline", "layer": "OUTLINE",
                      "points": [[X(xm[i]),
                                  y0s + h["stations"][i]["pts"][0][1] * 1000]
                                 for i in range(len(xs))]})
        ent_s.append({"type": "polyline", "layer": "OUTLINE",
                      "points": [[X(xm[i]), y0s + zem[i]]
                                 for i in range(len(xs))]})
        for xl in cs["xlegs"]:      # side view: the X itself
            ent_s.append({"type": "line", "layer": "STATION",
                          "p1": [X(xl["x0"] * 1000), y0s + xl["z0"] * 1000],
                          "p2": [X(xl["x1"] * 1000), y0s + xl["z1"] * 1000]})
        ent_s.append(rect(X(x0), y0s + zu, X(x1), y0s + zr_top, "CENTER"))
        ent_s.append(rect(X(x0), y0s + zp0, X(x1), y0s + zp1, "WATERLINE"))
        for k in range(1, cs["npx"]):    # divider between panel COLUMNS,
            if k * cs["npy"] >= len(cs["panels"]):   # from their real x's
                break
            xr = cs["panels"][(k - 1) * cs["npy"]]["x0"] + cs["pan_x"]
            xl = cs["panels"][k * cs["npy"]]["x0"]
            xj = (xr + xl) / 2.0 * 1000.0
            ent_s.append({"type": "line", "layer": "WATERLINE",
                          "p1": [X(xj), y0s + zp0], "p2": [X(xj), y0s + zp1]})
        xmid = (x0 + x1) / 2.0
        dims_s.append({"kind": "linear", "axis": "y",
                       "p1": [X(xmid), y0s + interp(zem, xmid)],
                       "p2": [X(xmid), y0s + zu], "off": P(14)})
        dims_s.append({"kind": "linear", "axis": "y",
                       "p1": [X(xmid), y0s + interp(zem, xmid)],
                       "p2": [X(xmid), y0s + zp1], "off": P(30)})
        txt_s.append({"pos": [X(xm[0] + 60), y0s - P(24)],
                      "h": P(4), "s": "SIDE - canopy on the gunwales (1:20)",
                      "align": "left"})
        clear_mm = cs["clear"] * 1000
        _fmm = lambda t: f"{t * 1000:.0f}"   # metres -> mm text
        xleg_lens_mm = [math.hypot(b["x1"] - b["x0"], b["y1"] - b["y0"],
                                   b["z1"] - b["z0"]) * 1000
                        for b in cs["xlegs"]]
        rail_len = x1 - x0
        span_mm = 2 * (hw - RAIL_SY * 1000)
        pan_txt = (f"{len(cs['panels'])} x 450 W (1762x1134), factory holes"
                   if cs["panels"] else "none fit - need 1762x1134 + margin")
        notes1 = [
            f"1  tube 40x30x4: 2 x {rail_len:.0f} + 2 x {span_mm:.0f}; "
            f"cross 40x20x2: {len(cs['cross'])} x {span_mm:.0f}",
            f"2  X supports 40x40x4: {len(cs['xlegs'])} x "
            f"{min(xleg_lens_mm):.0f}-{max(xleg_lens_mm):.0f}, foot at one "
            f"roof end to the opposite frame corner; feet ON the gunwale cap "
            f"(push AND pull): plate {_fmm(FOOT_PL[0])}x{_fmm(FOOT_PL[1])} + "
            f"backing {_fmm(BACK_PL[0])}x{_fmm(BACK_PL[1])} under the "
            "inwale, M8 SS through-bolted; clamp both tubes at the crossing",
            f"4  clamps M6: {len(cs['clamps'])}; {pan_txt}; 25 mm gap",
            f"5  ~{cs['mass_kg']:.0f} kg at CG {cs['z_cg']:.2f} m (stability "
            "KG); isolate Al/SS",
        ]
        spec = {"title": "SOLAR CANOPY - FRAME + PANELS",
                "subtitle": f"roof {x1 - x0:.0f} x {hw * 2:.0f} mm, "
                            f"clear height {clear_mm:.0f} mm",
                "scale": "1:20", "sheet": "A3", "units": "mm",
                "date": time.strftime("%Y-%m-%d"), "drawn": "boat builder",
                "notes": notes1,
                "views": [{"name": "", "entities": ent_t, "texts": txt_t,
                           "dims": dims_t},
                          {"name": "", "entities": ent_s, "texts": txt_s,
                           "dims": dims_s}]}
        outs = []
        spath = plans_path("roof_plan.spec.json")
        with open(spath, "w") as f:
            json.dump(spec, f)
        res = run(["python3", DRAFT_PY, spath, "--out",
                   plans_path("roof_plan"), "--png"])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        outs += ["/files/plans/roof_plan.pdf", "/files/plans/roof_plan.dxf",
                 "/files/plans/roof_plan.png"]

        # ---------------- sheet 2: section A-A at 1:5 ----------------
        s2 = 5
        P2 = lambda mm: mm * s2
        M2 = P2(12)
        W2, H2 = 420 * s2, 297 * s2
        Y = lambda ym: M2 + P2(12) + ym
        Z = lambda zm: M2 + P2(3) + (zm - zbot)
        ent_c, dims_c, txt_c = [], [], []
        si = min(range(len(xs)), key=lambda i: abs(xm[i] - x0))
        pts = [(q[0] * 1000, q[1] * 1000) for q in
               h["stations"][si]["pts"][:-1]]
        zbot = min(min(p[1] for p in pts), cs["xlegs"][0]["z0"] * 1000) - 80
        stub = [[Y(0), Z(zbot)]]
        for a, b in zip(pts, pts[1:]):
            if a[1] >= zbot:
                stub.append([Y(a[0]), Z(a[1])])
            elif b[1] >= zbot:
                t = (zbot - a[1]) / (b[1] - a[1])
                stub.append([Y(a[0] + t * (b[0] - a[0])), Z(zbot)])
        stub.append([Y(pts[-1][0]), Z(pts[-1][1])])
        stub.append([Y(0), Z(pts[-1][1])])
        stub.append([Y(0), Z(zbot)])
        ent_c.append({"type": "polyline", "layer": "OUTLINE", "points": stub})
        ent_c.append({"type": "line", "layer": "CENTER",
                      "p1": [Y(0), Z(zbot)], "p2": [Y(0), Z(cs["z_pan1"] * 1000 + 120)]})
        leg = cs["xlegs"][0]        # the X member leaving this foot
        yb, zb = leg["y0"] * 1000, leg["z0"] * 1000
        zt = leg["z1"] * 1000
        yt = leg["y1"] * 1000
        ent_c.append({"type": "polyline", "closed": True, "layer": "STATION",
                      "points": [[Y(yb - LEG_SY * 500), Z(zb)],
                                 [Y(yb + LEG_SY * 500), Z(zb)],
                                 [Y(yt + LEG_SY * 500), Z(zt)],
                                 [Y(yt - LEG_SY * 500), Z(zt)]]})
        ent_c.append(rect(Y(yb - FOOT_PL[1] * 500), Z(zb),
                          Y(yb + FOOT_PL[1] * 500), Z(zb + FOOT_PL[2] * 1000),
                          "STATION"))
        ent_c.append(rect(Y(yb - BACK_PL[1] * 500), Z(zb - BACK_PL[2] * 1000),
                          Y(yb + BACK_PL[1] * 500), Z(zb), "STATION"))
        for bs in (-35, 35):
            ent_c.append(rect(Y(yb + bs - 4), Z(zb - 40), Y(yb + bs + 4),
                              Z(zb + 6), "STATION"))
        ent_c.append(rect(Y(hw - RAIL_SY * 1000), Z(zt), Y(hw), Z(zr_top),
                          "CENTER"))
        ent_c.append(rect(Y(hw - RAIL_SY * 1000), Z(zr_top), Y(hw), Z(zp0),
                          "STATION"))
        ent_c.append(rect(Y(0), Z(zp0), Y(hw), Z(zp1), "WATERLINE"))
        dims_c.append({"kind": "linear", "axis": "y",
                       "p1": [Y(950), Z(zb)], "p2": [Y(950), Z(zt)],
                       "off": P2(18)})
        dims_c.append({"kind": "linear", "axis": "y",
                       "p1": [Y(900), Z(zt)], "p2": [Y(900), Z(zr_top)],
                       "off": P2(60)})
        dims_c.append({"kind": "linear", "axis": "x",
                       "p1": [Y(yb - 50), Z(zb + 3)], "p2": [Y(yb + 50), Z(zb + 3)],
                       "off": -P2(12)})
        dims_c.append({"kind": "linear", "axis": "x",
                       "p1": [Y(yb - 15), Z(zb + 250)], "p2": [Y(yb + 15), Z(zb + 250)],
                       "off": P2(16)})
        dims_c.append({"kind": "linear", "axis": "x",
                       "p1": [Y(yb), Z(zb + 80)], "p2": [Y(interp(yem, x0)), Z(zb + 80)],
                       "off": P2(10)})
        txt_c.append({"pos": [Y(560), Z(1385)], "h": P2(5),
                      "s": "SECTION A-A (1:5)", "align": "left"})
        _fmm = lambda t: f"{t * 1000:.0f}"
        for tz, lab in (
                (zp1 - 40, "panel + 25 mm air gap (see sheet 1)"),
                (cs["zu"] * 1000 + 15, "rail 40x30x4"),
                ((zb + zt) / 2.0, "X support 40x40x4"),
                (zb + 4, f"foot plate {_fmm(FOOT_PL[0])}x{_fmm(FOOT_PL[1])}"
                         f"x{_fmm(FOOT_PL[2])}"),
                (zb - 8, f"backing {_fmm(BACK_PL[0])}x{_fmm(BACK_PL[1])}"
                         f"x{_fmm(BACK_PL[2])} (inside)"),
                (zb - 20, "4 x M8 SS nyloc")):
            txt_c.append({"pos": [Y(150), Z(tz)], "h": P2(4), "s": lab,
                          "align": "left"})
        notes2 = [
            "Foot: plate + backing clamp the topside; drill in place after "
            "tacking.",
            "Isolate steel from aluminium (Tefgel); nyloc nuts; seal all "
            "penetrations.",
            "Panels clamp at factory holes only; check the maker's wind "
            "rating and remove/cover the panels in storm.",
        ]
        spec2 = {"title": "SOLAR CANOPY - SECTION A-A",
                 "subtitle": "gunwale foot + frame detail",
                 "scale": "1:5", "sheet": "A3", "units": "mm",
                 "date": time.strftime("%Y-%m-%d"), "drawn": "boat builder",
                 "notes": notes2,
                 "views": [{"name": "", "entities": ent_c, "texts": txt_c,
                            "dims": dims_c}]}
        spath2 = plans_path("roof_section.spec.json")
        with open(spath2, "w") as f:
            json.dump(spec2, f)
        res = run(["python3", DRAFT_PY, spath2, "--out",
                   plans_path("roof_section"), "--png"])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        outs += ["/files/plans/roof_section.pdf",
                 "/files/plans/roof_section.dxf",
                 "/files/plans/roof_section.png"]
        return {"ok": True, "files": outs}


def make_layers(layer_h=0.2):
    """Slice boat.stl into per-layer images for the page's layer viewer.
    Cached; rebuilt when the STL changes or the layer height differs."""
    stl = os.path.join(STATE["outdir"], "boat.stl")
    if not os.path.exists(stl):
        return {"ok": False, "error": "export the STL first"}
    ldir = os.path.join(STATE["outdir"], "layers")
    meta_path = os.path.join(ldir, "meta.json")
    try:
        layer_h = float(layer_h)
        if not math.isfinite(layer_h):
            layer_h = 0.2
    except (TypeError, ValueError):
        layer_h = 0.2
    layer_h = min(max(layer_h, 0.05), 2.0)
    mtime = os.path.getmtime(stl)
    if os.path.exists(meta_path):
        try:
            meta = json.load(open(meta_path))
            if (abs(meta.get("layer", 0.2) - layer_h) < 1e-9
                    and abs(meta.get("mtime", 0) - mtime) < 1e-6):
                return {"ok": True, "total": meta["total"], "zs": meta["zs"],
                        "urls": meta["urls"]}
        except Exception:
            pass
    with LOCK:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import slice_check as SL

        tris = SL.parse_stl(stl)
        zmin = float(tris[:, :, 2].min())
        zmax = float(tris[:, :, 2].max())
        nlay = max(1, int(np.ceil((zmax - zmin) / layer_h)))
        ox = tris.reshape(-1, 3)
        x0, x1 = ox[:, 0].min() - 2, ox[:, 0].max() + 2
        y0, y1 = ox[:, 1].min() - 2, ox[:, 1].max() + 2
        os.makedirs(ldir, exist_ok=True)
        zs, urls = [], []
        fig = plt.figure(figsize=(7.0, 7.0 * (y1 - y0) / (x1 - x0) + 0.6))
        ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
        for li in range(nlay):
            zc = min(zmin + (li + 0.5) * layer_h + 1e-5 * (li % 7),
                     zmax - 1e-4)   # never sample above the top
            segs, _own = SL.slice_layer(tris, zc)
            loops, _op = SL.chain_loops(segs) if segs else ([], [])
            ax.clear()
            for lp in loops:
                lp2 = np.vstack([lp, lp[:1]])
                ax.plot(lp2[:, 0], lp2[:, 1], "k-", lw=0.9)
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
            ax.set_aspect("equal")
            ax.axis("off")
            name = f"l{li:04d}.png"
            fig.savefig(os.path.join(ldir, name), dpi=110, facecolor="white")
            zs.append(round(zc, 2))
            urls.append("/files/layers/" + name)
        plt.close(fig)
        json.dump({"layer": layer_h, "mtime": mtime, "total": nlay,
                   "zs": zs, "urls": urls}, open(meta_path, "w"))
    return {"ok": True, "total": nlay, "zs": zs, "urls": urls}


def make_stl():
    """One model at 1:24 - resize in the slicer for other scales.  The
    stl_roof toggle leaves the solar canopy (frame, X supports, feet,
    clamps, panels) OUT of the printed file while the design keeps it."""
    with LOCK:
        out = os.path.join(STATE["outdir"], "boat.stl")
        p = STATE.get("params") or {}
        roof = int(p.get("roof_n", 0) or 0)
        bare = roof > 0 and int(p.get("stl_roof", 1) or 0) == 0
        res = run(["python3", os.path.join(HERE, "export_stl.py"), hull_path(),
                   "-o", out, "--scale", "24",
                   "--ply-mm", str(int(p.get("ply_mm", 6) or 6)),
                   "--roof-n", str(0 if bare else roof),
                   "--seat-h", str(float(p.get("seat_h", 0.38) or 0.38)),
                   "--roof-h", str(float(p.get("roof_h", 1.0) or 1.0)),
                   "--people", str(int(p.get("people", 2) or 2))])
        if res.returncode != 0:
            return {"ok": False, "error": (res.stderr or res.stdout)[-300:]}
        return {"ok": True, "files": ["/files/boat.stl"],
                "note": res.stdout.strip().splitlines()[-1]
                + ("  -  solar roof + supports left out of this STL"
                   if bare else "")
                + "  -  printed at 1:24; resize in your slicer for other "
                  "scales"}


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>Boat builder</title><style>
body{font-family:system-ui,sans-serif;margin:0;display:flex;height:100vh;background:#eef1f5}
#panel{width:400px;min-width:400px;padding:18px 20px;overflow:auto;background:#fff;box-shadow:0 0 10px #0002}
#view{flex:1;display:flex;flex-direction:column;padding:10px 16px 14px;gap:8px;min-width:0}
#imgwrap{flex:1;display:flex;align-items:center;justify-content:center;min-height:0}
img{border:1px solid #ccd;border-radius:8px;background:#fff}
#img{max-width:100%;max-height:100%}
.row{margin:10px 0}label{display:flex;justify-content:space-between;font-size:13.5px;color:#223}
input[type=range]{width:100%}
.num{font-weight:600}
.desc{font-size:11.5px;color:#7a8a99;margin:-2px 0 2px}
h1{font-size:18px;margin:0 0 8px}
#stats{width:100%;font-size:14.5px;line-height:1.8;background:#fff;border-radius:10px;padding:12px 18px;box-shadow:0 1px 4px #0001;columns:2;column-gap:44px}
#stats b{font-size:16px}
button{padding:8px 10px;margin:3px 2px;border-radius:7px;border:1px solid #8aa;background:#eef4ff;cursor:pointer;font-size:13px}
button:hover{background:#dce8ff}
#status{font-size:12px;color:#a60;min-height:16px;margin-top:6px}
#outputs{font-size:12.5px;margin-top:8px;line-height:1.6;word-break:break-all}
h2{font-size:13px;color:#456;margin:14px 0 4px}
a{color:#26a}
details{margin:12px 0;border-top:1px solid #e3e8ef;padding-top:8px}
summary{cursor:pointer;font-size:13px;color:#456;font-weight:600}
summary:hover{color:#26a}
select{padding:5px 6px;border-radius:6px;border:1px solid #8aa;background:#f4f7ff;font-size:13px}
#export button{min-width:170px}
</style></head><body>
<div id="panel">
 <h1>&#9875; Boat builder</h1>
 <h2>Presets</h2>
 <div id="presets"></div>
 <div id="histwrap" style="display:none">
  <h2>Recent boats</h2>
  <div id="history"></div>
  <div class="desc">every preset click first saves your current boat here -
  click a chip to load one back</div>
 </div>
 <h2>Boat</h2>
 <div id="sliders"></div>
 <h2>Load &amp; solar</h2>
 <div class="row"><label>People aboard <span class="num" id="pv"></span></label>
  <input type="range" id="people" min="1" max="12" step="1" value="2">
  <div class="desc">Crew count (85 kg each) &mdash; weight added to the boat</div>
  <div class="desc" id="pvres" style="color:#345;font-weight:600"></div></div>
 <div class="row"><label>Boat + gear <span class="num" id="loadv"></span></label>
  <input type="range" id="gear_kg" min="100" max="3000" step="10" value="300">
  <div class="desc">Everything aboard EXCEPT people: hull, motor, battery,
  gear - the solar canopy adds itself. Real 5.2 m example (24 V datasheets):
  ~150 ply hull + 20 motor (3 kW class) + 25 LiFePO4 100 Ah + 65 wiring/
  anchor/misc = ~300 kg; bigger battery 200 Ah = ~45 kg</div></div>
 <div id="solar"></div>
 <details><summary>Advanced hull shape</summary>
  <div id="adv"></div>
  <div class="row"><label><input type="checkbox" id="sheet"> Build from flat sheets (hard chine)</label>
   <div class="desc">Straight panels: flat bottom + straight sides &mdash;
   plywood/aluminium sheet, no compound curves. Set Bottom rise to 0 for a
   dead-flat bottom</div></div>
 </details>
 <div id="status"></div>
 <h2>Export</h2>
 <div id="export">
  <button onclick="views()">3D views</button>
  <button onclick="plan()">Lines plan</button>
  <button onclick="stl()">Export STL (1:24 - resize in slicer)</button>
 </div>
 <details><summary>More: layout sheets, layers, audit</summary>
  <div class="row">
   <button onclick="constr()">Construction plan</button>
   <button onclick="mats()">Materials list</button>
   <button onclick="ga()">Layout: seats + roof</button>
   <button onclick="roofdetail()">Roof detail: frame</button>
   <button onclick="layers()">Layer viewer</button>
   <button onclick="audit()">Audit parts</button>
  </div>
  <div class="row"><label><input type="checkbox" id="stl_roof" checked> STL includes the solar roof + supports</label></div>
 </details>
 <div id="outputs"></div>
 <div class="desc" style="margin-top:10px">Drawings land in
 ~/boat/plans; boat.stl + live preview in ~/boat. Move any
 slider &mdash; the boat redraws in ~1 s. Rating check = ISO-style offset-load
 estimate (85 kg crew test), not a certified rating.</div>
</div>
<div id="view">
 <div id="imgwrap"><img id="img"></div>
 <div id="status2" style="font-size:13px;color:#567"></div>
 <div id="stats">moving a slider builds the boat&hellip;</div>
</div>
<div id="lv" style="display:none;position:fixed;inset:0;background:#0007;z-index:9">
 <div style="background:#fff;max-width:820px;margin:30px auto;padding:14px 18px;
  border-radius:10px;box-shadow:0 10px 40px #0005">
  <div style="display:flex;justify-content:space-between;align-items:center">
   <b>Layer viewer</b>
   <span id="lvinfo" style="font-size:13px;color:#456"></span>
   <span>layer height
    <select id="lvh"><option>0.2</option><option>0.1</option><option>0.3</option><option>0.5</option></select>
    <button onclick="lvClose()">Close</button></span>
  </div>
  <img id="lvimg" style="width:100%;border:1px solid #ccd;border-radius:6px;margin:8px 0;background:#fff">
  <input type="range" id="lvrange" min="0" max="0" value="0" style="width:100%">
  <div style="font-size:12px;color:#567">scroll the mouse over this window or drag the slider - each layer shown with its number and height</div>
 </div>
</div>
<script>
function blog(msg){try{var b=JSON.stringify({msg:String(msg)});
 if(navigator.sendBeacon){navigator.sendBeacon('/api/log',b);return;}
 fetch('/api/log',{method:'POST',headers:{'Content-Type':'application/json'},
  body:b}).then(function(){}).catch(function(){});
}catch(e){}}
window.addEventListener('error',e=>blog('JS error: '+e.message+' @line '+e.lineno+':'+e.colno));
window.addEventListener('unhandledrejection',e=>blog('JS rejection: '+(e.reason&&e.reason.message?e.reason.message:e.reason)));
const SPEC={
 loa:[3,8,0.1,5.2,"Length (m)","Overall hull length (3-8 m)"],
 beam:[1.2,2.6,0.05,1.9,"Width (m)","Beam at the widest point. More width = more stability and room"],
 depth:[0.4,1.3,0.01,0.78,"Depth (m)","Height of the hull sides (deck edge above the bottom). More = more freeboard"],
 sheer_rise:[0,0.3,0.01,0.08,"Bow lift (m)","How much the top edge rises toward the bow"],
 rocker:[0,0.3,0.01,0.07,"Bottom rise (m)","How much the bottom sweeps up toward the bow"],
 bottom_f:[0,1,0.05,0,"Bottom shape","0 = round/vee bottom, 1.0 = true BOX: flat bottom all the way to the chine (pair with flat sheets for hard straight sides)"],
 bow_f:[0.2,1.0,0.01,0.33,"Bow fullness","0.2 = pointy bow ... 1.0 = flat nose (jon / swamp boat)"],
 transom_f:[0.5,1.0,0.01,0.80,"Stern fullness","Low = narrow rear, high = wide transom"],
 roof_n:[0,8,1,1,"Solar panels","Number of standard 450 W modules (1762&times;1134 mm). Arranged 1 or 2 rows across depending on the boat's width, as many columns as the length allows; 0 = no roof. Frame + panels weight and CG go into the stability check"],
 roof_h:[0.6,1.8,0.05,1.0,"Roof height","Clear height of the roof underside over the deck edge (m). 1.00 = comfortable SITTING for a 1.80 m person (seat + 0.95 m seated height + 10 cm); raise toward 1.8 only if you want to stand under it - every cm up raises windage and the centre of gravity (worse stability)"],
 seat_h:[0.25,0.52,0.01,0.38,"Bench height","Seat height above the floor (m); the stats show the bench top in cm above the hull's lowest point. Every cm lower drops the whole crew centre of gravity = more stable; 0.42-0.45 is a normal adult chair, 0.30-0.38 a low fishing thwart"],
 ply_mm:[3,18,1,6,"Plywood (mm)","Panel thickness for the hull, benches, decks and floor - standard plywood. Used on the plans and in the print model (the STL never goes below the printable 1.2 mm at model scale)"]};
const PRESETS={
 "Dinghy 4.2 m":{loa:4.2,beam:1.5,depth:0.55,sheer_rise:0.09,rocker:0.06,bottom_f:0.60,bow_f:0.28,transom_f:0.72,gear_kg:140,people:2,roof_n:0,sheet:1},
 "Jon boat 4.3 m":{loa:4.3,beam:1.6,depth:0.50,sheer_rise:0.05,rocker:0.16,bottom_f:0.80,bow_f:0.95,transom_f:0.95,gear_kg:180,people:3,roof_n:0,sheet:1},
 "Day launch 5.2 m":{loa:5.2,beam:1.9,depth:0.78,sheer_rise:0.08,rocker:0.07,bottom_f:0,bow_f:0.33,transom_f:0.80,gear_kg:300,people:4,roof_n:1,sheet:0},
 "Workboat 6.5 m":{loa:6.5,beam:2.2,depth:0.95,sheer_rise:0.10,rocker:0.10,bottom_f:0,bow_f:0.38,transom_f:0.85,gear_kg:450,people:4,roof_n:2,sheet:0}};
let timer=null;
function el(id){return document.getElementById(id)}
function renderHist(items){
 const h=el('history');h.innerHTML='';
 if(!items||!items.length){el('histwrap').style.display='none';return;}
 el('histwrap').style.display='block';
 for(const it of items.slice(0,6)){
  const wrap=document.createElement('span');wrap.style.display='inline-flex';
  const b=document.createElement('button');
  b.style.fontSize='11.5px';b.style.padding='4px 8px';
  b.style.borderTopRightRadius='0';b.style.borderBottomRightRadius='0';
  b.textContent=(it.when||'').slice(11)+'  '+(it.label||'boat');
  b.title='click to load this boat back';
  b.onclick=()=>{post('/api/history',{params:params(),label:'my boat before loading back'})
   .then(r=>renderHist(r.items)).catch(()=>{});
   applyParams(it.params);queue();};
  const x=document.createElement('button');
  x.style.fontSize='11.5px';x.style.padding='4px 7px';x.style.marginLeft='0';
  x.style.borderLeft='0';x.style.color='#a33';
  x.style.borderTopLeftRadius='0';x.style.borderBottomLeftRadius='0';
  x.textContent='\u00d7';x.title='delete this boat from the list';
  x.onclick=(ev)=>{ev.stopPropagation();
   post('/api/history',{delete_when:it.when,delete_label:it.label})
    .then(r=>renderHist(r.items)).catch(()=>{});};
  wrap.appendChild(b);wrap.appendChild(x);h.appendChild(wrap);}
 const cl=document.createElement('button');
 cl.style.fontSize='11.5px';cl.style.padding='4px 8px';cl.style.marginLeft='6px';
 cl.style.color='#a33';cl.textContent='delete all';
 cl.title='clear the whole recent-boats list';
 cl.onclick=()=>{if(confirm('Delete ALL recent boats? Your current boat is not touched.'))
  post('/api/history',{clear:1}).then(r=>renderHist(r.items)).catch(()=>{});};
 h.appendChild(cl);}
async function loadHist(){try{renderHist((await post('/api/history',{})).items);}catch(e){}}
function buildUI(){
 const ps=el('presets');
 for(const name in PRESETS){const b=document.createElement('button');
  b.textContent=name;b.onclick=()=>applyPreset(name);ps.appendChild(b);}
 const mk=(box,k)=>{const [mn,mx,st,def,label,desc]=SPEC[k];
  const row=document.createElement('div');row.className='row';
  row.innerHTML=`<label>${label} <span class="num" id="v_${k}"></span></label>
   <input type="range" id="${k}" min="${mn}" max="${mx}" step="${st}" value="${def}">
   <div class="desc">${desc||""}</div>`;
  box.appendChild(row);
  el(k).addEventListener('input',()=>{el('v_'+k).textContent=el(k).value;queue()});
  el('v_'+k).textContent=def;};
 for(const k of ['loa','beam','depth','ply_mm']) mk(el('sliders'),k);
 mk(el('solar'),'roof_n');
 mk(el('solar'),'roof_h');
 mk(el('solar'),'seat_h');
 for(const k of ['sheer_rise','rocker','bottom_f','bow_f','transom_f'])
  mk(el('adv'),k);
 const _shx=el('sheet');if(_shx)_shx.addEventListener('change',()=>queue());
 const _srx=el('stl_roof');if(_srx)_srx.addEventListener('change',()=>queue());
 el('gear_kg').addEventListener('input',()=>{el('loadv').textContent=el('gear_kg').value+' kg';queue()});
 el('loadv').textContent=el('gear_kg').value+' kg';
 el('people').addEventListener('input',()=>{el('pv').textContent=el('people').value;queue()});
 el('pv').textContent=el('people').value;
}
let extraParams={};   // SAVED keys with no slider (mid_f, stations, ...):
                      // they must survive every update, not reset to defaults
function params(){const p={};
 for(const k in SPEC){const e=el(k);if(e)p[k]=parseFloat(e.value);}
 const g=el('gear_kg'),pp=el('people'),sh=el('sheet'),sr=el('stl_roof');
 if(g)p.gear_kg=parseFloat(g.value);if(pp)p.people=parseFloat(pp.value);
 if(sh)p.sheet=sh.checked?1:0;
 if(sr)p.stl_roof=sr.checked?1:0;
 for(const k in extraParams)p[k]=extraParams[k];
 return p;}
let busy=false,pending=false,rev=0,doneRev=-1,lastOk=true;
function queue(){rev++;el('status2').textContent='updating the boat...';
 el('img').style.opacity=0.6;clearTimeout(timer);
 timer=setTimeout(update,250);}
async function post(url,body){const r=await fetch(url,{method:'POST',
 headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
 return await r.json();}
async function update(){
 if(busy){pending=true;return lastOk;}
 busy=true;const my=rev;
 try{
  const res=await post('/api/update',params());
  lastOk=!(res.error||res.ok===false);
  el('status2').textContent='';
  el('img').onerror=()=>{el('img').style.opacity=1;el('status2').textContent='image failed to render';};
  if(res.error){el('status').textContent='Error: '+res.error;el('img').style.opacity=1;}
  else if(!pending&&rev===my){  // no newer input arrived: safe to paint this result
   el('status').textContent='';
   el('img').onload=()=>{el('img').style.opacity=1;el('status2').textContent='';};
   el('img').src=res.image+'?t='+Date.now();
   el('outputs').innerHTML='';
   const n=res.numbers||{};const p=res.params||{};
   if(n.error){el('stats').innerHTML='<b>Cannot float this load</b><br>'+n.error+
    '<br><span style="color:#880;font-size:12px">reduce the load or make the hull bigger &mdash; plan/view exports stay disabled until it floats</span>';
    el('pvres').innerHTML='';}
   else{const pe=n.persons||{};let pl='';
    if(pe.sel){if(pe.sel.heel_deg==null){const gt=Number(n.gm_m);
      pl=`Rating check: <b>&#10007; `+(pe.sel.reason||'over the limit')+`</b>`+
      (isFinite(gt)?` &mdash; GM is ${n.gm_m} m`+(gt<=0?'; top-heavy even without heel. Levers that WORK: flat bottom shape, more beam, fewer/lighter panels. Bow lift / rocker / stern fullness do not change this.':'')+`<br>`:'');}
     else{const why=pe.sel.passes?'&#10003; passes':
       (pe.sel.heel_deg>pe.heel_limit_deg?'&#10007; heels too far':'&#10007; freeboard gone at that heel');
      pl=`Rating check (standard worst case - ALL ${pe.sel.n} crew on one side at once; normal seated crew heels it far less): `+
      `heel <b>${pe.sel.heel_deg}&deg;</b> of ${pe.heel_limit_deg}&deg; allowed `+why+
      `<br>Max crew by that rule: <b>${pe.max_persons}</b>`+
      (pe.flood_deg?` - the deck edge would go under at ~<b>${pe.flood_deg}&deg;</b> of heel, the real wave limit`:'')+`<br>`;}}
    else if(pe.error){pl='Rating check: '+pe.error+'<br>';}
    let short='';
    if(pe.sel){if(pe.sel.heel_deg==null){short='+'+pe.sel.n*85+' kg aboard · ISO one-side test: over — '+(pe.sel.reason||'over the limit')+' (GM '+n.gm_m+' m)';}
     else{short='+'+pe.sel.n*85+' kg aboard ('+pe.sel.n+' &times; 85 kg) · ISO one-side test: '+
       (pe.sel.passes?'passes':('over — '+(pe.sel.reason||'over the limit')));}}
    else if(pe.error){short=pe.error;}
    el('pvres').innerHTML=short;
    el('stats').innerHTML=
    `Draft at ${n.total_kg!=null?n.total_kg:''} kg (boat+gear ${p.gear_kg!=null?p.gear_kg:''}, crew ${p.people!=null?p.people:''}&times;85`+
    (n.roof_kg?`, roof ${n.roof_kg} kg`:'')+`, ply ${n.ply_mm!=null?n.ply_mm:''} mm): <b>${n.draft_m} m</b><br>`+
    `Freeboard left: <b>${n.freeboard_m} m</b><br>`+
    (n.kw_hull?`Cruise speed: ~<b>${n.cruise_kn} kn</b> (${n.cruise_kmh} km/h) - ~${n.kw_cruise} kW<br>`:'')+
    (n.lwl_m?`Hull speed: <b>${Number(n.hull_kn).toFixed(2)} kn</b> (${Number(n.hull_kmh).toFixed(1)} km/h) - ~${n.kw_hull} kW<br>`:'')+
    (n.tpc_kg?`Buoyancy: every 1 cm of extra sink = <b>${n.tpc_kg} kg</b> more load (TPC, real area at the waterline)<br>`:'')+
    ``+
    `Stability: KM ${n.km_m} m`+
    (n.kg_m!=null?` · KG <b>${n.kg_m} m</b>`+
     (n.roof_cg!=null?` (solar roof ${n.roof_kg} kg at ${n.roof_cg} m raises it)`:``)+
     ` · GM ${n.gm_m} m`:'')+`<br>`+
    `Seats: <b>${n.seated!=null?n.seated:''}</b> (${n.bench_n!=null?n.bench_n:''} cross benches; aft bench = helm, tiller)`+
    (n.seat_cm!=null?` &middot; bench top <b>${n.seat_cm} cm</b> above the bottom (lowest point)`:'')+
    (n.flotation_l?`<br>Flotation: <b>${n.flotation_l} L</b> of foam (or sealed compartments) under the benches to keep ${n.flotation_kg} kg afloat when swamped`:'')+
    (n.max_hp?`<br>Rated max power (USCG 33 CFR 183.53, factor ${n.hp_factor}): <b>${n.max_hp} hp</b> petrol - the 24 V electric is ~4 hp equivalent`:'')+
    (n.roof_kwp?`<br>Solar roof: <b>${n.roof_area} m&sup2;</b> = ${n.roof_kwp} kWp &asymp; ${n.roof_kwh} kWh/day (July) &middot; ${n.roof_pan} &times; 450 W`:'')+
    (n.roof_clear_floor!=null?`<br>Roof underside: <b>${n.roof_clear_floor} m</b> over the floor`+
     ` <span style="color:${n.roof_clear_seat-0.95<0.02?'#a60':'#345'}">(${Math.round((n.roof_clear_seat-0.95)*100)} cm above a seated 1.8 m head)`+
     (n.roof_clear_seat-0.95<0.02?' - too low to sit under':'')+`</span>`:'')+
    (n.roof_note?`<br><span style="color:#a60">${n.roof_note}</span>`:'')+
    `<br>`+pl+
    `Buoyancy cap (0.15 m freeboard): ${n.capacity_kg} kg `+
    `<span style="color:#880;font-size:12px">(absolute hull limit, not a rating)</span>`;}
  }
 }catch(e){lastOk=false;el('status').textContent='Cannot reach the builder server - is it running? ('+e+')';
  el('status2').textContent='';el('img').style.opacity=1;blog('update failed: '+e);}
 busy=false;doneRev=Math.max(doneRev,my);
 if(pending){pending=false;update();}
 return lastOk;
}
function applyPreset(name){
 post('/api/history',{params:params(),label:'my boat before '+name})
  .then(r=>renderHist(r.items)).catch(()=>{});
 const p=PRESETS[name];
 for(const k in p){if(k==='gear_kg'){el(k).value=p[k];el('loadv').textContent=p[k]+' kg';}
  else if(k==='people'){el('people').value=p[k];el('pv').textContent=p[k];}
  else if(k==='sheet'){el('sheet').checked=!!p[k];}
  else if(k==='stl_roof'){el('stl_roof').checked=!!p[k];}
  else{el(k).value=p[k];el('v_'+k).textContent=p[k];}}
 queue();}
async function settle(){clearTimeout(timer);await update();
 while(busy||pending||doneRev<rev){
  if(!busy&&!pending&&doneRev<rev){await update();}
  await new Promise(r=>setTimeout(r,80));
 }
 return lastOk;}
async function ga(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='drawing layout...';
 try{const res=await post('/api/layout',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Layout: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ');}
 catch(e){el('status').textContent='Export failed: '+e;blog('layout failed: '+e);}}
async function roofdetail(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='drawing canopy detail...';
 try{const res=await post('/api/roof',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Canopy detail: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ');}
 catch(e){el('status').textContent='Export failed: '+e;blog('roof detail failed: '+e);}}
async function views(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='rendering 6 views...';
 try{const res=await post('/api/views',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Views: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ');}
 catch(e){el('status').textContent='Export failed: '+e;blog('views failed: '+e);}}
async function constr(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='drawing the construction plan...';
 try{const res=await post('/api/construction',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Construction: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ');}
 catch(e){el('status').textContent='Export failed: '+e;blog('construction failed: '+e);}}
async function mats(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='listing the materials...';
 try{const res=await post('/api/materials',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Materials: <a href="/files/materials.md" target="_blank">materials.md</a><pre style="font-size:11px;white-space:pre-wrap;max-height:320px;overflow:auto;background:#f6f8fb;padding:8px;border-radius:6px">'+(res.report||'').replace(/&/g,'&amp;').replace(/</g,'&lt;')+'</pre>';}
 catch(e){el('status').textContent='Export failed: '+e;blog('materials failed: '+e);}}
async function plan(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='drawing plan...';
 try{const res=await post('/api/plan',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='Plan: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ');}
 catch(e){el('status').textContent='Export failed: '+e;blog('plan failed: '+e);}}
async function stl(){if(!await settle()){el('status').textContent='Cannot export: the last build failed (see message above)';return;}
 el('status').textContent='exporting STL...';
 try{const res=await post('/api/stl',{});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  el('outputs').innerHTML='STL: '+res.files.map(f=>`<a href="${f}" target="_blank">${f.split('/').pop()}</a>`).join(' | ')+'<br>'+(res.note||'');}
 catch(e){el('status').textContent='Export failed: '+e;blog('stl failed: '+e);}}
let lvUrls=[],lvZs=[];
async function layers(){if(!await settle()){el('status').textContent='Cannot open: the last build failed';return;}
 el('status').textContent='slicing the model into layers (first time ~15 s)...';
 try{const res=await post('/api/layers',{layer:parseFloat(el('lvh').value)||0.2});
  if(!res.ok){el('status').textContent='Error: '+res.error;return;}
  el('status').textContent='';
  lvUrls=res.urls;lvZs=res.zs;
  el('lvrange').max=lvUrls.length-1;el('lv').style.display='block';lvShow(0);}
 catch(e){el('status').textContent='Layer viewer failed: '+e;blog('layers failed: '+e);}}
function lvShow(i){if(!lvUrls.length)return;
 i=Math.max(0,Math.min(lvUrls.length-1,i));
 el('lvrange').value=i;
 el('lvimg').src=lvUrls[i];
 el('lvinfo').textContent='layer '+(i+1)+' / '+lvUrls.length+'  -  z = '+lvZs[i]+' mm';}
function lvClose(){el('lv').style.display='none';}
async function audit(){
 el('status2').textContent='auditing every part (gaps + intersections)... ~1 min';
 try{const res=await post('/api/audit',{});
  el('status2').textContent='';
  el('outputs').innerHTML='<b>Part audit '+(res.ok?'PASS':'FAIL')+'</b>'+
   '<pre style="font-size:11px;white-space:pre-wrap;max-height:340px;overflow:auto;background:#f6f8fb;padding:8px;border-radius:6px">'+
   ((res.report||res.error||'no report').replace(/&/g,'&amp;').replace(/</g,'&lt;'))+'</pre>';
 }catch(e){el('status2').textContent='audit failed: '+e;}}
function applyParams(p){
 for(const k in p){if(!(k in SPEC)&&k!=='gear_kg'&&k!=='people'&&k!=='sheet'
   &&k!=='stl_roof')
  extraParams[k]=p[k];}
 for(const k in SPEC){const e=el(k);if(e&&p[k]!==undefined){
  e.value=p[k];const v=el('v_'+k);if(v)v.textContent=e.value;}}
 if(p.gear_kg!==undefined){el('gear_kg').value=p.gear_kg;
  el('loadv').textContent=p.gear_kg+' kg';}
 if(p.people!==undefined){el('people').value=p.people;
  el('pv').textContent=p.people;}
 if(p.sheet!==undefined)el('sheet').checked=!!p.sheet;
 if(p.stl_roof!==undefined)el('stl_roof').checked=!!p.stl_roof;}
window.SAVED=null;
buildUI();
if(window.SAVED)applyParams(window.SAVED);
update();
loadHist();
el('lvrange').addEventListener('input',()=>lvShow(parseInt(el('lvrange').value,10)));
el('lv').addEventListener('wheel',e=>{e.preventDefault();
 lvShow(parseInt(el('lvrange').value,10)+(e.deltaY>0?1:-1));},{passive:false});
el('lvh').addEventListener('change',()=>{if(el('lv').style.display==='block')layers();});
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        try:
            n = int(self.headers.get("Content-Length", 0))
        except (TypeError, ValueError):
            return None
        if n < 0 or n > 200_000:
            return None
        raw = self.rfile.read(n) if n else b"{}"
        try:
            obj = json.loads(raw)
        except (json.JSONDecodeError, ValueError):
            return None
        return obj if isinstance(obj, dict) else None

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = PAGE.replace(
                "window.SAVED=null;",
                "window.SAVED=" + json.dumps(STATE.get("params") or {})
                + ";").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path.startswith("/files/"):
            name = self.path[len("/files/"):].split("?")[0]
            rel = name.replace("\\", "/").lstrip("/")
            root = os.path.realpath(STATE["outdir"])
            path = os.path.realpath(os.path.join(root, rel))
            if not path.startswith(root + os.sep):
                path = None          # no path escapes out of the outdir
            with LOCK:  # never serve a file while it is being rewritten
                data = None
                if path and os.path.isfile(path):
                    with open(path, "rb") as f:
                        data = f.read()
            if data is not None:
                ctype = ("image/png" if path.endswith(".png") else
                         "application/pdf" if path.endswith(".pdf") else
                         "text/csv" if path.endswith(".csv") else
                         "application/octet-stream")
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self._json({"error": "not found"}, 404)
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        body = self._body()
        if body is None:
            self._json({"ok": False, "error": "bad request body"}, 400)
            return
        t0 = time.time()
        notable = self.path in ("/api/plan", "/api/stl", "/api/views",
                                "/api/layout", "/api/roof", "/api/layers",
                                "/api/audit", "/api/construction",
                                "/api/materials")
        if notable:
            log(f"POST {self.path} {json.dumps(body)[:120]}")
        try:
            if self.path == "/api/update":
                self._json(generate(body))
            elif self.path == "/api/plan":
                self._json(make_plan())
            elif self.path == "/api/construction":
                self._json(make_construction())
            elif self.path == "/api/materials":
                self._json(make_materials())
            elif self.path == "/api/stl":
                self._json(make_stl())
            elif self.path == "/api/history":
                hp = body.get("params")
                if isinstance(hp, dict):
                    try:
                        hist_snapshot(sanitize(hp),
                                      str(body.get("label", "previous"))[:60])
                    except ValueError:
                        pass
                dw = body.get("delete_when")
                if body.get("clear") or isinstance(dw, str):
                    with LOCK:      # ONE acquisition: hist_load() locks too,
                        #             # nesting it here deadlocks (not reentrant)
                        try:
                            items = json.load(open(_hist_path()))
                        except (OSError, ValueError):
                            items = []
                        if not isinstance(items, list):
                            items = []
                        if body.get("clear"):
                            items = []
                        elif isinstance(dw, str):
                            items = [it for it in items
                                     if not (it.get("when") == dw
                                             and it.get("label")
                                             == body.get("delete_label"))]
                        json.dump(items[:10], open(_hist_path(), "w"),
                                  indent=1)
                self._json({"items": hist_load()[:10]})
            elif self.path == "/api/audit":
                stl = os.path.join(STATE["outdir"], "boat.stl")
                if not os.path.exists(stl):
                    self._json({"ok": False,
                                "error": "export the STL first"})
                else:
                    res = run(["python3",
                               os.path.join(HERE, "part_audit.py"), stl],
                              timeout=300)
                    self._json({"ok": res.returncode == 0,
                                "report": (res.stdout or res.stderr)[-4000:]})
            elif self.path == "/api/layers":
                self._json(make_layers(body.get("layer", 0.2)))
            elif self.path == "/api/views":
                self._json(make_views())
            elif self.path == "/api/layout":
                self._json(make_layout())
            elif self.path == "/api/roof":
                self._json(make_roof_plan())
            elif self.path == "/api/log":
                msgs = body.get("lines") or [body.get("msg", "")]
                for m in list(msgs)[:10]:
                    log(f"BROWSER: {str(m)[:400]}")
                self._json({"ok": True})
            else:
                self._json({"error": "not found"}, 404)
            if notable:
                log(f"  done in {time.time() - t0:.2f}s")
        except Exception as e:
            log(f"ERROR in {self.path}: {type(e).__name__}: {e}")
            log(traceback.format_exc())
            self._json({"ok": False, "error": f"{type(e).__name__}: {e}"}, 500)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--outdir", default=os.path.expanduser("~/boat"))
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    STATE["outdir"] = os.path.abspath(args.outdir)
    setup_log(STATE["outdir"])
    log(f"builder starting: outdir={STATE['outdir']} port={args.port}")
    saved = {}
    try:  # start from where the user left off, not from the defaults
        with open(os.path.join(STATE["outdir"], "last_params.json")) as fh:
            saved = json.load(fh)
        log(f"restored last params: {json.dumps(saved)[:120]}")
    except (OSError, ValueError):
        pass
    try:
        generate(saved)  # build the last-used (or default) boat up front
    except ValueError as e:
        log(f"last_params unusable ({e}); falling back to defaults")
        generate({})
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Boat builder running: http://127.0.0.1:{args.port}/  "
          f"(plans in {STATE['outdir']}/plans)")
    srv.serve_forever()


if __name__ == "__main__":
    main()
