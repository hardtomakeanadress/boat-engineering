"""Shared hull geometry model for the boat-engineering skill.

Units: meters, kilograms. A hull is a "table of offsets":

{
  "name": "demo launch",
  "units": "m",
  "stations": [
    {"x": 0.0, "section": [[y, z], ...]},
    ...
  ]
}

Conventions:
- x: longitudinal datum, 0 at the first station (e.g. transom), increasing
  forward. Stations must be evenly spaced for integration.
- z: 0 at the baseline (bottom of hull at centerline), positive up.
- section: HALF-section polyline, ordered from the centerline keel point
  (y=0, lowest) up the hull side to the deck edge (max y), then back to the
  centerline deck point (y=0, top). First and last points must be y=0.
  For an open boat the deck edge is the sheer; for a decked boat it is the
  deck edge / bulwark top — this is where water first enters.
"""

import json
import math

# --- shared engineering standards: ONE source for the whole skill --------
RHO_FRESH = 1000.0      # fresh water density [kg/m3]
RHO_SEA = 1025.0        # sea water [kg/m3]
PERSON_KG = 85.0        # crew mass allowance [kg] (ISO 14946-ish)
FREEBOARD_CAP = 0.15    # freeboard at which the capacity cap is read [m]
FB_MIN = 0.10           # ISO offset-load residual freeboard limit [m]
GM_MIN = 0.15           # below this GM the boat is "tender" (rule of thumb)
DECK_CLEARANCE = 0.001  # solver keeps this waterline-to-deck [m]
UNIFORM_TOL = 1e-4      # station-spacing uniformity (relative, ~0.1 mm)


def load_hull(path):
    with open(path) as f:
        d = json.load(f)
    if "units" not in d:
        raise ValueError('offsets file has no "units" key: add "units": "m" '
                         "(values are metres; convert them, do not relabel)")
    units = d["units"]
    if units != "m":
        raise ValueError(f'offsets file declares units={units!r}; must be "m" - '
                         "convert the values, do not change just the label")
    stations = sorted(d["stations"], key=lambda s: float(s["x"]))
    if len(stations) < 3:
        raise ValueError("need at least 3 stations")
    seen_x = set()
    for s in stations:
        x = float(s["x"])
        if not math.isfinite(x):
            raise ValueError(f"non-finite station x: {s['x']!r}")
        s["x"] = x                       # normalise "1.5" -> 1.5 once, here
        pts = [(float(p[0]), float(p[1])) for p in s["section"]]
        if any(not (math.isfinite(y) and math.isfinite(z)) for y, z in pts):
            raise ValueError(f"station x={s['x']}: non-finite coordinates")
        if len(pts) < 3:
            raise ValueError(f"station x={s['x']}: need >= 3 section points")
        if abs(pts[0][0]) > 1e-9 or abs(pts[-1][0]) > 1e-9:
            raise ValueError(
                f"station x={s['x']}: section must start and end on the centerline (y=0)")
        if any(y < -1e-9 for y, _z in pts):
            raise ValueError(f"station x={s['x']}: negative half-breadth "
                             "(sections live on the +y half only)")
        if any(pts[i + 1][1] < pts[i][1] - 1e-9 for i in range(len(pts) - 1)):
            raise ValueError(f"station x={s['x']}: section z must not "
                             "decrease (overhangs / internal tunnels are not "
                             "supported by the waterline code)")
        widest = deck_edge(pts)
        if abs(widest[0] - pts[-2][0]) > 1e-9:
            raise ValueError(f"station x={s['x']}: the widest point is not "
                             "the deck edge (tumblehome?) - not supported; "
                             "mark the sheer as the widest point")
        if x in seen_x:
            raise ValueError(f"duplicate station at x={x}")
        seen_x.add(x)
        s["pts"] = pts
    d["stations"] = stations
    return d


def deck_edge(pts):
    """Deck-edge point of a half-section (where water first enters).

    Outermost point; among equal breadths the topmost wins (vertical sides,
    chines). Boats with tumblehome (topsides leaning in) are NOT supported by
    this rule - mark such deck edges explicitly instead.
    """
    return max(pts, key=lambda p: (round(p[0], 9), p[1]))


def integrate_wet(xs, vals, xa, xb):
    """Integrate over the WET span [xa, xb] (value 0 at the keel-crossing
    ends - matching Lwl): Simpson over the full wet stations, trapezoid
    caps out to xa/xb.  The old callers integrated every station with dry
    ones as zero, smearing up to half a station of length into V whenever
    the keel rises (rocker); V and Lwl disagreed for the same waterline."""
    i0 = next((i for i, x in enumerate(xs) if x >= xa - 1e-9), len(xs) - 1)
    i1 = max(i for i, x in enumerate(xs) if x <= xb + 1e-9)
    if i1 < i0:
        return 0.0
    sub_x, sub_v = xs[i0:i1 + 1], vals[i0:i1 + 1]
    core = simpson_uniform(sub_x, sub_v) if len(sub_x) >= 2 else 0.0
    core += 0.5 * (sub_x[0] - xa) * sub_v[0]
    core += 0.5 * (xb - sub_x[-1]) * sub_v[-1]
    return core


def min_deck_edge(h):
    """Lowest deck edge over the stations - the open-boat flooding limit."""
    return min(deck_edge(s["pts"])[1] for s in h["stations"])


def full_section(pts):
    """Close a half-section into the full symmetric section polygon."""
    mirrored = [(-y, z) for (y, z) in reversed(pts)]
    return list(pts) + mirrored


def clip_halfplane(pts, a, b, c):
    """Sutherland-Hodgman clip: keep points with a*y + b*z <= c."""
    out = []
    n = len(pts)
    for i in range(n):
        p = pts[i]
        q = pts[(i + 1) % n]
        fp = a * p[0] + b * p[1] - c
        fq = a * q[0] + b * q[1] - c
        if fp <= 0.0:
            out.append(p)
        if (fp <= 0.0) != (fq <= 0.0):
            t = fp / (fp - fq)
            out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
    return out


def poly_area_centroid(pts):
    """Shoelace area and centroid of a polygon. Returns (abs_area, ybar, zbar)."""
    a2 = my = mz = 0.0
    n = len(pts)
    for i in range(n):
        y0, z0 = pts[i]
        y1, z1 = pts[(i + 1) % n]
        cr = y0 * z1 - y1 * z0
        a2 += cr
        my += (y0 + y1) * cr
        mz += (z0 + z1) * cr
    area = a2 / 2.0
    if abs(area) < 1e-12:
        return 0.0, 0.0, 0.0
    return abs(area), my / (6.0 * area), mz / (6.0 * area)


def immersed(poly_full, theta, c):
    """Submerged part of a full section at heel theta and waterline constant c.

    Submerged condition (body frame): z*cos(theta) - y*sin(theta) <= c.
    theta > 0 = heeled to starboard (+y side down).
    Returns (area, ybar, zbar) of the submerged region.
    """
    a = -math.sin(theta)
    b = math.cos(theta)
    clipped = clip_halfplane(poly_full, a, b, c)
    if len(clipped) < 3:
        return 0.0, 0.0, 0.0
    return poly_area_centroid(clipped)


def waterline_halfwidth(pts, T):
    """Max half-breadth of a half-section at height z=T (0 if dry)."""
    best = 0.0
    n = len(pts)
    for i in range(n):
        y0, z0 = pts[i]
        y1, z1 = pts[(i + 1) % n]
        if (z0 - T) * (z1 - T) <= 0.0 and z0 != z1:
            t = (T - z0) / (z1 - z0)
            y = y0 + t * (y1 - y0)
            best = max(best, y)
    return best


def simpson_uniform(xs, ys):
    """Composite Simpson integration over uniformly spaced ordinates.

    1/3 rule; if the interval count is odd, the last three intervals use the
    3/8 rule. Falls back to trapezoid for 2 points.
    """
    n = len(xs) - 1
    if n < 1:
        raise ValueError("need at least 2 ordinates")
    if n == 1:
        return 0.5 * (xs[1] - xs[0]) * (ys[0] + ys[1])
    h = (xs[-1] - xs[0]) / n
    for i in range(n):
        # relative tolerance: ignores input rounding, still catches
        # genuinely non-uniform stations (shared with linesplan)
        if abs((xs[i + 1] - xs[i]) - h) > UNIFORM_TOL * max(1.0, abs(h)):
            raise ValueError("stations must be uniformly spaced for Simpson integration")
    if n % 2 == 0:
        s = ys[0] + ys[-1]
        for i in range(1, n, 2):
            s += 4.0 * ys[i]
        for i in range(2, n, 2):
            s += 2.0 * ys[i]
        return h / 3.0 * s
    # odd interval count: 3/8 on the last three intervals, 1/3 on the first k
    k = n - 3
    s1 = 0.0
    if k >= 2:
        s1 = ys[0] + ys[k]
        for i in range(1, k, 2):
            s1 += 4.0 * ys[i]
        for i in range(2, k, 2):
            s1 += 2.0 * ys[i]
        s1 *= h / 3.0
    s2 = 3.0 * h / 8.0 * (ys[k] + 3.0 * ys[k + 1] + 3.0 * ys[k + 2] + ys[n])
    return s1 + s2


def keel_crossing(x1, z1, x2, z2, w):
    """x where the keel line crosses height w between two stations
    (None if it does not cross within the segment)."""
    if z2 != z1 and (z1 - w) * (z2 - w) < 0.0:
        t = (w - z1) / (z2 - z1)
        return x1 + t * (x2 - x1)
    return None
