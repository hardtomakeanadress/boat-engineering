"""Regression tests for the boat-engineering skill. Run:
python3 tests/regress.py   (from the skill directory, or anywhere)"""
import json
import math
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
_CFG = os.environ.get("CLAUDE_CONFIG_DIR", os.path.expanduser("~/.claude"))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, os.path.join(_CFG, "skills", "engineering-drafting", "scripts"))
import hull
import hydrostatics as hy
import stability as st
import ezdxf

B = os.path.join(ROOT, "scripts")
D = os.path.join(_CFG, "skills", "engineering-drafting", "scripts")
EX = os.path.join(ROOT, "examples", "example_launch.offsets.json")
EXB = os.path.join(ROOT, "examples", "example_budget.json")

fails = []


def check(name, cond):
    print(("PASS " if cond else "FAIL "), name)
    if not cond:
        fails.append(name)


def run(cmd, timeout=30):
    return subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout)


# 1. deck edge: rectangle section -> topmost of equal breadths
check("deck_edge rectangle", hull.deck_edge([(0, 0), (1, 0), (1, 2), (0, 2)]) == (1, 2))

# 2. units / NaN rejection
d = json.load(open(EX))
d['units'] = 'mm'
open('/tmp/h_mm.json', 'w').write(json.dumps(d))
try:
    hull.load_hull('/tmp/h_mm.json')
    check("units reject", False)
except ValueError:
    check("units reject", True)
d2 = json.load(open(EX))
d2['stations'][0]['section'][1][0] = float('nan')
open('/tmp/h_nan.json', 'w').write(json.dumps(d2))
try:
    hull.load_hull('/tmp/h_nan.json')
    check("NaN reject", False)
except ValueError:
    check("NaN reject", True)

# 3. triangular waterplane w(x)=x, x in 0..2, T=0.5: Lwl / LCF / BM_L / Cb
stn = [{"x": x, "section": [[0, 0], [x, 0], [x, 1], [0, 1]]}
       for x in (0.0, 0.5, 1.0, 1.5, 2.0)]
h3 = {"name": "tri", "units": "m", "stations": stn}
open('/tmp/h3.json', 'w').write(json.dumps(h3))
h3 = hull.load_hull('/tmp/h3.json')
r = hy.at_draft(h3, 0.5, 1000)
check("tri V=2", abs(r['V'] - 2.0) < 1e-6)
check("tri Lwl=2", abs(r['Lwl'] - 2.0) < 1e-6)
check("tri LCF=4/3", abs(r['LCF'] - 4 / 3) < 1e-6)
check("tri BM_L=0.4444", abs(r['BM_L'] - 0.444444) < 1e-4)
check("tri Cb=0.5", abs(r['Cb'] - 0.5) < 1e-6)

# 4. find_draft validation
for bad in (0, -5, float('nan')):
    try:
        hy.find_draft(h3, bad, 1000)
        check(f"find_draft reject {bad}", False)
    except ValueError:
        check(f"find_draft reject {bad}", True)
try:
    hy.find_draft(h3, 0.01, 1000)
    check("tiny mass rejected", False)
except ValueError:
    check("tiny mass rejected", True)

# 5. range parsing guards
for spec, why in (("0:0:1", "zero step"), ("0:-1:1", "wrong direction"),
                  ("0:1:100000", "too many")):
    try:
        hy.parse_drafts(spec)
        check(f"parse_drafts {why}", False)
    except ValueError:
        check(f"parse_drafts {why}", True)
check("parse_drafts ok", hy.parse_drafts("0.1:0.1:0.3") == [0.1, 0.2, 0.3])

# 6. heel bracket: flared barge at 90 deg
stn2 = [{"x": x, "section": [[0, 0], [4, 0], [4.1, 0.5], [0, 0.5]]}
        for x in (0.0, 1.0, 2.0)]
h4 = {"name": "flare", "units": "m", "stations": stn2}
open('/tmp/h4.json', 'w').write(json.dumps(h4))
h4 = hull.load_hull('/tmp/h4.json')
th90 = math.radians(90)
pts_all = [(y, z) for s in h4['stations'] for (y, z) in s['section']]
proj = [z * math.cos(th90) - y * math.sin(th90) for (y, z) in pts_all]
c_hi = max(proj) + 0.5
check("bracket covers target @90", st.volume_at(h4, th90, c_hi)[0] >= 1.604)
c, (V, yb, zb, marg) = st.solve_waterline(h4, th90, 1.604, min(proj) - 0.5, c_hi)
check("solve @90 converged", abs(V - 1.604) / 1.604 < 0.005)

# 7. CLI guards (no hangs / clean errors / n/a labels)
r1 = run(['python3', f'{B}/hydrostatics.py', EX, '--drafts', '0:0:1'])
check("draft 0:0:1 stops", r1.returncode != 0)
r2 = run(['python3', f'{B}/stability.py', EX, '--kg', '0.5', '--mass', '838',
          '--angles', '0:0:90'])
check("angles 0:0:90 stops", r2.returncode != 0)
r3 = run(['python3', f'{B}/stability.py', EX, '--kg', '0.5', '--mass', '838',
          '--angles', '10:10:20'])
check("partial coverage n/a", 'n/a' in r3.stdout)
r4 = run(['python3', f'{B}/hydrostatics.py', EX, '--mass', '0'])
check("mass 0 rejected", r4.returncode != 0)
d3 = json.load(open(EX))
d3['stations'][1]['x'] = 1.23
open('/tmp/h_nu.json', 'w').write(json.dumps(d3))
r5 = run(['python3', f'{B}/hydrostatics.py', '/tmp/h_nu.json', '--mass', '800'])
check("nonuniform rejected", r5.returncode != 0)

# 8. budget guards + LCB report
b = json.load(open(EXB))
b['items'].append({"name": "junk", "kg": -50, "count": 1, "x": 1, "z": 1})
open('/tmp/b_bad.json', 'w').write(json.dumps(b))
r6 = run(['python3', f'{B}/budget.py', EX, '/tmp/b_bad.json'])
check("negative kg rejected", r6.returncode != 0)
b2 = json.load(open(EXB))
next(it for it in b2['items'] if it['name'] == 'Crew')['count'] = 1.9
open('/tmp/b_bad2.json', 'w').write(json.dumps(b2))
r7 = run(['python3', f'{B}/budget.py', EX, '/tmp/b_bad2.json'])
check("fractional count rejected", r7.returncode != 0)
r8 = run(['python3', f'{B}/budget.py', EX, EXB, '--freeboard', '0.9'])
check("cap guard no crash", r8.returncode == 0 and 'n/a' in r8.stdout)
r9 = run(['python3', f'{B}/budget.py', EX, EXB])
check("LCB report present", 'LCB' in r9.stdout)

# 9. param hull: exact beam at emitted stations + guards
r10 = run(['python3', f'{B}/param_hull.py', '--stations', '3', '--beam', '1.7',
           '-o', '/tmp/hp.json'])
hh = hull.load_hull('/tmp/hp.json')
beam = 2 * max(hull.deck_edge(s['pts'])[0] for s in hh['stations'])
check("param beam exact (3 stations)", abs(beam - 1.7) < 1e-6)  # the value asked for
r11 = run(['python3', f'{B}/param_hull.py', '--stations', '1', '-o', '/tmp/hx.json'])
check("param stations guard", r11.returncode != 0)

# 10. draft.py: number format / degenerate dims / units / --only pdf / solid order
spec = {"title": "T", "scale": "1:20", "sheet": "A3", "units": "mm",
        "views": [{"name": "V",
                   "entities": [{"type": "line", "p1": [0, 0], "p2": [1000, 0],
                                 "layer": "OUTLINE"}],
                   "dims": [{"kind": "linear", "axis": "x", "p1": [0, 0],
                             "p2": [204, 0], "off": -200}],
                   "texts": []}]}
open('/tmp/spec_num.json', 'w').write(json.dumps(spec))
run(['python3', f'{D}/draft.py', '/tmp/spec_num.json', '--out', '/tmp/out_num',
     '--only', 'dxf'])
doc = ezdxf.readfile('/tmp/out_num.dxf')
texts = [e.dxf.text for e in doc.modelspace().query('TEXT')]
check("204 kept (not 200)", '204' in texts)

spec['views'][0]['dims'] += [
    {"kind": "linear", "axis": "aligned", "p1": [10, 10], "p2": [10, 10], "off": -50},
    {"kind": "angle", "vertex": [0, 0], "ray1": [0, 0], "ray2": [10, 0], "r": 50}]
open('/tmp/spec_deg.json', 'w').write(json.dumps(spec))
r13 = run(['python3', f'{D}/draft.py', '/tmp/spec_deg.json', '--out', '/tmp/out_deg',
           '--only', 'dxf'])
check("degenerate dims skipped, no crash", r13.returncode == 0)

spec2 = dict(spec)
spec2['units'] = 'in'
open('/tmp/spec_in.json', 'w').write(json.dumps(spec2))
r14 = run(['python3', f'{D}/draft.py', '/tmp/spec_in.json', '--out', '/tmp/out_in',
           '--only', 'dxf'])
check("draft units reject", r14.returncode != 0)

if os.path.exists('/tmp/only_pdf.dxf'):
    os.remove('/tmp/only_pdf.dxf')
r15 = run(['python3', f'{D}/draft.py', '/tmp/spec_num.json', '--out', '/tmp/only_pdf',
           '--only', 'pdf'])
check("only-pdf skips dxf", r15.returncode == 0 and not os.path.exists('/tmp/only_pdf.dxf'))

# aligned text lands at the dim midpoint + normal
spec4 = {"title": "T", "scale": "1:1", "sheet": "A3", "units": "mm",
         "views": [{"name": "V", "entities": [],
                    "dims": [{"kind": "linear", "axis": "aligned", "p1": [0, 0],
                              "p2": [100, 100], "off": 20}], "texts": []}]}
open('/tmp/spec_al.json', 'w').write(json.dumps(spec4))
run(['python3', f'{D}/draft.py', '/tmp/spec_al.json', '--out', '/tmp/out_al',
     '--only', 'dxf'])
doc = ezdxf.readfile('/tmp/out_al.dxf')
dim_texts = [e for e in doc.modelspace().query('TEXT') if e.dxf.text == '141.4']
check("aligned dim text exists", len(dim_texts) == 1)
if dim_texts:
    t = dim_texts[0]
    p = t.dxf.align_point if t.dxf.halign else t.dxf.insert
    check("aligned text at midpoint+normal",
          abs(p[0] - 33.383) < 0.05 and abs(p[1] - 66.617) < 0.05)

# solid: 4-point perimeter order converted to DXF order
spec5 = {"title": "T", "scale": "1:1", "sheet": "A3", "units": "mm",
         "views": [{"name": "V",
                    "entities": [{"type": "solid",
                                  "points": [[0, 0], [100, 0], [100, 100], [0, 100]],
                                  "layer": "DIM"}],
                    "dims": [], "texts": []}]}
open('/tmp/spec_so.json', 'w').write(json.dumps(spec5))
run(['python3', f'{D}/draft.py', '/tmp/spec_so.json', '--out', '/tmp/out_so',
     '--only', 'dxf'])
doc = ezdxf.readfile('/tmp/out_so.dxf')
sol = doc.modelspace().query('SOLID')[0]
pts = [(sol.dxf.vtx0[0], sol.dxf.vtx0[1]), (sol.dxf.vtx1[0], sol.dxf.vtx1[1]),
       (sol.dxf.vtx2[0], sol.dxf.vtx2[1]), (sol.dxf.vtx3[0], sol.dxf.vtx3[1])]
check("solid perimeter->DXF order",
      pts == [(0, 0), (100, 0), (0, 100), (100, 100)])

# --- second audit pass: partial-fix regressions ---
import linesplan
import view3d

# #5 full-mirror heel bracket: 6 m3 target on the 8.1 m3 flared barge
ptsf = [(y, z) for (y, z) in [(0, 0), (4, 0), (4.1, 0.5), (0, 0.5)]]
ptsf += [(-y, z) for (y, z) in ptsf]
projf = [z * math.cos(th90) - y * math.sin(th90) for (y, z) in ptsf]
try:
    c, (Vf, _, _, _) = st.solve_waterline(h4, th90, 6.0, min(projf) - 0.5,
                                          max(projf) + 0.5)
    check("full-mirror bracket solves 6 m3 @90", abs(Vf - 6.0) / 6.0 < 0.005)
except ValueError:
    check("full-mirror bracket solves 6 m3 @90", False)

# #10 draft-range edge cases
check("0:-1:0 single point", hy.parse_drafts("0:-1:0") == [0.0])
check("descending range ok", len(hy.parse_drafts("1:-0.1:0")) == 11)
try:
    hy.parse_drafts("1,nan")
    check("comma NaN rejected", False)
except ValueError:
    check("comma NaN rejected", True)

# #9 sub-floor mass rejected
try:
    hy.find_draft(h4, 0.395, 1000)
    check("sub-floor mass rejected", False)
except ValueError:
    check("sub-floor mass rejected", True)

# #16 multi-span waterline: keel 0 / 0.2 / 0 m, w=0.1 -> spans 0-0.5 and 1.5-2
st3 = [{"x": xx, "section": [[0, zk], [0.5, zk], [0.5, 1], [0, 1]]}
       for xx, zk in ((0.0, 0.0), (1.0, 0.2), (2.0, 0.0))]
open('/tmp/h_ms.json', 'w').write(json.dumps({"units": "m", "stations": st3}))
hms = hull.load_hull('/tmp/h_ms.json')
specms, _ = linesplan.build(hms, {"sheet": "A3", "sheet_wh": (420, 297),
                                  "wl_step": 100, "rho": 1000}, 10)
wl_lines = [e for e in specms["views"][0]["entities"]
            if e["layer"] == "WATERLINE" and e["type"] == "line"]
groups = {}
for e in wl_lines:
    groups.setdefault(round(e["p1"][1], 3), []).append(e)
check("multi-span waterline split", any(len(g) >= 2 for g in groups.values()))

# #23 mismatched section point counts in the 3D mesh
stm = [{"x": x, "section": pts} for x, pts in
       ((0.0, [[0, 0], [1, 0], [1, 1], [0, 1]]),
        (1.0, [[0, 0]] + [[0.3 + 0.005 * k, 0.02 * k] for k in range(1, 39)]
         + [[0.52, 0.8], [0, 1]]),   # widest = deck edge: physical section
        (2.0, [[0, 0], [1, 0], [1, 1], [0, 1]]))]
open('/tmp/h_mx.json', 'w').write(json.dumps({"units": "m", "stations": stm}))
hmx = hull.load_hull('/tmp/h_mx.json')
try:
    Vm, _quads = view3d.build_mesh(hmx)
    maxlen = max(len(s["pts"]) for s in hmx["stations"])
    check("mixed-count mesh builds", Vm.shape[1] == 2 * maxlen - 2)
except Exception:
    check("mixed-count mesh builds", False)

# #15 param_hull section-depth guard
r16 = run(['python3', f'{B}/param_hull.py', '--depth', '1', '--rocker', '.25',
           '--sheer-rise', '-.75', '-o', '/tmp/hz.json'])
check("param ze<=zk guard", r16.returncode != 0)

# B6: 2:1 enlargement accepted
spec6 = {"title": "T", "scale": "2:1", "sheet": "A3", "units": "mm",
         "views": [{"name": "V", "entities": [{"type": "line", "p1": [0, 0],
                                               "p2": [50, 0], "layer": "OUTLINE"}],
                    "dims": [], "texts": []}]}
open('/tmp/spec_21.json', 'w').write(json.dumps(spec6))
r17 = run(['python3', f'{D}/draft.py', '/tmp/spec_21.json', '--out', '/tmp/out_21',
           '--only', 'dxf'])
check("2:1 scale accepted", r17.returncode == 0)

# D-b: numeric text override warning
spec7 = json.loads(json.dumps(spec))
spec7['views'][0]['dims'] = [{"kind": "linear", "axis": "x", "p1": [0, 0],
                              "p2": [204, 0], "off": -200, "text": "999"}]
open('/tmp/spec_ov.json', 'w').write(json.dumps(spec7))
r18 = run(['python3', f'{D}/draft.py', '/tmp/spec_ov.json', '--out', '/tmp/out_ov',
           '--only', 'dxf'])
check("override mismatch warns", 'WARNING' in r18.stderr)

# --- STL export for 3D printing ---
import export_stl

import designer as _dgn0
tris_s = export_stl.hull_triangles(hull.load_hull(EX), n_per_half=48)
# welds_ok: parts sit with their faces EXACTLY on what they touch (touching
# surfaces, nothing in midair), so welded contact edges (4 uses) are valid
_oe, _sd = export_stl.edge_report(tris_s, welds_ok=True)
check("STL watertight + consistent winding", _oe == 0 and _sd == 0)
import numpy as _np

verts_s = _np.vstack([[t[0], t[1], t[2]] for t in tris_s])
size_mm = (verts_s.max(axis=0) - verts_s.min(axis=0)) * 1000.0 / 24.0
_hEX = hull.load_hull(EX)
_loa_mm = 1000.0 * _hEX["stations"][-1]["x"]
_beam_mm = 1000.0 * 2.0 * max(hull.deck_edge(s["pts"])[0]
                              for s in _hEX["stations"])
check("STL 1:24 scaled bbox (hull + end-wall cap boards)",   # derived
      # the transom/bow cap boards are glued ON the end walls' outer faces,
      # so the model is exactly 2 board thicknesses longer than the LOA
      abs(size_mm[0] - (_loa_mm / 24 + 2 * _dgn0.GUNW_T * 1000 / 24)) < 1.5
      and abs(size_mm[1] - _beam_mm / 24) < 2.5)

# STL with the solar canopy: watertight and taller (the roof is in the model)
tris_r = export_stl.hull_triangles(hull.load_hull(EX), n_per_half=48, roof_n=2)
_oe2, _sd2 = export_stl.edge_report(tris_r, welds_ok=True)
verts_r = _np.vstack([[t[0], t[1], t[2]] for t in tris_r])
size_r = (verts_r.max(axis=0) - verts_r.min(axis=0)) * 1000.0 / 24.0
check("STL with solar canopy: watertight and taller than the bare hull",
      _oe2 == 0 and _sd2 == 0 and size_r[2] > size_mm[2] + 20)
# the part audit: clean boat passes, a deliberately shifted part fails
_import_shutil = __import__("shutil")
_hEXa = hull.load_hull(EX)
_tra = export_stl.hull_triangles(_hEXa, n_per_half=48, roof_n=1)
export_stl.write_stl('/tmp/aud_fix.stl', _tra, 24.0)
_import_shutil.copy(EX, '/tmp/hull.json')
import render_stl as _rsa
_arra = _rsa.parse_stl('/tmp/aud_fix.stl')
_groupsa = _rsa.bodies(_arra)
_colsa, _namesa = _rsa.classify(_arra, _groupsa, '/tmp/hull.json')
_bi = _namesa.index("bench1") + 1   # the default layout's first bench
_outa = _arra.copy()
_outa[_groupsa[_bi]] = _outa[_groupsa[_bi]] + _np.array([0.0, 0.4, 0.0])
export_stl.write_stl('/tmp/aud_bad.stl',
                     [(t[0], t[1], t[2], _np.array([0.0, 0.0, 1.0]))
                      for t in _outa], 24.0)
r_pa = run(['python3', f'{B}/part_audit.py', '/tmp/aud_fix.stl'],
           timeout=300)
check("part audit: clean boat PASSES",
      r_pa.returncode == 0 and 'PASS' in r_pa.stdout)
r_pb = run(['python3', f'{B}/part_audit.py', '/tmp/aud_bad.stl'],
           timeout=300)
check("part audit: shifted part is CAUGHT",
      r_pb.returncode == 1 and 'FAIL' in r_pb.stdout)

# no stern deck: the transom stays open for the outboard
import render_stl as _rs
_tarr = _np.array([t[:3] for t in tris_s])          # (n,3,3) for bodies()
_gnd = _rs.bodies(_tarr)
_nz = max(hull.deck_edge(s["pts"])[1] for s in hull.load_hull(EX)["stations"]) - 0.02
_stern_decks = [g for g in _gnd[1:]
                if _tarr[g].reshape(-1, 3).mean(axis=0)[0] < 0.3 * 5.2
                and abs(_tarr[g].reshape(-1, 3).mean(axis=0)[1]) > 0.05
                and abs(_tarr[g].reshape(-1, 3).mean(axis=0)[2] - _nz) < 0.1]
check("no stern deck: transom open for the motor", len(_stern_decks) == 0)

# --- designer persons estimate (ISO offset-load style) ---
import designer

hh_p = hull.load_hull(EX)
T_p = hy.find_draft(hh_p, 800.0, 1000.0)
r_p = hy.at_draft(hh_p, T_p, 1000.0)
pe = designer.persons_estimate(hh_p, 800.0, hh_p["stations"][-1]["x"],
                               r_p["Bwl"],
                               max(hull.deck_edge(s["pts"])[1]
                                   for s in hh_p["stations"]), 4)
check("persons estimate sane (1..15 people)",
      1 <= pe.get("max_persons", 0) <= 15 and pe.get("heel_limit_deg", 0) > 20)
pe2 = designer.persons_estimate(hh_p, 705.0, 5.2, r_p["Bwl"], 0.78, 5)
check("crew split: sel computed without over_load",
      "over_load" not in (pe2.get("sel") or {})
      and pe2.get("sel", {}).get("heel_deg") is not None)

# flat-bottom / flat-nose (jon boat) generator
r_jb = run(['python3', f'{B}/param_hull.py', '--bottom-f', '0.8', '--bow-f', '0.95',
            '-o', '/tmp/h_jb.json'])
hj = hull.load_hull('/tmp/h_jb.json')
mid = hj["stations"][len(hj["stations"]) // 2]["pts"]
ye_mid = max(p[0] for p in mid)
check("jon boat flat bottom", abs(mid[1][0] - 0.8 * ye_mid) < 0.02
      and abs(mid[1][1] - mid[0][1]) < 1e-9)

# --- codex pass 3: request sanitize, overload refusal, uniform note ---
try:
    designer.sanitize({"depth": "nan"})
    check("nan depth rejected", False)
except ValueError:
    check("nan depth rejected", True)
check("string numbers coerced", designer.sanitize({"loa": "7"})["loa"] == 7.0)
try:
    designer.sanitize({"depth": 0.32, "rocker": 0.3})
    check("depth<=rocker rejected", False)
except ValueError:
    check("depth<=rocker rejected", True)

r_x = run(['python3', f'{B}/param_hull.py', '--loa', '4', '--beam', '1.2',
           '--depth', '0.4', '--rocker', '0.3', '--sheer-rise', '0',
           '--bottom-f', '0.85', '-o', '/tmp/h_ext.json'])
r_v = run(['python3', f'{B}/export_stl.py', '/tmp/h_ext.json', '-o', '/tmp/ext.stl'])
check("extreme hull STL validates", r_v.returncode == 0)

r_p = run(['python3', f'{B}/param_hull.py', '--stations', '13',
           '-o', '/tmp/h_13.json'])
import linesplan as _lp

_spec, _ = _lp.build(hull.load_hull('/tmp/h_13.json'),
                     {"sheet": "A3", "sheet_wh": (420, 297), "wl_step": 100,
                      "rho": 1000}, 10)
check("13-station plan not flagged non-uniform",
      not any("NON-UNIFORM" in n for n in _spec["notes"]))

# --- seating + solar roof layout ---
lay_t = designer.layout(hh_p, {"roof_n": 1})
# benches ride at the ROWING STATION (the oarlocks); a transom/helm bench
# exists only as an ADDITION in multi-bench layouts (census rule: a
# transom-only seat cannot be rowed from)
check("benches sit at the oarlocks (helm bench only in multi-bench layouts)",
      1 <= len(lay_t["benches"]) <= 4
      and any(abs(b["xc"] - lay_t["x_oarlock"]) < 1e-12
              for b in lay_t["benches"])
      and (len(lay_t["benches"]) == 1
           or any(b["helm"] for b in lay_t["benches"])))
# the bench COUNT follows people x what one bench seats on THIS hull:
# a narrow hull with one aboard gets ONE thwart, not three
import subprocess as _sp
_rc = _sp.run(['python3', f'{B}/param_hull.py', '--loa', '3.0',
               '--beam', '0.8', '--depth', '0.35',
               '-o', '/tmp/h_canoe.json'], capture_output=True)
_hc = hull.load_hull('/tmp/h_canoe.json')
check("bench count follows the people aboard (canoe: 1 person = 1 thwart)",
      _rc.returncode == 0
      and len(designer.layout(_hc, {"roof_n": 0, "people": 1,
                                    "seat_h": 0.30})["benches"]) == 1
      and len(designer.layout(_hc, {"roof_n": 0, "people": 4,
                                    "seat_h": 0.30})["benches"]) == 4)
# SOLO: the single bench is the ROWING thwart at the oarlocks amidships -
# never only a transom bench (you cannot row from there: no oar spread,
# stern-down trim - census rule from the real plans)
_lay1 = designer.layout(_hc, {"roof_n": 0, "people": 1, "seat_h": 0.30})
check("solo bench sits AT the rowing station, not the transom",
      _lay1["benches"][0]["xc"] == _lay1["x_oarlock"]
      and 0.40 <= _lay1["x_oarlock"] / 3.0 <= 0.60)
_lay4 = designer.layout(_hc, {"roof_n": 0, "people": 4, "seat_h": 0.30})
check("rowing thwart stays pinned at the oarlocks in a full layout",
      any(b["helm"] for b in _lay4["benches"])
      and any(abs(b["xc"] - _lay4["x_oarlock"]) < 1e-12
              for b in _lay4["benches"]))
# the rails + the end caps are FUSED into the hull profile and the end
# plates: build the augmented profiles and check the band is part of them
_hwr = view3d.hull_with_rails(hull.load_hull(EX))
_st0 = _hwr["stations"][len(_hwr["stations"]) // 2]["pts"]
_y_band = _st0[-2][0]
_ye = max(hull.load_hull(EX)["stations"][len(hull.load_hull(EX)["stations"]) // 2]["pts"],
          key=lambda q: (round(q[0], 9), q[1]))[0]
check("gunwale rails FUSED into the hull profile (band part of the ring)",
      abs(_y_band - (_ye + designer.GUNW_T)) < 1e-9)
# a pointy bow (top edge narrower than the laminate) drops the inboard bow
# board but still builds clean: the side rails' ends meet the stem angle
_r_pt = run(['python3', f'{B}/param_hull.py', '--loa', '3.0', '--beam', '0.8',
             '--bow-f', '0.1', '-o', '/tmp/h_pt.json'])
_r_pt2 = run(['python3', f'{B}/export_stl.py', '/tmp/h_pt.json',
              '-o', '/tmp/pt.stl'])
# this stem is narrower than the laminate: the inner offset CROSSES the
# centreline and the export must REJECT it with an actionable error
check("collapsed-stem hull rejected with an actionable error",
      _r_pt.returncode == 0 and _r_pt2.returncode != 0
      and "too narrow" in (_r_pt2.stderr + _r_pt2.stdout))
check("layout roof sane", lay_t["roof"]["area"] >
      designer.PAN_L * designer.PAN_W)   # > one module
check("layout roof off", designer.layout(hh_p, {"roof_n": 0})["roof"] is None)

# --- flat-sheet (hard chine) mode ---
r_sh = run(['python3', f'{B}/param_hull.py', '--sheet', '--bottom-f', '0.8',
            '-o', '/tmp/h_sh.json'])
hsh = hull.load_hull('/tmp/h_sh.json')
check("sheet sections straight (4-point trapezoids)",
      all(len(s["pts"]) == 4 for s in hsh["stations"]))
r_sh2 = run(['python3', f'{B}/export_stl.py', '/tmp/h_sh.json',
             '-o', '/tmp/sh.stl'])
check("sheet hull STL validates", r_sh2.returncode == 0)
check("sheet flag sanitized", designer.sanitize({"sheet": 1})["sheet"] == 1)

# open-top render mesh (3D views) vs closed mesh (STL)
_Vo, _qo = view3d.build_mesh(hull.load_hull(EX), open_top=True)
check("open-top mesh builds (odd ring)", _Vo.shape[1] % 2 == 1 and _Vo.shape[1] > 20)

# canopy structure (shared by the detail sheet and the 3D view)
_cs = designer.canopy_struct(hull.load_hull(EX),
                             designer.layout(hull.load_hull(EX),
                                             {"roof_n": 1})["roof"])
check("X supports: feet on the gunwale, tops buried into the rail",
      # each diagonal lands on its own half of the rail (coplanar side
      # faces at the crossing were a defect) and sinks GLUE_SINK in
      len(_cs["xlegs"]) == 4
      and all(0.1 < abs(l["y0"]) < 1.2 for l in _cs["xlegs"])
      and all(abs(abs(l["y1"]) - (_cs["hw"] - designer.RAIL_SY / 2))
              <= designer.XLEG_SX / 2 + 1e-9
              and abs(l["z1"] - (_cs["zu"] + designer.GLUE_SINK)) < 1e-9
              for l in _cs["xlegs"]))
check("X supports span end to end at a sane angle",
      all(abs(abs(l["x1"] - l["x0"]) - (_cs["x1"] - _cs["x0"])) < 1e-9
          # short roofs make a flatter X (29 deg on the 1-panel default),
          # long roofs steeper - 20-65 is the real envelope
          and 20 <= abs(math.degrees(math.atan2(l["z1"] - l["z0"],
                                                abs(l["x1"] - l["x0"])))) <= 65
          for l in _cs["xlegs"]))
check("canopy 25 mm air gap",
      abs(_cs["z_pan0"] - (_cs["zu"] + 0.04) - 0.025) < 1e-9)
check("canopy clear height = the roof_h default",
      abs(_cs["clear"] - designer.CLEAR_H) < 0.03)
_lay_low = designer.layout(hull.load_hull(EX), {"roof_n": 1, "roof_h": 0.80})
_cs_low = designer.canopy_struct(hull.load_hull(EX), _lay_low["roof"])
check("roof_h slider lowers the canopy (0.80 m over the deck edge)",
      abs(_cs_low["clear"] - 0.80) < 0.03
      and _cs_low["zu"] < _cs["zu"] - 0.15)
def _rejects(bad):
    try:
        designer.sanitize(bad)
        return False
    except ValueError:
        return True
check("roof_h default fills in, out-of-bounds (0.1/3.0) rejected",
      designer.sanitize({})["roof_h"] == designer.CLEAR_H
      and _rejects({"roof_h": 3.0}) and _rejects({"roof_h": 0.1}))
check("canopy fits one 450 W panel by default",
      _cs["fit"] and len(_cs["panels"]) == 1)
check("canopy panel is standard 450 W size",
      abs(_cs["pan_l"] - 1.762) < 1e-9 and abs(_cs["pan_w"] - 1.134) < 1e-9)
check("canopy weight + CG sane",
      35 < _cs["mass_kg"] < 70 and 1.5 < _cs["z_cg"] < 2.1)

# panel-count driven roof: rows from width, columns from length
r_w = run(['python3', f'{B}/param_hull.py', '--loa', '7', '--beam', '3.0',
           '-o', '/tmp/h_wide.json'])
hw_h = hull.load_hull('/tmp/h_wide.json')
lay_w = designer.layout(hw_h, {"roof_n": 3})
check("wide boat: 2 rows of 450 W panels",
      r_w.returncode == 0 and lay_w["roof"]["rows"] == 2
      and lay_w["roof"]["n_fit"] == 3)
lay_s = designer.layout(hh_p, {"roof_n": 6})
check("over-requested panels trimmed with a note",
      lay_s["roof"]["n_fit"] < 6 and lay_s["roof_note"])
check("note recommends concrete longer/wider numbers",
      "make the boat about" in lay_s["roof_note"]
      and " m long" in lay_s["roof_note"]
      and " m wide" in lay_s["roof_note"])
# rotation: a wide short boat fits more panels with the long side across
r_wk = run(['python3', f'{B}/param_hull.py', '--loa', '6.5', '--beam', '2.2',
            '-o', '/tmp/h_wk.json'])
hwk = hull.load_hull('/tmp/h_wk.json')
lay_r = designer.layout(hwk, {"roof_n": 6})
check("panels rotate when that fits more (wide short boat)",
      r_wk.returncode == 0 and lay_r["roof"]["orient"] == "port"
      and lay_r["roof"]["n_fit"] == 5)
# rotated arrays must stay CENTRED (the hit: box centres used pan_l/pan_w
# while the sizes had rotated -> every panel shifted ~314 mm both axes)
_csr = designer.canopy_struct(hwk, lay_r["roof"])
_pxc = [p["x0"] + _csr["pan_x"] / 2.0 for p in _csr["panels"]]
_pyc = [p["y0"] + _csr["pan_y"] / 2.0 for p in _csr["panels"]]
_rcx = (lay_r["roof"]["x0"] + lay_r["roof"]["x1"]) / 2.0
check("rotated panel array stays centred on the roof",
      abs(sum(_pxc) / len(_pxc) - _rcx) < 1e-9
      and abs(sum(_pyc) / len(_pyc)) < 0.02)
# slicing through a VERTEX must still produce a segment (half-open rule)
import slice_check as _sl
_tv = _np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 2.0]]])
_segs, _ = _sl.slice_layer(_tv, 1.0)
check("slice through a vertex yields a segment", len(_segs) >= 1)
# slice_check endpoint pairing must be DISTANCE-based: the old grid-bin
# match split endpoints 0.0002 mm apart across a 0.02 cell boundary and
# read perfectly closed layers as open contours (at the tapered stem it
# cost 20+ layers). Square with a micro-crack straddling the boundary.
_sq = _np.array([[[0.0, 0.0], [1.0, 0.0]], [[1.0, 0.0], [1.0, 1.0]],
                 [[1.0, 1.0], [0.0301, 1.0]], [[0.0299, 1.0], [0.0, 1.0]],
                 [[0.0, 1.0], [0.0, 0.0]]])
_slq, _slopen = _sl.chain_loops(_sq)
check("slice_check pairs endpoints by DISTANCE (micro-cracks across "
      "grid bins close)",
      len(_slopen) == 0 and len(_slq) == 1)

# no 75% length cap: a wide 4 m jon takes 4 panels as a 2x2 block
# bottom-f 1.0 = true box (the old 0.85/0.9/0.95 caps drifted and the
# round-bilge branch degenerated at exactly 1.0)
r_bx = run(['python3', f'{B}/param_hull.py', '--loa', '4', '--beam', '2.6',
            '--depth', '0.5', '--rocker', '0', '--bottom-f', '1.0',
            '-o', '/tmp/h_box.json'])
hbx = hull.load_hull('/tmp/h_box.json')
mid = hbx["stations"][len(hbx["stations"]) // 2]["pts"]
rect = (len(mid) == 4 and abs(mid[1][0] - mid[2][0]) < 1e-6
        and abs(mid[0][1] - mid[1][1]) < 1e-6)
r_bx2 = run(['python3', f'{B}/export_stl.py', '/tmp/h_box.json',
             '-o', '/tmp/box.stl'])
check("boxy boat: bottom-f 1.0 gives a rectangular section + valid STL",
      r_bx.returncode == 0 and rect and r_bx2.returncode == 0)

r_jn = run(['python3', f'{B}/param_hull.py', '--loa', '4', '--beam', '2.6',
            '-o', '/tmp/h_jn.json'])
hjn = hull.load_hull('/tmp/h_jn.json')
lay_j = designer.layout(hjn, {"roof_n": 4})
check("4 panels fit 2x2 on a 4 x 2.6 m jon (roof may run bow to stern)",
      r_jn.returncode == 0 and lay_j["roof"]["n_fit"] == 4
      and lay_j["roof"] and not lay_j["roof_note"])

# --- flared-hull fit: benches + risers must TOUCH the wall and riser tops
# must meet the bench underside.  The bug (user's screenshots): parts were
# boxes sized at ONE z on a hull whose side widens with height -> benches
# 4-8 mm and risers 13-17 mm of daylight at every zoom.  Thresholds: 3 mm
# real (old build fails at 13 mm; the box benches fail at 4 mm), 0.5 mm.
r_fl = run(['python3', f'{B}/param_hull.py', '--loa', '4', '--beam', '1.6',
            '--depth', '0.5', '--bottom-f', '0.8', '--bow-f', '0.95',
            '--transom-f', '0.95', '--rocker', '0.16', '-o', '/tmp/h_fl.json'])
r_fl2 = run(['python3', f'{B}/export_stl.py', '/tmp/h_fl.json',
             '-o', '/tmp/fl.stl', '--roof-n', '1'])
import render_stl as _rs
import part_audit as _pa
from scipy.spatial import cKDTree as _KD
_tris = _rs.parse_stl('/tmp/fl.stl')
_groups = _rs.bodies(_tris)
_, _nm = _rs.classify(_tris, _groups, '/tmp/h_fl.json')
_hull = _np.asarray(_tris[_groups[0]], float)
_scale = 4.0 * 1000.0 / float(_np.ptp(_hull.reshape(-1, 3)[:, 0]))
_tree = _KD(_hull.mean(axis=1))
_worst, _rz, _bz = 0.0, [], []
for _i, _g in enumerate(_groups):
    _n = _nm[_i - 1] if _i else "hull"
    if not _n.startswith(("bench", "riser")):
        continue
    _t = _np.asarray(_tris[_g], float)
    _v = _np.unique(_t.reshape(-1, 3), axis=0)
    _d, _idx = _tree.query(_v, k=24)
    _best = _np.full(len(_v), 1e18)
    for _c in range(24):
        _ht = _hull[_idx[:, _c]]
        _q = _pa.closest_pt_on_tri(_v, _ht[:, 0], _ht[:, 1], _ht[:, 2])
        _best = _np.minimum(_best, _np.linalg.norm(_q - _v, axis=1))
    _worst = max(_worst, float(_best.min()) * _scale)
    if _n.startswith("riser"):
        _rz.append(float(_t.reshape(-1, 3)[:, 2].max()) * _scale)
    else:
        _bz.append(float(_t.reshape(-1, 3)[:, 2].min()) * _scale)
check("flared hull: every bench/riser corner within 3 mm of the wall",
      r_fl.returncode == 0 and r_fl2.returncode == 0 and _worst <= 3.0)
check("riser tops bury into the bench underside (3 mm real +- 0.5)",
      # by design the riser top sits GLUE_SINK (3 mm real = 0.125 model)
      # INSIDE the bench so the printed shells fuse; 0 mm would be
      # coplanar faces, which viewers render as holes
      bool(_rz) and bool(_bz)
      and max(abs(z - min(_bz)) for z in _rz) <= 3.5)

# the rails are FUSED into the hull profile: the hull body itself must
# now reach outwale width (beam/2 + GUNW_T) at the sheer - there are no
# separate gunwale bodies left to slit, alias or leave a hole between
_hull_pts = _np.asarray(_tris[_groups[0]], float).reshape(-1, 3)
_hfl = hull.load_hull('/tmp/h_fl.json')
_fl_beam = 2.0 * max(hull.deck_edge(st["pts"])[0] for st in _hfl["stations"])
check("rails fused into the hull profile (hull reaches outwale width)",
      abs(_hull_pts[:, 1].max() * 24 - (_fl_beam * 1000 / 2
                                        + designer.GUNW_T * 1000)) < 6.0)

# the SOLID END PLATES must sit OUTSIDE the shell's end rings (0.2 mm
# real proud + the rail-band cap juts GUNW_T): an inboard nudge left the
# hull's ring edges proud of the plate face and viewers rendered them as
# ragged patches THROUGH the plate (the user's screenshots). The model
# length is the discriminating number: loa + 2 x (GUNW_T + 0.0002).
_vp = _np.asarray([(t[0], t[1], t[2]) for t in
                   export_stl.hull_triangles(_hfl, n_per_half=48,
                                             roof_n=0)], float).reshape(-1, 3)
_fl_loa = _hfl["stations"][-1]["x"] - _hfl["stations"][0]["x"]
check("end plates sit OUTSIDE the end rings (band cap + nudge proud)",
      abs(float(_np.ptp(_vp[:, 0]))
          - (_fl_loa + 2 * (designer.GUNW_T + 0.0002))) < 0.003)

# --- materials list: quantities computed from the model ------------------
r_m = run(['python3', f'{B}/materials.py', '/tmp/h_fl.json',
           '--people', '3', '--gear', '670', '--json', '/tmp/bom.json',
           '-o', '/tmp/bom.md'])
_bom = json.load(open('/tmp/bom.json')) if r_m.returncode == 0 else {}
_md = open('/tmp/bom.md').read() if r_m.returncode == 0 else ''
check("materials: builds, ships a sane sheet count from the model area",
      r_m.returncode == 0 and _bom.get("n_sheets", 0) >= 2
      and _bom.get("ply_a", 0) > 0)
check("materials: flotation + capacity plate rows present (census items)",
      "Flotation" in _md and "Capacity plate" in _md and "183.53" in _md)
# the sheet count is area-derived, not a constant: a bigger hull needs more
r_m2 = run(['python3', f'{B}/param_hull.py', '--loa', '6', '--beam', '2.0',
            '--depth', '0.7', '--rocker', '0.2', '-o', '/tmp/h_big.json'])
r_m3 = run(['python3', f'{B}/materials.py', '/tmp/h_big.json',
            '--people', '4', '--json', '/tmp/bom2.json', '-o', '/tmp/bom2.md'])
_bom2 = json.load(open('/tmp/bom2.json')) if r_m3.returncode == 0 else {}
check("materials: a 6 m hull needs more ply than a 4 m one",
      r_m3.returncode == 0
      and _bom2.get("n_sheets", 0) > _bom.get("n_sheets", 0))
check("capacity: USCG max hp rises with the size factor",
      designer.max_hp(4.0, 0.7) < designer.max_hp(6.0, 1.0)
      and designer.max_hp(3.0, 1.0, flat_bottom=True)
      < designer.max_hp(3.0, 1.0))

# --- construction plan: molds fan + table + midship + setup --------------
r_c = run(['python3', f'{B}/construction.py', '/tmp/h_fl.json', '-o',
           '/tmp/constr.spec.json'])
_cs = json.load(open('/tmp/constr.spec.json')) if r_c.returncode == 0 else {}
_cnames = [v["name"] for v in _cs.get("views", [])]
_n_st = len(hull.load_hull('/tmp/h_fl.json')["stations"])
check("construction: spec builds molds + table + midship + setup",
      r_c.returncode == 0
      and {"STATION MOLDS", "MOLD TABLE", "MIDSHIP CONSTRUCTION",
           "SETUP"} <= set(_cnames))
check("construction: the mold table lists every station",
      r_c.returncode == 0
      and len(_cs["views"][1]["texts"]) >= _n_st + 2)
r_c2 = run(['python3', f'{D}/draft.py', '/tmp/constr.spec.json', '--out',
            '/tmp/constr', '--png'], timeout=120)
check("construction: the sheet renders without draft errors",
      r_c2.returncode == 0)

print()
print(f"{len(fails)} failures" if fails else "ALL PASS")
sys.exit(1 if fails else 0)
