# boat-engineering

A Claude Code **skill** for designing small craft (up to ~10 m) — parametric
hull design, hydrostatics, stability, solar/electric budgets, drawing sheets,
and a **print-ready 1:24 STL** with the interior fitted (benches, risers,
gunwale sandwich, solar canopy with X-supports).

Everything is computed from a hull model — every number is derived or listed
as an assumption, never sketched by eye.

## What's inside

| Path | What it is |
|---|---|
| `SKILL.md` | The skill definition (what Claude reads when using it) |
| `scripts/param_hull.py` | Parametric hull generator → offsets JSON |
| `scripts/designer.py` | Local web app (sliders → live boat, exports). `python3 designer.py` → http://127.0.0.1:8765 |
| `scripts/export_stl.py` | Watertight 1:24 STL: hull shell + rails + benches + risers (+ canopy), every part fused |
| `scripts/view3d.py` | 3D preview + the shared profile/rail geometry |
| `scripts/hull.py`, `hydrostatics.py`, `stability.py`, `budget.py` | The numbers |
| `scripts/linesplan.py` | Lines plan (PDF/DXF/PNG) |
| `scripts/part_audit.py` | Part-fit gate: floating parts, wall pokes, overlaps (mm at model scale) |
| `scripts/slice_check.py` | Layer closedness gate (every slice must close) |
| `scripts/render_stl.py` | Color-coded check renders of the STL |
| `tests/regress.py` | Regression suite (ALL PASS is the gate) |
| `GUIDELINES.md` | Hard rules for changing this code (learned the hard way) |

## Use as a CLI (no server needed)

```bash
python3 scripts/param_hull.py --loa 4.0 --beam 1.4 --depth 0.5 \
    --bottom-f 1.0 --rocker 0.16 -o hull.json
python3 scripts/export_stl.py hull.json -o boat.stl --roof-n 0
python3 scripts/part_audit.py boat.stl          # fit gate
python3 scripts/slice_check.py boat.stl         # print gate
```

## Use as a Claude Code skill

Clone into `~/.claude/skills/boat-engineering` (or add this repo as a plugin
source) and ask Claude for a boat; the skill drives the workflow from
requirements to a print-ready model.

## 3D printing notes

- The STL is 1:24, millimetres — resize in the slicer for other scales.
- **All parts are fused**: no coplanar contact faces (viewers/slicers render
  those as holes) and no floating parts. Mating parts are either part of one
  surface (hull, rails, end caps) or buried a fraction of a print layer into
  what they bond to (benches, risers, canopy).
- Verification is built in: `part_audit.py` (attachment + pokes),
  `slice_check.py` (closed layers), `tests/regress.py` (behavior).

## Requirements

Python 3.10+, `numpy`, `scipy`, `ezdxf` (for the lines plan).

## Known limits (measured, documented)

- `part_audit` treats samples within 0.25 model mm (6 mm real) of the wall
  as print-burial, not pokes. Derived: 0.125 burial + a measured ≤0.235
  ceiling for curved-hull row chords at narrowing stems. The defect class
  this gate exists for (parts 13+ mm real off / through the wall) reads
  0.54+, so the band separates cleanly.
- Zero-area triangles: a few remain where horizontal profile runs collapse
  mirror strips; the exporter drops them unless that would open the mesh.
  Slicers ignore them.
- The audit gates are calibrated at the default 1:24 print scale. Much
  smaller scales shrink the 1.2 mm printed laminate below the burial; do
  not print this model below ~1:48 without re-checking.
- Numbers are engineering estimates, not a certified stability booklet.

## License

See repository settings.
