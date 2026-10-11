---
name: boat-engineering
description: Naval-architecture calculations and design workflow for small craft (up to ~10 m): flotation and displacement, weight/load capacity (persons and kg), hydrostatics from an offsets table (KB, BM, KM, TPC, MCT, LCB), stability (GM, GZ curve, inclining-test KG), freeboard and reserve buoyancy, and electric/solar weight and energy budgets. Use when designing or sizing a boat hull, computing how much a boat can carry, checking flotation/stability, planning a boat build, or doing weight/energy budgets for small boats — especially home builds with electric drive and solar. Produces a numbers sheet with formulas; hands dimensioned plans to the engineering-drafting skill.
---

# Boat engineering (small craft)

Engineering calculations for small craft, driven by a table of offsets.
Every number in output must be **computed from the hull model or explicitly
listed as an assumption** — never sketched in by eye. If a value is unknown,
mark it REF and ask the user; do not invent dimensions.

- Calculation units: **meters + kg**. Drawings: **mm**, 1:1 model space.
- Water density: fresh **rho = 1000**, sea **1025 kg/m3** (default 1000).
- State assumptions with results: even keel, no trim, static, polygonal hull.
- Deliverables are engineering estimates, not a certified plan. Say this once
  per deliverable; an inclining test of the real boat and (for open water) a
  naval architect's review are separate steps.

## Workflow

0. **Requirements** — fill this checklist with the user before any geometry:
   type (displacement/planing/sailing), waters (design category C/D),
   crew count (persons x 75 kg), propulsion (motor kW, battery kWh, solar kWp),
   target LOA, material/build method, max draft, load cases
   (lightship / day trip / max). Ask for what's missing.
1. **Hull model** — an offsets JSON (format below). Sources: the user's
   sketch/photos of mm-paper (extract dimensions, mark derived values REF and
   confirm), `scripts/param_hull.py` for a working starter hull, or refined
   lines from an existing design. Stations must be evenly spaced.
2. **Hydrostatics** — `hydrostatics.py`; pick the design draft/displacement.
3. **Weight budget** — `budget.py`; gives total, LCG, KG, equilibrium draft,
   freeboard, GM, and the capacity cap. Iterate weights if KG is too high.
4. **Stability** — `stability.py` with that KG; check the GZ curve and the
   deck-edge immersion angle against `references/standards-capacity.md`.
5. **Capacity & standards** — persons/load marking logic and swamped-flotation
   reserve buoyancy: `references/standards-capacity.md`. Electric/solar sizing:
   `references/electric-solar-budget.md`.
6. **Plans** — `linesplan.py` emits a drawing spec; render it with the
   engineering-drafting skill. Always also export the offsets table
   (`--offsets`) for lofting. For the build itself: `construction.py`
   (station molds + setup table + midship details + sequence) and
   `materials.py` (bill of materials with sheet counts, flotation and
   USCG capacity). `view3d.py` renders shaded 3D views from six
   directions plus the waterline at design draft (visualization aid). For
   physical/3D-printed models: `export_stl.py` writes a watertight STL at
   any scale (default 1:24).
7. **Numbers sheet** — Markdown with: inputs, assumptions, results table,
   checks (GM, freeboard, capacity vs standard), warnings, and every command
   that produced a number (reproducible).

## Offsets format (hull JSON)

```json
{ "name": "5.2 m launch", "units": "m",
  "stations": [
    { "x": 0.0, "section": [[0,0.00],[0.40,0.10],[0.66,0.25],[0.79,0.45],[0.85,0.62],[0.85,0.78],[0,0.78]] },
    { "x": 0.65, "section": [ "...same point count, next station..." ] }
  ] }
```
- `x` from station 0 (transom) forward; evenly spaced; >= 3 stations (odd = best).
- `section`: half-section from centerline keel (y=0, low) up the hull side,
  over the deck edge (max y), back to centerline deck (y=0, top). First and
  last points y=0. Deck edge = where water first enters (open boats: the sheer).

## Scripts (run with python3; hull/hydrostatics/stability/budget/linesplan are stdlib-only,
param_hull/designer/export_stl/view3d/render_stl need numpy, slice_check also scipy,
occlusion_test numpy+matplotlib+scipy)

| script | what |
|---|---|
| `scripts/hull.py` | shared model/geometry (imported, not run) |
| `scripts/param_hull.py` | starter hull from LOA/beam/depth/rocker/sheer/transom/mid/bow/bottom factors (`--sheet` = hard-chine flat panels) |
| `scripts/hydrostatics.py` | table or single draft for a mass; V, Aw, Cb, KB, BM, KM, TPC, MCT, LCB, freeboard |
| `scripts/stability.py` | GM + GZ curve (heave solved per heel angle, no trim), deck-immersion angle |
| `scripts/budget.py` | items -> total/LCG/KG, equilibrium draft, GM, capacity cap |
| `scripts/linesplan.py` | drawing spec (profile/half-breadth/body plan) + offsets CSV/MD |
| `scripts/view3d.py` | shaded 3D views from six sides + waterline plane (needs numpy/matplotlib) |
| `scripts/export_stl.py` | watertight scaled STL (hull + benches + gunwale + solar canopy; `--scale 24`, `--roof-n`) |
| `scripts/render_stl.py` | per-part coloured z-buffer sheets of an STL (+ parts + exploded) |
| `scripts/slice_check.py` | layer-by-layer slice audit of an STL: closed contours, wall gaps |
| `scripts/occlusion_test.py` | marker-wall see-through test (hull must be opaque) |
| `scripts/designer.py` | easy local web UI (sliders → live 3D + numbers, one-click plan/STL export; port 8765, files in ~/boat) |

Pipeline example (demo boat: the numbers below come from
`examples/example_budget.json` — swap in your own):

```bash
S=${CLAUDE_SKILL_DIR}
python3 $S/scripts/param_hull.py --loa 5.2 --beam 1.9 --depth 0.78 -o hull.json
python3 $S/scripts/hydrostatics.py hull.json --drafts 0.15:0.05:0.45
python3 $S/scripts/budget.py hull.json budget.json
python3 $S/scripts/stability.py hull.json --kg 0.53 --mass 838 --angles 0:10:90
python3 $S/scripts/linesplan.py hull.json -o plan.spec.json --draft 0.25 --offsets offsets.csv
python3 ~/.claude/skills/engineering-drafting/scripts/draft.py plan.spec.json --out linesplan --png
```

## References (load when relevant)

- `references/hydrostatics.md` — formulas, coefficients, sanity ranges.
- `references/stability.md` — GZ method, assumptions, inclining test procedure.
- `references/standards-capacity.md` — ISO 12217 / design categories, capacity
  and persons marking, swamped flotation.
- `references/electric-solar-budget.md` — motor power vs speed, solar yield,
  battery/panel weights.
- `references/materials-weights.md` — mass data for budgets.
- `references/small-boat-plans-survey.md` — real plan survey: seat/riser practice,
  helm position, gunwale attachments the builder follows.

## Guidelines (binding)

Any change to this skill's code must follow `GUIDELINES.md` in this
directory: named/derived constants, units discipline, negative-tested
verifiers, honest test names, and the mandatory completion checklist.

## Limits

No trim computation, no dynamic/CFD effects, no scantling/structural sizing
(v2: panel development for plywood builds, structure drawings). GM/GZ from
this tool are static, even-keel estimates — good for design iteration, not a
stability booklet. Verify the built boat's KG by inclining test
(`references/stability.md`).
