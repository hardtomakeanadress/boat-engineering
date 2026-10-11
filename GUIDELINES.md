# boat-engineering — hard rules for changing this code

Scope: every update, fix, or feature in this skill directory. These rules come
from bugs that actually happened here (hardcoded geometry that broke when the
boat changed, constants that drifted between modules, unit mixups, verifiers
that passed vacuously, tests named after behavior they never exercised).
The end-of-change checklist is mandatory — a change is not done until it runs.

- Inspect the relevant source, schema, and configuration before changing behavior.
- Separate verified constraints from assumptions; identify unresolved constraints before implementation.
- Stop dependent work when evidence invalidates a proposed design; obtain agreement before changing architecture or dependencies.

## Constants and derivation

- Define each physical constant once; import or pass it to every consumer (hull.py and designer.py hold the shared blocks — do not re-type a number that exists there).
- Derive dimension-dependent geometry from the current boat parameters and actual attachment geometry; never from another boat's numbers.
- No unexplained literal in a formula that tracks the hull: name it (a spec, a tolerance, or a design constraint) or derive it.
- A retained literal must be documented as a physical specification, a numerical tolerance, or an explicit design constraint.
- Recompute dependent geometry whenever a controlling dimension changes; invalidate affected caches.
- Sweep LOA and beam independently through minima, interior values, and maxima; assert clearances and attachment alignment at each.

## Units

- Declare units for every dimensional input, persisted field, calculation boundary, and drawing output.
- Internal calculation units are metres and kilograms; drawing sheets are millimetres — convert explicitly at boundaries.
- Convert both operands to the same unit before arithmetic or comparison. Never subtract millimetres from metres in a drawing expression.
- Apply drawing scale after physical-unit conversion; keep paper coordinates separate from model coordinates.
- Reject missing or unknown unit keys with an actionable error; never default missing units to metres.
- Verify unit consistency with a test that feeds equivalent metre and millimetre inputs and expects equivalent geometry.

## Verification (renders, STLs, checks)

- Derive verification fixtures from each boat's geometry, or declare the fixture's applicability limits. A fixture tuned to one demo boat is a dead test.
- Assert the intended target exists, lies inside the view, and produces the required number of eligible samples.
- Fail verification when required samples or intersections are absent; never count an empty evaluation as a pass (state coverage in the output).
- Include a known-visible case that must read visible and a known-occluded case that must read occluded.
- Deliberately break the tested condition and confirm the check FAILS; then restore and confirm it passes (negative test, every time).
- Keep expected results specified independently; never derive the oracle by calling the implementation under test.
- Measure contact on SURFACES, not only vertices: a box face resting flush on another surface still has its nearest VERTICES millimetres apart (the foot-plate false alarm), and a face floating 5 mm above a curved surface still has a closest vertex on it. Sample face interiors AND part corners.
- A part that meets a curved or sloped hull surface must be built from the hull's own geometry at EVERY height/station it spans (bevelled bench/riser ends measured at each corner's x and z; riser tops derive from the bench underside as built) — never a box sized at one z, and never a stale thickness assumption (the 40 mm bench that never existed left risers 11 mm short).
- NO COPLANAR CONTACT FACES: mating parts must either share topology or overlap by GLUE_SINK (3 mm real = 0.125 model = one eighth of a print layer). Faces exactly on each other are coplanar; viewers and Tinkercad's importer render coplanar opposite-facing pairs as white holes/slots (1,014 such pairs shipped before this rule; the exact-count check for them is part of every geometry review). Nothing floats (gap) and nothing is coplanar (touching face-to-face); if a render still shows an artifact, measure, don't guess — the ray sweep and the coplanar-pair counter in this session's history are the tools.
- Look at the rendered output yourself before claiming geometric or visual correctness — open the image, check attachments, clearances, clipping, labels, scale.
- Verification tools return nonzero exit status when they fail, so scripts can gate on them.

## Tests

- Make every test name describe the behavior its setup and assertions actually exercise (`--stations 13` in the "13-station" test, explicit `--beam` when asserting beam).
- Assert output values and structure; reject tests that only check a file exists or an exit code.
- Add a regression test that fails on the previous implementation for every corrected bug.
- Test rejected inputs alongside accepted ones (missing units, negative dimensions, NaN, unsupported shapes).
- Use numerical tolerances expressed in the quantity's unit and justified by the requirement.
- Never silently substitute a default demo boat in a parameterized test; report skipped/empty cases explicitly.

## Adding features

- Identify every new feature's inputs, derived values, UI controls, saved fields, history behavior, and endpoints before coding.
- Derive slider bounds from supported model constraints (backend BOUNDS and UI ranges must agree); do not copy ranges from one example boat.
- Enforce the same bounds in UI controls, direct entry, loading, and API validation.
- Test each slider's minimum, maximum, step, and values just outside its range.
- Define persistent keys, units, schema handling, and compatibility behavior before adding saved state; test save/load round trips for nondefault values.
- Record each user edit as an undoable action (the Recent-boats history snapshots before a preset click); group one slider drag into one entry.
- New geometry draws in ALL consumers the same way: web preview, colored renders, STL export, drawing sheets, classifier expectations — one definition, several views.

## Completion checklist — mandatory

- [ ] Run the regression suite; record command, result, failures, skips.
- [ ] Run the new regression against the previous behavior and confirm it detects the defect.
- [ ] Check authoritative constants and their consumers for remaining duplicated values.
- [ ] Check unit conversions, missing-unit rejection, and drawing dimensions.
- [ ] Exercise supported dimension extremes and independently varied LOA and beam.
- [ ] Run positive AND negative verification cases; confirm nonempty fixture coverage.
- [ ] Generate representative renders and drawing sheets; open and inspect them yourself.
- [ ] Exercise every endpoint in the tool being changed; record expected response and state effects.
- [ ] After ANY page/JS (PAGE) edit: run `scripts/page_check.sh` (fails on browser console errors and on stats that never rendered). A one-character quote typo in the page JS once took the whole site down silently.
- [ ] Exercise endpoint validation failures and confirm rejected requests leave state unchanged.
- [ ] Exercise slider bounds, persistence round trips, undo, and redo for affected features.
- [ ] Mark checks without an applicable feature as N/A with a reason; disclose anything unverified.
- [ ] Report verified results and unresolved constraints; claim completion only when required checks pass.
