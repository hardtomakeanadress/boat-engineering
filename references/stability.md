# Transverse stability - method, checks, inclining test

## What the script computes

For each heel angle theta (rotation about the longitudinal axis, +theta =
starboard down), the boat is re-floated with its original displacement: the
waterline slope is fixed at theta and the height is solved (bisection) so
V(theta) = mass/rho. No trim (rotation about the transverse axis) is modeled.

- Submerged condition in body frame: z*cos(theta) - y*sin(theta) <= c
  (y from centreline, z from baseline; c solved per angle).
- B(theta) = centroid of displaced volume, (ybar, zbar).
- KN(theta) = ybar*cos(theta) + zbar*sin(theta)   (lever of buoyancy from
  the keel point K).
- GZ(theta) = KN(theta) - KG*sin(theta)           (righting arm at angle).
- Small angles: GZ -> GM*sin(theta), GM = KM - KG; the script prints GM from
  the even-keel hydrostatics as an independent check.

## Reading the output

- **GM**: initial stiffness. GM ~ 0.3 - 0.5 m is comfortable for small
  launches; GM < 0.15 m will feel tender; GM <= 0 means it does not float
  upright. (Tenderness is not instability - but check criteria.)
- **GZ curve shape**: max GZ and the angle where it occurs (good small-craft
  shapes peak around 40 - 70 deg); the angle where GZ returns to zero = range
  of stability.
- **Deck-edge margin** per row: distance from the water surface to the
  modeled deck edge (both sides, min). Once <= 0, an open boat ships water -
  that is the practical limit, and GZ beyond that angle is only formal. For
  decked boats with openings, the true downflooding point is the LOWEST
  OPENING, not the modeled edge - opening positions are not modeled here;
  check sills/hatches separately against the waterline.
- Areas under GZ (0-30, 0-40) are printed for comparison with criteria.

## Wall-sided approximation (hand check)

GZ ~ sin(theta) * (GM + 0.5 * BM * tan^2(theta)). Use it to sanity-check a
couple of points of the numerical curve; deviations of a few % are normal
until deck immersion.

## Inclining test (measure the REAL boat's KG after building)

Purpose: verify the KG used in these calculations. Small-craft procedure:

1. Calm, sheltered water; boat free of loose weights; mooring lines slack.
2. Record absolute draft marks both sides (-> displacement & KM at that
   draft from the hydrostatics table).
3. Put known inclining weights (typically 1-2% of full displacement total)
   as one movable mass w; measure transverse shift distance d.
4. Heel measurement: a pendulum (string + plumb) of length L with deflection
   reading x gives tan(phi) = x/L; or a digital inclinometer. Target.
   2-4 degrees of heel - enough to read, small enough for linearity.
5. GM = w * d / (W * tan(phi)), with W = total displacement INCLUDING the
   inclining weights and everyone on board in their test positions.
6. Repeat shifting to both sides, several readings, average.
7. KG = KM(draft) - GM. Compare with the budget's KG; if different, redo the
   stability and load checks with the measured value.

## Assumptions / limits

- Static, still water, even keel start, no trim, rigid-hull, no free-surface
  effects from liquids (keep tanks full or model the free-surface correction
  separately - v2).
- Persons moving = weight moving; where the crew sits IS the KG input.
- Criteria (what GZ/heel limits to pass) depend on the chosen standard -
  see `standards-capacity.md`.
