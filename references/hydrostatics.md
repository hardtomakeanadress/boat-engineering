# Hydrostatics - formulas and sanity ranges

What `scripts/hydrostatics.py` computes and why. Units m / kg.

## Displaced volume and mass

- V(T) = volume below waterline T, integrated from section areas:
  half-section is closed into a full section, clipped at z <= T, area via
  shoelace; V = Simpson integral of full-section areas over x.
- Displacement mass = rho * V. rho = 1000 (fresh), 1025 (sea).
- Block coefficient Cb = V / (Lwl * Bwl * T).

Typical Cb at rest (sanity - verify per design):
| craft | Cb |
|---|---|
| rowing dinghy / runabout | 0.35 - 0.45 |
| moderate motor launch | 0.45 - 0.55 |
| sailboat (fin keel) | 0.30 - 0.40 |

## Simpson integration

For evenly spaced ordinates (our stations MUST be evenly spaced):

- 1/3 rule, 3 ordinates: A = h/3 * (y0 + 4 y1 + y2)
- 1/3 composite (even interval count): h/3 * (y0 + 4y1 + 2y2 + 4y3 + ... + yn)
- odd interval count: 3/8 rule on the last three intervals:
  A = 3h/8 * (y0 + 3y1 + 3y2 + y3), 1/3 rule on the rest.

Worked micro-example: ordinates 0, 0.5, 1 at x = 0, 0.5, 1.0 -> area =
0.5/3 * (0 + 4*0.5 + 1) = 0.5. Reverse-check by geometry: triangle area 0.5.

## Centers and radii

- KB (height of centre of buoyancy) = integral(A(z_centroid)) / V.
  Sanity for round/V sections: KB ~ 0.5 - 0.65 T.
- LCB (longitudinal centre, from datum) = integral(A * x) / V.
- I_T (waterplane transverse moment of inertia) = (2/3) * integral(y_half^3 dx)
- BM (transverse metacentric radius) = I_T / V.
  Magnitude sanity: BM ~ (Bwl^2 / (12 T)) * f(Cw, Cb), f roughly 0.8 - 1.1.
- KM = KB + BM.
- Longitudinal: I_L = 2 * integral(y * (x - xm)^2 dx), BM_L = I_L / V.

## Practical numbers

- TPC (mass to sink 1 cm parallel) = rho * Aw / 100 [kg/cm]. Cross-check:
  displacement growth between two drafts ~= TPC * draft difference (in cm).
- MCT (moment to change trim 1 cm) ~= disp * BM_L / (100 * L) [kg*m/cm]
  (uses BM_L for GM_L - adequate for small craft, note it in reports).
- Fresh vs sea water: moving a 1000 kg boat from fresh to sea raises it by
  about disp/TPC*0.025 cm ... simpler: displacement changes 2.5% at the same
  draft; TPC and freeboard follow rho.

## Assumptions and pitfalls

- Even keel, no trim. A sizeable weight off-centre changes LCB matters -
  check LCG from the budget against LCB.
- Sections are polygons - refine the input point count where curvature
  matters (bilge).
- The deck edge matters: freeboard_min in the output is the first place
  water enters. For open boats the boat is "full" there - not a legal
  capacity by itself (see standards-capacity.md).
- Lwl/Bwl/Cb from the station span are approximations; with >= 7 stations the
  error is small on a fair hull. More stations near the bow help.
- A boat can be stable and still unsafe: reserve buoyancy and downflooding
  are separate checks (standards ref).
