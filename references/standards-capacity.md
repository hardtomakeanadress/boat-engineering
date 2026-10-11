# Small-craft standards: design categories, capacity, flotation

Compiled 2026-10-10 from national reproductions of ISO 12217 (GOST ISO
12217-1/-3, KRS "Guidance for Recreational Crafts 2026"), RCD 2013/53/EU
Annex I and published assessment reports. The ISO texts are paywalled and
editions differ (2015 vs 2022) - verify against the edition you would
certify with. This reference maps the rules to what the scripts compute; it
is NOT the standard and not an assessment.

## ISO 12217 parts (no Part 4 exists)

| part | applies to | categories assignable |
|---|---|---|
| 12217-1 | non-sailing, hull length 6-24 m | A, B, C, D |
| 12217-2 | sailing, hull length 6-24 m | A, B, C, D |
| 12217-3 | all boats < 6 m | **C or D only** |

A < 6 m boat that is fully decked with ISO 11812 quick-draining recesses may
be assessed to Part 1/2 instead and get a higher category.

Design categories (RCD Annex I): A wind > 8 Bft / waves > 4 m; B <= 8 / <= 4 m;
C <= 6 / <= 2 m; D <= 4 / <= 0.3 m (occ. 0.5 m). Calculation wind speeds used
by the standard: A 28, B 21, C 17, D 13 m/s.

## Boats < 6 m (Part 3) - the checks that matter

**Offset-load test** (the main binding check for small open boats):
- Allowed heel: phi(O) = 11.5 + (24 - L_H)^3 / 520 degrees.
  2.5 m: 30.6 deg; 4.0 m: 26.9; 5.0 m: 24.7; 6.0 m: 22.7.
- People at 85 kg each, all seated on one side.
- Residual freeboard at that heel (deck edge above water): Cat C >= 0.10 m
  (0.15 for some options), Cat D >= 0.01 m.
- The boat's righting moment before downflooding must exceed the heeling
  moment. With our toolkit: get heel angle for the test heeling moment from
  the GZ curve; check heel <= phi(O) and freeboard.

**Downflooding height** (Annex A formula, shared with Part 2):
h_D(R) = H1 * F1 * F2 * F3 * F4 * F5, with H1 = L_H / 15; factors F1 (bow
position, 0.5-1.0 up to 1.2 near the bow), F2 (opening size, <= 1.0, based on
50*L_H^2 mm2), F3 (recess 0.7-1.2), F4 from displacement, F5 (0.8 for some
options). Absolute bounds: C 0.3-0.75 m, D 0.2-0.4 m.
Fixed minimums: hatch/portlight sills >= 0.2 m (C) / 0.1 m (D) above the
loaded waterline; drains rise >= 0.06 / 0.04 m; the outboard-start case (one
75 kg person 0.5 m forward of the motor) needs least opening height > 0.1 m.

**Capacity and marking**:
- Max load = the load at which ALL checks still pass (offset load, freeboard,
  downflooding, and flotation option). Persons = crew limit from the test
  mass / 85 kg, rounded down, also capped by ISO 14946 seating geometry
  (0.40 x 0.75 m per seat; 0.75 x 0.75 m per standing person).
- Marked on the builder's plate per ISO 14945: category, max persons, max
  load, max motor power (power is a builder declaration; ISO 12217 caps power
  only for one < 6 m option at 3 kW).
- **Our budget.py displacement cap is NOT the rating.** The offset-load /
  freeboard / flotation checks normally bind long before the hull runs out of
  buoyancy. Report the cap as "physical limit", never as capacity.

**Swamped flotation** (required for the open-boat options, not for
fully-decked fast-draining ones): buoyancy volume V_B > m_TEST / 930 [m3]
(basic flotation; one sailing option uses /850). V_B sums hull-structure
volume by material density, fixed tanks/batteries, foam and air boxes -
trapped air and crew do not count. Rules of thumb: 1 m3 trapped air = 1000 kg
fresh-water buoyancy; foam net buoyancy ~ (rho_water - rho_foam - absorbed
water); plywood counts as lift = mass * (1-SG)/SG (SG 0.55 -> ~0.8 kg lift
per kg). Level-flotation test criteria: heel < 45 deg after 5 min with
weights at L_H/3 each side; > 2/3 of the gunwale length above water.

## Boats 6-24 m (Part 1, non-sailing) - headline criteria

- Cat A/B (fully decked): righting moment at 30 deg >= 25 kN*m (A) / 7 kN*m
  (B) with GZ(30) >= 0.2 m; residual righting moment >= 2.0 / 1.0 / 0.5
  times the loaded-arrival mass (equivalent GZ ~0.20 / 0.10 / 0.05 m).
- Minimum downflooding angle: 30 deg (A) / 25 deg (B).
- Cat C/D wind heel: heel < 70% of the offset-load heel limit AND < 70% of
  the downflooding angle. Offset-load heel limit table extends:
  6 m 22.7, 7 m 20.9, 8 m 19.4, 10 m 16.8, 12 m 14.8, 15 m 12.9 deg.
- Minimum operating-condition mass added to the light craft: 75 kg (<= 8 m),
  150 kg (8-16 m), 225 kg (16-24 m).
- Swamped-flotation option: when swamped, support a dry mass = 133% of the
  max total load.

## Sailing (Part 2, brief)

Downflooding angle >= 40 (A,B) / 35 (C) / 30 (D) deg; AVS >= max(130 -
0.002*m, 100) (A), >= max(130 - 0.005*m, 95) (B), 90 (C), 75 (D) deg;
minimum righting energy 172,000 / 57,000 kg*m*deg (A/B); STIX > 32 / 23 / 14
/ 5; flotation volume > m_LDC / 850 m3.

## How to use with this toolkit

1. Decide part: < 6 m -> Part 3 (C/D), >= 6 m -> Part 1/2.
2. Control the offset-load case by hand - stability.py takes only KG and
   total mass; it does NOT solve offset loading. Heeling moment
   M_h = sum(m_i * g * y_i) with the 85 kg persons on ONE side, y measured
   from the centreline (the transverse lever - x/z do not enter the heeling
   moment). Righting moment = delta * g * GZ(theta): find theta where the
   two balance (interpolate the GZ curve), then check theta <= phi(O) limit,
   residual freeboard at that heel, and that the curve is still valid there
   (no deck immersion). A transverse-CG/offset-load solver is not
   implemented (v2).
3. Check downflooding heights/sills against the boat's actual openings.
4. For a real rating (CE plate): use the standard text and a professional
   assessment; these numbers get you a design that is in the right region,
   which is a different thing.

Confidence notes: heel-limit formula, RM values and category wind speeds
matched across independent sources (high). Residual-freeboard tables and the
V_B/m_TEST divisor vary slightly between editions (medium) - re-check in the
edition you use. Person weight: 85 kg is the boat-standard test/capacity
weight and is the default in budget.py (`person_kg`); 75 kg is a fine
planning weight for calm-water KG work.
