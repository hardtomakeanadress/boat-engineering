# Small-boat plans survey (studied 2026-10-10)

Why: learn from REAL plans how small open boats (4-6 m class) are laid out and
built — especially **how seats/benches attach** and **where the helm sits**.
Studied across types: pram, shallow-V skiff, dinghy, jon boats, punts, skiffs.

## Plans studied

| Boat | Type / size | Build method | Seat construction — the real detail |
|---|---|---|---|
| **Robin** (Duckworks, D. Munkelt) | 8-9 ft skin-on-frame pram | SOF + plywood floor | Seat = ply rectangle + 2 rails 3/4"x1-1/4". **Risers 3/4"x1-1/4", top BEVELLED** so the seat sits level; risers span 4-5 ribs. Seats clamped/lashed for launching, glued only after trim is found |
| **Kastri 12** (A. Walters, Duckworks) | 12' x 5' shallow-V skiff, short-shaft outboard | stitch & glue | Longitudinal **seat risers**; seats used as the build jig, then removed; hull **taped + glued to the frames and seat riser**; **perimeter battens** added to fix seats/foredeck; riser cut to the outer edge, gap made up with epoxy filler/resin/tape |
| **FL12** (Bateau.com, S&G 101 tutorial) | 12' dinghy | stitch & glue | Seats = **seat tops + seat frames = a closed BOX**; battens glued around frames, seat top glued on, edges covered by **fiberglass tape**; loose fits are fine — tape + epoxy do the structural work |
| **12' jon boat** (3dboatdesign build text) | 12' flat-bottom jon | plywood over frames, glue + nail | **Seat stringers nailed/glued to the frames**; 3 seats installed onto the stringers afterwards; the seats are structural cross-members (they hold the sides apart) |
| **11' plywood jon** (free-boat-plans.com) | 3.35 m | stitch & glue | seats and frames from 12 mm ply (details behind the paywall) |
| **Duckworks free plans list** | Duckskiff, One Sheet Skiff, Julie Skiff, West Mersea Duck Punt, Fritz tender, Lisa B Good, Bluestone, Cinderella | mixed | catalogue collected for reference |
| **Glen-L** (300+ designs since 1953) | all types; skiffs/driftboats/jon-style among outboards & workboats | ply over frames / S&G | design pages per model; useful precedent library |

Sources:
- Robin 18 PDF: https://s3.amazonaws.com/plan_downloads/ROBIN+18+March+2014.pdf
- Kastri 12 study plans: https://s3.amazonaws.com/planswalters/Kastri_12_Study_Plans.pdf
- Stitch-and-glue 101 (Bateau/BoatBuilderCentral): https://boatbuildercentral.com/support-tutorials/Tutorials/stitch-and-glue-101.pdf
- 12' jon build text: http://www.3dboatdesign.com/affiliates/plr/building_your_own_jon_boat.txt
- 11' S&G jon: https://free-boat-plans.com/product/11-foot-3-35m-plywood-jon-boat-plans/
- Duckworks free plans: https://duckworks.com/free-plans/
- Glen-L: https://www.glen-l.com/
- Jon design discussion: https://www.boatdesign.net/threads/trying-to-figure-out-how-to-design-jon-boat.50101/
- Seat mounting thread (risers, fillet + tape): https://www.songofthepaddle.co.uk/how-to-mount-seats-t37293.html
- Seat blocking / levelling (Glen-L build): http://barnaclemikeboats.blogspot.com/2015/08/musings-on-how-to-make-seats.html
- Seat riser in an aluminium boat: https://www.flyfishbc.com/threads/seat-riser.26409/

## Rules extracted (all confirmed by ≥2 sources)

1. **Seats never stand on legs.** A bench/thwart rides on **risers / cleats**
   (or stringers/battens on frame-built boats) fastened to the hull sides:
   glued with thickened epoxy, **filleted and glass-taped** (~50 mm onto hull
   and onto the part). On S&G builds the seat is often a **box** (frame +
   top) tabbed to the hull — the tape is structural.
2. Riser stock ≈ 3/4" x 1-1/4" (19 x 32 mm), **top bevelled** so the seat
   plank sits level; risers span 4-5 frames/ribs.
3. Jon boats: 3 cross benches (thwarts) + transom; the benches **hold the
   sides apart** — they are structure, not furniture.
4. Mixed layouts are normal: side benches + rear seat (Kastri), fore/mid/aft
   thwarts (rowing boats), transom seat.
5. Seat height: ~8" below the oarlocks in rowing boats; seats are positioned
   LAST, by boat trim — don't drill oarlocks/mounts until the seat position
   is settled.
6. **Helm with a tiller outboard = the aft bench/transom seat**, tiller in
   reach (see also the tiller-seat research: starboard common with a
   port-mounted tiller; a centre-mount electric tiller is position-free).
7. Build order wisdom: hull first, seats used as alignment jigs, seat
   risers/battens added and taped, seats dropped on top.

## How this changed our builder (designer.py / view3d.py)

- Seating = **cross benches (thwarts)**, 320 mm planks, aft bench = helm —
  matches the jon-boat/launch family (rule 3+6).
- Each bench is now drawn/noted as **plank on two side risers**, the risers
  glued + filleted/taped to the hull sides (rule 1+2); a centre post only on
  spans over 1.6 m. No legs on the floor (that was wrong).
- GA sheet notes carry the riser/taping callout so the plan shows intent.

## Rowing station + solo seating (plan census 2026-10-11)

Follow-up census (60+ plans, 19 authoritative sources; raw reports in
`~/boat/online plans/`). The rowing-boat practice, confirmed across the
Duckworks free plans (OarMouse, Light Trow, Cinderella, Julie Skiff),
Apple Pie, Western Skiff, Simbo II:

8. **The rowing thwart sits AT the oarlocks, near maximum beam.** Oars
   cross the gunwale where the boat is widest; the oarsman's thwart is at
   that station (plans give "oarlock position" as a gunwale dimension,
   one hand-span forward of the thwart edge).
9. **A solo boat gets the rowing thwart amidships - NEVER only a transom
   bench.** From the transom you cannot row (no oar spread, no foot
   bracing) and one person aft trims the bow up. This is what the user
   caught in our builder: people=1 used to put the single bench at the
   helm/transom station.
10. Helm bench aft (tiller reach) exists when there is a motor; with both,
    the layout is helm aft + rowing thwart at the oarlocks + any further
    benches spread toward the bow.
11. Oarlock hardware (sockets/rowlocks + oars) is part of the package -
    priced and drawn, not assumed.

Builder change: `designer.layout()` places the solo bench at
`x_oarlock` (widest station within the middle half), keeps a rowing
thwart pinned at `x_oarlock` in every multi-bench layout, exposes
`x_oarlock` for the GA sheet (oarlock marks + note) and the BOM
(oarlocks + oars row).
