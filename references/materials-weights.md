# Materials and weights (for the budget)

Approximate values for first-pass budgets. Weigh or look up real components
before committing; keep a 10-15% margin on estimates until then. Record
as-built masses as you go - weight creep is how boats lose freeboard.

## Structural materials

| material | density [kg/m3] | per m2 per mm |
|---|---|---|
| marine plywood | 550 - 750 (use 600) | 0.6 kg/m2/mm |
| cedar strip | 380 - 450 | - |
| pine / fir | 480 - 550 | - |
| oak | 650 - 750 | - |
| aluminium 5083/6061 | 2700 | 2.7 kg/m2/mm |
| steel | 7850 | 7.85 kg/m2/mm |
| GRP hand layup (laminate) | 1400 - 1600 | 1.4-1.6 kg/m2/mm |

Sheathing/lamination rough mass: each layer of glass cloth adds about
its areal weight in resin (50/50): 200 g/m2 cloth -> ~0.4-0.5 kg/m2
finished; a typical ply hull sheathed both sides, primed and painted roughly
adds 15-25% to bare plywood mass. Paint/antifoul: ~0.3-0.5 kg/m2 per coat.

## People and gear

- Design weight per person: **85 kg** for capacity/marking (ISO offset-load
  test weight, default in budget.py); 75 kg is fine for calm-water KG work
  and is the deck-crew planning figure. Gear counts against the same
  capacity either way.
- Typical day gear per person: 5-15 kg.

## Electric drive and solar (verify per product)

| item | rough mass | note |
|---|---|---|
| LiFePO4 cells | 6 - 9 kg/kWh | cell level |
| LiFePO4 pack, BMS + case | 10 - 14 kg/kWh | what goes in the boat |
| Electric outboard, 1 kW class | 10 - 20 kg | incl. tiller/battery version |
| Electric outboard, 3 - 6 kW class | 20 - 45 kg | incl. mount |
| Electric outboard, 10 kW class | 45 - 70 kg | incl. mount/cables |
| Rigid solar panel | ~10-12 kg/m2 (~45-55 kg/kWp) | framed, mounted |
| Flexible solar panel | 2 - 4 kg/m2 (~15-25 kg/kWp) | glued/strapped |
| Charge controller, wiring, fuses | add 5-10% of battery+PV mass | |

Worked check (used in the demo budget): 5 kWh pack ~65 kg, 6 kW outboard
~45 kg, 2 panels + light roof ~40 kg on a 5.2 m launch - these
dominate the lightship KG; low placement matters (battery low and central).

## Budget discipline

- Hull structural mass from surface area x table masses, then +15% for
  fasteners, frames, glue, fairing.
- The `budget.py` items need x (longitudinal) and z (height) - estimate
  both; the LCG must land near the hydrostatic LCB, and KG drives GM.
- Keep battery/tanks low and near midship; heavy gear (anchor, motor)
  positions are the easiest levers if GM or trim is bad.

These component numbers are typical ranges from the industry - the
electric-solar-budget reference has the yield/power side; replace all
ranges with real datasheet values as soon as the hardware is chosen.
