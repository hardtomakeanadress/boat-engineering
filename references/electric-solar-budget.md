# Electric drive + solar: power, energy, weights

Compiled 2026-10-10 from vendor documentation (Vetus, Torqeedo, ePropulsion,
Oceanvolt, Fischer Panda), PVGIS 5.2 data for Cluj-Napoca and published
measurements. Ranges are real-world; verify against the chosen hardware.

## Motor power vs speed (displacement hulls)

Power scales with speed^3 at fixed displacement. Anchors measured:

- Vetus E-Line example, 6 m / 1.1 t boat: 3.3 kn -> 0.7 kW; 4.7 kn -> 2.1 kW;
  5.9 kn -> 4.1 kW (**0.64 / 1.9 / 3.7 kW per tonne**).
  Vetus formula: P_shaft [hp] = delta[t]^(2/3) * V[kn]^3 / 45, then +25%.
- Rules of thumb: **1 kW/t** calm inland (Fischer Panda), **1.25 kW/t**
  (Oceanvolt, cruise at ~60% hull speed), **2.5 kW/t** current / open water.
- Hull speed: 1.34 * sqrt(LWL_ft) = 2.43 * sqrt(LWL_m) [kn].
- Motor classes (vendor guidance): Torqeedo Cruise 3.0 -> up to ~3 t;
  Cruise 6.0 -> up to ~5 t; Vetus ELINE050 (5 kW) -> ~7 m / 3 t.

Efficiency chain: propeller 40-65% (small fast props lower); motor+controller
~88-92% of battery-side DC power. Net rule: **propulsive (thrust) power ~
0.4-0.6 x battery DC power at cruise - use 0.5 when estimating range/speed**
(this single factor already folds in motor and propeller losses). When a
figure is quoted as shaft power, convert separately: DC = shaft / ~0.9.

Battery sizing practice: kWh ~ 1.3-2 x motor kW (Oceanvolt); vendors quote
"1 h at full power" requirements directly (ePropulsion Navy 3.0 needs
>= 3.0 kWh, Navy 6.0 >= 6.0 kWh).

## Solar, Cluj-Napoca (46.77N, 23.59E, ~350 m) - PVGIS 5.2

Per 1 kWp, 14% system losses:
- Optimum tilt 38 deg: **1,202 kWh/kWp/year**; July 4.42, June 4.30,
  Jan 1.75, Dec 1.50 kWh/kWp/day.
- Horizontal (boat deck): **1,012 kWh/kWp/year**; July 4.56, June 4.53,
  May 4.05, Dec 0.75 kWh/kWp/day.
- Reproducible API:
  `https://re.jrc.ec.europa.eu/api/v5_2/PVcalc?lat=46.77&lon=23.59&peakpower=1&loss=14&angle=0&aspect=0&outputformat=json`

Real-world boat derate: expect **0.3-0.4 kWh/day per 100 W** in decent sun
(0.4-0.55 only in unshaded tropics; up to -50% if shading-prone; MPPT
92-97%). July sanity: 1 kWp horizontal ~ 3.6-4.3 kWh/day after boat losses;
December ~ 0.6-0.75 kWh/day.

Panel mass: **rigid ~55 kg/kWp** (a 400 W panel weighs ~22 kg); **flexible
~20 kg/kWp** (100 W ~ 2 kg; walkable deck-rated models heavier).

## LiFePO4 (48 V marine practice)

- Pack mass incl. BMS: **~8-14 kg/kWh; use 10 kg/kWh** for design
  (ePropulsion E60 3.07 kWh/33 kg; E175 8.96 kWh/87 kg). Beware: some vendor
  packs (Torqeedo Power 48-5000) are NMC, lighter - don't mix the data.
- Depth of discharge: 80% routine (10-90% SoC window), 90% occasional;
  3,000-6,000 cycles at 80% DoD.
- 48 V nominal = 51.2 V (16S) with BMS.

## Worked example: the 5.2 m demo launch

Loaded 838 kg = 0.84 t. Vetus scales with delta^(2/3): at 4.7 kn it gives
**~1.7 kW motor output (shaft)** for this hull.

- Battery-side DC = shaft / ~0.90 (motor + controller) = **~1.9 kW**. The
  0.4-0.6 propulsive chain applies between shaft and effective thrust -
  keep the two efficiencies separate (shaft is NOT battery power divided
  by the propulsive chain).
- 5 kWh battery at 80% usable = 4 kWh -> **~2.1 h at 4.5-4.7 kn**; at 3 kn
  power falls to (3/4.5)^3 ~ 30% (~0.57 kW DC) -> **~7 h**.
- Solar 2 x 400 Wp flat = 0.8 kWp (legacy example, 400 W modules; the builder's default module is the 450 W - rescale): July ~0.8 * 4.0 = **~3.2 kWh/day** =
  ~1.7 h of 4.5 kn cruising per sunny day (or most of a 3 kn day).

Light hull, low drag - bigger boats scale with delta^(2/3).

## Mass overhead for the budget

Cabling: 35 mm^2 marine cable ~0.3-0.5 kg/m (a 6 m round trip both poles
~3-6 kg); controller/throttle/fuse box, panel frames (~1-2 kg/m2 of panel)
and fasteners: add **5-15% to drive + battery + solar mass** in the budget
until real components are weighed.

## Confidence

kW/tonne figures and efficiency ranges: medium-high (multiple vendors,
consistent physics). Solar yield: high (PVGIS for the exact location). Weights:
high (vendor datasheets), but the overhead percentage is an engineering
estimate - replace with real numbers as hardware is chosen.
