# 02 — Structural Budget

> **⚠ Resized planform.** Authored for the 8.4 m² / 7.4 m wing; the wing is now
> **6.5 m² / 6.3 m** (BRIEF findings #5/#6). `analysis/struct/out/` data + plots
> are regenerated — re-run `make struct`. Headline deltas: bending-sized front
> spar root **67 mm** OD (was 73) — shorter span → less root moment; wing-system
> mass **~16.3 kg**; telescoping boom 2.9 → **2.4 m/side**.

**Status:** First-cut closed. The analysis has surfaced two binding tensions
with `BRIEF.md` that need a decision before structural design proceeds:

1. **The locked spar dimensions (front 40 mm OD / 2 mm wall) fail the bending
   case at 3 g limit by a factor of ~3 in stress.** Recommended sizing: 73 mm
   OD root / 2.5 mm wall. (Section "Spar bending" below.)
2. **The 15.5 kg wing-system mass target cannot be met** with the
   bending-correct spar. Recommended budget: **~16.6 kg** with a properly
   sized front spar, or a different load-case decision. (Section "Mass
   budget".)

Both are evidence-driven flags per the BRIEF rule "Architecture decisions
locked unless evidence forces a change." Resolution paths are in the
recommendations at the end.

## Methodology

- Span loading from the Weissinger lifting-line solver at design CL = 0.5
  (`analysis/aero/weissinger/`). This captures the actual sweep-and-taper
  loading shape rather than approximating with elliptical.
- Cantilever bending-moment integration along each half-wing, validated
  against analytical cases (uniform line load and elliptical loading) in
  `analysis/struct/tests/test_struct.py` to <1 %.
- Section properties from a parametric telescoping-spar model
  (`analysis/struct/spar_model.py`).
- Material allowables from `analysis/struct/materials.py` (T800/epoxy UD
  CFRP tube, knockdown 0.55, FAR 1.5× ultimate-over-limit factor).
- Brazier critical-moment check for thin-walled CFRP tubes under bending.

## Materials

| Property | Value | Source |
|---|---|---|
| ρ (CFRP UD tube) | 1580 kg/m³ | Toray T800S/3900-2 datasheet, 2024-09 |
| σ_ult tension | 1900 MPa | T800 layup ~85 % axial UD |
| σ_ult compression | 1100 MPa | fiber-kinking limit |
| knockdown (env × fatigue × notch) | 0.55 | DOT/FAA composite handbook + Niu Ch. 5 |
| safety factor (limit→ult) | 1.5 | FAR Part 23 / EASA convention |
| **σ_design_compression_limit** | **403 MPa** | knockdown × ult / SF |

DCF skin areal density: 50 g/m² (mid-weight CT3K series).

## Span loading at design CL

The Weissinger solver gives the normalized span loading shape at α = 7.66°,
CL = 0.5. Total wing lift at n_load · m_total · g is distributed across
the span proportional to `cl(y)·c(y)`, then integrated as a cantilever
moment about each spanwise station.

For sizing, the **upper-end pilot mass** in the BRIEF envelope is used:

| Quantity | Value |
|---|---|
| Pilot mass (sizing) | 95.0 kg |
| Wing + rig mass | 23.5 kg |
| Total mass for sizing | 118.5 kg |
| Lift at 1 g | 1162 N |
| Lift at 3 g | 3487 N |
| Lift at 4.5 g | 5230 N |

## Spar bending — `analysis/struct/spar_bending.py`

### Chordwise load split

The two-spar layout splits the wing's lift between front and rear spars
based on the chordwise position of each spar relative to the lift action
line (the AC at ~0.25·c).

With the BRIEF nominal spar positions (front at 0.20·c, rear at 0.65·c):

> f_front = (x_rear − x_AC) / (x_rear − x_front) = (0.65 − 0.25) / (0.65 − 0.20) = **0.889**

So the front spar carries ~89 % of the wing bending load. Moving the front
spar to 0.10·c would drop this to 0.73, but at the cost of LE structural
real estate (interferes with stowed-state packaging and tape-spring rib
coil routing).

### Stress at the root — BRIEF dimensions

With BRIEF spar dimensions (front 40 mm OD / 32 mm mid / 25 mm tip / 2 mm
wall; rear 30 mm / 24 mm / 18 mm / 2 mm wall):

| Load case | Spar | M_root (N·m) | σ_root (MPa) | SF_strength | SF_buckling (Brazier) |
|---|---|---|---|---|---|
| 1 g cruise | front | 830 | **384** | **1.05** | 110 |
| 3 g limit | front | 2491 | **1153** | **0.35** | 37 |
| 4.5 g ult. | front | 3737 | **1729** | **0.23** | 24 |
| 1 g cruise | rear | 104 | 90 | 4.49 | 494 |
| 3 g limit | rear | 311 | 270 | 1.50 | 165 |
| 4.5 g ult. | rear | 467 | 404 | 1.00 | 110 |

**The front spar fails:** SF_strength = 0.35 at 3 g limit means the spar
is undersized by a factor of ~3 in stress capacity. At 1 g cruise it's
right at the design-limit margin (SF = 1.05) — no growth room and no
gust-load tolerance.

The rear spar passes (SF = 1.00 at 4.5 g ultimate, exactly the FAR
threshold). It does not need re-sizing. **Buckling (Brazier) is not the
binding constraint** — buckling SF is two orders of magnitude above the
strength SF for both spars.

### Wall-thickness sensitivity (BRIEF OD progression)

Holding the BRIEF OD progression and varying wall:

| Wall (mm) | Front mass (kg/side) | σ_root @ 3 g (MPa) | SF_strength |
|---|---|---|---|
| 1.5 | 0.96 | 1480 | 0.27 |
| 2.0 | 1.24 | 1153 | 0.35 |
| 2.5 | 1.51 | 958 | 0.42 |
| 3.0 | 1.76 | 830 | 0.49 |

Doubling the wall (to 4 mm) would still leave SF < 0.7. Wall thickness
alone cannot save the BRIEF dimensions.

### OD sensitivity at 2.5 mm wall

The binding parameter is OD (because I scales with D³ for thin walls).
At 2.5 mm wall, with stage progression OD_root / 0.7·OD_root / 25 mm:

| OD_root (mm) | Front mass (kg/side) | σ_root @ 3 g (MPa) | SF_strength |
|---|---|---|---|
| 40 | 1.44 | 958 | 0.42 |
| 50 | 1.71 | 590 | 0.68 |
| 60 | 1.98 | 400 | 1.01 |
| 65 | 2.12 | 337 | 1.20 |
| 70 | 2.25 | 288 | 1.40 |
| 80 | 2.52 | 218 | 1.85 |

### Recommended front spar

- **OD_root = 73 mm**, OD_mid = 51 mm, OD_tip = 25 mm
- **Wall = 2.5 mm**
- 3-stage telescoping, stage length 1.283 m each (joint overlap 50 mm)
- Per-side mass: **2.34 kg** (vs. BRIEF default 1.24 kg → **+1.10 kg/side**)

Result: σ_root = 264 MPa at 3 g limit, SF_strength = 1.53. Comfortable
margin against the 403 MPa allowable.

## Mass budget — `analysis/struct/mass_budget.py`

### Component roll-up — BRIEF spar dimensions (NB: spar fails bending)

| Component | Mass (kg) | % of 15.5 target |
|---|---|---|
| Spars (4 spars, both sides) | 4.32 | 27.9 % |
| Ribs (9 per side, 18 total, tape-spring) | 1.14 | 7.3 % |
| Skin (DCF + bond overhead) | 0.95 | 6.1 % |
| Root fittings + cutters | 1.60 | 10.3 % |
| Pneumatic deployment | 0.73 | 4.7 % |
| Flight control system | 0.86 | 5.5 % |
| Actuators + reversion | 0.97 | 6.3 % |
| Drogue + bridle | 0.35 | 2.3 % |
| Harness shell + interface | 2.00 | 12.9 % |
| Margin (10 % of allocated) | 1.29 | 8.3 % |
| **Total** | **14.21** | **91.7 %** |

`14.21 kg < 15.5 kg target — but the spar in this configuration fails
bending. Not a usable design.`

### Component roll-up — bending-sized front spar (73 mm/2.5 mm)

| Component | Mass (kg) | % of 15.5 target |
|---|---|---|
| Spars (4 spars, both sides) | **6.51** | **42.0 %** |
| Ribs | 1.14 | 7.3 % |
| Skin | 0.95 | 6.1 % |
| Root fittings + cutters | 1.60 | 10.3 % |
| Pneumatic deployment | 0.73 | 4.7 % |
| FCS | 0.86 | 5.5 % |
| Actuators + reversion | 0.97 | 6.3 % |
| Drogue | 0.35 | 2.3 % |
| Harness | 2.00 | 12.9 % |
| Margin (10 %) | 1.51 | 9.7 % |
| **Total** | **16.62** | **107.2 %** |

**+1.12 kg over the 15.5 kg BRIEF target.**

### Sensitivity sweep

The wing system's mass is most sensitive to spar sizing. Skin areal
density and rib count are second-order:

| Front spar | Ribs/side | Skin g/m² | Total (kg) | Δ vs target |
|---|---|---|---|---|
| BRIEF | 7 | 40 | 13.72 | −1.78 |
| BRIEF | 9 | 50 | 14.21 | −1.29 |
| BRIEF | 11 | 60 | 14.70 | −0.81 |
| **SIZED** | 7 | 40 | 16.13 | **+0.63** |
| **SIZED** | 9 | 50 | 16.62 | **+1.12** |
| **SIZED** | 11 | 60 | 17.11 | **+1.61** |

Even the leanest sized configuration (7 ribs, 40 g/m² skin) is +0.63 kg
over budget. Reducing ribs below 7 risks skin sag between ribs and tip
flutter; reducing skin below 40 g/m² loses tear resistance.

## Recommendations

### 1. Adopt the bending-sized front spar.

OD_root 73 mm, OD_mid 51 mm, OD_tip 25 mm, wall 2.5 mm. This is the
smallest spar that closes the bending case at 3 g limit / 4.5 g ultimate
with a 1.5× margin on the design-limit stress. **Update BRIEF
architecture decision #1 to reflect this.**

### 2. Update wing-system mass budget to ~16.6 kg.

That's the lowest credible budget given a structurally adequate spar
and the rest of the systems sized as in the build-up. Wing loading
becomes ~12.9 kg/m² (vs. BRIEF nominal 10.5 kg/m²). The aero pipeline's
stall-speed and best-glide numbers would shift slightly — re-run
`make aero` after the change.

### 3. Alternatively, reduce the design n_load.

A non-aerobatic glider with mandatory alpha-limiter envelope protection
could justify a 2.5 g limit (3.75 g ultimate) instead of 3 g/4.5 g. At
2.5 g limit the front spar requirement drops to ~62 mm OD / 2.5 mm wall
and the total budget closes at ~15.7 kg — within ~0.2 kg of the BRIEF
target.

This requires a separate gust-load analysis to defend (a 7.5 m/s gust at
V = 25 m/s gives Δn ≈ 2 g for this wing loading and CL_α — so 2.5 g
limit leaves only 0.5 g margin against a moderate gust, which is
borderline). Not recommended without a serviceable alpha-limiter prototype
first.

### 4. Don't attempt to save mass by reducing knockdowns.

The 0.55 knockdown is conservative but defensible. Reducing it requires
coupon test data the program doesn't yet have — no shortcut here.

## Sized-spar 3D model

`cad/spars/build.py` exports both BRIEF and sized configurations:

- `cad/spars/out/spars_brief.step`  (40/32/25 mm, fails bending)
- `cad/spars/out/spars_sized.step`  (73/51/25 mm, recommended)

Same for `.stl`. The sized spar visibly larger at the root — it's a
significant change to the wing's stowed-package thickness budget too,
since the stowed (telescoped) bundle is now 73 mm in the front-spar
diameter (vs. 40 mm BRIEF) plus the rear spar plus the tape-spring
ribs and skin folded around them. The BRIEF "stowed package thickness
< 15 cm off body profile" constraint may also need re-examination.

## Drogue snatch — harness-mount load case

The drogue dynamics analysis ([`analysis/deployment/drogue_dynamics.py`](../analysis/deployment/drogue_dynamics.py))
sizes the drogue at 1.84 m diameter (CdA = 1.47 m²) and predicts a
**peak bridle tension of 3.94 kN at t = 0.45 s** post drogue-extract
command. This corresponds to a 3.83 g equivalent vehicle decel —
between 3 g flight limit and 4.5 g ultimate.

The bridle attaches to the **harness**, not the spar roots. The spar
bending analysis above does NOT see this load in the nominal sequence.
But the harness mount (and the bridge that carries the wing-mount
sub-frame) must be sized to the drogue case rather than to the 3 g
flight load — the drogue snatch is the binding harness-mount case.

Cross-coupling exception: an asymmetric-deploy event with the drogue
still attached can transmit drogue tension into the spar roots via
the wing-harness interface for ~75 ms before jettison fires. This is
not yet sized; current `analysis/struct/spar_bending.py` treats only
the symmetric flight load. A coupled-asymmetric-deploy load case is
in the open-issues list below.

## Open verification gates

1. **FEA validation of the root joint** under combined bending + cutter
   interface load — the analytical hand-calc here treats the root as a
   simple cantilever with no joint compliance. The pyrotechnic-cutter
   joint is a stress concentrator and a discontinuity; FEA needed.
2. **CFRP coupon tests** to defend knockdown 0.55 for the actual layup
   chosen. Vendor B-basis from a qualified prepreg + qualified shop
   could reduce the knockdown and unlock mass savings.
3. **Gust-load analysis** to defend the 3 g (or proposed 2.5 g) limit.
4. **Tape-spring rib actual mass** — the 50 g/rib estimate here is a
   first-cut; lab-measured prototypes will tighten this.
5. **Stowed-package thickness budget** with the sized spar in the
   stowed configuration. The BRIEF < 15 cm constraint may bind.
6. **Asymmetric-deployment transient load** — at the moment we have not
   sized for the bending+torsion case during a partial-deploy event.
   That goes into `analysis/deployment/` and a follow-up update of
   this document.

## Reproducibility

```sh
make struct      # full pipeline: bending + budget
PYTHONPATH=. .venv/bin/python -m pytest analysis/struct/tests/ -v
PYTHONPATH=. .venv/bin/python cad/spars/build.py
```
