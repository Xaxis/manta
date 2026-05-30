# 01 — Aero Sizing

> **⚠ Resized planform.** This write-up was authored for the original
> 8.4 m² / 7.4 m wing. The wing was downsized to **6.5 m² / 6.3 m, AR 6.1, 7°
> washout** (BRIEF findings #5/#6). The scripts + `analysis/aero/*/out/` data +
> plots are regenerated for the current planform — re-run `make aero` for live
> numbers. Headline deltas: CL_α 4.24 → **4.17 /rad**; V_bg ~16 → **18.3 m/s**;
> (L/D)ₘₐₓ 12.0 → **11.6**; root chord 1.62 → **1.47 m**; trim SM at 7° washout
> **5.6 %** (binding CG-based margin ~2.7 %).

**Status:** First-cut closed. AVL and XFOIL verification pending; trim
parameters and architecture decisions flagged for review where the analysis
surfaces tension with `BRIEF.md`.

This document summarizes deliverable #1: planform sizing, airfoil selection,
trim, stability, and L/D verification. The supporting analysis lives in
`analysis/aero/`. Numbers in this doc are reproducible from
`make aero` (and the AVL/XFOIL targets when those tools are installed).

## Planform — locked from BRIEF, derived in `analysis/aero/planform/geometry.py`

| Quantity | Value |
|---|---|
| Wing area  S | 8.400 m² |
| Span b | 7.400 m |
| Aspect ratio | 6.519 |
| Taper λ | 0.400 |
| Root chord | 1.6216 m |
| Tip chord | 0.6486 m |
| MAC | 1.2046 m |
| y_MAC | 1.5857 m |
| Sweep LE / c/4 / TE | 25.00° / 21.83° / 11.49° |
| Wetted area S_wet | 17.30 m² |
| Geometric twist (current default) | 5° washout |

Top-view planform: `cad/wing/out/planform_top.png`.

## Airfoil — `analysis/aero/airfoil/`

**Primary candidate: MH 78** (Hepperle), 10 % t/c, mildly positive Cm0,
appropriate stall character for a tailless wing. **Alternate: MH 60.**
Selection criteria, hard-rejection thresholds, and the operational
Reynolds range (5×10⁵ at the tip near stall through 2×10⁶ at the root at
best-glide) are documented in `analysis/aero/airfoil/README.md`.

A first-cut analytic polar
(`analysis/aero/airfoil/polar_analytic.py`) stands in for the airfoil
section everywhere downstream. Uncertainty bracket on each parameter is
explicit in the file. **Replace with XFOIL data** by:

1. Drop `MH78.dat` into `analysis/aero/airfoil/airfoils/`.
2. `make xfoil` — runs polars at Re ∈ {5×10⁵, 7×10⁵, 1×10⁶, 1.5×10⁶, 2×10⁶}.

Working polar at design CL (cruise):

| α (°) | Cl | Cd | Cm₀.₂₅ | L/D₂D |
|---|---|---|---|---|
| 4 | 0.50 | 0.0086 | +0.003 | 58 |

Cm0 ≈ +0.005 (slightly positive, helping tailless trim).

## Lifting-line — `analysis/aero/weissinger/`

Method: Prandtl-style lifting line with bound vortex along the panel c/4
sweep line and section a₀ coupling. Validated against:

- Helmbold AR-correction formula (rectangular AR = 8: 4.86 /rad solver vs
  5.03 /rad analytic, **−3.4 %** — single-row chord paneling underestimates
  by a few %, AVL is the higher-fidelity follow-up).
- 2D limit (AR = 100 → 6.09 /rad vs 2π = 6.28 /rad, **−3.1 %**).
- Unswept-rectangle NP at MAC c/4.

Results for the locked MANTA planform with 5° washout, MH-78-class section:

| Quantity | Value |
|---|---|
| CL_α | **4.24 /rad** (0.0740 /deg) |
| α at design CL = 0.5 | **7.66°** |
| 3D zero-lift α | +0.90° (washout pulls it positive) |
| Neutral point (aft of root LE) | **1.118 m**  =  0.928 · MAC |
| Geometric MAC c/4 | 1.041 m  =  0.864 · MAC  (NP is 6.5 % MAC aft of geom c/4) |

Span loading is in `analysis/aero/weissinger/out/span_loading.png` — peak
section Cl is mid-outboard (y ≈ ±2 m), root cl is depressed by the swept
trailing wake's downwash, tip cl is depressed by washout.

## CD0 build-up — `analysis/aero/lift_drag/cd0.py`

Component build-up referenced to S = 8.4 m². Wing profile drag is small;
the body fairing dominates and is the first-cut unknown.

| Bracket | Body CdA | CD0_wing | CD0_body | CD0_misc | **CD0** |
|---|---|---|---|---|---|
| optimistic | 0.15 m² | 0.0076 | 0.0179 | 0.0019 | **0.0273** |
| nominal | 0.20 m² | 0.0076 | 0.0238 | 0.0022 | **0.0336** |
| pessimistic | 0.30 m² | 0.0076 | 0.0357 | 0.0029 | **0.0464** |

References: Raymer Ch. 12 friction + form factor; Hoerner §13 streamlined
bodies; CdA bracket sourced from prone-pilot freefall (~0.4–0.5 m²
unfaired) and small ultralight prone-pilot fairings (~0.10–0.18 m² when
done well).

## Glide polar — `analysis/aero/lift_drag/glide_polar.py`

CD = CD0 + CL² / (π·AR·e), with e = 0.95.

**Best-glide condition** (CDi = CD0):

| Pilot | Bracket | V_bg | CL_bg | (L/D)_max | Sink |
|---|---|---|---|---|---|
| 82.5 kg | optimistic | 16.7 m/s | 0.73 | **13.4** | 1.25 m/s |
| 82.5 kg | nominal | 15.8 m/s | 0.81 | **12.0** | 1.31 m/s |
| 82.5 kg | pessimistic | 14.6 m/s | 0.95 | **10.2** | 1.42 m/s |

**L/D at the BRIEF target V = 25 m/s** (pilot 82.5 kg):

| Bracket | CL | CD | L/D |
|---|---|---|---|
| optimistic | 0.32 | 0.0326 | **9.91** |
| nominal | 0.32 | 0.0390 | **8.29** |
| pessimistic | 0.32 | 0.0517 | **6.25** |

### Tension with BRIEF performance targets

The locked planform's natural best-glide airspeed is **~16 m/s**, not the
**25 m/s** stated in BRIEF. Hitting 25 m/s as a *cruise* speed produces
L/D ≈ 8 (nominal CD0) — below the 10:1 design target, well short of the
13:1 stretch.

There are three resolution paths; one needs to be selected before the
mass/structural budget is sized:

1. **Restate V_bg as ~16 m/s.** Carries best L/D and lowest sink rate.
   Reduces ground-cover speed; gust penetration becomes worse.
2. **Hold V = 25 m/s as a cruise speed**, accept ~8:1 L/D, and revise
   `BRIEF.md` glide-ratio targets. Best for tactical mission profiles
   that need ground speed; bad for distance.
3. **Reopen the planform.** Higher wing loading (smaller S, or more
   weight on a smaller wing) lifts V_bg. Architecture decision #4
   (BRIEF) would have to change. Not recommended without other forcing
   pressure.

**My recommendation:** Path 1. The 10:1 target is closely met at the
natural V_bg with a defensible body-fairing CdA budget; the BRIEF V_bg
appears to have been set without solving for it.

## Trim + washout iteration — `analysis/aero/trim/`

For a tailless wing, longitudinal trim closes via CG placement:

> Cm_cg(α_trim, CL_design) = Cm_apex(α_trim) + (x_cg / MAC) · CL = 0

Sweeping washout in {3°, 4°, 5°, 6°, 7°} at the BRIEF design CL = 0.5:

| Washout | α_trim | x_CG_trim | x_CG / MAC | SM | α_tip_eff | tip stall margin |
|---|---|---|---|---|---|---|
| 3° | 6.90° | 1.086 m | 0.901 | **2.71 %** | +1.88° | 9.69° |
| 4° | 7.28° | 1.075 m | 0.892 | 3.61 % | +1.75° | 9.81° |
| 5° | 7.66° | 1.064 m | 0.883 | 4.51 % | +1.63° | 9.93° |
| **6°** | **8.04°** | **1.053 m** | **0.874** | **5.42 %** | **+1.51°** | **10.05°** |
| 7° | 8.42° | 1.042 m | 0.865 | 6.32 % | +1.39° | 10.18° |

**Recommendation:** **washout = 6°** (top of BRIEF range). Gives 5.4 %
static margin at the design point — within the conventional 5–15 % band,
albeit at the low end. Tip stall margin is healthy at ~10° across the
band; static margin (not stall) is the binding constraint.

Sensitivity at 5° washout across CL (cruise vs slow-flight vs
fast-cruise):

| Design CL | α_trim | SM | α_tip_eff |
|---|---|---|---|
| 0.30 | 4.96° | 7.52 % | +0.73° |
| 0.40 | 6.31° | 5.64 % | +1.18° |
| 0.50 | 7.66° | 4.51 % | +1.63° |
| 0.60 | 9.01° | 3.76 % | +2.08° |
| 0.70 | 10.36° | 3.22 % | +2.53° |

Static margin **decreases with CL**. The high-CL end of the operating
envelope (slow flight, near stall) is the worst case for stability; the
FCS alpha-limiter (mandatory per BRIEF) is the safety-net. At CL = 0.7
with 5° washout, SM = 3.2 % — that's marginal. AVL must confirm.

## CG envelope and pilot mass

For x_CG_trim = 1.053 m (6° washout, design CL = 0.5):

- m_total = m_pilot + 15.5 kg wing + 8.0 kg rig
- x_CG_total = (m_pilot · x_pilot + 15.5 · x_wing + 8.0 · x_rig) / m_total
- For 70 kg pilot, x_pilot ≈ 1.21 m aft of root LE; for 95 kg pilot, x_pilot ≈ 1.16 m
- A ~0.05 m CG-shift envelope across the 70–95 kg pilot range is acceptable;
  this does NOT include the much larger CG perturbation from head/torso
  motion in flight, which is `analysis/flightdynamics/` work and a
  binding constraint on the alpha limiter.

## CAD — `cad/wing/`

The lofted wing surface is generated parametrically from
`analysis/aero/planform/geometry.py`:

- `cad/wing/out/wing.step` — solid for OnShape / Fusion / FreeCAD import.
- `cad/wing/out/wing.stl` — visualization mesh.
- `cad/wing/out/planform_top.png` — top view with MAC and c/4 annotation.

Section is the parametric reflexed placeholder. Drop a real `MH78.dat`
into `cad/wing/airfoils/` and re-run for the production geometry.

## Open issues / verification gates

1. **AVL verification (gating).** Targets in `analysis/aero/avl/README.md`:
   CL_α within ±5 %, NP within ±0.05 MAC, e within physical bounds.
2. **XFOIL polars** for MH 78 and MH 60 across the operational Re sweep.
3. **Cm0 of the actual section** must be ≥ +0.005 — analytic polar has
   that as nominal but with ±0.010 uncertainty.
4. **V_bg vs BRIEF tension** — needs a decision (Path 1/2/3 above)
   before mass and structural budget closes.
5. **Static margin marginal at 5° washout, design CL.** Pin washout at
   6° in `Planform.washout_deg` after AVL confirms the NP location.
6. **High-CL static margin** (CL = 0.7 → SM = 3.2 %): near the lower
   limit. Flight envelope should not extend below CL ≈ 0.65 in trim
   without further analysis; the alpha-limiter target should reflect this.
7. **Pilot CG shift in flight** (head/torso motion) — owned by
   `analysis/flightdynamics/`. The alpha-limiter must close this loop or
   the 3.2–5.4 % static margin band is not enough.

## Reproducibility

```sh
make venv     # one-time setup of .venv with required packages
make aero     # weissinger + trim + glide polar in sequence
make cad      # parametric wing CAD (STEP + STL)
make avl      # optional, requires avl on PATH
make xfoil    # optional, requires xfoil + airfoil .dat
```

All numbers in this doc come from those targets.
