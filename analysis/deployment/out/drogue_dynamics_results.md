# MANTA drogue dynamics

  Total mass for sizing: 105.0 kg (sized-config wing)
  Terminal V (no drogue): 55.0 m/s (BRIEF reference)
  Target V after drogue:  30.0 m/s
  Pilot CdA (prone):      0.4 m²
  Drogue CD (ringslot):   0.55

## Sizing — drogue area required to produce target equilibrium descent

  CdA_total required at V = 30.0 m/s:  1.868 m²
  CdA_drogue (subtracting pilot):                 1.468 m²
  Drogue area (A = CdA / C_D):                    2.669 m²
  Drogue diameter (round canopy):                 1.843 m

## Snatch load + deceleration

  Peak bridle tension:    3944 N  (3.94 kN)
    occurs at t = 0.445 s, V = 51.0 m/s
    dynamic amplification factor: 1.7

  Time to V ≤ 31 m/s: 4.66 s

## Snatch vs. flight loads

  Pilot weight (1 g):           1030 N  (1.00 g)
  3 g limit flight:             3089 N  (3.00 g)
  4.5 g ultimate flight:        4634 N  (4.50 g)
  **Drogue snatch peak**:       3944 N  (3.83 g)

  ⚠ Drogue snatch exceeds 3 g limit flight load. The harness mount has to be sized to this case rather than 3 g flight.

## Where does this load go?

  Drogue bridle attaches to the **harness**, NOT the spar roots.
  The wing is still stowed when the drogue is loaded; the spars
  do not see the snatch load directly in the nominal sequence.

  Coupling cases that DO put drogue tension on the spars:
  - Asymmetric deploy with drogue still attached: brief
    cross-coupling via the bound wing-harness interface.
    Bounded by the ground-rig data once that exists.
  - Drogue-cut-release with one cutter not firing: drogue
    drag asymmetry into the deployed wing. Mitigated by
    redundant drogue release.
