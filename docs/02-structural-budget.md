# 02 — Structural Budget

**Status:** stub.

## Scope

- Mass rollup against the 15.5 kg wing-system budget.
- Load cases: 1g cruise, 3g maneuver limit, 4.5g ultimate, asymmetric deployment transient.
- Spar sizing: bending, buckling, joint capacity at telescoping junctions.
- Rib (tape-spring) deployment force and locked-state stiffness.
- Skin tension and attachment.
- Root joint and pyrotechnic cutter interface — the joint must carry full flight loads but sever cleanly on command.
- Safety factors: limit / ultimate per locked policy (TBD here, propose 1.5× ultimate over limit, plus material knockdowns).

## Mass budget structure

| Subsystem | Allocation (kg) | Owner doc |
|---|---|---|
| Front spar (both sides, 3-stage telescoping CFRP, 40/25 mm OD, 2 mm wall) | TBD | this doc |
| Rear spar (both sides, 3-stage, 30/18 mm, 2 mm) | TBD | this doc |
| Ribs (18 × bistable CFRP tape-spring booms) | TBD | this doc |
| Skin (DCF, ~50 g/m² × wetted area) | TBD | this doc |
| Root fittings + cutters (4 pyrotechnic) | TBD | doc 05 |
| Pneumatic deployment (CO2 cartridges, valve, manifold, lines) | TBD | doc 03 |
| FCS + servos + wiring | TBD | doc 04 |
| Drogue + bridle + reefing | TBD | doc 03 |
| Harness shell + interface to skydiving rig | TBD | this doc |
| Margin (≥10 % of allocated) | TBD | this doc |
| **Total** | ≤ 15.5 | BRIEF |

## Deliverables

- `analysis/struct/mass-budget.{py,csv}` — sensitivity to spar wall thickness, rib count, skin g/m².
- `analysis/struct/spar-bending.py` — parametric root bending vs. n-load, validates 40/2 spar at 3g limit.
- FEA (CalculiX or FreeCAD FEM) of the root joint under combined bending + cutter-interface load.
- 3D models under `cad/spars/`, `cad/ribs/`, `cad/jettison/`.
