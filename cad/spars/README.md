# cad/spars/

Telescoping CFRP spars — front and rear, 3-stage each, both sides.

**Driven by:** `docs/02-structural-budget.md` and `analysis/struct/spar-bending.py`.

**Locked dimensions (from BRIEF):**

| Spar | Root OD | Tip OD | Wall | Stages |
|---|---|---|---|---|
| Front | 40 mm | 25 mm | 2 mm | 3 |
| Rear | 30 mm | 18 mm | 2 mm | 3 |

**Generation strategy:** Python-parametric. Stage lengths, overlap regions, locking-pin positions, and sealing geometry (water/ice ingress is unsolved-problem #4) all flow from `analysis/struct/`. Update the script when wall thickness or stage count changes from sensitivity analysis.

To be populated alongside deliverable #3.
