# Skin membrane form-finding — deployment into a controlled surface

The deployed skin is a pretensioned membrane stretched over the bistable ribs; the telescoping booms + rib snap put it in tension. Bay sag (the waviness off the design airfoil) follows from membrane statics `δ = q·s²/(8·N)`, cross-checked by discrete relaxation.

| Quantity | Value |
|---|---|
| Cruise dynamic pressure q | 211 Pa |
| Net skin pressure (cruise / 3 g) | 158 / 475 Pa |
| Skin pretension (deployment-set) | 2200 N/m |
| Bay spacing (rib pitch) | 266–266 mm |
| **Worst bay sag @ 3 g** | **1.9 mm** |
| **Surface waviness δ/c @ 3 g** | **0.27 %** (within the 0.4 % tol) |
| Skin stays in tension (no wrinkles) | yes |
| Billow fraction handed to the 3D model | 0.015 |

**Result.** With the deployment putting ~2200 N/m of pretension into the skin, the worst inter-rib sag is ~1.9 mm even at the 3 g limit — a surface waviness of 0.27 % chord, within the 0.4 % aero tolerance. So the deployed wing is a smooth, controlled airfoil (and a clean trailing-edge flaperon), NOT a billowing canopy. The earlier 14 %-of-thickness 'billow' was ~10× too large for a rigid pretensioned skin; the model now uses the physical 0.015.

The pretension is itself a deployment requirement: the booms and the bistable rib snap must deliver it for the surface to come out fair — linking the deployment mechanism to the final aerodynamic quality.