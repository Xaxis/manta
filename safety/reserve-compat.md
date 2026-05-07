# Reserve Parachute Compatibility

**Status:** stub.

This document is the closure of the BRIEF's hardest constraint:

> Reserve parachute compatibility is non-negotiable. The skydiving rig functions normally for canopy flight and landing after wing jettison. Anything that compromises that is a non-starter.

## What it must show

1. **Pre-jettison geometry** — the wing harness mounted on the piggyback rig does not interfere with reserve container access, ripcord/RSL geometry, or AAD function.
2. **Post-jettison geometry** — after the four spar-root cutters fire and the wing assembly leaves cleanly, no protrusions remain that would foul the reserve canopy or its lines during inflation.
3. **Reserve deployment trajectory** — pilot-chute extraction path, bag launch, line stretch, canopy inflation: all clear of any residual structure or harness elements.
4. **Worst-credible attitude case** — pilot in unusual attitude at jettison (e.g. the trigger event was an asymmetric deploy that induced a roll). The reserve must still be able to find clean air.
5. **AAD compatibility** — physical compatibility with at least one production AAD (Cypres or Vigil), and signal-level integration for FCS-aware AAD trigger.
6. **Standard rig functionality after jettison** — pilot can fly canopy and land normally; nothing in the wing-harness mount creates a hazard during canopy descent.

## Inputs

- `cad/harness/` — full geometry of harness + rig stack.
- `cad/jettison/` — root fittings in intact, severed, and post-jettison stub states.
- `analysis/deployment/` — jettison-trigger conditions and resulting body state.

## Verification

- Geometric clearance: CAD analysis with margin reporting, plus physical mock-up.
- Reserve deployment trajectory: CAD trajectory + drop-test article evidence (`test/drop/`).
- Pre-flight inspection procedure that any rigger or pilot can follow.
