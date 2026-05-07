# test/ground/

Ground deployment rig. **Deliverable #6** in BRIEF — the first piece of MANTA hardware to actually build.

## What it is

A fixture that holds the full wing assembly off the body, with no flight loads, instrumented to characterize and prove the deployment kinematics:

- Cycles deploy → restow repeatably (target: ≥200 cycles before re-build).
- Records left-right deploy timing per stage, both spars.
- Records spar-stage lock confirmations (microswitches).
- Records skin tension at multiple chord stations per side.
- Records valve actuation feedback and CO2 cartridge pressure / temperature.
- Records ambient T and RH; conditions to cold soak / hot / humid extremes.

## Hard gate (BRIEF)

> Do not move to flight-relevant test articles until the ground deployment rig has demonstrated reliable, symmetric, sensed deployment over the full thermal and humidity envelope, in at least 200 cycles without intervention.

## To produce

- `spec.md` — the rig specification: structural fixture, instrumentation, DAQ, environmental conditioning, cycle automation.
- `test-cards/` — per-condition run cards.
- `data-handling.md` — file formats, naming, retention, review process.
- `cad/test-rig/` (yes, the rig itself gets CAD) — fixture and harness.
