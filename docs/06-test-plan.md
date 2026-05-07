# 06 — Test Plan

**Status:** stub.

## Scope

The full progression of test articles and their gating criteria. The brief is explicit: do not move to flight-relevant test articles until the ground deployment rig has demonstrated reliable, symmetric, sensed deployment over the full thermal and humidity envelope, in **at least 200 cycles without intervention**.

## Progression

1. **Bench / component** — individual subsystem characterization. CO2 cartridge variability. Tape-spring rib unfurl dynamics (high-speed video, force trace). Pyrotechnic cutter no-fire/all-fire margins. Skin tension calibration.
2. **Ground deployment rig (`test/ground/`)** — full wing assembly on a fixture. Repeatable deploy/restow cycles. Instrumented: deploy timing per-side, spar-lock confirmations, skin tension at multiple chord stations, valve actuation feedback. Conditioned over thermal envelope (cold soak, hot, humid). **Gate: 200 cycles symmetric within 10ms 3-σ, no intervention.**
3. **Tow article (`test/tow/`)** — wing already deployed before launch. Boat or vehicle tow. Verifies aerodynamic predictions, control authority, stall behavior, envelope-protection effectiveness — without ever needing to deploy in flight. **Gate: trim, stability, control authority within predicted envelopes; envelope protection demonstrated.**
4. **Drop article (`test/drop/`)** — ballasted instrumented article released from aircraft, executes the full deploy sequence, including drogue, jettison, and reserve. No human on board. **Gate: end-to-end sequence including jettison + reserve verified across the full speed/altitude envelope.**
5. **Manned tow** — first human flights, deployed-on-tow only, no deploy-in-flight. Pilot tested progression. **Gate: handling qualities acceptable, recovery procedures rehearsed.**
6. **Manned exit, deployed (cold release)** — towed exit at altitude, released into a known glide. Avoids the deploy event but adds the exit dynamic.
7. **Manned exit, deploy-in-flight** — full operational profile. Earliest after ALL prior gates pass and a separate go/no-go review.

## Cross-cutting

- Every test has documented success criteria, instrumentation list, abort criteria, and a written go/no-go review before article work begins.
- All flight-test data is retained, indexed, and reviewable.
- Failure investigations are required before progression — no "ran it again, it worked" closures.

## Deliverables

- Per-stage detailed test-card series (one per article).
- Instrumentation specifications.
- Range/site selection notes.
- Schedule with explicit gate reviews.
