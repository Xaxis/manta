# cad/fcs/

FCS bay layout: flight computer(s), IMUs, magnetometer, GNSS, pitot-static plumbing, AoA sensing, servo locations, wiring/cable runs, mechanical-reversion cable routing.

**Driven by:** `docs/04-fcs-architecture.md`.

Particular attention to:

- Independent failure-domain separation (redundant FCS units must not share single-point physical hazards — coolant path, harness chafe, etc.).
- Servo placement vs. mechanical-reversion cable run — the reversion path needs to remain operable even with servos jammed or unpowered.
- Sensor placement vs. spar/skin obstructions and EMI sources.
- Battery placement and CG impact (couples to `analysis/flightdynamics/pilot-cg-perturbations/`).
