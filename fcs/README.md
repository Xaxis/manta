# fcs/

Flight Control System: firmware, simulation, envelope-protection logic.

**Pairs with:** `docs/04-fcs-architecture.md`, `analysis/flightdynamics/`, and the deployment state machine in `docs/03-deployment-sequence.md`.

## Subdirectories

| Path | Purpose |
|---|---|
| [`firmware/`](firmware/) | PX4 fork (or ArduPilot — pick after architecture review). MANTA-specific modules: deployment state machine, alpha limiter, jettison logic, spar-lock sensor handling. |
| [`sim/`](sim/) | SITL configurations, vehicle model wired to `analysis/flightdynamics/`, scripted mission scenarios for envelope-protection development. |
| [`envelope-protection/`](envelope-protection/) | Standalone development of the alpha limiter and other limiters with unit tests, gain schedules, and HIL fixtures before they land in firmware. |

## Locked from BRIEF

- Redundant Pixhawk-class hardware.
- EKF at 400 Hz.
- Mechanical reversion to direct cable as last-resort backup — the FCS code must not assume it is the only path to the surfaces.
- Alpha limiter is a structural design assumption, treated as required for safe operation.
