# FMEA — Failure Mode and Effects Analysis

**Status:** stub. Living document.

## Format

For each failure mode:

| Field | Description |
|---|---|
| ID | Stable identifier, e.g. `FM-DEP-003`. |
| Subsystem | wing / spar / rib / skin / pneumatics / FCS / sensor / cutter / harness / drogue / reserve. |
| Failure mode | What fails. |
| Cause(s) | Plausible causes. |
| Effect | Consequence on flight and pilot. |
| Severity | Catastrophic / Hazardous / Major / Minor. |
| Likelihood | Per-flight, with rationale. |
| Detection | How is it sensed (or — explicit — that it isn't). |
| Mitigation | Design / procedural / operational. |
| Response | Pilot or FCS action. |
| Residual risk | After mitigation. |
| Test evidence | What test backs the mitigation. |

## Top failure modes to populate first

1. **Asymmetric wing deployment** (the dominant unrecoverable case per BRIEF). Detection budget, jettison threshold, false-positive rate, time-to-decision. Has its own file at `failure-modes/asymmetric-deployment.md`.
2. **Spar root joint failure under flight load.** Mitigation: design margin, NDI, retirement life.
3. **Pyrotechnic cutter no-fire.** Mitigation: redundant initiators, hot/cold logic, manual reserve path.
4. **Pyrotechnic cutter inadvertent fire on ground or in stable flight.** Mitigation: arming logic, lockouts.
5. **Reserve canopy occlusion or fouling on jettison stub.** Mitigation: severed-state geometry verified clean.
6. **Alpha limiter failure / saturation → stall departure.** Mitigation: redundant FCS path, mechanical reversion, structural design assumes limiter present (so loss-of-limiter is a recoverable event only at low alpha).
7. **Telescoping joint binding from water/ice ingress.** Mitigation: sealing design, pre-flight check, ground-rig environmental envelope.
8. **CO2 cartridge under-pressure (cold).** Mitigation: temperature monitor + abort, dual-cartridge per side under study.
9. **Sensor dropout (IMU, pitot, AoA, spar-lock).** Mitigation: redundancy + EKF behavior + degraded-mode envelope.
10. **AAD interface fault.** Mitigation: independence from FCS, periodic check.
11. **Drogue mal (no inflation / asymmetric).** Mitigation: bypass + jettison-and-reserve.
12. **Mechanical reversion cable fouled or jammed.** Mitigation: routing analysis, pre-flight check, FCS redundancy is primary.
