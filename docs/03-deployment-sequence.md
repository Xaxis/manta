# 03 — Deployment Sequence

**Status:** stub — **deliverable #4** per `BRIEF.md`. The safety case is built around this document.

## Scope

The full sequence from pilot exit through stable flight, with sensed handshakes and abort logic at every transition. Deployment loads scale with dynamic pressure $q$, so timing and ordering are first-order safety items.

## Phases (to be detailed)

1. **Exit** — pilot leaves aircraft / object in stowed configuration. FCS armed, deployment inhibited until stable freefall sensed.
2. **Stabilize** — body in prone-stable freefall, accelerating toward terminal (~55 m/s).
3. **Drogue deploy** — small ringslot drogue extracted by spring/pilot-chute; reefed inflation; decelerates to ~30 m/s.
4. **Drogue-stable check** — sensed: airspeed below 32 m/s, body axes within attitude window, drogue load nominal. Pass → continue. Fail → abort path (see below).
5. **Wing deploy command** — single solenoid valve releases CO2 to both sides simultaneously through matched-impedance manifold; tape-spring ribs unfurl passively; skin tensions.
6. **Symmetry check** — sensed: left/right deploy timing within budget (target <10 ms 3-σ), spar-stage lock sensors confirm full extension both sides, skin tension OK.
   - Pass → continue.
   - Fail (asymmetric or partial) → fire jettison, deploy reserve.
7. **Drogue release** — drogue cut clear after wing-stable confirmed.
8. **Trim acquisition** — FCS captures trimmed glide, alpha limiter active, pilot input enabled.

## Sensed handshakes

Each transition has explicit go/no-go criteria from independent sensor channels:

- IMU (attitude, body rates, accel)
- pitot-static (airspeed, altitude rate)
- spar-lock microswitches (per stage, per side, per spar)
- skin tension (load cells at 2-4 chord stations per side)
- drogue load cell at bridle attach
- valve actuation feedback

Single-sensor failures must not cause unsafe progression — every gate is multi-channel.

## Abort paths

| Condition | Action | Abort window |
|---|---|---|
| Drogue fails to inflate or load <50 % nominal | Bypass drogue, command immediate jettison if armed; otherwise pilot reserve | freefall stable phase |
| Asymmetric wing deploy detected (>10 ms, lock-sensor mismatch, or roll rate >X) | Fire all 4 spar-root cutters, deploy reserve | within 200 ms of deploy command |
| Wing deploys but skin tension fails | Continue or jettison? — TBD; depends on flight-quality assessment criteria here |
| FCS fault during deploy | Fall back to mechanical reversion if airworthy; otherwise jettison + reserve | continuous |
| AAD altitude trigger (low + still falling fast) | Independent jettison + reserve, FCS-bypassed | continuous |

## Timeline (target)

To be filled with budget for each phase, including 3-σ tolerances. The symmetry budget specifically is broken out into `analysis/deployment/symmetry-budget.md` (deliverable #5).

## Deliverables

- This doc, complete and review-passed.
- State machine diagram (commit alongside).
- `analysis/deployment/symmetry-budget.md` (deliverable #5).
- `fcs/firmware/deploy-sm/` — implementing state machine.
- Integration with safety/fmea.md.
