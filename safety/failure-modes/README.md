# safety/failure-modes/

Per-failure-mode write-ups. Each file: detection, mitigation, response, residual risk, test evidence.

## Naming

`{subsystem}-{short-name}.md` — e.g. `asymmetric-deployment.md`, `cutter-no-fire.md`, `reserve-fouling.md`.

## Priority queue

In rough order of severity × likelihood, populate:

1. `asymmetric-deployment.md` — dominant unrecoverable failure mode per BRIEF. First file written here.
2. `cutter-no-fire.md` — pyrotechnic spar-root cutter fails to fire on jettison command.
3. `cutter-inadvertent-fire.md` — cutter fires when it shouldn't.
4. `reserve-fouling.md` — reserve canopy or lines snag on residual wing structure.
5. `alpha-limiter-loss.md` — envelope protection lost or saturated.
6. `joint-water-ice-ingress.md` — telescoping joint binding from environmental contamination.
7. `co2-cold-underpressure.md` — cartridge insufficient at low ambient T.
8. `sensor-dropout.md` — primary sensor fails; per-channel analysis.
9. `aad-fault.md` — AAD interface fault.
10. `drogue-mal.md` — drogue malfunction.
11. `reversion-fault.md` — mechanical reversion path unavailable.

Each file links back to its row in `safety/fmea.md`.
