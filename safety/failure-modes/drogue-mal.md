# Failure mode: drogue malfunction

**FMEA ID:** `FM-DEP-002`
**Severity:** Catastrophic
**Pre-mitigation likelihood:** ~10⁻³ per deployment (single drogue, no bypass logic)
**Post-mitigation likelihood:** < 10⁻⁵ per deployment

## What it means

The drogue chute fails to inflate, fails to inflate symmetrically, or
fails to release after wing deploy. Each sub-case has different
consequences:

| Sub-case | Consequence |
|---|---|
| Pilot chute (PC) misfire — drogue never extracts | Pilot stays at terminal; no deceleration; wing deploy at high q is structurally unsafe |
| Drogue inverts during inflation | Greatly reduced drag; deceleration to wing-deploy V takes much longer or never reaches it |
| Bridle line snag on the harness | Drogue inflates but doesn't load the bridle; same effect as no drogue |
| Drogue release after wing-stable fails | Drogue continues to load aft; affects wing trim acquisition; may damage wing or pull pilot off-trim |

## Time budget

From [`analysis/deployment/drogue_dynamics.py`](../../analysis/deployment/drogue_dynamics.py)
the nominal drogue inflates in ~0.45 s and decelerates the system from
55 m/s to 30 m/s in ~4.7 s. The deployment state machine
([`analysis/deployment/state_machine.py`](../../analysis/deployment/state_machine.py))
gives the drogue 4.0 s to reach 50 % nominal load before declaring a
malfunction. That timeout is conservatively the ~time required to
clear the drogue extract and inflation transient with margin.

## Detection

| Channel | What it shows |
|---|---|
| Drogue load cell on the bridle | Direct measurement; nominal trace is a fast rise to ≥ 50 % within 1.5 s of extract |
| Pitot-static airspeed | Nominal V drops from 55 → ~30 m/s in 4.7 s; flat trace = drogue not effective |
| Body axis accel | Drogue inflation produces ~0.5 g forward decel pulse; absence indicates problem |
| IMU body rates | Drogue stable should keep body rates low; spinning indicates inversion or asymmetric inflation |

The state-machine gate uses **drogue load AND airspeed-below-32 m/s**.
Either signal failing within 4.0 s triggers the abort path.

## Response

```
drogue mal detected at 4.0 s post extract
       │
       ├──► (preferred)  bypass drogue, command immediate jettison
       │                 → wing has not deployed → "jettison" of stowed wing
       │                   means firing the cutters on a still-stowed wing
       │                   to release wing assembly to fall clear
       │                 → pilot manual reserve (no AAD trigger yet at altitude)
       │
       └──► (fallback)   pilot manual reserve (cutaway main path)
                         only used if FCS jettison-fire path is faulted
```

The "jettison stowed wing" sub-case is unusual in skydiving — pre-deploy
abort that includes pyrotechnic separation of an undeployed wing. The
wing assembly is still close to the body in the stowed configuration;
firing cutters at that point releases the assembly to fall away. The
geometry is exercised once on the drop article ([`test/drop/`](../../test/drop/)).

## Mitigation chain

1. **Drogue bag + PC pre-flight check** — visual inspection that the
   PC is properly cocked, the bag is closed, the bridle is routed
   through the tunnel without snags. Standard skydiving rigger
   protocol; rigger-signoff required pre-flight.
2. **Drogue PC firing redundancy** — both spring-loaded PC and a
   timed mechanical extraction backup if PC doesn't pull the bag at 1 s.
3. **Drogue load cell as gate** — explicit detection threshold in the
   state machine, not relying on attitude or pilot perception.
4. **Drogue bypass logic** — state machine fires jettison + reserve
   on drogue mal rather than continuing into wing deploy at terminal q.
5. **Drogue release mechanism** redundancy — two independent paths to
   sever the drogue bridle after wing-stable; failure to release on the
   primary path triggers the secondary on a 0.5 s timeout.

## Residual risk

After mitigation:

- **Drogue PC and timed backup BOTH fail** (~10⁻⁶ per flight, mitigated by
  pre-flight inspection and field-tested PC hardware): residual ~10⁻⁶.
- **Drogue load cell signals "good" but is faulty** (sensor failure):
  cross-checked against airspeed. Both signals failing simultaneously
  requires correlated faults that should not happen with independent
  channels. Residual ~10⁻⁷.
- **Drogue release fails on both paths**: the trailing drogue at deployed
  wing condition affects trim. Pilot can manually cut bridle if pilot has
  reach (depends on `cad/harness/`). Residual major (loss of glide
  performance), not catastrophic.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Drogue inflation profile bench | Inflation time, peak load, dispersion | Bench drogue test |
| Drogue load cell calibration | Load cell reads bridle tension accurately at the rates of interest | Bench |
| Drogue PC firing reliability | PC fires within 1 s in 50+ trials | Bench / drop article |
| Drop article — drogue mal sub-cases | Each sub-case (PC misfire, inversion, late inflation, release fail) exercised on instrumented test article | [`test/drop/`](../../test/drop/) |
| State-machine bypass logic | Software fires jettison + reserve within budget on drogue-mal trigger | SITL + drop |
| Drogue release redundancy | Both paths exercised independently | Bench + drop |

## Open issues

- Drogue stability at terminal V — ringslot drogues have characteristic
  body-axis stability but can squidge at certain V/q combinations.
  Bench dynamic stability characterization needed.
- Bridle attachment-point load distribution on the harness — same
  3.94 kN snatch peak (per `analysis/deployment/drogue_dynamics.py`)
  needs to be in the harness mount load case.
- Drogue release: pyrotechnic vs. mechanical line-cutter is not
  selected. Pyrotechnic is faster and more reliable but adds another
  initiator to the system.
