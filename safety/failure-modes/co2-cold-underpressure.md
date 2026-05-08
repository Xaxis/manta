# Failure mode: CO2 cartridge under-pressure (cold)

**FMEA ID:** `FM-CO2-001`
**Severity:** Catastrophic if it causes asymmetric deploy or no-deploy
**Pre-mitigation likelihood:** ~10⁻² per flight at T < 0 °C, < 10⁻⁴ at T > +20 °C (single cartridge per side)
**Post-mitigation likelihood:** Bound by `FM-DEP-001` mitigation (option B + shared reservoir + active modulation)

## What it means

CO2 cartridge mass-flow rate depends on the cartridge's internal
pressure, which is set by its temperature (vapor pressure of CO2 at
ambient T). Cold cartridges flow significantly less mass per second:

| Cartridge T | Vapor pressure | Effective mass-flow rate (relative to +20 °C) |
|---|---|---|
| +30 °C | ~71 bar | 1.05× |
| +20 °C | ~57 bar | 1.00× (reference) |
| 0 °C | ~35 bar | ~0.85× |
| −10 °C | ~26 bar | ~0.75× |
| −20 °C | ~19 bar | ~0.65× |

Below ~−15 °C the regulator may not be able to maintain the working
pressure for the full deploy duration, even on a fresh cartridge. The
two failure sub-cases:

| Sub-case | Consequence |
|---|---|
| Both cartridges run out of pressure before stages lock | Wing partially deploys; stage-lock-fail timeout fires jettison + reserve |
| One cartridge underflows asymmetrically (the colder one or the leaker) | Asymmetric Δt > 10 ms gate; jettison + reserve |
| Both cartridges underflow symmetrically | Slow-deploy on both sides; may still complete within the 0.5 s wing-deploy timeout, but with reduced energy and possible partial lock |

## Detection

- **Cartridge T + P telemetry** before fire — sensors at each cartridge
  read T and (via the regulator) starting pressure. The FCS pre-flight
  check refuses to arm if either cartridge is below the operational
  threshold.
- **Manifold pressure trace during fire** — the ground rig measures
  this; in flight the FCS records it but doesn't gate on it (too late).
- **Stage-lock progression** — the abort logic catches the downstream
  consequences (asymmetric, no-lock, lock timeout).

## Mitigation chain

The dominant mitigation is at the symmetry-budget architecture level
(per [`analysis/deployment/symmetry-budget.md`](../../analysis/deployment/symmetry-budget.md)):

1. **Shared reservoir (option A)** — the per-cartridge variance becomes
   common-mode and cancels in Δt, so a cold-batch cartridge can't
   produce asymmetric flow. CdA budget closes from CO2 contribution
   10.6 ms 3-σ to ~3 ms.
2. **Active per-side flow modulation (option B)** — closed-loop sensing
   of stage-lock progress modulates per-side flow. If both sides are
   underflowing symmetrically (cold-soak), the FCS can decide the deploy
   is no longer credible and abort to jettison + reserve before either
   side completes (preventing partial deploy).
3. **Combined A + B** — both architectural changes together. This is
   the recommended end-state for the program.

In addition:

4. **Heated cartridges.** Each cartridge has a small resistive heater
   (~2 W) that maintains it at +20 °C from arm to fire. Battery cost
   is small; mass cost ~30 g per cartridge.
5. **Operational envelope discipline.** No flight at ambient T < −10 °C
   until the ground rig demonstrates the 200-cycle gate at the next-
   colder temperature with no failures. The envelope grows with
   demonstrated reliability.
6. **Cartridge pre-flight check.** Each cartridge's T + P verified
   before the wing assembly is stowed. Out-of-spec cartridges are
   replaced. Fresh cartridges sourced from a single batch with traceable
   lot numbers.
7. **Cartridge installation procedure.** Riggers install fresh
   cartridges per shipset; cartridges are NOT carried on the aircraft
   between flights at low ambient T.

## Residual risk

After mitigation:

- **Heated cartridge fails** during a cold flight (heater open or
  power lost): residual is the un-heated case, which the option-B
  active-modulation handles. ~10⁻⁴ per flight.
- **Operational-envelope discipline fails** (pilot dispatches outside
  envelope): procedural-discipline residual; ~10⁻⁴ per flight assumes
  a properly trained operator and rigger sign-off.
- **Cartridge actual T + P deviate from datasheet** (manufacturing
  defect): ~10⁻⁵ per cartridge; pre-flight T + P check catches most.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Cartridge mass-flow vs. T | Each lot characterized over operational T range | Bench |
| Heater system test | Cartridge holds +20 °C in −10 °C ambient with 2 W draw | Bench |
| Cartridge T + P sensor calibration | Sensors read accurately to within 0.5 °C and 1 bar | Bench |
| Shared-reservoir test | Common-mode CO2 effect verified — single bottle drives two outlets | Bench |
| Active-modulation test | FCS modulates valve flow based on simulated stage-lock-progress sensor input | Bench HIL |
| Ground-rig cold-condition runs | 200-cycle gate at the cold-conditioned envelope | [`test/ground/spec.md`](../../test/ground/spec.md) |

## Open issues

- Heater design and cartridge mounting. The 2 W heater is a small mass
  but a real wiring change to the harness; needs to be in the FCS bay
  CAD.
- Single-batch cartridge sourcing — operationally enforced, but the
  enforcement is a procedural document that needs to be written
  (ops manual, deferred from this phase).
- Higher-pressure alternative to CO2 (e.g. compressed N₂ at 200 bar):
  trades higher pressure for higher mass and complexity. Not currently
  considered but worth mentioning as an architecture-level fallback if
  CO2 cannot close cold-condition reliability.
