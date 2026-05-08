# Failure mode: telescoping joint water/ice ingress

**FMEA ID:** `FM-JNT-001`
**Severity:** Major (fails to deploy on one or more stages — drives jettison)
**Pre-mitigation likelihood:** Unknown — ground-rig characterization gate
**Post-mitigation likelihood (target):** < 10⁻³ per flight in operational envelope

## What it means

The telescoping CFRP spar joints have small clearances (2–5 mm radial
between adjacent stages) for the inner stage to slide free of the outer.
Water that enters the joint can:

- Increase friction (water film between sliding surfaces)
- Freeze in cold conditions, locking the joint
- Corrode the locking-pin hardware (less of an issue with CFRP +
  stainless pins, but if any aluminum is present in the joint stack,
  galvanic corrosion accelerates)

In any of those conditions a stage may fail to extend, fail to lock,
or extend slowly. Per [`analysis/deployment/symmetry-budget.md`](../../analysis/deployment/symmetry-budget.md)
the spar-friction term is 2 ms 3-σ contribution to the symmetry
budget — second-largest contributor after CO2 cartridge variance, and
the contribution **doubles or triples in cold/wet conditions** per
field experience with telescoping CF spar systems.

## Effect

Spar-friction induced asymmetry feeds directly into the symmetry budget:

| Ambient state | Spar-friction σ_t (modeled) | Combined 3-σ |Δt| (modeled, all contributors) |
|---|---|---|
| Nominal (+20 °C, 50 % RH) | 2.0 ms | 16.3 ms |
| Cold dry (−10 °C) | 3.0 ms | ~19 ms |
| Cold wet (−5 °C, soaked) | 4.5 ms | ~24 ms |
| Cold iced (−5 °C, frozen ingress) | unknown — could be infinite (binding) | unknown |

The cold-iced case is what the BRIEF specifically calls out as
unsolved-problem #4. If a stage doesn't extend at all, the per-side
deploy time on one side becomes much greater than on the other — the
asymmetric-deploy detector fires jettison ([`safety/failure-modes/asymmetric-deployment.md`](asymmetric-deployment.md)).
**No-deploy on one side is recoverable via jettison + reserve**, but the
flight is over.

## Detection

- **Per-stage spar-lock microswitch** — direct evidence that the stage
  did or didn't reach its locked position. Failure to latch within 50 ms
  of deploy command on one or more locks → state machine treats as
  asymmetric deploy / lock-fail abort.
- **Pre-flight environmental conditioning check** — pilot or rigger
  verifies the spar joints are clean and dry before stowing the wing.
  This is a procedural mitigation; effectiveness depends on operator
  discipline.

## Mitigation chain

1. **Sealed joints.** Each telescoping stage joint has lip seals
   (silicone or fluorocarbon) at the outer-tube end that scrape water
   and debris from the inner tube as it extends. Same approach as
   high-end fishing rods and surveying tripods — well-characterized
   technology.
2. **Drainage paths.** Small drilled holes near the joint allow any
   water that does get past the lip seal to drain out as the spar
   extends. Holes are positioned so they're internal in the stowed
   configuration (so they don't whistle or admit water in flight).
3. **Hydrophobic coatings** on the sliding surfaces (PTFE-based dry
   lubricant, applied during build and refreshed periodically).
4. **Ground-rig environmental conditioning gate.** Per
   [`test/ground/spec.md`](../../test/ground/spec.md):
   25 cycles in cold/wet, 25 in cold/iced, with 50 % failure rate
   in either category aborting the gate run. The rig is the
   measurement that converts this from "unknown" to a quantified
   distribution.
5. **Pre-flight check** — visual inspection of the lip seals (tear or
   lift = no fly), drainage holes (clogged = no fly), and spar
   surface condition (visible water film = wipe dry; visible ice =
   no fly until thawed).
6. **Dispatch envelope.** No flight at ambient T < −10 °C until the
   ground rig demonstrates the 200-cycle gate at the next-colder
   temperature with no failures. The operational envelope GROWS with
   demonstrated reliability.
7. **Asymmetric-deploy detection** as the safety net — even if one
   stage fails to extend, the asymmetric detector fires jettison
   within 75 ms of detection.

## Residual risk

After mitigation:

- **Sealed-joint failure (seal tear or lift) admitting water** that the
  pre-flight check misses: ~10⁻³ per flight, residual major (asymmetric
  deploy → jettison). Catastrophic only if the cutters also fail
  (`FM-CUT-001`), which combines to < 10⁻⁸ per flight.
- **Cold-soak deeper than the operational envelope**: pilot procedural
  mitigation only. Residual is dispatching outside the envelope, which
  is a discipline problem, not a hardware problem.
- **Microswitch fault at the same time as joint binding** (so the
  state machine doesn't see the no-lock signal): mitigated by the
  dual-contact microswitch design (per [`docs/03`](../../docs/03-deployment-sequence.md)).
  Residual <10⁻⁵ per flight.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Lip-seal coupon test | Seal integrity over the operating-temperature range and 1000 sliding cycles | Bench |
| Drainage geometry test | Water introduced at joint drains under representative deploy kinematics | Bench |
| Cold-soak deploy test (uninstalled spar) | Joint friction measured at −10 °C, dry vs. wet vs. iced | Bench |
| Ground-rig cold/wet runs (×25) | Full wing assembly deploys symmetrically with cold-soaked + spray-soaked spars | [`test/ground/spec.md`](../../test/ground/spec.md) |
| Ground-rig cold/iced runs (×25) | Same with frozen ingress — the binding case | Same |
| Pre-flight checklist | Documented and rehearsed; rigger sign-off required | Operational doc TBD |

## Open issues

- Lip-seal vendor selection. Off-the-shelf options (Trelleborg, SKF,
  custom fluorocarbon) need to be matched to the operating-temperature
  range and sliding-cycle life.
- Drainage hole geometry — must drain water but not admit it during
  flight. The flow direction reverses between stowed and deployed
  configurations.
- Galvanic-corrosion analysis if any non-CFRP hardware (locking pins,
  spring-energizers) is used in the joint stack. Stainless-on-CFRP is
  acceptable; aluminum-on-CFRP is not.
- The cold-iced characterization on the ground rig is a binding gate.
  If the rig cannot achieve 200 cycles in the cold-iced category, the
  operational envelope contracts (no flight in conditions where
  ingress can freeze) — a substantial mission limitation for
  cold-weather skydiving operations.
