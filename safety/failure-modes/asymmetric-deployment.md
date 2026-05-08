# Failure mode: asymmetric wing deployment

**FMEA ID:** `FM-DEP-001`
**Severity:** Catastrophic
**Pre-mitigation likelihood:** ≈ 25 % per deploy (locked architecture)
**Post-mitigation likelihood:** ≤ 0.3 % per deploy (option-B architecture)

The dominant unrecoverable failure mode for MANTA per `BRIEF.md` and the
unsolved-problem #1 the program is built around. This document is the
single-failure-mode write-up that the safety case turns on; everything
else in `safety/` cross-references it.

## What "asymmetric deployment" means

The two wings deploy with a left-right deploy-time delta exceeding the
10 ms 3-σ envelope the BRIEF requires. The aerodynamic and mechanical
consequences are:

1. **One wing locks before the other** → momentary asymmetric lift →
   roll-rate impulse before the second wing finishes locking.
2. **Skin tension applies asymmetrically** → if it persists, wing twist
   diverges from design.
3. **Drogue is still attached during the deploy event** → drogue tries
   to keep the system facing the relative wind, but the asymmetric
   roll-rate fights it.
4. **Pilot-induced response is too slow** → human reaction time ≥ 200 ms
   is an order of magnitude longer than the deploy event, so manual
   recovery is impossible. Either the FCS catches it, or the abort path
   fires.

If un-aborted, asymmetric deploy ends in:
- Fast roll, possibly into a non-recoverable spin
- Skin/spar damage that prevents stable flight
- Loss of vehicle control before pilot can react
- Reserve canopy deploys into a tumbling configuration with high risk
  of fouling

## Why it is the dominant case

- It is **rapid**: the entire deploy completes in ~50 ms. There is no
  pilot in this loop.
- It is **load-amplifying**: the higher the asymmetry, the higher the
  roll torque, the worse the spar root reaction load on the slow side
  (which still has to absorb the snap-deploy impulse from the fast side
  via cross-spar coupling at the root).
- It is **architecture-coupled to the most complex subsystem** — the
  pneumatic deployment plumbing — and the deployment is a one-shot
  operation with no rehearsal between exit and trim.
- It is **the failure case that compromises the reserve**: a wing
  asymmetric in mid-deploy can foul the reserve deployment path if
  not jettisoned cleanly. Reserve compatibility (`safety/reserve-compat.md`)
  depends on jettison happening cleanly even in the asymmetric case.

## Detection

Two independent detection channels, both wired into the deployment state
machine ([`analysis/deployment/state_machine.py`](../../analysis/deployment/state_machine.py)):

1. **Per-side spar-lock timestamps.** Six microswitches (3 stages × 2
   sides) log lock events with ≤ 1 ms latency. The state machine
   computes Δt_LR between left-fully-locked and right-fully-locked.
   Δt > 10 ms (the BRIEF gate) → fire jettison.

2. **Roll-rate divergence.** During the deploy window (a fixed time box
   of 200 ms after the wing-deploy command), if body roll rate exceeds
   1.5 rad/s, the system fires jettison without waiting for the lock-
   timestamp evaluation. This catches the case where one side locks but
   the other doesn't lock at all (no second timestamp to compare).

Either trigger → jettison + reserve.

## Time budget

| Event | Time after deploy cmd |
|---|---|
| First side begins to deploy | 0 ms (cmd issued) |
| First-side stage 1 lock | ≤ 30 ms |
| First-side stage 3 lock (first side fully open) | ≤ 50 ms |
| Decision window for Δt evaluation | first-locked + 10 ms = ≤ 60 ms |
| Cutter fire on jettison | + 5 ms (electrical), + 10 ms (LSC propagation) |
| Wing departs | + 30 ms (mechanical separation) |
| Reserve drogue extracted | + 200 ms (AAD or pilot) |
| Reserve canopy inflated | + 1.5–3 s |

The detection-to-jettison-fire path has to complete in **under 75 ms** to
ensure the wing is clear before reserve deployment begins, even if the
pilot starts the reserve immediately on noticing the failure.

## Pre-mitigation likelihood

From `analysis/deployment/symmetry-budget.md` (50 k-trial Monte Carlo over
the locked architecture's 5 contributors): combined 3-σ |Δt| = 16.3 ms vs
the 10 ms gate. False-positive jettison rate at the 10 ms gate: ~25 %.

That's not the failure rate; that's the **rate at which the gate fires
for an arguably-recoverable deployment**. The real catastrophic-asymmetry
rate is dominated by extreme outliers in the same distribution — about
1 in 1,000 trials show |Δt| > 18 ms, where the wing is fully open on
one side but the other isn't, and the roll impulse is severe.

## Mitigation — the architecture amendment

The nominal jettison path (Δt > 10 ms → fire all 4 cutters → wing
departs, reserve deploys) is *post-failure remediation*. The primary
mitigation is **preventing the asymmetry in the first place**.

Per `analysis/deployment/symmetry-budget.md`, the BRIEF architecture
(decision #5: passive sequencing through a single valve) cannot meet the
10 ms 3-σ gate. The recommended amendment is **Option B — active per-side
flow modulation**: the FCS senses per-side stage-lock progress and
modulates per-side valve flow to slow whichever side is leading, in
real-time. With Option B's modeled variance, 3-σ drops to 8.1 ms and the
catastrophic-asymmetry tail (|Δt| > 18 ms) drops by ~3 orders of
magnitude.

Option B *requires* the FCS to be in the deployment loop (vs. open-loop
sequencing). This is a real architecture change: the FCS becomes
deployment-critical, not just flight-critical. The mitigation for FCS
fault-during-deploy is unchanged — fire jettison, reserve.

## Response

```
asymmetric deploy detected
        │
        ├──► fire all 4 spar-root cutters (LSC, dual initiators per cutter)
        │
        ├──► wing assembly departs in < 50 ms
        │
        ├──► reserve canopy command issued (or AAD-initiated)
        │
        └──► pilot canopy descent and landing as a normal skydive scenario
```

After jettison, the underlying piggyback skydiving rig functions normally
(reserve canopy inflates clean over the head — verified geometrically in
`safety/reserve-compat.md`). The pilot lands under the reserve canopy.

## Residual risk

After mitigation:

- **Catastrophic asymmetry that escapes detection:** requires both the
  spar-lock channel AND the roll-rate channel to fail to detect. With
  redundant sensors and independent logic paths, residual probability is
  estimated < 10⁻⁵ per deploy. Acceptable.
- **Cutter no-fire on one or more cutters:** if 2 of 4 fail to fire,
  partial separation could foul the reserve. Residual risk addressed in
  `safety/failure-modes/cutter-no-fire.md` (TBD) — dual initiators per
  cutter and B-basis no-fire/all-fire margins on the LSC are the
  mitigations.
- **Reserve fouling on residual structure:** geometric clearance verified
  in `safety/reserve-compat.md` (TBD content); also a verification gate
  on the drop article (`test/drop/`).

## Verification gates

The following must be measured / demonstrated before manned-deployment
flight is permitted:

1. **Ground-rig 200-cycle gate** ([`test/ground/spec.md`](../../test/ground/spec.md)): demonstrates that
   the implemented architecture (Option B + the bench-tightened input
   distributions) actually closes the 10 ms 3-σ gate over the full thermal
   and humidity envelope, in 200 cycles without intervention.
2. **Lock-sensor latency** ≤ 1 ms hardware-verified per channel before
   that channel is used in the deploy gate logic.
3. **Cutter no-fire/all-fire margins** demonstrated per cutter at LSC
   acceptance test, plus dual-initiator independence verification.
4. **Drop-article asymmetric-deploy demonstration**: a deliberately
   asymmetric deploy (one side delayed) is induced and the jettison +
   reserve sequence completes cleanly on an instrumented ballast article
   ([`test/drop/`](../../test/drop/)).
5. **Reserve-canopy clearance** with severed-state stub geometry — both
   geometric (CAD) and physical (drop-article) verification.

Until **all five gates** are passed, the residual-risk numbers above are
predictions, not measurements, and manned-deployment flight is not
authorized.

## Open issues

- The roll-rate threshold (1.5 rad/s) is an engineering placeholder; once
  6DOF closed-loop simulation is in place, calibrate against the actual
  asymmetric-deploy roll profile.
- The 200 ms deployment-monitoring window is also a placeholder — it
  should be set to (longest-credible-deploy-time + 50 ms margin) once
  bench data tightens that distribution.
- Cross-coupling of asymmetric-deploy roll impulse into the bending
  load case for the slow-side spar root is not yet sized. The current
  `analysis/struct/spar_bending.py` uses symmetric loading; an asymmetric
  case at the worst credible Δt would be a load-case addition.
