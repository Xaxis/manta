# Failure mode: pyrotechnic cutter no-fire

**FMEA ID:** `FM-CUT-001`
**Severity:** Catastrophic
**Pre-mitigation likelihood:** ~10⁻³ per cutter, per fire command
(single-initiator, untested LSC)
**Post-mitigation likelihood:** < 10⁻⁵ per cutter (B-basis on dual-initiator
qualified hardware)

## What it means

One or more of the four spar-root cutters fails to sever its spar when
commanded to fire. Possible causes:

- Initiator dud (NSI failed to fire)
- LSC defect (charge fails to propagate, partial cut)
- Electrical fault (firing current did not reach initiator)
- Mechanical / lockout fault (lockout pin not removed pre-flight, or
  arm sequencing prevented fire)
- Cold-soak fault (LSC sensitivity outside operating envelope)

Consequence: when jettison is commanded but fewer than 4 cutters fire,
the wing **does not separate cleanly**. Failure modes downstream include:

- **Partial separation**: 2 or 3 cutters fire, the wing tilts about the
  unfired joint(s) and may flap, exposing the reserve canopy path.
- **Asymmetric separation**: front-pair fires, rear-pair doesn't (or
  vice versa). Wing rotates about the still-attached spar root.
- **No separation**: 0 fires. Wing remains attached; pilot must use
  manual reserve through-the-wing if reserve geometry permits, or
  accept loss of vehicle.

## Why this is binding on the safety case

Cutter-no-fire is the failure mode that defeats the rest of the safety
architecture. Every upstream abort path (asymmetric-deploy detect,
drogue-mal, AAD trigger, pilot manual) ends with **fire the cutters**.
If the cutters don't fire, the abort doesn't recover the situation.

This is the *last-line* failure for ~80 % of credible flight emergencies
in MANTA. Reliability requirements are correspondingly high.

## Detection

The cutter system is **not directly self-monitoring** in the sense of
providing a "fired OK" feedback signal during the fire event itself
(LSC firing happens too fast, ~5 ms, and the resulting current/voltage
signature is destructive to the firing path). Detection is *indirect*:

1. **Spar separation sensors** at each cutter: a microswitch or
   continuity loop that opens when the spar physically separates.
   Closes within 30 ms of fire on a successful cut. Absence of the
   signal at 50 ms after fire = no-fire.
2. **Body rate signature**: a successful 4-cutter fire produces an
   abrupt mass change and a brief asymmetric force pulse. Absence of
   this signature in the IMU within 100 ms of fire confirms no-fire.
3. **Cross-check**: both signals must agree. Single-channel
   disagreement → degraded confidence; both saying "fired" → fired;
   both saying "didn't fire" → re-fire attempt then manual reserve.

## Mitigation chain (in order of independence)

1. **Dual independent initiators per cutter.** Two NSI initiators
   wired to two separate firing paths, each capable of sustaining the
   LSC propagation alone. Probability of both failing simultaneously
   ~ 10⁻⁶ per cutter (vendor B-basis).
2. **Redundant firing circuits.** FCS-A and FCS-B each have an
   independent firing-current path to each cutter, plus the AAD
   FCS-bypass path. Three independent fire-command sources per cutter.
3. **Re-fire on no-fire detection.** If the no-fire signature is
   detected within 50 ms of the first fire command, the FCS commands
   a re-fire on the redundant initiator path. (Programmed delay 30 ms
   between fire and re-fire to allow the no-fire detection window.)
4. **B-basis-tested LSC.** No-fire/all-fire margins per shipset, plus
   acceptance test on every cutter.
5. **Cold-soak qualification.** Cutter operating envelope verified at
   −20 °C (lower than the operational envelope of −10 °C in
   `test/ground/spec.md`).
6. **Pilot manual reserve as final fallback.** If 4-cutter fire fails
   even after re-fire, the pilot pulls the reserve handle. Reserve
   canopy may need to deploy through partial wing structure — risk
   accepted, but quantified in `safety/reserve-compat.md`.

## Time budget for the re-fire path

```
   t = 0     fire command issued
   t = 5 ms  cutters fire (or attempt to)
   t = 30 ms spar separation detected on healthy cuts
   t = 50 ms no-fire detection threshold reached on any unfired cutter
   t = 60 ms re-fire command issued via redundant path
   t = 65 ms second-attempt cutters fire
   t = 95 ms separation detected (or final no-fire confirmation)
```

Total time-to-confirm-jettison-success: **< 100 ms**. The reserve
canopy deployment delay (programmed 100 ms after wing-cutter fire on
AAD-trigger path) is sized to cover this window.

## Residual risk after mitigation

- **Both initiators on one cutter fail** (manufacturing defect, common-
  mode like batch contamination): ≤ 10⁻⁶ per cutter, ≤ 4·10⁻⁶ per
  jettison. **Mitigation:** vendor lot-acceptance testing; do not pull
  initiators from a single batch for all 4 cutters on a given vehicle
  (procedurally enforce two-batch initiator sourcing).
- **All four cutters fail simultaneously** (catastrophic common-mode):
  ≤ 10⁻⁸ per jettison. Plausible only with a systematic fault in the
  firing-circuit power rail. **Mitigation:** independent power rails
  (FCS-A, FCS-B, AAD-bypass), all qualified independently.
- **Lockout fails to disengage** post arm: detected by built-in test
  pre-flight. Aircraft does not fly with a failed lockout test.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Vendor coupon test on the LSC and initiator | No-fire/all-fire B-basis margins | Vendor lab |
| Lot acceptance test per shipset | Margins maintained for the specific cutters being flown | Vendor lab + program acceptance file |
| Bench fire of one cutter on a representative spar root | Cut quality (clean severance, no fiber snag) | In-house bench |
| EMI immunity test (DO-160 H) | Cutter does not inadvertent-fire under HIRF / lightning | EMI test lab |
| Lockout cycle test | 100 cycles of lockout engage/disengage with no failures | In-house bench |
| Cold-soak fire test | Cutter fires nominally at −20 °C | Climate chamber |
| Ground-rig integrated fire test | First live cutter fire on a complete wing assembly in the program | [`test/ground/`](../../test/ground/) |
| Drop article asymmetric-deploy | Real-flight no-fire case is exercised on instrumented test article | [`test/drop/`](../../test/drop/) |
| Re-fire path timing test | Re-fire fires within the 60-ms window post no-fire detection | Bench + drop article |

Until **all gates close**, the residual-risk numbers are predictions.

## Open issues

- Lot-acceptance sample size and reject criteria not yet specified.
  Defer to the vendor's standard practice plus program independent
  review.
- Re-fire timing window (60 ms) is engineering placeholder. Real
  number is the time to confirm a no-fire on the spar separation
  sensor — tightens once the sensor is selected.
- Cutter firing in the asymmetric-deploy *while-deploying* case (i.e.
  the spar is in mid-extension when the cutter is asked to fire on
  it). LSC behavior on a moving spar is plausibly different from on
  a static one. Bench characterization required.
