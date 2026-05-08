# FMEA — Failure Mode and Effects Analysis

**Status:** Top failure modes populated from the analyses closed in
deliverables #1–#6. Living document — every architecture decision change
or new analysis result updates the relevant rows. Per-mode write-ups
under [`safety/failure-modes/`](failure-modes/) carry the detail and the
verification gates.

## Severity scale

| Severity | Definition |
|---|---|
| Catastrophic | Loss of vehicle and / or pilot fatality |
| Hazardous | Substantial reduction in safety margin or pilot ability to handle adverse conditions |
| Major | Significant reduction in safety margin; substantially increases workload |
| Minor | Small reduction in safety margin; impairs ability to perform normal mission |

## Top failure modes

| ID | Subsystem | Failure mode | Cause(s) | Severity | Likelihood (post-mit) | Detection | Mitigation | Test evidence required |
|---|---|---|---|---|---|---|---|---|
| `FM-DEP-001` | deployment | [Asymmetric wing deployment](failure-modes/asymmetric-deployment.md) | CO2 fill variance, manifold imbalance, spar friction, snap dispersion | Catastrophic | ≤ 0.3 % per deploy (option-B arch) | Per-side spar-lock Δt + roll-rate divergence | Active per-side flow modulation (option B); jettison + reserve on detection | Ground rig 200-cycle gate; drop article asymmetric-deploy demonstration |
| `FM-DEP-002` | deployment | Drogue malfunction | Drogue PC misfire, drogue inversion, bridle snag | Catastrophic | < 0.1 % per deploy | Drogue load cell < 50 % within 4 s + accel signature | Bypass drogue → jettison if airworthy; pilot reserve | Drop article — drogue-mal scenario; bench drogue inversion characterization |
| `FM-STR-001` | structures | Spar root failure under flight load | Bending overload, bond defect, fatigue | Catastrophic | < 10⁻⁶ per flight | Skin tension load cells (advisory); pilot perception of flutter | Bending-sized spar (73 mm OD, 2.5 mm wall, SF 1.5 at 3 g limit); FEA of root joint; coupon tests of bonded layup | Static load test of full root assembly; FEA correlation |
| `FM-CUT-001` | jettison | Pyrotechnic cutter no-fire | Initiator dud, LSC defect, electrical fault | Catastrophic | < 10⁻⁵ per cutter (B-basis) | Cutter-fired-OK signal absent within 50 ms | Dual initiators per cutter; redundant firing circuits; manual reserve as final fallback | LSC vendor coupon tests; integrated cutter-firing tests at the drop article |
| `FM-CUT-002` | jettison | Pyrotechnic cutter inadvertent fire | Static, EMI, lockout failure, software bug | Catastrophic if airborne; major if on ground | < 10⁻⁷ per cutter operating hour | (Detection is lockout monitoring) | Two-stage arming (mechanical lockout + electrical arm); inhibits while the deployment SM is in pre-deploy state; AAD signal independent path | Lockout cycle test; EMI immunity test (military-style HIRF / lightning); first-fire on ground rig only |
| `FM-RSV-001` | reserve | Reserve canopy fouling on jettison stub | Stub geometry too large, severed-fiber snag, bridle wrap | Catastrophic | < 10⁻⁴ per jettison | (Detection is post-failure; mitigation is preventive) | Stub geometry verified clean of reserve cone in CAD + bench mock-up; rounded ferrule on cut spar; drop article verification | [`safety/reserve-compat.md`](reserve-compat.md) gates 1–6 |
| `FM-FCS-001` | FCS | Alpha-limiter loss | Both FCS units fault; AoA vane + airspeed both fail; software bug | Hazardous | < 10⁻⁵ per flight | FCS health monitor; sensor-fault EKF flags | Triplex IMUs; redundant FCS units; degraded-mode α estimate; mechanical reversion if FCS unrecoverable; pilot procedure: jettison + reserve > 200 m AGL | Sensor-fault scenario in PX4 SITL; HIL test of degraded modes |
| `FM-FCS-002` | FCS | EKF divergence in trim acquisition | Sensor noise, magnetic deviation, GNSS dropout at exit | Major | < 10⁻³ per flight | EKF residual monitor; cross-check between FCS-A / FCS-B / aux IMU | EKF tuning per environment; magnetometer calibration on every aircraft; GNSS-degraded fallback to ground-speed proxy | SITL Monte Carlo across exit conditions |
| `FM-PIT-001` | pitch stability | Pilot CG perturbation pushes wing into negative SM | Postural change in flight; gust-induced pilot motion | Hazardous | ~ 1 in 10² flights (un-mitigated) | EKF body-rate divergence; α nearing limit | Alpha limiter mandatory; pitch damper in the inner loop; harness restraints to limit pilot torso travel; pilot training | Closed-loop SITL across CG-shift envelope; wind-tunnel or tow-test verification |
| `FM-JNT-001` | spars | Telescoping joint binding (water/ice ingress) | Sub-zero ambient, rain on jump run, condensation | Major (fails to deploy on one or more stages) | Unknown — ground-rig characterization gate | Per-stage lock sensor doesn't latch within timeout | Sealed joints (lip seals + drainage), thermal conditioning of stowed assembly, pre-flight check after exposure | Ground rig cold/wet/iced conditioning runs (25 cycles each per [`test/ground/spec.md`](../test/ground/spec.md)) |
| `FM-CO2-001` | pneumatics | CO2 cartridge under-pressure (cold) | Sub-zero ambient or stored cartridge | Catastrophic if it causes asymmetric deploy or no-deploy | Bound by `FM-DEP-001` mitigation (option B) | Per-cartridge T + P sensor pre-firing | Heated cartridges OR shared CO2 reservoir + Option B; pre-flight check | Ground rig cold-condition firings |
| `FM-AAD-001` | safety | AAD fault | Electronics failure, dead battery, mis-armed | Hazardous | ~ 10⁻⁵ per flight (skydiving baseline) | Built-in self-test pre-flight | Production-grade AAD with current cert; pre-flight self-test mandatory; pilot-initiated reserve always available | AAD cert + integration test |
| `FM-DRG-001` | drogue | Reserve canopy descent obstructed by harness mount | Mount geometry not optimized for canopy phase | Major (slow descent rate, hard landing) | < 10⁻³ per landing | Pilot perception under canopy | Harness mount geometry verified for canopy descent; flight-test under reserve; pilot-rated landing-speed envelope | First-cut: `safety/reserve-compat.md`; later: drop article + manned-canopy rehearsal |
| `FM-RVR-001` | mech reversion | Mechanical reversion path blocked / fouled | Cable routing chafe, internal interference with deployed wing | Hazardous | < 10⁻⁴ per flight | Pre-flight functional check; reversion-mode entry test once per pre-flight | Routing analysis; pre-flight check in deployed-mock-up configuration; pilot procedure: jettison + reserve if reversion fails | First-cut: `cad/fcs/` routing study; later: bench reversion test |

## Progression / interconnections

These rows are not independent. The dominant chain is:

```
   FM-CO2-001 → FM-DEP-001 → FM-RSV-001 (if jettison stub fouls reserve)
                       ↓
                  FM-CUT-001 (if cutters don't fire on the asymmetry)
                       ↓
                  pilot manual reserve (last fallback)
```

Mitigation of `FM-DEP-001` (the dominant case) cascades downstream — the
better the symmetry budget closes, the less reliance on the
jettison-and-reserve chain.

## What the BRIEF rule "would this hold up if a coroner asked for it"
means in this context

For each row above, the program needs:
1. The cited test data exists and is documented.
2. The mitigation is in the as-built hardware and software, not just on paper.
3. The independence of redundancy paths is verified, not assumed.
4. Failure investigations from any test campaign are closed before
   moving on (no "ran it again, it worked").

Until all rows have completed verification gates, the residual-risk
likelihoods are predictions, not measurements.

## To populate next

Per-mode write-ups in [`failure-modes/`](failure-modes/) for the rows
above that don't yet have one. Priority: `FM-CUT-001`, `FM-CUT-002`,
`FM-RSV-001`, `FM-AAD-001`, `FM-FCS-001`, `FM-JNT-001`.
