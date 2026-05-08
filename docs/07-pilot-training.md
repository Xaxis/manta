# 07 — Pilot Training Transition

**Status:** First-cut. Defines the syllabus structure and the gate-by-gate
progression from a current fabric-wingsuit pilot to a MANTA-rated pilot.
Tied directly to the test-article progression in
[`docs/06-test-plan.md`](06-test-plan.md).

This addresses BRIEF unsolved-problem #5:

> Pilot training transition path from fabric wingsuit to fly-by-wire
> rigid wing.

A wingsuit pilot, even a very experienced one, does NOT have flight
hours on a fly-by-wire rigid wing. The skill set is genuinely new:
- The vehicle has a stable trim point that the FCS holds; the pilot
  is not constantly fighting body shape to maintain glide.
- The pilot has direct stick / yoke / body-tilt input to flaperons,
  not differential body shape.
- The vehicle has an alpha limiter that **prevents** the pilot from
  pulling into stall — a behavior that's the OPPOSITE of wingsuit
  stall mitigation, where pilot pulls into a flare and accepts the
  drop.
- Reserve deployment is via a wing-jettison-then-reserve sequence,
  not a wingsuit cutaway-and-reserve.

A pilot who carries wingsuit muscle memory into MANTA will:
- Try to pitch the body for trim (fights the FCS).
- Try to hold body-stable into stall (the alpha limiter blocks them).
- Try to deploy reserve before jettisoning the wing (creates a
  fouling case).

The training plan is built to break that muscle memory and replace it.

## Pilot prerequisites

| Requirement | Source |
|---|---|
| Active skydiving rating (USPA A or international equivalent) | Standard prerequisite |
| ≥ 100 wingsuit jumps | Same skill base; not a transferable license but a baseline |
| Current main+reserve+AAD packing within 12 months | Standard skydiving safety |
| Medical (Class 3 or skydiving medical) | Same |
| ≥ 5 hr glider time, dual-rated preferred | Stable-flight intuition |
| Cleared by program safety review for the specific airframe shipset | Program-internal — verifies the pilot has read the BRIEF and the safety case for the version of the vehicle they will fly |

## Transition syllabus

### Phase A — Ground school (~40 hours)

Lecture + simulator + paper.

| Module | Content |
|---|---|
| A.1 Aerodynamic basics | Tailless flying-wing flight characteristics; the static-margin trade; pilot CG perturbation effects (per `analysis/flightdynamics/longitudinal.py`) |
| A.2 FCS architecture | What the alpha limiter does; how mechanical reversion works; sensor fault modes (per `docs/04`) |
| A.3 Deployment sequence | The state machine, every gate, every abort path. Memorize the cockpit display interpretation for each phase (per `docs/03`) |
| A.4 Emergency systems | Cutter system, AAD interface, manual abort handle; reserve compatibility considerations (per `docs/05`) |
| A.5 PX4 SITL simulator | Hands-on with the closed-loop simulator from a representative mock cockpit. Practice handling QUALITIES but not yet real flight |
| A.6 Failure modes | Walk through every entry in `safety/fmea.md`; the appropriate response for each |
| A.7 Practical assessment | Written exam covering all six modules; passing required to advance |

### Phase B — Tow article (manned tow, no in-flight deploy)

Pilot flies the MANTA wing assembly in its deployed configuration,
towed behind a vehicle (slow at first) or boat. The deployment event
is excluded; the pilot only experiences the deployed-wing flight
characteristics.

| Card | What it builds | Gate to advance |
|---|---|---|
| B.1 Static ground hold + control surface check | Familiarization with stick / yoke / body-tilt response | Pilot completes 5 cards with no anomalies |
| B.2 Low-altitude tow (< 5 m AGL), short duration | Pilot maintains tow line tension without porpoising | Pilot rates Cooper-Harper ≤ 4 |
| B.3 Step-up tow altitude | Pilot maintains stable trim at increasing altitude | Same |
| B.4 Release at altitude into a known glide | Pilot transitions from tow to free flight without overshoot | Cooper-Harper ≤ 4 across release configurations |
| B.5 Slow flight characterization | Pilot flies at low V; alpha limiter kicks in; pilot experiences and recognizes the saturation feel | Pilot consistently respects the limit feedback without "fighting" it |
| B.6 Asymmetric-control demonstration | Differential flaperon for roll authority | Roll rate matches predicted; pilot smooth on inputs |
| B.7 Mechanical reversion mode | FCS power deliberately removed; pilot flies on the manual cable | Cooper-Harper rating documented; ≥ 5 minutes successful flight |
| B.8 Reserve descent rehearsal | Deliberate jettison at safe altitude over water; reserve descent and landing | At least 3 successful reserve descents |

### Phase C — Manned exit, deployed (cold release)

Tow up to altitude, release into a stable glide. The pilot exits
the cold-release fixture (no deployment event) and flies the
deployed wing to landing.

| Card | What it adds | Gate |
|---|---|---|
| C.1 First cold-release exit | Exit dynamic on top of deployed flight | Successful flight to landing; no anomalies |
| C.2 Cold-release at higher altitudes | Familiarization with longer glides | Same |
| C.3 Cold-release with deliberate alpha-limiter activation | Pilot demonstrates respect for the limiter in real flight | Pilot rates the experience consistent with simulator |
| C.4 Cold-release with sensor-fault injection | Pilot operates in degraded mode (AoA-fault, etc.) | Pilot maintains control; reserve descent if cued |

### Phase D — Manned exit, deploy-in-flight

The full operational profile. Pilot exits aircraft (or BASE object),
deployment sequence executes, pilot flies under the deployed wing.

**Entry gate:** Phases A, B, C all complete; independent program-level
go/no-go review; AAD verified; recovery procedures rehearsed.

| Card | What it adds | Gate |
|---|---|---|
| D.1 First in-flight deploy | The actual deployment event | Successful deploy + trim acquisition + landing |
| D.2 Subsequent in-flight deploys | Familiarization with deploy variability | Pilot rates qualitative consistency with prior deployments |
| D.3 Asymmetric-deploy abort drill | Forced asymmetric deploy (controlled by FCS injection) | Jettison fires; reserve descends to landing |
| D.4 Drogue-mal abort drill | Forced drogue mal | Same |
| D.5 Operational envelope expansion | Flights to expand the V envelope, the alpha envelope, the altitude envelope | Each flight stays within demonstrated envelope; CHQ ≤ 4 |

## Recurrency

After phase D rating, pilot maintains currency by:

- Minimum 4 flights per quarter
- Phase B equivalent (tow / cold-release) refresher every 12 months
- Re-do the simulator and ground school A.6 (failure modes) every
  24 months
- Any extended layoff > 90 days requires a re-qualification flight
  in phase C before phase D operations

## Open issues

- The simulator (A.5) is not yet built. Need PX4 SITL with the
  MANTA-specific flight dynamics + FCS modules + a representative
  cockpit interface. Significant SW effort before this phase can run.
- Cooper-Harper rating in mechanical reversion (B.7) is an unknown.
  If poor, B.7 becomes a "demonstrate jettison capability before
  HQ becomes unworkable" exercise rather than a "fly in reversion
  for 5 minutes" exercise.
- AAD vendor selection affects card D.3 — different AADs have
  different forced-trigger interfaces. Spec the cards once vendor
  is locked.
- Pilot input device — yoke, stick, or body-tilt sensor — affects
  A.5 simulator and B.1+ control surface check. Whatever is selected
  needs to be the same across the syllabus and the production fleet.
