# 06 — Test Plan

**Status:** First-cut. Defines the progression from bench through manned
deploy-in-flight, with explicit gates that map to the analyses closed in
deliverables #1–#6.

## Overall progression — no skipping

```
       ┌─────────────────┐
       │  bench /        │   component-level: cutters, drogue, snap-springs,
       │  component      │   spar-friction characterization, sensor latency
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  ground         │   200-cycle reliability gate per BRIEF;
       │  deployment rig │   the 10 ms 3-σ symmetry gate is verified here
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  tow article    │   wing already deployed before launch;
       │                 │   verifies aero predictions, control authority,
       │                 │   envelope protection — without ever needing to
       │                 │   deploy in flight
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  drop article   │   ballasted instrumented article released from
       │                 │   aircraft; full deploy + jettison + reserve
       │                 │   sequence; no human aboard
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  manned tow     │   first human flights in the deployed wing,
       │                 │   towed launch, no in-flight deploy
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  manned exit,   │   towed exit at altitude, released into a known
       │  deployed       │   glide. Avoids the deploy event but adds the
       │  (cold release) │   exit dynamic.
       └────┬────────────┘
            ▼
       ┌─────────────────┐
       │  manned exit,   │   FULL operational profile. After ALL prior
       │  deploy-in-     │   gates pass and a separate program-level
       │  flight         │   go/no-go review.
       └─────────────────┘
```

## Bench / component characterization

These run in parallel with the analysis work and feed empirical numbers
back into the predictions. They are *not* gated; results are inputs to
the ground-rig and downstream gates.

| Article | Purpose | Outputs |
|---|---|---|
| **Pyrotechnic cutter coupon** | No-fire / all-fire margins on the LSC and initiators per shipset. EMI immunity per DO-160. | Vendor cert + program acceptance file |
| **Drogue ringslot** | Inflation profile, snatch load, body-axis stability under representative q. | Time-history trace of bridle load + canopy area |
| **Tape-spring rib snap test** | Per-rib unfurl time and dispersion across temperature. Force trace via instrumented hinge. | Distribution of snap-energy and snap-time |
| **Spar joint friction** | Stage-lock-time variance under thermal + humidity conditioning, water/ice ingress. | Distribution of stage friction force vs. T, RH |
| **Microswitch latency** | Hardware response from physical contact close to FCS GPIO read. ≤ 1 ms requirement. | Per-channel latency distribution |
| **CO2 cartridge variance** | Mass-flow rate vs. temperature, batch-to-batch, ≥ 200 cartridges. | Distribution of effective fill mass and flow rate |
| **Skin tension and bond** | Tear-strength of the rib/spar bond after thermal cycling. | Stress-to-failure of bonded coupons |

Each is a self-contained test plan written when the article is built.
Distributions feed the symmetry-budget Monte Carlo
([`analysis/deployment/symmetry_budget.py`](../analysis/deployment/symmetry_budget.py))
to replace the engineering estimates with measurements.

## Ground deployment rig (deliverable #6)

Specification: [`test/ground/spec.md`](../test/ground/spec.md).

**Gate:** 200 cycles of symmetric, sensed deployment over the full
thermal and humidity envelope, no operator intervention. 3-σ |Δt_LR|
≤ 10 ms across the run.

Conditions and counts:

| Condition | T | Humidity / state | Cycles |
|---|---|---|---|
| Hot/dry | +50 °C | < 20 % RH | 25 |
| Hot/humid | +35 °C | 95 % RH | 25 |
| Nominal | +20 °C | 50 % RH | 50 |
| Cold | −10 °C | dry | 25 |
| Cold/wet | −5 °C | spray-soaked, drained | 25 |
| Cold/iced | −5 °C | spray-soaked, frozen | 25 |
| Re-test in nominal | +20 °C | 50 % RH | 25 |
| **Total** | | | **200** |

Failure-investigation protocol per [`test/ground/spec.md`](../test/ground/spec.md):
no "ran it again, it worked" closures.

Pre-gate also: the **first cutter live-fire on a complete wing assembly
in the program** happens here, in a controlled test cell. The 200-cycle
deploy gate uses the FCS-mediated mechanical-only deploy-restow path;
cutter firings are a separate, single-shot test campaign on the same
fixture.

## Tow article (no in-flight deploy)

Article: structurally representative wing assembly, pre-deployed and
secured open, towed behind a vehicle (vehicle tow at low speed for
initial cards) or boat (extended speed envelope).

**Gate to enter:** Ground-rig 200-cycle gate passed.

Test cards:

| Card | What it shows | Pass criteria |
|---|---|---|
| Static tow at low speed | Trim alignment with prediction | CL_α matches Weissinger ±10 % |
| Speed envelope sweep | L/D vs V across operational range | Curve shape matches prediction; (L/D)_max within ±15 % of nominal |
| Slow-flight characterization | α_stall location and stall character | Gentle stall, no abrupt rolloff |
| Alpha-limiter activation | Limiter holds α at limit when commanded into stall | No departure |
| Pilot-CG-perturbation simulation | Ballast shifted at altitude on the tow boom | Recovery within 5 s; no departure |
| Asymmetric-control demonstration | Differential flaperon command produces predicted roll rate | Roll rate matches AVL prediction |
| Gust ingestion | Tow through a marked turbulent area | Pilot rates handling qualities Cooper-Harper ≤ 4 |

**Gate to exit:** Trim, stability, control authority within predicted
envelopes. Envelope protection demonstrated under realistic disturbances.
Cooper-Harper ≤ 4 across the operational envelope.

## Drop article (no human; full sequence)

Article: ballasted instrumented test article in the wing-and-harness
configuration, released from a chase aircraft at altitude. Executes the
full deploy sequence including drogue, wing deploy, and (in
deliberately-induced abort cases) jettison + reserve.

**Gate to enter:** Tow article passed all cards.

Test cards:

| Card | What it shows | Pass criteria |
|---|---|---|
| Nominal deploy at design speed | Full sequence executes per `docs/03` | All states transitioned in budget; trim acquired |
| Cold-condition deploy | Conditioned article deploys nominally | Same |
| Asymmetric-deploy abort (induced) | One side delayed by 30 ms; abort fires | Jettison fires within 75 ms of detection; reserve clears stubs |
| Drogue-mal abort | Drogue PC deliberately disabled | Bypass-and-jettison logic fires; reserve inflates clean |
| AAD-induced abort | AAD altitude trigger forced | Wing departs ahead of reserve; reserve inflates clean |
| Spar-lock-fail abort | One lock sensor disconnected | Timeout fires; jettison + reserve |
| Reserve compatibility (asymmetric) | Asymmetric-deploy + jettison + reserve in worst-credible attitude | Reserve inflates; no fouling on stubs |

**Gate to exit:** End-to-end sequence verified; jettison + reserve clean
across all credible failure modes. Drop-article has been recovered
intact across the full envelope.

## Manned tow

First human flight. Towed launch, deployed wing only — no in-flight
deploy.

**Gate to enter:** Drop article all-cards-passed. Pilot transition
training plan in place. Pilot rated for skydiving + tow operations.

Test cards (progression, NOT a single article):

1. Static ground hold + control surface check
2. Low-altitude tow (1 m AGL), short duration
3. Step-ups in altitude and speed
4. Release at altitude into a known glide
5. Reserve-canopy descent rehearsal (deliberate jettison at safe
   altitude over water with retrieval boat)

**Gate to exit:** Pilot rates handling qualities acceptable across the
flight envelope. Recovery procedures rehearsed. Reserve descent
configurations verified for canopy phase.

## Manned exit, deployed (cold release)

Tow up to altitude, release into a stable glide, fly to landing under
the wing. Avoids the deploy event entirely.

**Gate to enter:** Manned tow passed. Independent program-level go/no-go
review.

This adds the exit-from-aircraft dynamic and the unpowered glide phase
without the deployment risk.

## Manned exit, deploy-in-flight

The full operational profile per BRIEF: pilot exits aircraft (or BASE
object), deployment sequence executes, pilot flies under the deployed
wing.

**Gate to enter:** ALL prior gates passed, AND a separate
program-level go/no-go review with:
- Independent failure-mode review
- Updated FMEA reflecting all test data
- Pilot proficiency in all prior phases
- Range / site approvals
- Recovery procedures rehearsed in real conditions
- AAD interface integration verified

This is the first time the deploy-in-flight event has a human in the
loop. It is irreducibly the highest-risk event in the program. Treat
the gate-to-enter as binding.

## Range / site requirements

| Article | Site requirement |
|---|---|
| Bench | In-house lab with range-rated EMI and pressure capabilities |
| Ground rig | Lab with environmental conditioning chamber, instrumented test bay; ≥ 10 m vertical clearance for tip-swing |
| Tow article | Open water (boat tow) or long airfield runway (vehicle tow); chase boat / chase truck |
| Drop article | Drop range with telemetry coverage; chase aircraft for release; recovery boat / recovery vehicle |
| Manned tow | Open water; drop zone with reserve-landing area downwind |
| Manned exit | Skydiving operation drop zone; chase aircraft; AAD-rated reserve packing |

## Schedule (representative)

The following is a representative critical path. Adjust as analyses and
hardware bench tests update predictions.

| Quarter | Activity |
|---|---|
| Q1 | Bench / component characterization; cutter vendor pick; CO2 cartridge data |
| Q2 | Ground rig fabrication; first nominal-condition runs |
| Q3 | Ground rig 200-cycle gate (full envelope) |
| Q4 | Tow article fabrication |
| Y2 Q1 | Tow article test campaign |
| Y2 Q2 | Drop article fabrication; first nominal drop |
| Y2 Q3 | Drop article asymmetric-deploy + reserve verification |
| Y2 Q4 | Manned tow campaign |
| Y3 Q1 | Manned cold-release flights |
| Y3 Q2 | Manned deploy-in-flight (first) |

This is a research-aircraft schedule; it slips when the gates don't
close cleanly. The program does not skip ahead.

## Reproducibility

Each article carries its own `test/<article>/` subdirectory with:

- `spec.md` — what the article is and what it measures
- `cards/` — per-card test sheets
- `data/` — retained data files, indexed by cycle/run number
- `failure-investigations/` — closed-out write-ups for any anomaly

Already populated: [`test/ground/spec.md`](../test/ground/spec.md). Tow,
drop, manned-tow, and manned-exit articles get specs as the program
reaches them.
