# 03 — Deployment Sequence

**Status:** First-cut closed. The state machine, sensed handshakes, and abort
paths are specified and a runnable simulator validates them
(`analysis/deployment/state_machine.py`). The companion **symmetry budget
fails** the BRIEF 10 ms 3-σ gate with the locked architecture — see
`docs/02b-symmetry.md` (or section "Symmetry budget closure" below) for the
architecture revisions that close it. Resolution is required before any
flight-relevant test article is built.

This document defines the safety case the program is built around. Every
transition gate, every abort path, every sensor channel listed below has
implications downstream: spar-lock sensors get specified in `cad/jettison/`,
the alpha limiter inherits the FCS-fault path, and the ground deployment rig
exists to characterize every contributor in the timing budget.

## State machine

```
                        +──── pilot_arm ────────────────────►
                        │                                     │
       ┌────────────┐   │   ┌────────────┐                   │
       │   INIT     ├───┘   │   ARMED    │   accel < 0.3 g, 200 ms dwell
       └────────────┘       └─────┬──────┘
                                  │
                                  ▼
                            ┌──────────┐
                            │ FREEFALL │   body_rates < 0.5 rad/s, 1 s dwell
                            └─────┬────┘
                                  ▼
                            ┌────────────┐
                            │ STABILIZE  │ ──── (issue drogue extract cmd)
                            └─────┬──────┘
                                  ▼
                       ┌───────────────────────┐
                       │  DROGUE_INFLATING     │
                       └─────┬─────────────────┘
                load ≥ 50 %  │   load fail / timeout 4 s
                airspeed < 32│   ──────► JETTISON + RESERVE
                m/s          ▼
                      ┌────────────────┐
                      │ DROGUE_STABLE  │ ──── (issue wing deploy cmd)
                      └──────┬─────────┘
                             ▼
                     ┌────────────────────┐
                     │   WING_DEPLOY      │
                     └──────┬─────────────┘
       all 6 spar-lock OK   │   any-lock-fail / Δt > 10 ms 3-σ / timeout 0.5 s
       Δt_LR ≤ 10 ms        │   ──────► JETTISON + RESERVE
                             ▼
                     ┌─────────────────────┐
                     │ WING_TRIM_ACQUIRE   │
                     └──────┬──────────────┘
            FCS healthy +   │
            airspeed window ▼
                     ┌─────────┐
                     │  GLIDE  │   ← steady state
                     └─────────┘
```

**Parallel abort monitors** active from ARMED onward override every state and
go directly to `JETTISON_RESERVE`:

- `pilot_abort` (manual handle pull, two-action requirement to prevent inadvertent fire)
- `aad_fire AND altitude < 200 m AGL` (AAD-triggered, FCS-bypassed)
- `fcs_fault` (irrecoverable; if airworthy, `MECHANICAL_REVERSION` instead of jettison)

## Phase budgets (nominal)

From the simulator's nominal scenario (`make_nominal_scenario`):

| Phase | Entry trigger | Duration (s) | Exit gate |
|---|---|---|---|
| ARMED → FREEFALL | exit detected | 0.2 (dwell) | accel < 0.3 g, 200 ms dwell |
| FREEFALL → STABILIZE | rates settled | 1.0 (dwell) | rate_mag < 0.5 rad/s, 1 s dwell |
| STABILIZE → DROGUE_INFLATING | drogue extract cmd | < 5 ms | command issued |
| DROGUE_INFLATING → DROGUE_STABLE | drogue load + airspeed | ~2.6 (typical) | load ≥ 50 % AND V < 32 m/s |
| DROGUE_STABLE → WING_DEPLOY | wing deploy cmd | < 5 ms | command issued |
| WING_DEPLOY → WING_TRIM_ACQUIRE | spar locks + symmetry | ~0.45 | all 6 locks AND Δt ≤ 10 ms |
| WING_TRIM_ACQUIRE → GLIDE | trim captured | ~0.005 | FCS healthy, airspeed window |
| Total exit-to-glide | | **~5 s** | |

## Sensor channels and gates

Every gate is multi-channel; single-sensor failures must not cause unsafe
progression.

| Sensor | Used for gate(s) | Redundancy |
|---|---|---|
| IMU (3-axis accel + gyro) | ARMED→FREEFALL, FREEFALL→STABILIZE, abort | Triplex IMUs (Pixhawk + redundant FCS + standalone aux) |
| Pitot-static | DROGUE_INFLATING→STABLE airspeed gate, trim capture | Dual pitot booms; if both lost, GNSS-derived ground speed as degraded fallback |
| Drogue load cell | DROGUE_INFLATING→STABLE | Dual-bridge cell on bridle; cross-check vs accel signature |
| Spar-lock microswitches (6) | WING_DEPLOY → WING_TRIM_ACQUIRE | One per stage per side (3 stages × 2 sides); dual contacts on each |
| Skin tension load cells (4) | (advisory only — not a gate) | 2 per side at chord stations |
| AAD signal | global abort | independent of FCS power and bus |
| Pilot manual abort | global abort | mechanical handle + two electrical switches |
| FCS health monitor | global abort, mechanical reversion | both FCS units cross-check |

**Critical:** the WING_DEPLOY→WING_TRIM_ACQUIRE gate has two parts:
1. *all 6 spar-lock sensors confirm full extension*
2. *time delta between left-side fully-locked and right-side fully-locked is
   ≤ 10 ms (the BRIEF symmetry gate)*

The second part requires high-resolution timestamping of the lock events.
Lock-sensor channels need ≤ 1 ms latency from physical close to FCS read,
which drives both sensor selection (no software-debounced microswitches with
> 1 ms timer) and bus design (interrupt-driven GPIO, not polled).

## Abort paths

| Trigger | Time-to-decision | Action |
|---|---|---|
| Drogue load < 50 % at 4 s after extract | 4 s timeout | Bypass drogue, command immediate jettison if airworthy; pilot reserve otherwise |
| Spar-lock sensor mismatch (any side incomplete) at 0.5 s after deploy cmd | 0.5 s | Jettison + reserve |
| Asymmetric deploy: Δt_LR > 10 ms when both sides complete | < 1 ms after second side locks | Jettison + reserve |
| Pilot manual abort | continuous, < 50 ms | Jettison + reserve |
| AAD altitude trigger | continuous, < 100 ms | Jettison (FCS-bypassed) + reserve |
| FCS irrecoverable fault during deploy | continuous | Jettison + reserve |
| FCS irrecoverable fault during glide | continuous | Mechanical reversion to direct cable (preferred); jettison if airworthy revision lost |

The asymmetric-deploy detector is the dominant case per BRIEF. Detection is
in two channels:
- per-side completion timestamps (the ≤ 10 ms gate)
- attitude / roll-rate divergence outside the ARMED window

When **either** triggers, jettison fires.

## Symmetry budget closure

The Monte-Carlo error stack-up
(`analysis/deployment/symmetry_budget.py`, 50,000 trials) gives the locked
architecture's combined left-right deploy-time variance:

| Contributor | σ (ms) | 3-σ contribution to Δt (ms) |
|---|---|---|
| CO2 cartridge fill mass / temperature | 2.5 | 10.6 |
| Valve port differential | 0.8 | 3.4 |
| Manifold flow impedance match | 1.5 | 6.3 |
| Spar telescope friction (cold/wet) | 2.0 | 8.5 |
| Tape-spring snap dispersion (9 ribs/side) | 1.2 | 5.1 |
| **Combined 3-σ |Δt|** | | **16.3 ms** |

> 16.3 ms 3-σ vs the 10 ms BRIEF gate. **Fails by 6 ms.**

The CO2 cartridge variance alone (10.6 ms 3-σ) busts the gate before any
other contributor is added. Sensitivity analysis confirms it's the single
biggest knob.

### Architecture options that close the budget

| Option | Description | 3-σ |Δt| (ms) | Status |
|---|---|---|---|
| A | Shared CO2 reservoir (one bottle, one regulator, common-mode flow) | 12.9 | FAIL |
| B | Active per-side flow modulation (sense progress, modulate valve) | **8.1** | **PASS** |
| C | Mechanical-spring primary, CO2 only unlocks the latches | 10.1 | FAIL by 0.1 ms |

**Recommendation:** adopt **option B** — active per-side flow modulation
with closed-loop sensing of stage-lock progress on each side. This is a
first-order architecture change to BRIEF decision #5 ("sequenced from a
single valve to enforce sub-10ms left/right symmetry") — replace passive
sequencing with active control.

Combined options (A+B or B+C) close the budget with much more margin and are
worth comparing once bench data tightens the input distributions.

## Ground rig requirements driven by this analysis

The symmetry budget's input distributions are conservative engineering
estimates. Each one needs to be measured on the bench before treating the
final 3-σ as authoritative. The ground deployment rig
(`test/ground/spec.md`) is the instrument that does the measurement —
each contributor maps to a specific instrumented test campaign:

- **CO2 cartridge variance** ← thermal + cartridge-batch sweep, mass-flow
  metering at the manifold output.
- **Manifold balance** ← differential pressure across each side's port
  during firing, repeated.
- **Spar friction** ← stage-lock-time recordings under temperature and
  humidity conditioning, with intentional water/ice ingress.
- **Tape-spring snap dispersion** ← per-rib unfurl-time recordings via
  high-speed video frame extraction.

Once those inputs are measured, the budget closes (or doesn't). The
current 16 ms result is a *prediction* — the rig is the *measurement*.

## Open issues / verification gates

1. **Adopt option B.** Update BRIEF architecture decision #5. Active flow
   modulation requires per-side stage-lock progress sensors — already in
   the sensor list above, but their use needs to be tighter than just
   gating completion.
2. **Lock-sensor latency.** Specify ≤ 1 ms hardware response from contact
   close to FCS register; verify on bench.
3. **Asymmetric-deploy false-positive rate.** With a 10 ms gate and the
   current input distributions, the false-positive jettison rate at deploy
   is ~25 % (option B drops it to ~1 %). Unacceptable until option B is
   in place.
4. **AAD interface independence.** Wire AAD trigger directly to cutter
   firing circuit, not through FCS. Detail in `safety/failure-modes/aad-fault.md`.
5. **Pilot abort handle reachability.** Mechanical and reach geometry under
   deployed-wing pilot configuration — closes via `cad/harness/`.
6. **Mechanical reversion.** Surfaces, cable routing, transition logic in
   `docs/04-fcs-architecture.md`.

## Reproducibility

```sh
# Nominal scenario trace + state machine validation
PYTHONPATH=. .venv/bin/python -m analysis.deployment.state_machine

# Symmetry budget Monte Carlo with sensitivity sweep
PYTHONPATH=. .venv/bin/python -m analysis.deployment.symmetry_budget

# Tests
PYTHONPATH=. .venv/bin/python -m pytest analysis/deployment/tests/ -v
```
