# 04 — FCS Architecture

**Status:** First-cut closed. Limiter prototype, longitudinal dynamics
analysis, and closed-loop characterization are runnable. Outstanding work
is the lateral-directional axis and the actual hardware/firmware build
(PX4 fork or ArduPilot equivalent).

## Locked from BRIEF

- Redundant Pixhawk-class FCS.
- EKF at 400 Hz.
- MEMS IMUs.
- Flaperons (servo-driven, brushless waterproof) on the outer trailing
  edge each side.
- **Alpha limiter is a structural design assumption** — not a feature, an
  invariant the structural sizing depends on.
- Mechanical reversion to direct cable as last-resort backup.

## Why the alpha limiter is load-bearing

Two findings from the upstream analyses converge here:

1. **Static margin is tight.** At design CL the wing has 5.4 % MAC margin
   (top-of-BRIEF-range washout). At higher CL (slow flight, near stall)
   it drops below 4 %. ([`docs/02`](02-structural-budget.md), [`analysis/aero/trim/`](../analysis/aero/trim/))

2. **Pilot motion eats most of the margin.** The pilot is 79 % of the
   vehicle mass and not rigid. A 50 mm head/torso CG shift produces a
   ±3.26 % MAC vehicle-CG shift — comparable to the entire static margin.
   At the trim CG, short-period damping ζ = 0.97 (heavily damped). At a
   50 mm aft pilot shift, the wing goes neutrally stable; at larger
   shifts, divergent. ([`analysis/flightdynamics/longitudinal.py`](../analysis/flightdynamics/longitudinal.py))

The alpha limiter, combined with body-rate damping in the inner loop, is
the only thing that prevents departure during a worst-case posture
perturbation at high CL. Loss of the limiter is a **hazardous failure
mode**, listed in the FMEA.

## Topology

```
                                     ┌──── pitot/static (×2) ────┐
                                     │                            │
                                     │     AoA vane (×1)          │
                                     │     IMU triplex            │
                                     │     spar-lock GPIO (×6)    │
                                     │     skin-tension cells (×4)│
                                     │     drogue load cell       │
                                     │                            ▼
                                     │                       ┌─────────────┐
                                     │                       │ EKF (400Hz) │
                                     │                       └─────┬───────┘
   pilot stick / abort handle        │                              │
   AAD trigger (FCS-bypassed)        │                              ▼
            │                        │      ┌─────────────────────────────────┐
            │                        ├─────►│ Outer loop: trim, glide, alpha  │
            │                        │      │  hold + ALPHA LIMITER           │
            │                        │      └─────────────────┬────────────────┘
            │                        │                         ▼
            │                        │        ┌──────────────────────────────┐
            │                        ├───────►│ Inner loop: pitch-rate,      │
            │                        │        │  roll-rate, yaw-rate damping │
            │                        │        └─────────────┬────────────────┘
            ▼                        │                       ▼
     ┌──────────────────────────┐    │             ┌────────────────────┐
     │ Deployment state machine │────┘             │ Mixer + servo cmd  │
     │ + jettison interface     │                  └─────────┬──────────┘
     └──────────────────────────┘                            ▼
                                                  ┌──────────────────────┐
                                                  │ Flaperon servos (×4) │
                                                  └──────────────────────┘
```

## Hardware redundancy

- **Two Pixhawk-class FCS units** (FCS-A and FCS-B), running cross-checked
  EKFs. Either can fly the vehicle alone; swap is automatic on health failure.
- **Independent power rails** for FCS-A and FCS-B; a third independent rail
  for the mechanical-reversion logic + cutter firing circuit so jettison
  remains possible if both FCS units lose power.
- **Standalone aux IMU** wired to FCS-B (and physically separated from the
  FCS-A IMUs) — provides a third attitude solution for cross-check.
- **AAD signal** wired directly to the cutter firing circuit, bypassing
  both FCS units. (See `safety/failure-modes/aad-fault.md`.)
- **Mechanical reversion**: direct cable from a stick-style pitch input to
  the flaperons. Operable with FCS fully unpowered; pilot reach geometry
  verified in `cad/harness/`.

## Sensor suite

| Sensor | Count | Purpose | Notes |
|---|---|---|---|
| 9-DoF IMU | 3 (FCS-A, FCS-B, aux) | attitude, body rates, accel | Cross-checked via EKF |
| Magnetometer | 2 | heading | Magnetic deviations from skydiving rig metalwork need calibration on every aircraft |
| GNSS | 2 | position, ground speed | Degraded-mode ground speed if pitot fails |
| Pitot-static | 2 | airspeed, altitude rate | Boom location TBD; must avoid wing wake during deploy |
| AoA vane | 1 | direct α | Single-channel; loss → degraded-mode α estimate from trim equation |
| Spar-lock micro | 6 | deployment confirmation | ≤ 1 ms latency requirement (see `docs/03`) |
| Skin tension load cells | 4 | deploy diagnostic | Advisory only — does not gate deployment |
| Drogue load cell | 1 | deployment gate | Cross-checked vs. accel signature |

## Control laws

### Outer loop — alpha limiter (the central piece)

Implementation: [`fcs/envelope_protection/alpha_limiter.py`](../fcs/envelope_protection/alpha_limiter.py).

Limit schedule:

| V (m/s) | α_limit (°) | α_limit, sensor degraded (°) |
|---|---|---|
| 10 | 7.86 | 6.36 |
| 14 | 9.00 | 7.50 |
| 16 (V_bg) | 9.00 | 7.50 |
| 20 (cruise) | 9.00 | 7.50 |
| 25 | 9.00 | 7.50 |
| 30 | 9.00 | 7.50 |
| 35 | 9.50 | 8.00 |
| 40 | 10.00 | 8.50 |

Stall α is 11.5° (analytic polar). Margin 2.5° in the cruise band, +1.5° additional in degraded mode (sensor failure).

Behavior:
- Pilot stick → α_pilot_cmd
- α_cmd = min(α_pilot_cmd, α_limit(V, sensor_state))
- saturated flag → inner loop integrator anti-windup
- degraded flag → α estimate from trim equation: α = α₀ + (1/CL_α) · (m·g)/(½·ρ·V²·S)

Validated by 7 unit tests in [`fcs/envelope_protection/tests/`](../fcs/envelope_protection/tests/).

### Inner loop — pitch / roll / yaw rate damping

PI on α error + Kq on pitch rate (longitudinal). Same structure for the
lateral axes. Anti-windup hooks via the α-saturated flag from the outer
loop. Tunings are placeholders pending high-fidelity 6DOF in SITL.

### Trim hold

Slow (~1 Hz) outer loop adjusts α_cmd to maintain a target airspeed. Pilot
override available; otherwise the FCS holds best-glide trim by default.

## Closed-loop characterization

Three scenarios in [`fcs/sim/closed_loop.py`](../fcs/sim/closed_loop.py):

| Scenario | What it shows |
|---|---|
| `pilot_overcmd` | Pilot demands α = 12° at cruise. Limiter clamps at α_limit ≈ 9° for 83 % of the run; anti-windup holds; α is held at the model's hard ceiling, not exceeded. |
| `cg_shift_50mm` | 50 mm aft CG shift (worst-credible posture change). With no other disturbance the trim point holds — the dynamics rebuild against the shifted A matrix do not destabilize. |
| `combined_overcmd_cg` | Both at once. Limiter still saturates correctly. |

**Caveats made explicit in the simulator docstring:** the plant is a
linearized state-space about trim. Outside ±20 % from trim it does not
represent the real wing. **Gust ingestion is deferred to a non-linear 6DOF
simulator** that's out of scope for first-cut. The closed-loop sim is a
limiter-logic characterization tool, not a flight-validation simulator.

## Sensor-fault modes

| Failed sensor | FCS response | Vehicle behavior |
|---|---|---|
| AoA vane | Degraded-mode α estimate from trim equation; α-limit margin widens by 1.5° | Flyable; pilot informed; reduced authority near stall |
| One pitot-static channel | Use the other; cross-check against GNSS ground speed | No degradation if both pitots agree pre-failure |
| Both pitot-static channels | Use GNSS-derived ground speed as airspeed proxy (degraded — wind error); widen α-limit margin further | Flyable in calm air; risky in gusty conditions; recommend reserve |
| One IMU | EKF flags + voting drops bad channel | No degradation |
| Two IMUs | Single remaining IMU is authoritative | Substantially degraded; consider deploy-completed reserve |
| Spar-lock sensor | If pre-deploy: jettison if any-lock-fail timeout. If post-deploy: advisory only | See `docs/03` |
| FCS-A failure | FCS-B takes over; transition transient < 100 ms | Functional with full authority |
| FCS-A + FCS-B failure | Mechanical reversion (manual cable to flaperons) | Substantially degraded; pilot must hand-fly with no envelope protection — α-limiter loss is a **hazardous** mode (FMEA `FM-FCS-001`) |

## Mechanical reversion

- Direct mechanical linkage from pilot stick to a single set of flaperon
  surfaces (the *outer* flaperon segment per side; inner segment locked
  out in reversion mode for simplicity).
- Engaged automatically when both FCS units flag fault, or manually from
  a guarded cockpit switch.
- Pilot has no envelope protection in this mode. Recovery procedure is
  immediate jettison + reserve at any altitude > 200 m AGL where
  jettison is still safe.

## To-build (firmware)

The architecture above is implementable on PX4 or ArduPilot. Both have
existing infrastructure for IMUs, EKF, SITL, mixers, and servos. Choice
of which to fork is deferred to firmware-team selection; the limiter and
state-machine modules in [`fcs/`](../fcs/) are framework-agnostic Python
prototypes that translate cleanly to either.

## Lateral-directional dynamics

Implementation: [`analysis/flightdynamics/lateral.py`](../analysis/flightdynamics/lateral.py).

First-cut estimates of the stability derivatives (Roskam Vol. I tables for
swept tailless wing, no vertical tail) feed a 4-state lateral state-space
[β, p, r, φ]. Headline findings across the operational speed range:

| Mode | V = 16 m/s (V_bg) | V = 20 m/s (cruise) | V = 25 m/s | Verdict |
|---|---|---|---|---|
| Dutch roll | ω_n = 2.40, ζ = +0.29 | ω_n = 2.35, ζ = +0.26 | ω_n = 2.55, ζ = +0.27 | **Level 2** (acceptable but light) |
| Roll mode | τ = 0.064 s | τ = 0.049 s | τ = 0.039 s | crisp |
| Spiral mode | **DIVERGENT, T₂ = 15 s** | **DIVERGENT, T₂ = 80 s** | convergent, τ = 80 s | unstable below cruise |

Two consequences for the FCS architecture:

1. **Yaw damper required.** Dutch roll at 0.26 ζ is acceptable for a
   gliding aircraft but right at the threshold of pilot-irritating;
   handling qualities improve markedly above 0.40. The FCS adds an
   artificial Cn_r contribution by feeding sensed yaw rate into
   differential flaperon (right-flaperon-down + left-flaperon-up
   produces a yaw moment via wing drag asymmetry). Tuning is a
   placeholder until 6DOF SITL is in place.

2. **Spiral mode autopilot or persistent pilot attention.** At V_bg the
   spiral doubles in 15 s — pilot can easily fly through this but
   cannot leave the controls. The FCS should provide bank-hold
   functionality (close a φ loop with the ailerons) for normal
   cruise. Loss of this is **major** (Cooper-Harper drift), not
   hazardous — pilot can still fly.

These are first-cut numbers — Cn_β and Cn_r in particular for a tailless
wing are sensitive to fuselage / harness / pilot-body shadowing of the
sweep contribution, which is exactly the kind of effect a vortex-lattice
code (AVL) captures and our hand-calc does not. Validate with AVL before
treating as load-bearing.

## Open issues

1. **AVL validation of lateral derivatives.** Cn_β especially — if AVL
   reports a smaller value than our 0.020 placeholder, dutch-roll damping
   could drop into Level 3 territory and the yaw damper becomes
   *mandatory* not *recommended*.
2. **Servo selection.** Need a waterproof brushless servo with
   ≥ 100 deg/s slew, ≥ 5 N·m holding torque, and < 5 ms control-input
   latency. Vendor survey pending.
3. **Pilot input device.** Yoke? Body-tilt sensor? Stick? Constrained by
   the deployed-wing pilot ergonomics. Owns `cad/harness/`.
4. **Anti-windup tuning.** The current PI inner loop is illustrative; real
   gains come from high-fidelity 6DOF simulation in PX4 SITL.
5. **EKF tuning.** Sensor-fault transition behavior, particularly the
   pitot/GNSS handoff during gusts, needs simulation work.
6. **Hardware-in-the-loop test fixture.** Should sit alongside the ground
   deployment rig (`test/ground/`) so the FCS exercises real spar-lock
   GPIO and real cutter-firing circuits during integration.

## Reproducibility

```sh
PYTHONPATH=. .venv/bin/python -m analysis.flightdynamics.longitudinal
PYTHONPATH=. .venv/bin/python -m fcs.envelope_protection.alpha_limiter
PYTHONPATH=. .venv/bin/python -m fcs.sim.closed_loop
PYTHONPATH=. .venv/bin/python -m pytest fcs/ -v
```
