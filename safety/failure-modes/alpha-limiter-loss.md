# Failure mode: alpha-limiter loss

**FMEA ID:** `FM-FCS-001`
**Severity:** Hazardous
**Pre-mitigation likelihood:** ~10⁻³ per flight (single-FCS, AoA-vane-only)
**Post-mitigation likelihood:** < 10⁻⁵ per flight (redundant FCS + degraded-mode α estimate)

## Why this matters more for MANTA than for a conventional aircraft

Per `BRIEF.md`:

> Stall departure on a tailless high-AR wing is unforgiving. Alpha limiter
> in the FCS is mandatory; not a feature, a structural design assumption.

The structural sizing in `analysis/struct/spar_bending.py` assumes the
alpha limiter is functional — the wing is sized to 3 g limit / 4.5 g
ultimate, on the assumption that the FCS prevents the wing from being
operated at α > α_limit even under gust + pilot-CG-perturbation
disturbances. **Loss of the limiter does NOT immediately lose the
vehicle**, but it removes the protection that prevents the wing from
being kicked into an unsizable load case.

The pilot-CG-perturbation analysis in
[`analysis/flightdynamics/longitudinal.py`](../../analysis/flightdynamics/longitudinal.py)
makes this concrete: a 50 mm aft head/torso shift produces a 3.26 % MAC
vehicle CG shift, comparable to the entire 5.4 % MAC static margin.
Without active envelope protection, a worst-case posture transient at
high CL can put the wing into negative static margin and stall departure.

## Causes

| Cause | Effect on limiter | Detection |
|---|---|---|
| AoA vane mechanically stuck or iced | Limiter sees fixed α; clamps when vehicle isn't actually high-α (or vice versa) | EKF cross-check vs. body-rate-derived α; vane heater current monitoring |
| AoA vane disconnected / cabling fault | Limiter falls to degraded-mode α estimate | Vane signal validity flag |
| Pitot-static failure (both channels) | Degraded-mode estimate also unreliable (depends on V) | Pitot health flags + EKF-V-vs-GNSS-V cross-check |
| FCS-A fault | Limiter handed off to FCS-B; ~100 ms transition | FCS health monitor |
| FCS-A AND FCS-B both fault | Limiter unavailable; mechanical reversion has no envelope protection | Health monitor → mechanical-reversion mode |
| Software bug | Limiter logic doesn't engage when it should | Pre-flight built-in test + ground-rig validation |

## Detection chain

The limiter has *three* layers of monitoring, in order of immediacy:

1. **AoA vane validity flag** (per FCS, EKF output): set false when the
   vane is mechanically stuck, frozen, or signaling outside its
   physical range. Triggers degraded-mode α estimate.
2. **FCS health monitor**: cross-checks limiter outputs between FCS-A
   and FCS-B. Disagreement on `alpha_cmd_deg` by more than 1° at the
   same input → fault flag → swap to healthy unit (or both if both
   show fault).
3. **Body-rate plausibility**: if the EKF observes pitch rate q
   inconsistent with the commanded α (e.g. limiter says "saturated at 9°"
   but the vehicle is pitching up rapidly, suggesting α is rising past
   the limit), the FCS flags limiter-ineffective.

## Response

### Limiter degraded but still functional

- Layer 1 trip (AoA vane fault): switch to degraded-mode α estimate
  (trim equation: α = α₀ + (1/CL_α)·(m·g)/(½·ρ·V²·S)). Widen the limit
  margin by 1.5° (per [`fcs/envelope_protection/alpha_limiter.py`](../../fcs/envelope_protection/alpha_limiter.py)).
  Inform pilot via cockpit alert.

- Layer 2 trip (FCS-A vs FCS-B disagreement): swap to the healthy unit;
  pilot informed; flight may continue at reduced authority near stall
  if the degraded-mode estimate is also active.

### Limiter fully unavailable

- Layer 3 trip OR both FCS units faulted simultaneously:
  - Engage **mechanical reversion**. Pilot has direct cable to the
    flaperons; *no* envelope protection.
  - Pilot procedure: **Jettison + reserve** at any altitude > 200 m AGL
    where jettison is still safe, OR continue the glide if stable trim
    can be maintained manually and altitude / terrain permit a landing
    under canopy without recurring instability.
  - Do NOT attempt aerobatic maneuvers, slow flight, or aggressive
    maneuvering in this mode.

The procedure favors jettison + reserve because:
- The wing's natural instability with full pilot CG perturbations
  exceeds typical pilot bandwidth at high CL.
- A flapping reserve canopy is a far more forgiving descent vehicle
  than a tailless flying wing without envelope protection.

## Mitigation

| Layer | Mitigation |
|---|---|
| Hardware redundancy | Two FCS units (independent power rails, independent IMUs, independent cabling); aux IMU on FCS-B; cross-checked EKFs |
| Sensor redundancy | AoA vane → degraded-mode estimate from trim equation when vane fails. The trim equation only needs V and m·g; widely-redundant inputs |
| Software | Limiter is testable in isolation ([`fcs/envelope_protection/tests/`](../../fcs/envelope_protection/tests/)). 7 unit tests cover saturation, degraded mode, low-V tightening, never-exceeds-stall. Adding scenarios is cheap |
| Pre-flight | Built-in test exercises the limiter saturation path with an injected pilot α command at the limit; observed `saturated` flag must agree on FCS-A and FCS-B |
| Pilot training | Mechanical-reversion procedure rehearsed in tow article (`test/tow/`) before manned-deploy flights |

## Residual risk

After mitigation, the catastrophic-loss-of-limiter scenarios that remain:

- **Common-mode software bug** that affects both FCS units identically.
  Mitigated by code review, scenario-based testing, and PX4/ArduPilot
  software-quality inheritance from the upstream project. Residual
  ~ 10⁻⁵ per flight.
- **Common-mode environmental fault** (lightning strike, mass EMI event)
  that takes both FCS units. Mitigated by DO-160 EMI testing and
  separated power rails. Residual ~ 10⁻⁶ per flight.
- **Mechanical reversion AND structural overload simultaneously** —
  i.e. the limiter loss and a pilot-CG transient both occur in the same
  flight before pilot can jettison. Residual ~ 10⁻⁷ per flight.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Limiter unit tests | Logic is correct in isolation | [`fcs/envelope_protection/tests/`](../../fcs/envelope_protection/tests/) (7 tests passing) |
| Closed-loop scenarios | Limiter saturates correctly under pilot over-command | [`fcs/sim/closed_loop.py`](../../fcs/sim/closed_loop.py) |
| Sensor-fault SITL | Degraded-mode estimate behavior in PX4 SITL across V envelope | TBD — PX4 SITL once it exists |
| Built-in test | Pre-flight verification that limiter saturates as expected | TBD — bench HIL fixture |
| Tow article validation | Limiter saturates when commanded into stall on a real wing | [`test/tow/`](../../test/tow/) |
| Mechanical reversion rehearsal | Pilot procedure for limiter-loss case practiced before manned-deploy | Tow article + manned-tow training |

## Open issues

- The body-rate plausibility check (layer 3 detection) is described
  here but not yet specified at the algorithmic level. Define the
  threshold (e.g. q > X rad/s while limiter reports saturated) once
  6DOF SITL is in place.
- Cooper-Harper rating in mechanical-reversion mode is unmeasured.
  Tow-article test will provide the first data; if HQR is worse than
  Level 4, mechanical reversion becomes a "jettison-only" mode rather
  than a "potentially fly-able" mode.
- Single-event-upset (SEU) hardening of the FCS firmware against
  cosmic-ray bit flips: skydiving-altitude operation doesn't have a
  cosmic-ray issue, but it's worth noting the issue exists for any
  high-altitude deployment scenario the program might extend to.
