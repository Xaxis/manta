# Failure mode: sensor dropout

**FMEA ID:** `FM-SEN-001` (umbrella; per-channel sub-IDs in parentheses below)
**Severity:** Varies by channel — see table
**Pre-mitigation likelihood:** ~10⁻³ per channel per flight (no redundancy)
**Post-mitigation likelihood:** < 10⁻⁵ per flight (overall vehicle outcome)

## What it means

Any sensor channel reports invalid data — either silent (no signal),
stuck (frozen value), out-of-range, or noisy beyond filter rejection.
The FCS detects via cross-check and either downgrades to a backup
channel or enters degraded-mode operation.

## Per-channel response matrix

| Sub-ID | Channel | Severity | Detection | Mitigation | Residual |
|---|---|---|---|---|---|
| `FM-SEN-IMU-001` | One IMU (of 3) | Minor | EKF residual + cross-check between FCS-A IMU, FCS-B IMU, aux IMU | Voting drops the bad IMU; 2-of-3 still active | < 10⁻⁵ per flight |
| `FM-SEN-IMU-002` | Two IMUs simultaneously | Hazardous | Same — two channels disagree with the third | Single remaining IMU is authoritative; Cooper-Harper degrades but flyable; recommend reserve at next safe altitude | < 10⁻⁷ per flight (correlated 2-of-3 failure) |
| `FM-SEN-IMU-003` | All three IMUs | Catastrophic | EKF cannot resolve; FCS faults | Mechanical reversion (no envelope protection); pilot procedure: jettison + reserve | < 10⁻⁸ per flight |
| `FM-SEN-PIT-001` | One pitot-static | Minor | Cross-check vs. the other pitot AND vs. GNSS ground speed | Use the healthy channel; mild EKF re-tune | < 10⁻⁴ per flight |
| `FM-SEN-PIT-002` | Both pitot-static | Hazardous (especially in gusty conditions) | Both channels disagree with GNSS-derived ground speed by > threshold | Use GNSS ground speed as airspeed proxy (degraded — wind error); widen alpha-limit margin further; cockpit advisory | < 10⁻⁵ per flight |
| `FM-SEN-AOA-001` | AoA vane | Major (degraded alpha-limiter authority) | Vane signal validity flag; cross-check against EKF body-rate-derived alpha estimate | Degraded-mode alpha estimate from trim equation; alpha-limit margin widened by 1.5° (per `fcs/envelope_protection/alpha_limiter.py`) | < 10⁻³ per flight |
| `FM-SEN-LCK-001` | One spar-lock microswitch | Catastrophic *only* during deploy gate evaluation | Within-budget time-out on the affected channel; redundant dual contacts on the same switch help | Lock-fail-timeout fires jettison + reserve; pilot proceeds under canopy | < 10⁻⁵ per deploy |
| `FM-SEN-LCK-002` | Multiple spar-lock channels | Catastrophic if the failure happens during the deploy gate window | Same — but multiple-channel timeouts are more decisive | Same response; jettison + reserve is fail-safe | < 10⁻⁶ per deploy |
| `FM-SEN-DRG-001` | Drogue load cell | Catastrophic if false-positive | Cross-check vs. accel signature and airspeed trace | If load cell silent, fall back to airspeed gate; if airspeed also fails, time-based bypass after 4 s | < 10⁻⁴ per deploy |
| `FM-SEN-MAG-001` | Magnetometer | Minor | EKF heading divergence vs. GNSS course | EKF runs without magnetometer using GNSS-derived course; brief degraded heading; not flight-critical | < 10⁻³ per flight |
| `FM-SEN-GNSS-001` | GNSS | Major (loss of altitude-rate / horizontal-position cross-check) | EKF residual + standard GNSS health flags | EKF runs without GNSS at the cost of slow position drift; fine for the duration of a glide; informs reserve-altitude alarm | < 10⁻³ per flight |

## Cross-channel correlations

Some sensor failures are **correlated** — a single fault can take
multiple channels down at once. The mitigation strategy assumes
INDEPENDENT failures; correlated failures bypass the redundancy.

| Correlated event | Channels affected | Mitigation |
|---|---|---|
| Common power-rail failure | All sensors on that rail | FCS-A, FCS-B, AAD on independent rails; rail-monitoring on each |
| EMI / lightning event | Many digital channels simultaneously | DO-160 hardening; transient suppression on every penetrating wire |
| Airframe vibration mode at sensor mounting frequency | All vibration-coupled MEMS sensors | Vibration isolation on the IMU mounts; calibration sweep across operational frequency range |
| Thermal-shock stripe | Sensors in a single thermal environment | Spatial separation of the redundant channels (FCS-A in fwd bay, FCS-B in aft bay); independent thermal management |
| Ground-strike or hard landing | Mechanically-coupled sensors | Each sensor mount sized to its channel's load case; G-load survival testing per DO-160 Cat S |

## Detection latency

The state machine and inner-loop control are time-sensitive. The EKF
runs at 400 Hz; sensor-fault flags must propagate within ≤ 10 ms of the
fault-condition becoming detectable, and the inner loop must respond
within ≤ 25 ms (10 control cycles) of the flag setting.

Fault-detection latency is a verifiable property of the FCS firmware;
target test in HIL fixture is 95 % of detected faults flag within 10 ms.

## Mitigation chain (overall)

1. **Triplex redundancy** on the most safety-critical channel (IMU).
2. **Duplex redundancy** on next-most-critical (FCS, pitot, spar-lock
   contacts on a single switch).
3. **Single-channel + degraded-mode fallback** on AoA vane, drogue
   load cell, magnetometer, GNSS.
4. **Cross-channel cross-checking** so a frozen / stuck value is
   detected as easily as a silent one.
5. **Heartbeat monitoring** on every digital sensor: missing heartbeat
   = invalid.
6. **Per-channel pre-flight self-test** that the sensor reports the
   expected value at the expected condition (e.g. AoA vane reads
   within ±1° of zero with the aircraft level on the ground).
7. **Cockpit advisories** on any sensor degraded-mode entry; pilot
   informed of which channel and what the consequence is for envelope
   protection authority.

## Residual risk

After mitigation:

- **Single-channel failures** are essentially eliminated by redundancy
  for the IMU and pitot, and well-handled by graceful degradation
  for AoA / mag / GNSS.
- **Correlated multi-channel failures** are the residual risk. Most
  credible: a vibration mode that hits all IMUs (mitigated by
  isolation testing); a power-rail fault (mitigated by independent
  rails); EMI / lightning (mitigated by DO-160 hardening — but
  unverified at MANTA-program scale).
- **Unknown sensor-fault modes** that the per-channel detection
  doesn't catch: hard to quantify. Mitigated by EKF residual
  monitoring (any sensor whose output is inconsistent with the rest
  of the state estimate gets flagged), but residual ~10⁻⁵ per flight.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Per-channel injection test | Each sensor channel forced into invalid / stuck / out-of-range states; FCS detects within latency budget | Bench HIL |
| Cross-channel disagreement test | Two sensors of the same kind disagree; voting works as designed | Bench HIL |
| Power-rail independence | Each rail can be cycled independently with no loss of redundancy on others | Bench |
| Vibration sweep | IMUs do not couple to airframe vibration modes through their mounts | Bench shaker test |
| EMI immunity | Sensor channels survive DO-160 G H HIRF + lightning transient | EMI test lab |
| End-to-end SITL with sensor faults | Full deployment scenario with each sensor injected as failed; state machine + control responds correctly | PX4 SITL |

## Open issues

- Sensor selection is still placeholder. Each per-channel reliability
  number above assumes "production-grade aerospace MEMS / load cell"
  sourcing. Vendor-specific reliability data would tighten the
  estimates.
- Vibration-isolation analysis is not yet done. Until the airframe is
  built and a representative mass model is shaken, the IMU-to-airframe
  resonance assumption is uncharacterized.
- HIRF / lightning testing for a non-cert program is expensive and
  rarely done. Compromise approach: vendor-cert components where
  possible; bench-level transient injection on the firing circuit only.
