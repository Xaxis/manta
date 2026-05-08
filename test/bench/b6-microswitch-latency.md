# B6 — Microswitch latency characterization

**Purpose:** Verify hardware close-to-GPIO-read latency on the
selected spar-lock microswitch. The deployment-sequence specification
([`docs/03-deployment-sequence.md`](../../docs/03-deployment-sequence.md))
requires ≤ 1 ms latency on every lock-sensor channel — without this,
the asymmetric-deploy detector cannot resolve the 10 ms 3-σ symmetry
budget. B6 is the one-day bench test that confirms or rejects each
candidate switch part.

## Article

A single microswitch + GPIO interface board. The switch is mounted
in a fixture that drops a calibrated impactor onto its actuator from
a known height; an accelerometer on the impactor captures the contact
event, and a separate GPIO logger captures the FCS-equivalent register
read. The two are time-aligned at 100 kHz sample rate.

This is one of the cheapest, fastest bench tests in the program. Run
it FIRST when the switch is selected, before the part gets specified
into the harness wiring.

## Cards

| Card | Switch condition | Samples | Pass criteria |
|---|---|---|---|
| 6.1 | Nominal, fresh switch | 200 | ≤ 1 ms latency on 95 % of trials |
| 6.2 | After 10,000 mechanical cycles | 100 | Same; no degradation |
| 6.3 | At −10 °C | 50 | ≤ 1 ms (some grease may stiffen at cold) |
| 6.4 | At +50 °C | 50 | Same |
| 6.5 | After exposure to spray-water and drainage | 50 | No false closures, latency unchanged |
| 6.6 | Bounce / contact chatter characterization | 100 | Bounce duration documented; firmware debounce strategy chosen |

## Instrumentation

- Impactor accelerometer (charge-mode, 100 kHz)
- GPIO logger / oscilloscope on the switch contact line (100 kHz)
- Drop fixture with adjustable impactor mass and height (energy
  matched to the actual rib unfurl impulse delivered to the lock pin)

## Output format

```
test/bench/data/B6-{card}/{switch_serial}/trial_{nnn}.csv
   columns: t_us, accel_g, gpio_state
```

Per-trial latency = (time of first GPIO transition) − (time of
accelerometer peak ≥ contact threshold).

## Pass / fail per card

A switch that fails B6 cannot be the production part. Switch
selection iterates until B6 passes, typically across 2–3 candidates.

## Output to the analysis pipeline

If the selected switch shows latency > 1 ms typical, the symmetry
budget tightens by that amount per side. The contributor in
[`analysis/deployment/symmetry_budget.py`](../../analysis/deployment/symmetry_budget.py)
is currently 0.8 ms (valve port differential) which assumes < 1 ms
sensor latency. A 3 ms switch latency adds ~3 ms 3-σ to that
contributor and pushes the budget out by ~3 ms 3-σ overall — a
significant change.

## Cost / schedule

Switches: ~$50 each for industrial-grade aerospace parts; 5 candidate
parts × 5 each = ~$1250.
Drop fixture build: ~$2k.
Lab time: 1–2 days per candidate.
Total: ~$5k, ~2 weeks.
