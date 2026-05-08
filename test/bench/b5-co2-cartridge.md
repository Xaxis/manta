# B5 — CO2 cartridge variance characterization

**Purpose:** Measure mass-flow rate and fill mass distribution per
cartridge across the operational temperature envelope, and per cartridge
lot. The dominant contributor to the symmetry budget is CO2 cartridge
variance (10.6 ms 3-σ in
[`analysis/deployment/symmetry_budget.py`](../../analysis/deployment/symmetry_budget.py)),
modeled with σ = 2.5 ms. B5 is the measurement that confirms or
tightens that engineering estimate.

## Article

Bench fixture that holds a single 88 g CO2 cartridge against a
pressure regulator and discharge valve, with the regulator output
plumbed to a calibrated sonic-orifice flow meter. The cartridge is
fired and the regulator pressure / mass-flow trace is captured.

For per-side variance studies, two fixtures run in parallel from a
common arm signal so that the differential between two cartridges
under nominally identical conditions can be measured directly.

## Cards

| Card | Condition | Samples | What it measures |
|---|---|---|---|
| 5.1 | +20 °C nominal, single fixture | 50 | Reference fill-mass distribution |
| 5.2 | +30 °C, +20 °C, 0 °C, −10 °C, −20 °C — single fixture | 25 each | Mass-flow vs. temperature curve per cartridge |
| 5.3 | +20 °C, dual-fixture differential | 50 pairs | Per-side σ_t differential at room temp |
| 5.4 | −10 °C, dual-fixture differential | 25 pairs | Per-side σ_t at the cold operational floor |
| 5.5 | Cartridge lot-to-lot variance | 50 cartridges from each of 3 lots | Lot-level σ; informs whether lot-segregation procedure is needed |
| 5.6 | Heater-on cold environment | 25 pairs at −10 °C ambient with cartridges held at +20 °C | Heater performance — does the cartridge deliver +20 °C performance under cold ambient |

## Instrumentation

- Pre-fire mass via load cell (measures cartridge dry mass; full vs. empty difference = fill mass)
- Cartridge T (thermocouple on cartridge body, +/−0.5 °C)
- Regulator output P at 10 kHz
- Mass-flow rate via sonic-orifice meter at 10 kHz
- Total discharge time
- Ambient T, RH

## Output format

```
test/bench/data/B5-{card}/{lot_id}/{cartridge_serial}.csv
   columns: t_ms, mass_flow_g_per_s, P_regulator_bar, T_cartridge_C
```

Plus per-cartridge metadata file with fill mass, manufacturer cert,
lot id.

## Pass / fail per card

For card 5.4 — the cold-soak differential — the gate is **σ_t < 4.0 ms
3-σ between the two fixtures**. A wider distribution forces option A
(shared reservoir) to be adopted to common-mode the cartridge variance.

## Output to the analysis pipeline

The measured σ_t per condition replaces the engineering-estimate value
in `analysis/deployment/symmetry_budget.py`:

```python
CONTRIBUTORS = (
    Contributor("CO2 cartridge fill mass / temp",
                sigma_ms=<measured>,    # was 2.5; replace with B5 result
                common_mode_fraction=<0.0 for option-A=False, 0.85 for option-A=True>),
    ...
)
```

Re-run the Monte Carlo with the empirical sigma. If the budget closes
at the target architecture, the bench has done its job.

## Cost / schedule

50 + 5×25 + 50 + 25 + 150 + 25 = 425 cartridge fires. Cartridge cost
~$1 each → $425. Bench fixture build ~$8k. Climate chamber rental ~$3k
for 2 weeks. Lab time at 20 fires/day → ~3 weeks.
Total: ~$12k, ~4 weeks.
