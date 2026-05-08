# MANTA deployment symmetry budget — Monte Carlo

## Contributors

| Contributor | σ (ms) | common-mode fraction | 3-σ contribution to Δt (ms) |
|---|---|---|---|
| CO2 cartridge fill mass / temp | 2.50 | 0.00 | 10.64 |
| Valve port differential | 0.80 | 0.00 | 3.40 |
| Manifold flow impedance match | 1.50 | 0.00 | 6.33 |
| Spar telescope friction (cold / wet) | 2.00 | 0.00 | 8.50 |
| Tape-spring snap dispersion (per-rib summed) | 1.20 | 0.00 | 5.10 |

## Combined  (50,000 trials)

  3-σ |Δt|   :  **16.25 ms**
  P99  |Δt|  :  13.94 ms
  P99.9 |Δt| :  17.98 ms
  max |Δt|   :  23.38 ms

  vs BRIEF gate (10.0 ms 3-σ): **FAIL** (margin = -6.25 ms)

## Sensitivity sweep — 3-σ of |Δt| as each contributor scales

| Contributor | scale | 3-σ |Δt| (ms) | vs nominal |
|---|---|---|---|
| CO2 cartridge fill mass / temp | 0.5× | 13.32 | -2.92 |
| CO2 cartridge fill mass / temp | 1.0× | 16.19 | -0.06 |
| CO2 cartridge fill mass / temp | 2.0× | 24.53 | +8.29 |
| Valve port differential | 0.5× | 15.91 | -0.34 |
| Valve port differential | 1.0× | 16.19 | -0.06 |
| Valve port differential | 2.0× | 17.24 | +0.99 |
| Manifold flow impedance match | 0.5× | 15.16 | -1.09 |
| Manifold flow impedance match | 1.0× | 16.15 | -0.09 |
| Manifold flow impedance match | 2.0× | 19.60 | +3.36 |
| Spar telescope friction (cold / wet) | 0.5× | 14.42 | -1.83 |
| Spar telescope friction (cold / wet) | 1.0× | 16.19 | -0.06 |
| Spar telescope friction (cold / wet) | 2.0× | 21.85 | +5.61 |
| Tape-spring snap dispersion (per-rib summed) | 0.5× | 15.72 | -0.53 |
| Tape-spring snap dispersion (per-rib summed) | 1.0× | 16.31 | +0.06 |
| Tape-spring snap dispersion (per-rib summed) | 2.0× | 18.53 | +2.28 |

## Closing the budget — alternate architectures

If the nominal architecture cannot close, what alternates do?

| Option | 3-σ |Δt| (ms) | vs gate (10.0 ms) | Status |
|---|---|---|---|
| A — shared CO2 reservoir (common-mode 0.85) | 12.86 | -2.86 | FAIL |
| B — active per-side flow modulation (all σ × 0.5) | 8.11 | +1.89 | PASS |
| C — mechanical-spring primary, CO2 unlock only | 10.12 | -0.12 | FAIL |

