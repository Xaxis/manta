# B1 — Pyrotechnic cutter coupon test

**Purpose:** Quantify pyrotechnic cutter behavior on a representative
spar root coupon BEFORE any cutter is integrated into a flight
article. Closes the no-fire / all-fire / EMI gates from
`safety/failure-modes/cutter-no-fire.md` and `cutter-inadvertent-fire.md`
on bench data, not just vendor cert.

## Article

A short length of CFRP tube matching the front-spar root OD (73 mm OD,
2.5 mm wall, sized config) bonded into an aluminum cup of the
production fitting geometry. The cutter assembly wraps the tube just
outboard of the bond region, exactly as in the flight configuration.

Initial article quantity: 4 (one per shipset of cutters tested), with
provision for replacements.

## Cards

| Card | Condition | Samples | Pass criteria |
|---|---|---|---|
| 1.1 | No-fire margin: 1 W / 1 A applied to initiator for 5 minutes; verify no fire | 10 cutters | 10 / 10 no-fire |
| 1.2 | All-fire margin: 3.5 A pulse, 50 ms duration; verify fire within spec | 10 cutters | 10 / 10 fire within timing window |
| 1.3 | EMI immunity per RTCA DO-160 G H — 200 V/m HIRF, lightning Cat A direct + Cat B indirect | 4 cutters | No fire under any condition |
| 1.4 | ESD immunity — 5 kV, 500 pF model on initiator pins | 4 cutters | No fire |
| 1.5 | Cold soak fire — cutter conditioned to −20 °C, then fire at the cold-soak temp | 4 cutters | Fire within timing spec; clean cut |
| 1.6 | Hot soak fire — +50 °C | 4 cutters | Same |
| 1.7 | Cut quality on a representative bonded spar root | 8 cutters | Severance complete; no spar-end fiber wisps; ferrule retains cut end |
| 1.8 | Dual-initiator independence | 4 cutters | Single initiator fires alone successfully (no degradation) |
| 1.9 | Vibration survival per DO-160 Cat S | 4 cutters | No degradation; no inadvertent fire under vibration |

## Instrumentation

- Cutter firing-circuit voltage and current at 100 kHz sample rate
- High-speed video at ≥ 1000 fps of the cutter and the spar end
- Thermocouple on the cutter housing
- Force transducer on the spar (verifies a complete cut releases the
  spar without residual mechanical resistance)

## Output format

```
test/bench/data/B1-{card}/{cutter_serial}_{condition}.csv
   columns: t_us, V_firing, I_firing, force_N, T_housing_C
```

Plus video files referenced by cycle id.

## Pass / fail per card

A failure on any card stops B1 progression and triggers a failure-
investigation write-up. Card 1.7 (cut quality) is the binding card for
flight integration — without clean cut quality demonstration the
cutter cannot be integrated into a flight article.

## Cost / schedule

Cutter coupons + spars: ~$2k each shipset of 10 cutters → ~$8k for
the full B1 campaign.
EMI lab access: ~$5k for 1 day; cards 1.3 + 1.4 batched into one day.
Cold/hot soak: lab climate chamber, 1 day.
Total: ~$15k materials + lab fees, ~3 weeks calendar time.
