# Bench characterization tests

These run BEFORE the ground deployment rig (`test/ground/`) and feed
the analyses with real input distributions instead of engineering
estimates.

Each bench article is self-contained: it characterizes one component
or one phenomenon, the data it produces gets plugged back into the
analysis it supports, and the analysis re-runs to update its
predictions.

## Articles

| # | Article | Characterizes | Feeds | Spec |
|---|---|---|---|---|
| B1 | Pyrotechnic cutter coupon | LSC + initiator no-fire/all-fire margins, EMI immunity, cut quality | `safety/failure-modes/cutter-no-fire.md`, `safety/failure-modes/cutter-inadvertent-fire.md` | [`b1-cutter.md`](b1-cutter.md) |
| B2 | Drogue ringslot | Inflation profile, bridle snatch peak, dynamic amplification, body-axis stability | `analysis/deployment/drogue_dynamics.py` | [`b2-drogue.md`](b2-drogue.md) |
| B3 | Tape-spring rib snap | Per-rib unfurl time + force trace, dispersion across temperature | `analysis/deployment/symmetry_budget.py` (CONTRIBUTORS row 4), `cad/ribs/build.py` | [`b3-rib-snap.md`](b3-rib-snap.md) |
| B4 | Spar telescope friction | Stage friction vs. T, RH, and water/ice ingress; lock latency distribution | `analysis/deployment/symmetry_budget.py` (row 3), `safety/failure-modes/joint-water-ice-ingress.md` | [`b4-spar-friction.md`](b4-spar-friction.md) |
| B5 | CO2 cartridge variance | Mass-flow vs. T per cartridge lot, distribution of fill mass, regulator behavior | `analysis/deployment/symmetry_budget.py` (row 0), `safety/failure-modes/co2-cold-underpressure.md` | [`b5-co2-cartridge.md`](b5-co2-cartridge.md) |
| B6 | Microswitch latency | Hardware close-to-GPIO-read latency per switch type | `docs/03-deployment-sequence.md` (≤ 1 ms requirement) | [`b6-microswitch-latency.md`](b6-microswitch-latency.md) |
| B7 | Skin tension + bond | DCF skin tear strength, rib-bond peel strength after thermal cycling | `analysis/struct/components.py` skin model | [`b7-skin-bond.md`](b7-skin-bond.md) |
| B8 | Spar root joint | Bonded fitting + cutter interface under combined bending + cutter load | `analysis/struct/spar_bending.py` (root joint FEA gate) | [`b8-root-joint.md`](b8-root-joint.md) |

## Per-article spec template

Each `b*.md` follows this structure:

1. **Purpose** — what this characterizes and why
2. **Article** — physical fixture description; what's representative,
   what's simplified vs. flight hardware
3. **Test cards** — per-condition runs with sample sizes and
   environmental brackets
4. **Instrumentation** — sensors, sample rates, acceptance criteria
5. **Output format** — CSV / HDF5 schema for the data the article
   produces; how it gets plugged back into the analysis
6. **Pass / fail per card** — what closes a card vs. what triggers
   a failure-investigation
7. **Cost / schedule estimate** — to inform program planning

## Why bench-before-rig

The ground deployment rig (`test/ground/`) uses a complete wing
assembly. That's expensive and slow. Bench articles are cheap and
fast, and they de-risk the rig:

- The rig assumes microswitch latency ≤ 1 ms. If B6 shows the
  selected switch is 3 ms, the architecture changes BEFORE the rig
  is built.
- The rig's symmetry-budget closure depends on the input
  distributions in `analysis/deployment/symmetry_budget.py`. If B3,
  B4, B5 show distributions WIDER than the engineering estimates,
  the rig may fail its 200-cycle gate. Better to know on the bench
  than after a 6-month rig build.
- The rig's first cutter live-fire is the FIRST cutter event in the
  program. B1 confirms cutter behavior on a coupon before any
  cutter is integrated into a wing assembly.

## Schedule

Bench articles are mostly independent of one another and can run in
parallel. Realistic timeline:

| Quarter | Bench articles |
|---|---|
| Q1 | B5 (CO2 cartridge), B6 (microswitch latency) — both fast, low-cost, high-value |
| Q1 | B1 (cutter coupon) — vendor-led, runs in parallel |
| Q2 | B3 (rib snap), B4 (spar friction) — produce the symmetry-budget inputs |
| Q2 | B2 (drogue) — needs a small drop tower or a wind tunnel |
| Q3 | B7 (skin bond), B8 (root joint) — structurally bigger, slower |

By end of Q3 the inputs to the symmetry-budget Monte Carlo have been
measured; the architecture-revision recommendations from
`analysis/deployment/symmetry-budget.md` either close cleanly or
require the bigger architecture change.

## Output retention

All bench data is retained per the same protocol as ground-rig data
(`test/ground/spec.md`): one HDF5 file per cycle / per condition,
indexed by article, condition, and timestamp. Failure-investigation
write-ups under each article's `failure-investigations/` subdirectory.
