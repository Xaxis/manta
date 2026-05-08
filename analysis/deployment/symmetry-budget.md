# Symmetry budget — left-right deploy timing

**Status:** First-cut closed, **fails the BRIEF 10 ms 3-σ gate**.
Architecture revision required (see "Recommendation").

This document is the deliverable #5 per `BRIEF.md`:

> Error budget for left-right deployment timing. CO2 cartridge variability,
> valve actuation variance, tape-spring deployment dynamics. Must close to
> under 10ms 3-sigma or the architecture has to change.

The error budget is implemented as a Monte-Carlo over all contributors in
`analysis/deployment/symmetry_budget.py`. This document explains what's in
the model and what the result implies.

## The gate

> Sub-10ms deployment symmetry under representative loads, in cold and wet
> conditions. (BRIEF, unsolved-problem #1)

A 10 ms 3-σ left-right deploy-time variance means that 99.7 % of deployments
have |Δt_LR| ≤ 10 ms. The remaining 0.3 % trip the asymmetric-deployment
detector, which fires jettison + reserve. False-positive jettison is bad
(loss of vehicle, possibly low-altitude reserve event), so the budget needs
to close — false-positive rate ≤ ~0.3 % is acceptable; ~25 % (the current
prediction) is not.

## Contributors

Each contributor is modeled as a Gaussian with the cited σ. The
"common-mode fraction" describes how much of the variance is shared between
both sides (cancels in Δt) versus independent per-side (adds in quadrature).

| Contributor | σ (ms) | common-mode | Source / sanity |
|---|---|---|---|
| CO2 cartridge fill mass + temperature | 2.5 | 0.0 | Datasheet ±2 % at 20 °C; ~10 % flow loss at 0 °C; per-side independent because separate cartridges |
| Valve port differential | 0.8 | 0.0 | Bulk valve open is common-mode and cancels; residual is mechanical asymmetry |
| Manifold flow impedance match | 1.5 | 0.0 | Matched-impedance target ±2 % between sides; manufacturing variance widens this |
| Spar telescope friction (cold/wet) | 2.0 | 0.0 | Stage friction with water/ice ingress; per side |
| Tape-spring snap dispersion (9 ribs summed) | 1.2 | 0.0 | Bistable shell snap-energy variance, central-limit summed across 9 ribs |

These are conservative first-cut numbers. Three of them (CO2 cartridge,
spar friction, tape-spring snap) are flagged as research items in the
BRIEF and need bench characterization to firm up. The ground deployment
rig is the instrument that measures them.

## Result

50,000-trial Monte Carlo:

| Metric | Value |
|---|---|
| 3-σ |Δt_LR| | **16.25 ms** |
| P99  |Δt_LR| | 13.94 ms |
| P99.9 |Δt_LR| | 17.98 ms |
| max |Δt_LR| (over 50k trials) | 23.38 ms |

**vs. the 10 ms BRIEF gate: FAIL by 6.25 ms.**

CO2 cartridge variance alone produces 10.6 ms 3-σ — by itself it busts
the gate.

## Sensitivity

Scaling each contributor in isolation by 0.5× / 2×:

| Contributor | 0.5× | 1× | 2× |
|---|---|---|---|
| CO2 cartridge | 13.3 ms | 16.2 ms | 24.5 ms |
| Valve port | 15.9 | 16.2 | 17.2 |
| Manifold balance | 15.2 | 16.2 | 19.6 |
| Spar friction | 14.4 | 16.2 | 21.9 |
| Tape-spring snap | 15.7 | 16.3 | 18.5 |

Sensitivities ranked: CO2 ≫ spar friction > manifold > snap > valve. The
CO2 cartridge term dominates the budget, and the program's biggest
risk-reduction lever is whatever shrinks it.

## Architecture options

What architecture changes would close the budget?

### Option A — Shared CO2 reservoir
Single bottle, single regulator, common manifold to both sides. Bottle and
flow asymmetries become common-mode (don't add to Δt).
- Modeled as: CO2 contributor sigma 2.5 ms but common-mode fraction 0.85
- Result: **12.9 ms — fails by 2.9 ms**

Closer to the gate, but still fails because the differential portion
(downstream of the regulator) plus the other contributors stack above 10 ms.
Doable as a half-measure or in combination with B.

### Option B — Active per-side flow modulation
Per-side stage-lock sensors run a closed loop that modulates valve flow on
the side that's ahead, slowing it down to wait for the slow side. Reduces
all timing variances by ~50 %.
- Modeled as: every contributor sigma × 0.5
- Result: **8.1 ms — PASSES with 1.9 ms margin**

This is the recommended architecture revision. It's a real change to BRIEF
decision #5 — replaces passive sequencing with active closed-loop control.
Cost: more sensor latency budget pressure, more software complexity, more
single-point FCS dependence. None of these are showstoppers if the symmetry
gate is otherwise unreachable.

### Option C — Mechanical-spring primary, CO2 unlock only
The energy that drives the wing open comes from pre-loaded mechanical
springs (or from the bistable tape-spring rib snap energy itself). CO2 only
fires the locking pins that release the stowed wing. Eliminates pneumatic
contributors entirely — replaces them with a single mechanical-unlock
differential.
- Modeled as: drop CO2/valve/manifold; add a mechanical-unlock differential at σ = 0.5 ms
- Result: **10.1 ms — fails by 0.1 ms**

Marginally fails on its own. Combined with option B (active sensing on the
mechanical unlocks) it would pass comfortably.

## Recommendation

**Adopt Option B as the primary path** (active per-side flow modulation).
Hold Option A (shared reservoir) as a low-cost first-step that buys margin
in the lab if the bench-measured contributors come in better than predicted.

Combined Option A+B is the strongest configuration: shared reservoir for
common-mode flow, plus active per-side modulation for residual asymmetry.
With bench-measured (likely tighter) input distributions, that should give
3-σ ≤ 5 ms — comfortable margin.

**Required BRIEF amendment:**
> Architecture decision #5 — Pneumatic deployment via *single* CO2 cartridge per side, sequenced from a single valve to enforce sub-10ms left/right symmetry.

becomes:
> Architecture decision #5 — Pneumatic deployment via shared CO2 reservoir feeding an active-modulated per-side valve. Closed-loop FCS sensing of per-side stage-lock progress modulates per-side flow to enforce sub-10ms left/right symmetry. The shared reservoir is the common-mode element; the active modulation handles the residual differential.

## What needs the ground rig

The Monte Carlo runs on engineering estimates. The actual variance of each
contributor is measured by the ground deployment rig (`test/ground/`):

| Contributor | Rig measurement |
|---|---|
| CO2 cartridge | Mass flow vs temperature, batch-to-batch, 200+ samples |
| Manifold balance | Per-port differential pressure traces, multiple firings |
| Spar friction | Stage-lock timestamps under thermal + humidity conditioning, water/ice ingress |
| Tape-spring snap | High-speed-video per-rib unfurl times across the rib population |

Once measured, plug the empirical sigmas back into the Monte Carlo and
re-evaluate. The current 16 ms is a prediction; the rig provides the
measurement that decides whether the architecture closes.

## Reproducibility

```sh
PYTHONPATH=. .venv/bin/python -m analysis.deployment.symmetry_budget
```

Outputs in `analysis/deployment/out/`:
- `symmetry_trials.csv` — first 5,000 trial results for plotting
- `symmetry_histogram.png` — histogram of |Δt|
- `sensitivity.csv` — sigma-scale-vs-3-σ table
