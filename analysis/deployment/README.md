# analysis/deployment/

Deployment dynamics. Pairs with `docs/03-deployment-sequence.md`.

## Tools

- 0-D / 1-D pneumatics for CO2 cartridge → manifold → actuator energy delivery.
- Python ODE models for tape-spring rib snap-through dynamics.
- High-speed video (frame extraction) once the ground rig is built — feeds parameter ID back into the models.

## Expected layout (to be populated)

```
deployment/
├── pneumatics/             # CO2 blowdown, manifold balance, valve opening
├── tape-spring/            # bistable shell snap-through energy + timing
├── timing/                 # composite end-to-end timeline + per-stage budgets
├── symmetry-budget.md      # deliverable #5
├── drogue/                 # ringslot reefed inflation, snatch loads
└── README.md
```

## Critical: symmetry budget (deliverable #5)

`symmetry-budget.md` quantifies left-right deployment timing variance, contributors include (non-exhaustive):

- CO2 cartridge fill mass tolerance and temperature dependence
- Valve opening time-to-flow variance
- Manifold flow-impedance balance
- Spar friction (telescoping stages, with water/ice ingress)
- Tape-spring snap-through dispersion
- Sensor-trigger threshold variance

**Hard gate:** total left-right variance must close to under **10 ms 3-σ** across the operational envelope. If the budget does not close, architecture decision #5 in BRIEF is reopened.
