# analysis/flightdynamics/

Flight dynamics, trim, stability, and 6-DOF response. Pairs with `docs/04-fcs-architecture.md`.

## Tools

- AVL stability output → linearized state-space.
- Python (numpy/scipy) for 6-DOF integration of nonlinear equations of motion.
- Connects to `fcs/sim/` (PX4 SITL) for closed-loop envelope-protection development.

## Expected layout (to be populated)

```
flightdynamics/
├── trim/                   # trim across pilot mass and CG-perturbation envelope
├── linearized/             # state-space at trim points; eigenvalue analysis
├── nonlinear-6dof/         # full equations of motion, gust response, departure cases
├── pilot-cg-perturbations/ # head/torso CG shift effects on tailless trim
└── README.md
```

## Why this is its own subdir

Aero (`aero/`) sizes the planform. Flight dynamics here closes the loop with the **moving fuselage** problem unique to MANTA: the pilot is the bulk of the mass and not rigid. Head and torso position perturbs CG, which perturbs trim, on a tailless wing where there is no margin to absorb that. This is unsolved-problem #2 in the BRIEF and gets its own analysis directory because of it.

## What "done" looks like

- Trim closes across pilot mass envelope and across plausible head/torso CG perturbations without exceeding alpha-limiter activation thresholds.
- Eigenvalues at trim are stable across the envelope with handling-quality-acceptable damping ratios (criteria pinned in `docs/04-fcs-architecture.md`).
- Closed-loop simulation in `fcs/sim/` shows the alpha limiter prevents departure under the worst credible CG perturbation + gust combination.
