# analysis/

Quantitative analysis. Anything with a number that downstream design or safety claims rest on lives here. Each subdirectory pairs with a `docs/` document that summarizes its conclusions.

## Subdirectories

| Path | Tools | Pairs with |
|---|---|---|
| [`aero/`](aero/) | AVL, XFOIL, OpenVSP (later: SU2/OpenFOAM) | `docs/01-aero-sizing.md` |
| [`struct/`](struct/) | Python hand calcs, CalculiX, FreeCAD FEM | `docs/02-structural-budget.md` |
| [`deployment/`](deployment/) | Python timing models, pneumatic 0-D, tape-spring dynamics | `docs/03-deployment-sequence.md` |
| [`flightdynamics/`](flightdynamics/) | AVL/Athena trim + stability, 6-DOF sim | `docs/04-fcs-architecture.md` |

## Conventions

- **Reproducibility first.** Each analysis has a runnable entry point (`run.py`, `Makefile` target, or documented command). Output is regeneratable from inputs in the repo. No "I have it on my laptop."
- **Inputs are versioned**, outputs are usually not. Commit input decks, scripts, and a small set of canonical figures/tables. Don't commit gigabyte solution dumps; regenerate.
- **Cite assumptions inline.** Materials properties, atmospheric model, drag coefficients — every number has a source.
- **Sensitivity analysis is part of the deliverable**, not optional.
- **Cross-references** to the document it supports go in the analysis README.
