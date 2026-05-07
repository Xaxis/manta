# analysis/struct/

Structural analysis. Pairs with `docs/02-structural-budget.md`.

## Tools

- Python (numpy, scipy) for hand calcs and parametric studies.
- CalculiX or FreeCAD FEM workbench for spar and root-joint FEA.
- Material properties cited from manufacturer datasheets (cite version + retrieval date).

## Expected layout (to be populated)

```
struct/
├── mass-budget.{py,csv}    # mass rollup with sensitivities
├── spar-bending.py         # parametric root bending vs. n-load — deliverable #3
├── joints/                 # telescoping junction analysis
├── ribs/                   # tape-spring deployment force, locked stiffness
├── skin/                   # tension, attachment, areal density studies
├── root-fea/               # CalculiX inputs and post-processing
└── README.md
```

## Load cases (working set, to be ratified in 02)

- 1g cruise (reference)
- 3g maneuver — limit
- 4.5g — ultimate (1.5× limit)
- Asymmetric deployment transient — TBD; defined by deployment dynamics work
- Drogue snatch + reefed-inflation — handled in deployment/, but spar-root reaction must close here

## What "done" looks like for deliverables #2 and #3

- **#2 mass-budget**: full rollup vs. 15.5 kg envelope, with sensitivities to (a) spar wall thickness, (b) rib count, (c) skin areal density. Show ≥10% margin or call out the exception.
- **#3 spar-bending**: parametric model showing 40 mm OD / 2 mm wall front spar at 3g limit closes with a defensible safety factor; include both bending and local buckling checks at telescoping joints.
