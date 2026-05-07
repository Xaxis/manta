# analysis/aero/

Aerodynamic analysis. Pairs with `docs/01-aero-sizing.md`.

## Tools

- **AVL** — vortex lattice; planform trim and stability derivatives. Primary tool for sizing and stability.
- **XFOIL** — 2D airfoil polars at representative Reynolds numbers.
- **OpenVSP** — geometry generation and visualization; can drive AVL inputs.
- **SU2 / OpenFOAM** *(later)* — high-fidelity verification for compressibility-free regime; only after AVL says we are close.

## Expected layout (to be populated)

```
aero/
├── airfoil/                # XFOIL inputs, polars, selection notes
├── planform/               # OpenVSP geometry, AVL .avl decks
├── trim/                   # AVL trim sweeps across pilot mass envelope
├── stability/              # stability derivatives, static margin
├── lift-drag/              # L/D vs. V curves, sensitivity to body Cd0
└── README.md               # this file
```

## What "done" looks like for deliverable #1

- A reproducible AVL run that produces a stability table consistent with the planform locked in BRIEF.
- An L/D curve at design weight that closes on the 10:1 target with a defensible body $C_{D0}$ assumption (literature-cited, with an uncertainty bracket).
- A trim study showing positive static margin across the 70–95 kg pilot envelope and across reasonable head/torso CG perturbations.
- An airfoil pick justified against tailless-suitable reflex options (MH/EH series and similar).
- A washout angle pinned (within the 4–6° BRIEF range) by the trim/stall trade.
- All of the above summarized in `docs/01-aero-sizing.md`.
