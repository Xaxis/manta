# analysis/aero/avl/

AVL (Athena Vortex Lattice) input deck for the MANTA wing. Higher-fidelity
verification of the Weissinger lifting-line work in `../weissinger/`.

## Files

- `manta.avl` — geometry deck. Mirror-symmetric half-wing with one SECTION at the root and one at the tip; MH-78-class airfoil reference (`airfoils/MH78.dat` — user must drop in the file from mh-aerotools.de). Reference point is at the geometric MAC c/4.
- `manta.mass` — mass case file for stability analysis (CG, inertia tensor placeholders — populated when the structural budget is closed).
- `manta_default.run` — runfile with α and β sweeps for trim and stability derivatives.
- `run.sh` — convenience driver. Requires `avl` on PATH (Drela/Youngren AVL 3.x). Outputs go to `out/`.

## Sources / references

- Drela, M. & Youngren, H. *AVL 3.36 User Primer*, MIT, 2017. https://web.mit.edu/drela/Public/web/avl/
- Locked planform parameters: `BRIEF.md` and `analysis/aero/planform/geometry.py`.

## What "running this" gets you

- Trim solution at design CL across pilot mass envelope.
- Stability derivatives table (CL_α, CM_α, CN_β, CL_β, CY_β, CN_p/r, CL_p/r, etc.).
- Neutral-point estimate (independent of the Weissinger code's NP — this is the verification).
- Spanwise loading at trim, comparable to `../weissinger/out/span_loading.csv`.

## How to run

```sh
# 1. Drop MH78 airfoil coordinates in airfoils/MH78.dat
#    (download from https://www.mh-aerotools.de/airfoils/, save the .dat coord file)

# 2. Run AVL with the default runfile
make avl       # from project root
# or
cd analysis/aero/avl && ./run.sh
```

## Verification gates against Weissinger

Before AVL is treated as the authoritative answer, check that for the locked
geometry without flap deflection:

| Quantity | Weissinger | AVL (target) | Acceptable spread |
|---|---|---|---|
| CL_α | 4.24 /rad | TBD | ±5 % |
| Neutral point (·MAC aft of apex) | 0.928 | TBD | ±0.05 |
| Span efficiency e at CL = 0.5 | ~1.05 (numerical) | TBD | within physical (0.85–1.0) |

Disagreements outside those bands indicate a real problem (deck error,
Weissinger limitation surfacing, or incompatible reference geometry). Resolve
before adopting AVL numbers downstream.
