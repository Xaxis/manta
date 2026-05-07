# cad/wing/

Outer mold line of the deployed wing. Planform, airfoil-lofted surface,
washout, deployed-state geometry.

**Driven by:** `analysis/aero/planform/geometry.py` (locked from `BRIEF.md`,
washout pinned by `analysis/aero/trim/`).

## Files

- `build.py` — CadQuery script that lofts the wing surface from the parametric
  airfoil through 16 spanwise stations, mirrors to full span, exports STEP +
  STL to `out/`. Will use `airfoils/MH78.dat` if present; otherwise falls
  back to a parametric reflexed-airfoil placeholder.
- `preview.py` — pure-matplotlib top-view renderer for inclusion in docs
  (no CadQuery required).
- `out/` — generated artifacts (STEP, STL, PNG planform). Regenerate with
  `make cad-wing` from the project root.

## Run

```sh
make cad-wing       # uses .venv set up by `make venv`
# or
PYTHONPATH=. .venv/bin/python cad/wing/build.py
PYTHONPATH=. .venv/bin/python cad/wing/preview.py
```

## What gets generated (current)

- `out/wing.step` — solid, importable into OnShape, FreeCAD, Fusion 360.
- `out/wing.stl` — tessellation for visualization and quick mesh tools.
- `out/planform_top.png` — top-view planform with MAC and c/4 line annotated.

Bounding box (full wing) at the locked planform with 5° washout:
- x-extent (chord direction):  ~2.39 m  (root chord + tip sweep offset)
- y-extent (span):              7.42 m
- z-extent (thickness):         ~0.21 m at the section deepest point

## Airfoil

The current section is parametric (NACA 4-digit thickness × cubic S-camber
line). It captures the qualitative shape needed for tailless flying-wing
sections (thin reflexed) but is **not the production airfoil**. To switch:

1. Download the MH 78 coordinate file from
   <https://www.mh-aerotools.de/airfoils/> (or your preferred archive).
2. Save it as `cad/wing/airfoils/MH78.dat`.
3. Re-run `build.py`. The script auto-detects the file.

## What's intentionally not in this model yet

- Dyneema skin attachment line / spar boss bonding regions.
- Spars (separate model in `cad/spars/`).
- Tape-spring ribs (separate model in `cad/ribs/`).
- Trailing-edge flaperon hinge cutouts (added once the FCS architecture
  pins servo placement).
- Stowed (rolled / coiled) state — that's a separate CAD configuration
  driven from the deployment dynamics analysis.
