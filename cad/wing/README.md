# cad/wing/

Outer mold line of the deployed wing. Planform, airfoil-lofted surface, washout, deployed-state geometry.

**Driven by:** `docs/01-aero-sizing.md` and `analysis/aero/`.

**Generation strategy:** parametric — wing surface is regenerated from planform parameters (root chord, tip chord, span, sweep, twist distribution) and the selected airfoil's section data. Python (CadQuery or FreeCAD API) recommended so geometry tracks the AVL deck.

To be populated alongside deliverable #1.
