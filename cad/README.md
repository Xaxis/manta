# cad/

3D models of every subsystem. Living artifacts that track analysis inputs — when an analysis parameter changes, the model updates.

## Standing rule

Every design step on MANTA produces CAD geometry alongside its analysis doc. Not a downstream artifact — part of the deliverable.

## Format policy

- **Source:** FreeCAD native (`.FCStd`). Free, scriptable, version-controllable.
- **Interchange:** STEP (`.step`) export committed alongside each `.FCStd`. Lets reviewers open in OnShape, SolidWorks, Fusion, etc.
- **Visualization:** STL committed for any model intended for figures or quick previews. Lightweight; regenerate if the source updates.
- **Parametric:** When geometry tracks analysis (telescoping spars, rib geometry derived from planform, swept wing surface, etc.), prefer **Python-driven** generation — FreeCAD's Python API or **CadQuery**. Commit the script; the `.FCStd` / `.step` are regenerated.

## Layout

```
cad/
├── wing/         # outer mold line: planform, airfoil-lofted surface, washout
├── spars/        # telescoping CFRP spars (front & rear, 3-stage)
├── ribs/         # bistable tape-spring rib booms, stowed and deployed
├── harness/      # wing-harness shell + interface to skydiving rig
├── jettison/     # spar-root fittings + pyrotechnic cutter assemblies
├── fcs/          # FCS bay, sensor mounting, servo locations, wiring routes
└── README.md
```

## Per-model README

Each subdirectory carries its own `README.md` describing:

- What the model represents.
- Which analysis it is dimensionally driven by (cite the file).
- Generation command, if Python-parametric.
- Last-updated commit and the change that triggered the update.

## Conventions

- One assembly per directory; child parts as separate files referenced into it.
- Units: SI (mm for length, kg for mass, N for force).
- Origin: wing apex at $(0, 0, 0)$, X aft, Y starboard, Z up — body axes consistent with AVL deck.
- Naming: `subsystem-component-revN.FCStd` (e.g. `spar-front-stage1-rev3.FCStd`). STEP export drops the `.FCStd` extension only.
