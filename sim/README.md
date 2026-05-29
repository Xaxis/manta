# sim — physics + deployment animation pipeline

Two layers, one source of truth for the geometry.

```sh
PYTHONPATH=. .venv/bin/python sim/aero.py              # aero-model closure check
PYTHONPATH=. .venv/bin/python sim/flight_dynamics.py   # trajectory + plot + telemetry
PYTHONPATH=. .venv/bin/python sim/build.py             # MuJoCo kinematics -> animated GLB
PYTHONPATH=. .venv/bin/python sim/build.py --render    # ... + 3 hero stills (Eevee)
```

## 1. Flight physics — `aero.py` + `flight_dynamics.py`

A real six-state longitudinal flight simulation of the whole sequence:
**belly-to-earth freefall → drogue stabilisation → wing deploy → load-limited
pull-out → captured best-glide.**

The aerodynamic coefficients in `aero.py` are derived from the locked planform
(`S = 8.4 m²`, `AR = 6.5`, `e = 0.85`, `CD0 = 0.025`) and are **not** tuned to
hit the BRIEF targets — the targets fall out of the geometry. `aero.py`
asserts the closure on every run:

| Quantity | Model | BRIEF target |
|---|---|---|
| Best-glide speed `V_bg` | **17.0 m/s** | ~16 m/s |
| Max L/D | **13.2** | 10:1 design, 13:1 stretch |
| Stall speed | **12.6 m/s** | < 14 m/s |
| Stowed terminal velocity | **43.9 m/s** | belly freefall ~ |

Coefficients are interpolated stowed→deployed by the deploy progress `p`, so a
single model spans the bluff freefalling body (`p=0`, no lift, vertical
terminal velocity) and the locked wing (`p=1`).

`flight_dynamics.py` integrates the trajectory at 1 ms and closes the pitch
loop with a fly-by-wire longitudinal autopilot (BRIEF #8): a load-factor-limited
flight-path tracker with speed-on-pitch energy regulation and a 12° alpha
limiter. The simulated sequence settles to:

```
V = 17.1 m/s   γ = -4.2°   L/D = 13.7   α = 6.3°   n = 1.0 g
deploy pull-out peak load = 3.1 g   (within the 3 g limit-load spar sizing)
```

Outputs: `out/flight_dynamics.png` (six-panel trace), `out/telemetry.json`
(per-frame series for the web viewer overlay).

## 2. Full mechanical model + deployment animation — `build.py`

`run_simulation()` integrates the deployment **schedule** in MuJoCo (shoulder/
hip yokes spread over Phase A; the wrist + ankle tip booms telescope out over
Phase B). That schedule drives the wing's open fraction.

The vehicle is built from the **locked planform** (BRIEF #5: `S = 8.4 m²`,
`b = 7.4 m`, `AR = 6.5`, `25°` LE sweep, taper `0.4`, `5°` washout) as **ONE
continuous cambered wing surface, tip-to-tip** — so the skin is continuous
across the body and the region between the legs, exactly like a rigid wing or a
paraglider canopy. The pilot is the fuselage, embedded under the translucent
tensioned skin, with arms along the leading-edge spar and legs along the
trailing-edge spar (BRIEF #2). Every BRIEF component is modelled as a named
material slot:

| slot | parts |
|---|---|
| `suit` | pilot / harness garment body |
| `cfrp` | spine yoke (box beam), shoulder + hip pivot hubs, LE spar, TE spar, 3-stage telescoping tip booms |
| `skin` | cambered NACA-4412 wing (camber + thickness + washout + billow scallop between ribs) |
| `rib`  | 9 bistable airfoil-profile ribs per side |
| `reserve` | reserve container on the back, above the spine yoke |
| `fcs`  | flight-control bay on the spine |
| `flaperon` | trailing-edge flaperons (deflect down on deploy) |
| `metal` | CO₂ canisters + tip-hub fittings |

The deployable geometry is **one mesh with a fixed topology that is identical on
every frame**; the 60 frames are stored as **morph targets (shape keys)** — a
single `manta.glb` with a real, continuously-interpolated deployment animation
(no opacity crossfades, no per-pose static meshes). Identical-topology morphing
sidesteps skin-weight / bone-roll bookkeeping and guarantees the exact geometry
on every frame, which is what makes the web scrub exact.

## 3. Aero field — surface pressure + streamlines

`build_pressure_object()` and `build_flow_object()` bake an **illustrative**
aerodynamic field onto the deployed wing as two *separate static* GLB objects
(`PRESSURE`, `FLOW`) so they never contaminate the morph mesh's materials:

* **Pressure colormap** — surface Cp on the wing (suction-blue over the leading
  edge, recovering aft), via a thin-airfoil `cp_at()` model **scaled by the real
  Weissinger span-loading** (`analysis/aero/weissinger/out/span_loading.csv` at
  the glide α≈6°).
* **Streamlines** — upwash ahead of the LE → acceleration over the suction peak
  → downwash behind the TE; coloured by local speed `√(1−Cp)`.

This is an **intuition field for the web viewer, NOT a CFD solution** — SU2 /
OpenFOAM is the rigorous follow-up per the BRIEF tool stack. The viewer labels
it as such. The web viewer (`site/src/components/viewer/Viewer.tsx`) plays the
deployment by scrubbing a `THREE.AnimationMixer` clock from the deploy slider,
and its **Flow field** toggle reveals `PRESSURE` + animated `FLOW` streamlines
(a small shader scrolls pulses along each line) over the fully-deployed wing.
