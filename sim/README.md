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

## 2. Deployment animation — `build.py`

`run_simulation()` runs the deployment **mechanism** kinematics in MuJoCo
(shoulder/hip yokes spread the spars over Phase A, the wrist + ankle tip booms
telescope out over Phase B). The geometry is then built as **one mesh with a
fixed topology that is identical on every frame**; the 60 deployment frames are
stored as **morph targets (shape keys)**. The result is a single
`manta.glb` containing a real, continuously-interpolated deployment animation —
no opacity crossfades, no per-pose static meshes.

The web viewer (`site/src/components/viewer/Viewer.tsx`) plays it by scrubbing a
`THREE.AnimationMixer` clock from the deploy slider, so any fractional deploy
state is a true blend of the two neighbouring frames.

Why morph targets instead of an armature: identical-topology morphing
sidesteps all skin-weight / bone-roll bookkeeping and guarantees the exact
intended geometry on every frame, which is what makes the scrub exact.
