# Project MANTA

## What this is

A deployable rigid-wing personal flight system. Pilot-worn airframe with telescoping carbon fiber spars and bistable composite tape-spring ribs that deploy from a stowed (body-conformal) state to a high aspect-ratio swept flying wing in flight, targeting hang-glider-class glide performance from a wingsuit-class form factor.

This is a real flight vehicle development program disguised as a piece of sporting goods. Treat it as such.

## Performance targets

- Glide ratio: 10:1 design target, 13:1 stretch (vs. 3:1 wingsuit, 16:1 hang glider)
- Best-glide airspeed: 25 m/s (56 mph)
- Stall speed: under 14 m/s (31 mph)
- Pilot mass envelope: 70-95 kg
- Wing area: 8.4 m² (90 sq ft)
- Span: 7.4 m (24.3 ft) deployed
- Aspect ratio: 6.5
- Wing loading: ~10.5 kg/m²
- Wing system mass budget: 15.5 kg
- Stowed package thickness: under 15 cm off body profile

## Architecture decisions (locked unless evidence forces a change)

1. Two-spar wing per side, both CFRP telescoping tubes, 3-stage. Front spar 40mm OD root / 25mm tip / 2mm wall. Rear spar 30mm/18mm/2mm.
2. 9 ribs per side, bistable CFRP tape-spring booms, passive snap-deploy, store coiled flat against the spar.
3. Skin: Dyneema Composite Fabric (DCF), ~50 g/m², bonded to ribs, tensioned by deployment.
4. Planform: 25° leading-edge sweep, taper ratio 0.4, tip washout 4-6° for tailless flying-wing pitch stability.
5. Pneumatic deployment via single CO2 cartridge per side, sequenced from a single valve to enforce sub-10ms left/right symmetry. Tape-spring ribs unfurl passively.
6. Drogue-first deployment sequence: small ringslot drogue decelerates pilot from terminal (~55 m/s) to ~30 m/s before main wing deployment. Non-negotiable; deployment loads scale with q.
7. Fly-by-wire control. Redundant Pixhawk-class FCS, EKF at 400 Hz, MEMS IMUs. Flaperons (servo-driven, brushless waterproof) on outer trailing edge each side. Mechanical reversion to direct cable as last-resort backup.
8. Wing harness sits ON TOP OF a standard piggyback skydiving rig (main + reserve). Four pyrotechnic spar-root cutters fully jettison the wing assembly on command, on AAD trigger, or on detected asymmetric deployment. Reserve canopy deploys clean over the head with no wing-structure occlusion.

## Hard constraints

- Reserve parachute compatibility is non-negotiable. The skydiving rig functions normally for canopy flight and landing after wing jettison. Anything that compromises that is a non-starter.
- Asymmetric deployment is the dominant unrecoverable failure mode. All design decisions defer to mitigating it.
- Stall departure on a tailless high-AR wing is unforgiving. Alpha limiter in the FCS is mandatory; not a feature, a structural design assumption.
- All deployment-critical components have at least one independent backup path or sensed-and-aborted failure mode.

## Known unsolved problems

These are the open research items, in rough order of risk:
1. Sub-10ms deployment symmetry under representative loads, in cold and wet conditions.
2. Tailless flying-wing pitch stability with a moving human "fuselage" (head and torso CG shifts perturb trim).
3. Stall behavior characterization on the specific airfoil and planform; departure prevention via active envelope protection.
4. Telescoping CFRP joint reliability with water and ice ingress.
5. Pilot training transition path from fabric wingsuit to fly-by-wire rigid wing.

## Repo structure

```
manta/
├── BRIEF.md                  # this file
├── docs/
│   ├── 00-design-rationale.md
│   ├── 01-aero-sizing.md
│   ├── 02-structural-budget.md
│   ├── 03-deployment-sequence.md
│   ├── 04-fcs-architecture.md
│   ├── 05-emergency-systems.md
│   └── 06-test-plan.md
├── analysis/
│   ├── aero/                 # XFOIL, AVL, OpenVSP models
│   ├── struct/               # spar bending, FEA inputs, mass budget
│   ├── deployment/           # pneumatic timing, rib unfurl dynamics
│   └── flightdynamics/       # AVL/Athena trim, stability derivatives
├── cad/                      # FreeCAD natives, STEP exports, parametric Python
├── fcs/
│   ├── firmware/             # Pixhawk fork or PX4 module
│   ├── sim/                  # SITL configs, model
│   └── envelope-protection/
├── test/
│   ├── ground/               # static deployment rig specs
│   ├── tow/                  # boat/vehicle tow article
│   └── drop/                 # static drop article
└── safety/
    ├── fmea.md
    ├── reserve-compat.md
    └── failure-modes/
```

## First deliverables (priority order)

Build in this order. Do not skip ahead.

1. `docs/01-aero-sizing.md` and `analysis/aero/`: AVL model of the swept-wing planform with the trim and stability derivatives. Verify the 10:1 L/D target is achievable at the design CL with reasonable Cd0 assumptions for the body fairing. Iterate sweep, taper, and washout for positive static margin without a tail.
2. `analysis/struct/mass-budget.xlsx` (or .py / .csv, your call): full mass rollup, with sensitivity to spar wall thickness, rib count, and skin areal density.
3. `analysis/struct/spar-bending.py`: parametric model of root bending moment vs. n-load, validates the 40mm/2mm spar at 3g limit with safety factor.
4. `docs/03-deployment-sequence.md`: full timeline, sensed handshakes, abort logic. This is the document a safety case will be built around.
5. `analysis/deployment/symmetry-budget.md`: error budget for left-right deployment timing. CO2 cartridge variability, valve actuation variance, tape-spring deployment dynamics. Must close to under 10ms 3-sigma or the architecture has to change.
6. Standalone deployment subsystem ground rig specification (`test/ground/`). This is the first piece of hardware to actually build: instrumented, no flight loads, just prove the deployment kinematics and timing.

Do not move to flight-relevant test articles until the ground deployment rig has demonstrated reliable, symmetric, sensed deployment over the full thermal and humidity envelope, in at least 200 cycles without intervention.

## Tools and software stack

- Aero: AVL (vortex lattice), XFOIL (airfoil), OpenVSP (geometry). Optional later: SU2 or OpenFOAM CFD for high-fidelity verification.
- Structural: hand calcs in Python, FEA via CalculiX or FreeCAD FEM workbench for spars and root joints.
- CAD: FreeCAD natives (.FCStd) with STEP exports for OnShape import. Parametric geometry driven from Python where it tracks analysis inputs (CadQuery acceptable).
- FCS: PX4 fork, ArduPilot as reference. SITL for envelope-protection development.
- Repo: standard git, conventional commits. Treat docs as code, review them.

## Engineering culture

This project kills people if done sloppily. Every analysis is reviewable. Every assumption is cited. Every safety-critical claim has a test that backs it. No vibes-based engineering. The bar is: would this analysis hold up if a coroner's office asked for it.

## Out of scope (for now)

- Powered variants (jet, electric ducted fan). Possible later. Not v1.
- Ground takeoff (foot-launch). v1 is exit-from-aircraft or BASE only.
- Powered landing without parachute. No.
- Certification path. Experimental category eventually; for now, treat as research aircraft under existing skydiving regs.
