# Project MANTA

## What this is

A **deployable rigid-wing extension to a wingsuit-style flight system**.
The pilot wears a fitted wingsuit-derivative garment with an integrated
deployable structure: a CFRP spine yoke mounted on the back, with
arm-aligned and leg-aligned spars that brace into position as the pilot
spreads to a flight pose. Telescoping tip extensions from the wrists and
ankles complete the wingspan. Deployable rapidly — in flight from
freefall posture, or on the ground from a packed configuration.

This is a real flight vehicle development program disguised as a piece
of sporting goods. Treat it as such.

## Architecture (locked unless evidence forces a change)

1. **Pilot is the fuselage.** Pilot wears a fitted, fabric-skinned
   harness garment (wingsuit-derivative) with an integrated CFRP spine
   yoke along the back. Pilot's body forms the central wing chord.

2. **Two-spar, four-arm-braced structure.** Per side:
   - **Leading-edge (LE) spar:** rigid CFRP boom, hinged at the spine
     yoke at the shoulder. Runs along the pilot's arm (underside,
     bonded to the harness sleeve) from shoulder to wrist. The pilot's
     arm fits alongside the spar; the spar carries the bending load,
     the arm provides the aerodynamic shape and control input.
   - **Trailing-edge (TE) spar:** mirror geometry along the pilot's
     leg, hinged at a hip yoke, from hip to ankle.
   - **Wrist tip extension:** 3-stage telescoping CFRP boom from the
     wrist hub outward to the wingtip leading edge. Pneumatic CO₂
     extension.
   - **Ankle tip extension:** mirror, from ankle hub to wingtip
     trailing edge. Pneumatic CO₂ extension, sequenced with the wrist
     extension for symmetric deploy.

3. **Bistable CFRP tape-spring ribs.** 9 per side, span chordwise
   between LE and TE spars. Coiled at the spar in stowed; snap-deploy
   as the spars reach their locked positions. Passive — no power
   required.

4. **DCF skin.** ~50 g/m², bonded to ribs and to the harness body
   panels, tensioned by full deployment of the structure.

5. **Wing planform — resized (finding #5).** S = 6.5 m², b = 6.3 m,
   AR = 6.1, 25° LE sweep, taper ratio 0.4, 6° tip washout. Downsized
   from the original 8.4 m² / 7.4 m: because MANTA lands under a reserve
   canopy (not on the wing), a low stall speed is not required, which
   frees the wing to shrink for a far more feasible ~2.4 m telescoping
   boom while holding the glide target (L/D ≈ 11.6 at V_bg ≈ 18 m/s).
   Source of truth: `analysis/aero/planform/geometry.py`.

6. **Deployment sequence (~0.6 s end-to-end):**
   - **Phase A (~0.3 s):** pilot extends arms and legs outward from
     stowed posture. Pneumatic shoulder/hip yokes assist and lock the
     spars at the deployed sweep angle.
   - **Phase B (~0.1 s):** CO₂ fires; telescoping tip extensions snap
     out from wrist and ankle hubs simultaneously. Sequenced through
     a single valve per side for symmetry (active modulation per the
     symmetry-budget analysis).
   - **Phase C (~0.05 s):** bistable ribs snap to open shape as the
     extended spars pass them.
   - **Phase D (passive):** skin tensions across the deployed
     structure; FCS captures trimmed glide.

7. **In-flight deployability.** Because the deployment is the pilot
   spreading from a tucked posture to spread-eagle, it works equally
   well from a stable freefall posture (no drogue stabilization
   needed — spread-eagle freefall is self-stable) or from the ground
   (e.g., for cliff-launch or aircraft-floor pre-deploy).

8. **Fly-by-wire control.** Redundant Pixhawk-class FCS, EKF at 400 Hz,
   MEMS IMUs. Flaperons (servo-driven, brushless waterproof) on the
   trailing-edge tip extensions. Pilot also has body-control authority
   via shoulder/hip rotation against the locked spar — a hybrid
   control path that maps directly to wingsuit pilot intuition. The
   alpha limiter remains a structural design assumption.

9. **Reserve canopy compatibility.** Reserve container is on the
   pilot's back, ABOVE the spine yoke (where the FCS bay sits in
   front of the reserve PC launch path). After tip-extension retract
   OR yoke release (the yoke spar pivots disengage from the spine on
   command), the rigid structure folds clear of the reserve cone.
   No pyrotechnic spar-root cutters required — disengagement is
   mechanical, latched, and reversible.

10. **Pilot fully retains wingsuit mode.** If the rigid structure is
    fully retracted (tip extensions in, yokes folded), the pilot is
    in a fabric-wingsuit configuration and can fly under fabric-wing
    control to a normal canopy descent. This is the architectural
    fallback.

## Performance targets

- Glide ratio: 10:1 design target, ~12:1 stretch (vs. 3:1 wingsuit, 16:1 hang glider)
- Best-glide airspeed: ~18 m/s (per the glide-polar analysis, resized planform)
- Stall speed: ~15-16 m/s (relaxed — landing is under reserve canopy, not the wing)
- Pilot mass envelope: 70-95 kg
- Wing area: 6.5 m²
- Span: 6.3 m deployed
- Aspect ratio: 6.1
- Wing loading: ~16.6 kg/m² (163 N/m²; structurally-correct mass per finding #2)
- Wing system mass budget: ~16.5 kg (per the bending-sized spar — see
  finding #2)
- Stowed package thickness (off body): TBD per the new architecture;
  the BRIEF v1 target of 150 mm was for a wing-on-rig-stack design that
  did not work — the arm-braced architecture is much thinner because
  the structure runs along the arms/legs rather than on top of the
  pilot's back.

## Open architecture-amendment findings (from the analysis)

These came out of deliverables #1–#6 + the architecture rebuild:

1. **V_bg ≈ 16 m/s, not 25 m/s** — the wing's natural best-glide is
   ~16 m/s with the locked planform (analysis/aero/lift_drag/).
2. **Front spar must grow** to 73 mm OD root, 2.5 mm wall (was
   40 mm/2 mm) — bending analysis surfaced the original spec failed
   at 3 g limit.
3. **Active per-side flow modulation** (rather than passive matched-
   impedance manifold) is required to close the 10 ms 3-σ symmetry
   budget.
4. **The fundamental architecture was wrong** — the prior BRIEF
   described an aircraft-on-pilot's-back configuration with
   pyrotechnic root cutters, a stowed-package-on-rig stack, and a
   wing planform floating above the pilot. The CORRECT concept is the
   arm-braced wingsuit-extension architecture defined above. This
   amendment supersedes architecture decisions in BRIEF v1.

## First deliverables (revised priority)

Build in this order. Do not skip ahead.

1. `cad/build.py` — single parametric build of the integrated
   architecture: pilot humanoid, spine yoke, arm/leg spars,
   telescoping tips, ribs, skin. Driven by `deploy_state ∈ [0,1]`.
2. `cad/render.py` — multi-view static render + animation frames.
3. `site/src/components/viewer/Viewer.tsx` — interactive 3D viewer
   with bone-hierarchy animation matching the deployment sequence.
4. `analysis/struct/spar_bending.py` — load case update for the
   arm-aligned spar (cantilever from the shoulder/hip yoke, not from
   a sub-frame on a rig). Magnitude similar but the load path is
   different.
5. `analysis/deployment/state_machine.py` — phases A-B-C-D timing per
   the new sequence.
6. `test/ground/spec.md` — ground rig spec updated for the new
   deployment kinematics (arm-spread, tip-extend, rib-snap).

## Tools and software stack

- Aero: AVL (vortex lattice), XFOIL (airfoil), OpenVSP (geometry).
  Optional later: SU2 or OpenFOAM CFD.
- Structural: hand calcs in Python, FEA via CalculiX or FreeCAD FEM
  workbench for the spine yoke and shoulder/hip pivots.
- CAD: CadQuery (FreeCAD-importable STEP) parametric, single source
  of truth at `cad/build.py`.
- FCS: PX4 fork (or ArduPilot reference). SITL for envelope-protection
  development.
- Site: Astro + Tailwind + R3F (Three.js) for the live 3D viewer.

## Engineering culture

This project kills people if done sloppily. Every analysis is
reviewable. Every assumption is cited. Every safety-critical claim
has a test that backs it. No vibes-based engineering. The bar is:
would this analysis hold up if a coroner's office asked for it.

## Out of scope (for now)

- Powered variants. v1 is unpowered glide.
- Ground takeoff (foot-launch). v1 deploys from spread-eagle posture
  in freefall or from a high stationary platform (cliff/tower); not a
  running takeoff.
- Powered landing without parachute. No.
- Certification path. Experimental research aircraft under existing
  skydiving regs.
