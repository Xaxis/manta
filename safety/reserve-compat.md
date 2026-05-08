# Reserve parachute compatibility

**Status:** Constraints defined; geometric clearances bracketed; bench
verification + drop-article verification gated as the BRIEF's hardest
constraint.

This document is the closure of the BRIEF's hardest constraint:

> Reserve parachute compatibility is non-negotiable. The skydiving rig
> functions normally for canopy flight and landing after wing jettison.
> Anything that compromises that is a non-starter.

## Hard requirements

The reserve canopy must deploy cleanly **on every credible failure path**
that gets to a reserve event:

1. **Pre-jettison harness geometry** must not interfere with the reserve
   container, ripcord, RSL, or AAD function.
2. **Jettison must remove the wing assembly cleanly.** No hanging spars,
   no flapping skin, no bridle entanglement.
3. **Post-jettison stub geometry** must leave the reserve canopy
   inflation envelope unobstructed.
4. **Reserve deployment trajectory** must be clear — pilot chute
   extraction, bag launch, line stretch, canopy inflation all in
   uncongested air.
5. **Worst-credible attitude case at jettison** (asymmetric-deploy
   induced roll, pilot in unusual attitude) must still allow clean
   reserve deployment.
6. **AAD compatibility:** the AAD signals jettison directly (FCS-bypassed)
   and operates as a standard skydiving AAD when the wing is gone.
7. **Canopy descent and landing**: the wing-harness mount, after
   jettison, must not be a hazard to the pilot under canopy.

## Pre-jettison geometry

The wing harness sits *on top of* the standard piggyback skydiving rig
(BRIEF architecture decision #8). The harness is bolted to the
sub-frame on the rig's main lift webs (or to a bridge that distributes
load across them — final mounting TBD with rigger consultation).

Constraints:

| Item | Constraint | Status |
|---|---|---|
| Reserve container access | Ripcord handle, RSL routing, AAD button must remain reachable from the inboard side without removing wing parts. | Geometry stub in `cad/harness/` — model TBD |
| Reserve PC routing | Pilot chute path from the reserve must not pass under any wing structure. | Verified geometrically once `cad/harness/` is built |
| AAD physical access | Cypres / Vigil / M2 powering and arming buttons must be reachable in pre-flight without partial wing disassembly. | Same |
| Cutaway handle reach | The pilot must be able to reach the main-canopy cutaway handle in the deployed-wing configuration (in case of a main-canopy mal post-jettison). | Verified once harness CAD is built |
| Reserve handle reach | Same — reachable in any flight or descent attitude. | Same |
| Skydiving rig main canopy compatibility | Wing harness must not block the main-canopy deployment path either. (Even if normal MANTA op doesn't need the main canopy, post-jettison the main is the primary descent canopy.) | Same |

## Jettison sequence — geometric requirement

Spar-root cutters fire at the four root fittings (front-left, front-
right, rear-left, rear-right). The wing assembly separates from the
bonded fittings, leaving stubs ≤ 80 mm long projecting from the harness
sub-frame.

Geometric constraints that the cutter design must meet (owned by
[`cad/jettison/`](../cad/jettison/)):

- **Stub end is rounded** (no sharp severed-fiber edges that snag lines).
  Achieved by an internal ferrule that captures the spar end during
  the cut.
- **Stub does not protrude into the reserve deployment cone.** The
  cone is ~30° half-angle from the reserve-canopy launch point;
  stubs (≤ 80 mm long, located at chordwise positions 0.20·c and
  0.65·c) are well outside that cone given the harness geometry —
  to be confirmed in CAD.
- **No loose internal fasteners** post-cut. The cup remains bonded to
  the harness; the spar departs with the wing assembly. Any attachment
  hardware (servo bolts, sensor leads, pneumatic tubing) is plumbed
  through the spar root and severed by the LSC alongside the spar.

## Post-jettison reserve trajectory

The reserve canopy launches from the reserve container on the pilot's
back. Trajectory in body-axes:

```
                ↑ z (skyward)
                │
                ┊  inflated reserve canopy
                ┊
                ┊
                │
   ┌───────────┴───────────┐    bag at line-stretch
   │                       │
   │                       │
   │                       │ ← reserve container on the rig
   │     pilot prone       │
   │                       │
   └───────────────────────┘
   ←—— x (aft) ——→
```

The pilot is prone; the reserve container faces upward (vehicle z+).
After deployment:
- pilot chute → drag-extracts bag in +z direction
- canopy clears the bag at line-stretch (~ 30 m above the pilot, ~ 1 s)
- canopy inflates, lift vector aligns with descent direction

Constraints from the wing-jettison case:

- **No wing structure in +z hemisphere** post-jettison. Verified by the
  stub geometry: stubs are in the horizontal plane (chord plane), no
  vertical projection.
- **No bridle / drogue residue trailing from the harness.** The drogue
  stays with the wing assembly when the wing leaves; the bridle is
  attached to the wing root, not to the harness. The drogue line must
  be designed to release cleanly with the wing.

## Worst-credible attitude at jettison

If asymmetric deployment induces a roll, the pilot may be in 30–90° bank
when jettison fires. Reserve deployment in unusual attitudes is a
well-characterized skydiving scenario (BASE jumpers and accelerated-
freefall instructors deal with it routinely). Skydiving reserve canopies
are designed to inflate from a wide range of body attitudes and at any
airspeed below ~70 m/s.

Specific concerns for MANTA:

- **Pilot in steep bank or inverted at jettison:** reserve still inflates
  but trajectory may be unusual. Acceptable; standard skydiving practice.
- **Pilot tumbling at jettison:** reserve drogue kicks in body-axis
  tumble; may take an extra 0.5–1.5 s to stabilize before canopy fully
  inflates. Altitude budget for the BRIEF AAD trigger threshold (200 m
  AGL) needs to allow for this — owned by `safety/failure-modes/aad-fault.md`.

## AAD compatibility

The AAD (Cypres / Vigil / M2) is wired in two paths:

1. **Standard skydiving function:** AAD fires the reserve directly via
   its existing cutter (which severs the closing loop on the reserve
   container). This path operates whether the wing is attached or not;
   does not depend on FCS or any MANTA-specific hardware.

2. **MANTA jettison signal:** AAD altitude/descent-rate trigger sends a
   signal directly to the MANTA cutter firing circuit, **bypassing both
   FCS units**. Independent power rail; independent wiring. Triggers
   wing jettison ahead of the AAD-initiated reserve cutter, by a
   programmed delay (target: 100 ms) so the wing departs before the
   reserve inflates.

Selection criteria for the AAD: any production skydiving AAD with an
auxiliary GPIO output for the MANTA jettison trigger. Cypres 2 supports
this via its serial output; Vigil and M2 require integrator review.

## Pre-flight inspection procedure (placeholder)

1. Verify wing-harness mounting bolts torqued.
2. Verify all 4 root fittings show no cracks / no bond defects.
3. Verify all 4 cutter assemblies armed (initiators connected, lockout
   pin removed).
4. Verify reserve container door free (no wing parts contacting it).
5. Verify ripcord handle reachable.
6. Verify cutaway handle reachable.
7. Verify AAD powered, armed, with current battery date.
8. Test FCS jettison-circuit continuity (built-in test).
9. Independent rigger check before jump.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Geometric clearance (CAD) | Stubs outside reserve deploy cone; PC path clear; handle reach | [`cad/harness/`](../cad/harness/), [`cad/jettison/`](../cad/jettison/) |
| Bench mock-up | Physical geometry agrees with CAD; rigger sign-off on access and reach | TBD |
| Drop article — symmetric jettison | Reserve clears stubs at design speed | [`test/drop/`](../test/drop/) |
| Drop article — asymmetric jettison | Reserve clears stubs in worst-credible attitude (induced roll at deploy) | [`test/drop/`](../test/drop/) |
| AAD interface | AAD fires both jettison and standard reserve cutter at programmed delay | bench + drop |
| Canopy descent | Pilot lands cleanly under reserve with the post-jettison stubs in place | drop article + first manned-deploy run |

Until all six gates pass, no manned-deployment flight is authorized.

## Open issues

- Final harness mount geometry (single-bridge vs. dual-strap) — depends
  on rigger consultation and structural test of the bridge.
- AAD vendor selection and integration sign-off.
- Precise placement of the spar-root fittings vs. reserve canopy launch
  geometry — needs the harness CAD to close.
- Rigger SOP for re-packing after a jettison event (the harness needs
  to be re-armed; the rigger is unlikely to have seen this hardware
  before — training material required).
