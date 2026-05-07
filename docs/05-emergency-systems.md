# 05 — Emergency Systems

**Status:** stub.

## Scope

Wing jettison, AAD integration, reserve compatibility, asymmetric-deployment detection, manual-abort interface.

## Locked from BRIEF

- Four pyrotechnic cutters at the spar roots fully jettison the wing assembly.
- Triggers: pilot command, AAD activation, FCS detection of asymmetric deployment.
- After jettison, the underlying piggyback skydiving rig (main + reserve) operates normally.
- Reserve canopy must deploy clean over the head with no wing-structure occlusion.

## Topics

- Cutter selection: redundant initiators, no-fire/all-fire margins, integrated lockout interlock.
- Trigger logic and arming sequence (deploy-command-sent disables ground-handling lockout, landed-and-stopped re-engages it, etc.).
- AAD integration: which AAD (Cypres / Vigil / M2 / etc.), interface signal, independence from FCS.
- Asymmetric-deployment detector: sensor inputs, threshold logic, time-to-decision (must beat the recoverable window).
- Reserve-compat geometry: post-jettison, no protruding stub, no fouling path for reserve bridle, no reserve canopy contact with severed root structure during inflation.
- Pilot manual abort: physical handle location, reach with arms in deployed configuration, two-action requirement to prevent inadvertent fire.
- Stub-end safety: severed root must not become a hazard to pilot or canopy lines.

## Deliverables

- This doc.
- `safety/reserve-compat.md` — full reserve compatibility analysis with geometry, trajectory, line-clearance margins.
- `safety/failure-modes/asymmetric-deployment.md` — detection and response, with budget and test plan.
- 3D model under `cad/jettison/`: cutter assemblies, root fittings, severed-state geometry.
- Drop-test article specification under `test/drop/` for end-to-end jettison-and-reserve verification.
