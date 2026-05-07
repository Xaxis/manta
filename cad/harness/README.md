# cad/harness/

Wing-harness shell and the interface that mounts it on top of a standard piggyback skydiving rig.

**Driven by:** `docs/05-emergency-systems.md`, `safety/reserve-compat.md`.

**Hard constraint:** the underlying skydiving rig (main + reserve) must function normally for canopy flight and landing after wing jettison. The harness mount must release cleanly with the wing when the spar-root cutters fire, leaving no protrusion that would foul the reserve canopy or its lines.

Models will need to include:

- Pilot torso/back representative envelope.
- Stowed wing package on top — verifying the <15 cm thickness off the body profile.
- The skydiving container (representative geometry — vendor-specific later).
- The reserve canopy deployment trajectory and an analyzed clearance envelope.
- Severed-state geometry post-jettison.
