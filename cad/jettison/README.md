# cad/jettison/

Spar-root fittings and the four pyrotechnic cutter assemblies.

**Driven by:** `docs/05-emergency-systems.md`, `safety/failure-modes/asymmetric-deployment.md`.

The root joint must:

1. Carry full flight loads (spar bending, skin tension reaction, drogue snatch reaction at deploy).
2. Sever cleanly on cutter command — no partial release, no jammed remnants, no hazardous protrusions left behind.
3. Provide a redundant-initiator interface (each cutter has independent fire paths).
4. Be reachable for inspection / installation by the pilot or rigger.

Models include intact, severed (immediately post-fire), and post-jettison stub geometry. The latter is the input to `safety/reserve-compat.md`.
