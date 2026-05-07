# 01 — Aero Sizing

**Status:** stub — this is **deliverable #1** per `BRIEF.md`. To be authored next.

## Scope

- Planform: span, area, sweep, taper, twist distribution.
- Airfoil selection (reflexed, suitable for tailless flying wing).
- Trim solution at design CL across pilot mass envelope (70–95 kg).
- Stability derivatives from AVL: static margin without a tail, $C_{m\alpha}$, $C_{n\beta}$, $C_{l\beta}$, dihedral effect.
- L/D verification at $V_{best} = 25$ m/s — must achieve 10:1 at design weight with body-fairing $C_{D0}$ assumption.
- Stall: $C_{Lmax}$ estimate and stall airspeed at MTOW.

## Inputs

| Quantity | Value | Source |
|---|---|---|
| Wing area $S$ | 8.4 m² | BRIEF |
| Span $b$ | 7.4 m | BRIEF |
| Aspect ratio | 6.5 | derived |
| LE sweep | 25° | BRIEF (locked) |
| Taper ratio | 0.4 | BRIEF (locked) |
| Tip washout | 4–6° | BRIEF — to be pinned by trim/stall study here |
| Pilot+rig mass | 70–95 kg | BRIEF |
| Wing mass budget | 15.5 kg | BRIEF |
| Best-glide airspeed | 25 m/s | BRIEF target |
| Stall airspeed | <14 m/s | BRIEF target |

## Deliverables

- AVL input deck under `analysis/aero/` for the swept planform.
- Airfoil polar (XFOIL) for the selected reflexed section, at representative Re.
- Trim sweep across pilot mass envelope.
- Stability derivative table.
- L/D vs. airspeed curve. Sensitivity to $C_{D0}$ assumption.
- 3D geometry under `cad/wing/` (FreeCAD parametric, STEP export) — drives downstream structural and CAD work.

## Open issues to resolve in this doc

- Airfoil: candidate set (MH-series reflexed, EH-series, custom?).
- Washout: linear vs. nonlinear, how much.
- Body-fairing $C_{D0}$: literature values for prone pilot configurations + uncertainty bracket.
- Whether 25° is enough sweep for tailless static margin at the chosen tip moment arm; if not, this loops back to BRIEF and architecture decision #4 gets reopened.
