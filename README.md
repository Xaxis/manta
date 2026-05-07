# MANTA

Deployable rigid-wing personal flight system. Pilot-worn airframe — telescoping CFRP spars and bistable tape-spring ribs — that snaps from a body-conformal stowed state into a high-aspect-ratio swept flying wing in flight. Target: hang-glider-class glide (10:1) from a wingsuit-class form factor.

This is a real flight vehicle program. Read [`BRIEF.md`](BRIEF.md) before touching anything else.

## Status

Pre-analysis. Repo scaffold only. No analysis or hardware exists yet.

## Where things live

| Path | What's there |
|---|---|
| [`BRIEF.md`](BRIEF.md) | The brief. Performance targets, locked architecture decisions, hard constraints, priority of first deliverables. |
| [`docs/`](docs/) | Numbered design documents (rationale → aero → structure → deployment → FCS → emergency → test plan). |
| [`analysis/`](analysis/) | Quantitative analysis: aero (AVL/XFOIL/OpenVSP), structural (spar bending, mass budget, FEA), deployment dynamics, flight dynamics. |
| [`cad/`](cad/) | 3D models. FreeCAD natives + STEP exports. Parametric where geometry tracks analysis inputs. |
| [`fcs/`](fcs/) | Flight control system: firmware, SITL, envelope-protection logic. |
| [`test/`](test/) | Test article specifications: ground deployment rig, tow article, drop article. |
| [`safety/`](safety/) | FMEA, reserve-parachute compatibility analysis, per-failure-mode write-ups. |

## How to read this repo

Start at `BRIEF.md`. Then `docs/00-design-rationale.md` for the why behind locked decisions. Then `docs/01-aero-sizing.md` and `analysis/aero/` once those exist — they anchor the rest of the design.

## Engineering bar

> Would this analysis hold up if a coroner's office asked for it?

If the answer to that question is no, the analysis is not done.
