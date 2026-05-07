# analysis/aero/planform/

Single source of truth for MANTA planform geometry. All other aero analyses (Weissinger lifting-line, AVL deck, trim, glide polar) and the parametric wing CAD import from `geometry.py` rather than re-deriving chord/sweep numbers from BRIEF.

## File

- `geometry.py` — `Planform` dataclass with locked BRIEF inputs and derived quantities (chords, MAC, sweep at any chord fraction, wetted area, twist distribution). Run as `python geometry.py` to print a Markdown summary table.

## Conventions

- Frame: x aft, y starboard, z up. Wing apex (root LE) is the origin.
- Sweep angles positive aft.
- Twist (washout) positive geometric leading-edge-down at the tip relative to the root.
- Areas reference total (both-sides) wing planform unless noted.

## Computed values (current)

Run `python geometry.py` for the live table. As of this commit, with the locked BRIEF parameters:

- Root chord 1.622 m, tip chord 0.649 m, MAC 1.205 m
- y_MAC 1.586 m, x_MAC c/4 1.041 m aft of root LE
- Sweep: 25° LE, 21.8° c/4, 11.5° TE
- Wetted area ≈ 17.3 m²

## What gets changed here

- BRIEF parameters never edit silently — change requires editing `BRIEF.md` and `docs/00-design-rationale.md`.
- Washout (currently 5°, within the BRIEF 4–6° band) gets pinned by the trim study in `analysis/aero/trim/`. If trim says 4° or 6° wins, edit it here and the rest of the pipeline re-derives.
