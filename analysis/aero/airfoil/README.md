# analysis/aero/airfoil/

Airfoil selection and 2D polar.

## The selection problem

A tailless flying wing needs a section with:

1. **Cm0 ≈ 0 or slightly positive** — reflexed (S-camber). Sweep and washout share the trim burden, but a reflexed section keeps the required washout small and preserves outer-wing $C_l$ margin against stall.
2. **Forgiving stall** — gentle $C_l$ rolloff post-stall is critical because we have no tail and the alpha limiter (mandatory per BRIEF) leans on the stall margin built into the section.
3. **$C_{l,max}$ ≥ ~1.2 at Re ≈ 1×10⁶** — sets stall speed via the wing-loading equation. Margin against the BRIEF 14 m/s stall target.
4. **Reasonable thickness (10–12 % t/c)** — needed for spar packaging (40 mm OD / 25 mm tip front spar at root-region chord 1.62 m gives t/c headroom ≈ 0.025; the 12 % section easily contains it).
5. **Low drag at design $C_l$ ≈ 0.5** — directly drives 10:1 L/D viability (BRIEF target).
6. **Insensitive to Re in 1–2 ×10⁶ band** — operational Re range (see below).

## Operational Reynolds number

| Condition | V (m/s) | c (m) | Re |
|---|---|---|---|
| Best glide | 25  | 1.20 | 2.0×10⁶ |
| Cruise (slow) | 18 | 1.20 | 1.4×10⁶ |
| Near stall | 14  | 1.20 | 1.1×10⁶ |
| Tip section near stall | 14 | 0.65 | 6×10⁵ |

Air at sea-level ISA: $\nu \approx 1.46\times10^{-5}$ m²/s.

Tip Re drops to 6×10⁵, where laminar separation can become a concern on under-conditioned sections. Selection prioritizes airfoils that have been characterized (XFOIL/wind-tunnel) at this Re range.

## Candidate set

| Airfoil | t/c | Cm0 | Notes | Source |
|---|---|---|---|---|
| **MH 60** | 10.1 % | ≈ 0 | Hepperle's classic flying-wing section. Widely used in flying-wing models and full-scale Horten-derivative builds. Mild reflex, gentle stall in available data. | Hepperle (mh-aerotools.de) |
| **MH 78** | 10.0 % | mild + | Successor to MH 60 with slightly higher $C_{l,max}$ and similar low-Cd bucket. | Hepperle |
| **MH 91 series** (e.g. MH 91-115) | 11.5 % | ≈ + | Targeted at flying wings; higher $C_{l,max}$, slightly more drag. | Hepperle |
| **HS 522** | 10.7 % | + | Selig's reflexed section. Reflex larger than MH 60. | UIUC airfoil database (Selig) |
| **EH 1.0/9.0** | 9 % | ≈ 0 | Eppler/Hepperle for Horten-style wings. Thinner than ideal for spar packaging. | Eppler |

## Selection (provisional, pending XFOIL verification)

**Primary: MH 78.** Reasons:

- 10.0 % t/c hits the spar-packaging sweet spot.
- Mildly positive Cm0 reduces washout requirement → less induced drag from washout-driven span loading.
- $C_{l,max}$ in published polars (Hepperle) is ~1.25–1.35 at Re = 1×10⁶, comfortably enabling sub-14 m/s stall.
- Documented Re-sensitivity behavior in the relevant range.

**Alternate: MH 60.** A more conservative pick with longer pedigree; switch back if MH 78 polars at MANTA Re reveal undocumented surprises.

**Hard rejection criteria** (any of these on XFOIL data → drop the candidate):

- $C_{l,max}$ < 1.15 at Re = 1×10⁶.
- Sharp post-stall $C_l$ drop (>0.4 within 2° α past stall).
- Cm0 < −0.015 (would force large washout).
- Cd0 > 0.011 at design $C_l$ (would push 10:1 L/D out of reach with realistic body Cd₀).

## Files

- `polar_analytic.py` — first-cut analytical polar (thin-airfoil theory + literature-bracketed Cm0 / Cd0 / $C_{l,max}$). Used by Weissinger and trim/glide-polar codes until XFOIL runs are available. Parameters are explicit so the polar is interpretable; uncertainty is documented.
- `xfoil_run.sh` *(stub)* — shell driver that runs XFOIL on the selected section across the operational Re range. Requires `xfoil` on PATH; runs are committed as CSV polars in `polars/`. **Until this is run, the analytic polar is what the rest of the pipeline uses, and conclusions are tagged "pending XFOIL".**
- `polars/` — XFOIL output, one CSV per (airfoil, Re). Loaded via interpolation in downstream code.

## What "done" looks like for this directory

1. XFOIL polars run for MH 78 + MH 60 at Re = 0.7, 1.0, 1.5, 2.0 ×10⁶, alpha sweep with transition prediction.
2. CSV polars committed.
3. The Cm0 / $C_{l,max}$ / Cd0 numbers in the analytic polar are replaced (or confirmed) against XFOIL.
4. Selection finalized in `docs/01-aero-sizing.md` with the polar attached.

Until then, the analytic polar is the working assumption with the uncertainty bracket carried into downstream sensitivity analysis.
