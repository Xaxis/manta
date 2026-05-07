"""
Top-level driver for the Weissinger lifting-line analysis on the MANTA wing.

Produces:
    out/alpha_sweep.csv     — CL, CDi, e, neutral-point row per α
    out/span_loading.csv    — y, chord, cl, span_load, α_ind at each α in the sweep
    out/results.md          — Markdown summary table
    out/span_loading.png    — span loading plot at design CL (matplotlib)
    out/glide_polar.png     — induced-drag-only Cl/Cd polar

Run:
    PYTHONPATH=. .venv/bin/python analysis/aero/weissinger/run.py

Numbers consumed by docs/01-aero-sizing.md and other downstream analyses
(trim, glide polar) come from out/results.md.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# allow `analysis...` imports when run as a script
_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.aero.airfoil.polar_analytic import AnalyticPolar  # noqa: E402
from analysis.aero.planform.geometry import Planform  # noqa: E402
from analysis.aero.weissinger.weissinger import (  # noqa: E402
    WingModel,
    alpha_sweep,
    neutral_point,
)


N_PANELS = 50
ALPHAS_DEG = list(range(-2, 13))  # -2° to +12° in 1° steps
DESIGN_CL = 0.50  # target cruise CL (sets best-glide α via the lift curve)


def _build_wing(p: Planform, polar: AnalyticPolar) -> WingModel:
    return WingModel(
        span=p.b,
        chord_at=p.chord_at,
        x_le_at=p.x_le_at,
        twist_deg_at=p.twist_at,
        section_alpha_0_deg=polar.alpha_0_deg,
        section_a0_per_rad=polar.cl_alpha_per_rad,
    )


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    p = Planform()
    polar = AnalyticPolar()
    wing = _build_wing(p, polar)

    # --- Alpha sweep --------------------------------------------------------
    sweep = alpha_sweep(wing, ALPHAS_DEG, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)

    cls = np.array([r.CL for r in sweep])
    cdis = np.array([r.CDi for r in sweep])

    # CL_α from the linear (small-α) portion of the sweep — fit α in [0°, 6°]
    a_arr = np.array(ALPHAS_DEG, dtype=float)
    fit_mask = (a_arr >= 0.0) & (a_arr <= 6.0)
    slope, intercept = np.polyfit(np.deg2rad(a_arr[fit_mask]), cls[fit_mask], 1)
    cl_alpha_per_rad = float(slope)
    cl_at_alpha_zero = float(intercept)
    alpha_zero_lift_deg = float(-intercept / slope * 180.0 / np.pi)

    # Neutral point — best to use a small α range fully in the linear regime
    x_np, mac_used = neutral_point(
        wing, alphas_deg=(2.0, 6.0), n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac
    )

    # --- Span loading at design CL ------------------------------------------
    # Find the alpha that gives DESIGN_CL via linear fit from the sweep
    alpha_design_deg = (DESIGN_CL - cl_at_alpha_zero) / cl_alpha_per_rad * 180.0 / np.pi

    # Re-solve at that alpha for accurate span loading
    from analysis.aero.weissinger.weissinger import solve

    r_design = solve(wing, alpha_design_deg, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)

    # --- CSV outputs --------------------------------------------------------
    sweep_csv = out_dir / "alpha_sweep.csv"
    with sweep_csv.open("w") as f:
        f.write("alpha_deg,CL,CDi,e,CL_over_CDi,cm_apex\n")
        for r in sweep:
            lod = r.CL / r.CDi if r.CDi > 1e-9 else float("nan")
            f.write(f"{r.alpha_deg:.3f},{r.CL:.5f},{r.CDi:.6f},{r.e:.5f},{lod:.3f},{r.cm_about_apex:.5f}\n")

    span_csv = out_dir / "span_loading.csv"
    with span_csv.open("w") as f:
        f.write("alpha_deg,y,chord,cl_section,span_load_cl_c,alpha_ind_deg\n")
        for r in sweep:
            for y, c, cl, sl, ai in zip(
                r.y, r.chord, r.cl_section, r.span_load, r.alpha_induced_deg
            ):
                f.write(f"{r.alpha_deg:.3f},{y:.4f},{c:.5f},{cl:.5f},{sl:.5f},{ai:.4f}\n")

    # --- Markdown summary ---------------------------------------------------
    summary = out_dir / "results.md"
    with summary.open("w") as f:
        f.write("# Weissinger lifting-line — MANTA wing\n\n")
        f.write(f"**Solver:** {N_PANELS} panels per side, cosine spaced. ")
        f.write("Section model: analytic MH-78-class polar, a0 = "
                f"{polar.cl_alpha_per_rad:.3f}/rad, α₀ = {polar.alpha_0_deg:.2f}°.\n\n")
        f.write("## Planform\n\n")
        from analysis.aero.planform.geometry import summary_markdown

        f.write(summary_markdown(p) + "\n\n")
        f.write("## Finite-wing aero properties\n\n")
        f.write(f"- **CL_α** (linear fit, 0–6°): `{cl_alpha_per_rad:.3f} /rad` "
                f"(`{cl_alpha_per_rad / 57.296:.4f} /deg`)\n")
        f.write(f"- **Zero-lift α** (3D, with twist): `{alpha_zero_lift_deg:.2f}°`\n")
        f.write(f"- **CL at α = 0°**: `{cl_at_alpha_zero:+.4f}`\n")
        f.write(f"- **α at design CL = {DESIGN_CL}**: `{alpha_design_deg:.2f}°`\n")
        f.write(f"- **Neutral point (x aft of root LE)**: `{x_np:.4f} m` "
                f"= `{x_np / mac_used:.3f}·MAC`\n")
        f.write(f"- **Geometric MAC c/4**: `{p.x_mac_c4:.4f} m` "
                f"= `{p.x_mac_c4 / p.mac:.3f}·MAC`. NP shift aft of geom MAC c/4: "
                f"`{(x_np - p.x_mac_c4) / p.mac * 100:+.1f}%` of MAC.\n\n")
        f.write("## α sweep\n\n")
        f.write("| α (°) |   CL    |   CDi    | CL/CDi |   e   | Cm_apex |\n")
        f.write("|---|---|---|---|---|---|\n")
        for r in sweep:
            lod = f"{r.CL / r.CDi:7.1f}" if r.CDi > 1e-9 else "    n/a"
            e_str = f"{r.e:5.3f}" if 0.4 < r.e < 1.5 else "  n/a"
            f.write(f"| {r.alpha_deg:+5.2f} | {r.CL:+.4f} | {r.CDi:+.5f} | {lod} | {e_str} | {r.cm_about_apex:+.4f} |\n")
        f.write("\n_Note: low-CL e values are numerically unstable as CL → 0; trust e in the working CL range._\n\n")
        f.write(f"## Span loading at design CL = {DESIGN_CL} (α = {alpha_design_deg:.2f}°)\n\n")
        f.write(f"Sample stations (every {N_PANELS // 10}th panel):\n\n")
        f.write("|  y (m)  | chord (m) | cl_section | span_load | α_ind (°) |\n")
        f.write("|---|---|---|---|---|\n")
        step = max(1, len(r_design.y) // 12)
        for i in range(0, len(r_design.y), step):
            f.write(f"| {r_design.y[i]:+.4f} | {r_design.chord[i]:.4f} | "
                    f"{r_design.cl_section[i]:.4f} | {r_design.span_load[i]:.4f} | "
                    f"{r_design.alpha_induced_deg[i]:+.2f} |\n")

    # --- Plots -------------------------------------------------------------
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        # Span loading
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(r_design.y, r_design.cl_section, label="cl(y)")
        ax.plot(r_design.y, r_design.span_load / r_design.span_load.max(),
                label="span loading (cl·c, normalized)", linestyle="--")
        ax.set_xlabel("y (m)")
        ax.set_ylabel("cl  /  normalized span loading")
        ax.set_title(f"MANTA span loading at design CL = {DESIGN_CL} (α = {alpha_design_deg:.2f}°)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "span_loading.png", dpi=140)
        plt.close(fig)

        # Lift / induced-drag polar
        fig, ax = plt.subplots(figsize=(7, 5))
        # Filter to physical CL>0 range
        mask = cls > 0
        ax.plot(cdis[mask], cls[mask], "o-", label="Wing alone (induced only)")
        ax.set_xlabel("CDi  (induced drag only)")
        ax.set_ylabel("CL")
        ax.set_title("MANTA lift / induced-drag polar  (Weissinger)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "induced_polar.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass

    # --- Console summary ----------------------------------------------------
    print(f"Output written to {out_dir}/")
    print()
    print(f"  CL_α            = {cl_alpha_per_rad:.4f} /rad  ({cl_alpha_per_rad/57.296:.4f} /deg)")
    print(f"  α at design CL  = {alpha_design_deg:.2f}°  (CL = {DESIGN_CL})")
    print(f"  α₀ (3D, twist)  = {alpha_zero_lift_deg:.2f}°")
    print(f"  Neutral point   = {x_np:.4f} m aft of apex  ({x_np/mac_used:.3f}·MAC)")
    print(f"  Geom MAC c/4    = {p.x_mac_c4:.4f} m aft of apex  ({p.x_mac_c4/p.mac:.3f}·MAC)")
    print()


if __name__ == "__main__":
    main()
