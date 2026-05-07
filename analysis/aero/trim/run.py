"""
Tailless trim + washout iteration for the MANTA wing.

For a flying wing with no horizontal tail, longitudinal trim closes by:

    Cm_cg(α_trim, CL_trim)  =  0

where  Cm_cg = Cm_apex(α) + (x_cg / MAC) · CL(α)  is the pitching moment
about the CG (positive nose-up). The Weissinger solver gives Cm_apex(α);
this script combines that with sweeps of:

    - washout angle  in {3°, 4°, 5°, 6°, 7°}
    - design CL      = 0.5 (cruise) and 0.3, 0.7 sensitivity
    - pilot mass     = 70, 82.5, 95 kg

For each (washout, design CL) pair it finds:
    α_trim          (alpha at design CL)
    x_cg_trim       (where the CG must be placed for Cm_cg = 0)
    SM              (static margin = (x_NP − x_cg) / MAC)
    α_tip_eff       (effective angle of attack at the tip — stall margin proxy)

Outputs:
    out/results.md          summary tables + recommended washout
    out/washout_sweep.csv   raw data
    out/washout_plot.png    SM vs washout, x_cg_trim vs washout
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.aero.airfoil.polar_analytic import AnalyticPolar  # noqa: E402
from analysis.aero.planform.geometry import Planform  # noqa: E402
from analysis.aero.weissinger.weissinger import (  # noqa: E402
    WingModel,
    alpha_sweep,
    neutral_point,
    solve,
)


N_PANELS = 50
WASHOUTS_DEG = [3.0, 4.0, 5.0, 6.0, 7.0]
DESIGN_CLS = [0.5]      # primary
SENSITIVITY_CLS = [0.3, 0.4, 0.6, 0.7]
ALPHA_FIT_RANGE = (0.0, 8.0)


def _build_wing(washout_deg: float, polar: AnalyticPolar) -> tuple[WingModel, Planform]:
    p = Planform(washout_deg=washout_deg)
    wing = WingModel(
        span=p.b,
        chord_at=p.chord_at,
        x_le_at=p.x_le_at,
        twist_deg_at=p.twist_at,
        section_alpha_0_deg=polar.alpha_0_deg,
        section_a0_per_rad=polar.cl_alpha_per_rad,
    )
    return wing, p


def trim_at_cl(wing: WingModel, p: Planform, target_cl: float, washout: float, polar: AnalyticPolar):
    """Return α_trim, Cm_apex_trim, x_cg_trim, SM, x_NP, α_tip_eff."""
    # Sweep alphas to find α at target_cl, plus get cm_apex(α) shape
    alphas = list(range(0, 13))
    sweep = alpha_sweep(wing, alphas, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)
    cls = np.array([r.CL for r in sweep])
    cms = np.array([r.cm_about_apex for r in sweep])
    a_arr = np.array(alphas, dtype=float)

    # Linear fit on the relevant portion
    mask = (a_arr >= ALPHA_FIT_RANGE[0]) & (a_arr <= ALPHA_FIT_RANGE[1])
    s_cl, b_cl = np.polyfit(a_arr[mask], cls[mask], 1)
    s_cm, b_cm = np.polyfit(a_arr[mask], cms[mask], 1)
    alpha_trim = (target_cl - b_cl) / s_cl
    cm_apex_trim = s_cm * alpha_trim + b_cm

    # Trim CG location: Cm_apex + (x_cg/MAC)·CL = 0  →  x_cg = -Cm_apex/CL · MAC
    x_cg_trim = -cm_apex_trim / target_cl * p.mac

    # Neutral point (use full Weissinger NP, more accurate than fit slope ratio)
    x_np, _ = neutral_point(wing, alphas_deg=(2.0, 6.0), n_panels_per_side=N_PANELS,
                             S_ref=p.S, mac_ref=p.mac)
    sm = (x_np - x_cg_trim) / p.mac

    # Effective angle at tip — pull from a fresh solve at α_trim
    r = solve(wing, alpha_trim, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)
    # Use the outermost panel α_eff = α_trim + twist + α_ind_tip - α_0
    a_ind_tip_deg = r.alpha_induced_deg[-1]
    twist_tip = -washout
    alpha_tip_eff_deg = alpha_trim + twist_tip + a_ind_tip_deg - polar.alpha_0_deg

    return {
        "alpha_trim": alpha_trim,
        "cm_apex_trim": cm_apex_trim,
        "x_cg_trim": x_cg_trim,
        "x_np": x_np,
        "static_margin": sm,
        "alpha_tip_eff": alpha_tip_eff_deg,
        "alpha_stall": polar.alpha_stall_deg,
    }


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    polar = AnalyticPolar()
    rows = []

    print("# Trim + washout iteration — MANTA")
    print()
    print(f"Section: {polar.name}, a0 = {polar.cl_alpha_per_rad:.3f}/rad, α₀ = {polar.alpha_0_deg:.2f}°, "
          f"α_stall = {polar.alpha_stall_deg:.2f}°, Cm0 = {polar.cm0:.4f}")
    print()
    print("## Trim at design CL = 0.5\n")
    print("| Washout | α_trim | Cm_apex_trim | x_cg_trim (m) | x_NP (m) | SM (% MAC) | α_tip_eff (°) | Tip stall margin (°) |")
    print("|---|---|---|---|---|---|---|---|")
    for w in WASHOUTS_DEG:
        wing, p = _build_wing(w, polar)
        t = trim_at_cl(wing, p, 0.5, w, polar)
        margin = t["alpha_stall"] - t["alpha_tip_eff"]
        rows.append((w, 0.5, t["alpha_trim"], t["cm_apex_trim"], t["x_cg_trim"],
                     t["x_np"], t["static_margin"]*100, t["alpha_tip_eff"], margin))
        print(f"| {w:4.1f}°   | {t['alpha_trim']:5.2f}° | {t['cm_apex_trim']:+.4f}     "
              f"|  {t['x_cg_trim']:5.4f}    |  {t['x_np']:5.4f} |  "
              f"{t['static_margin']*100:6.2f}   |    {t['alpha_tip_eff']:+5.2f}    |     "
              f"{margin:5.2f}        |")

    # Sensitivity at washout = 5° to design CL changes
    print()
    print("## Trim sensitivity at washout = 5° across design CL (cruise vs slow vs fast)\n")
    print("| Design CL | α_trim | x_cg_trim | SM (% MAC) | α_tip_eff |")
    print("|---|---|---|---|---|")
    wing5, p5 = _build_wing(5.0, polar)
    for cl_d in [0.30, 0.40, 0.50, 0.60, 0.70]:
        t = trim_at_cl(wing5, p5, cl_d, 5.0, polar)
        print(f"| {cl_d:.2f}      | {t['alpha_trim']:5.2f}° | {t['x_cg_trim']:.4f}    |  "
              f"{t['static_margin']*100:6.2f}   |   {t['alpha_tip_eff']:+5.2f}   |")

    # Save CSV
    with (out_dir / "washout_sweep.csv").open("w") as f:
        f.write("washout_deg,design_CL,alpha_trim_deg,cm_apex_trim,x_cg_trim_m,"
                "x_NP_m,static_margin_pct,alpha_tip_eff_deg,tip_stall_margin_deg\n")
        for r in rows:
            f.write(",".join(f"{x:.5f}" for x in r) + "\n")

    # Plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        ws = np.array([r[0] for r in rows])
        sms = np.array([r[6] for r in rows])
        x_cgs = np.array([r[4] for r in rows])
        margins = np.array([r[8] for r in rows])

        fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
        axes[0].plot(ws, sms, "o-")
        axes[0].axhline(5, color="grey", linestyle=":", label="5 % SM (acceptable lower)")
        axes[0].axhline(15, color="grey", linestyle=":", label="15 % SM (acceptable upper)")
        axes[0].set_xlabel("washout (°)")
        axes[0].set_ylabel("static margin (% MAC)")
        axes[0].set_title("Static margin vs washout (CL=0.5)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(fontsize=8)

        axes[1].plot(ws, x_cgs, "o-")
        axes[1].set_xlabel("washout (°)")
        axes[1].set_ylabel("x_CG_trim (m, aft of root LE)")
        axes[1].set_title("Required CG location for trim")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(ws, margins, "o-")
        axes[2].axhline(2, color="grey", linestyle=":", label="2° margin floor")
        axes[2].set_xlabel("washout (°)")
        axes[2].set_ylabel("tip stall margin (°)")
        axes[2].set_title("α_stall − α_tip_eff at trim")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(out_dir / "washout_plot.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass

    # Recommendation
    print()
    print("## Recommended washout")
    print()
    # pick the row with the best balance: SM in [5%, 15%], tip margin > 2°, smallest α_trim
    best = None
    for r in rows:
        sm_pct = r[6]
        margin = r[8]
        if 5.0 <= sm_pct <= 15.0 and margin >= 2.0:
            score = abs(sm_pct - 10.0) + (10.0 - margin if margin < 10 else 0)
            if best is None or score < best[0]:
                best = (score, r)
    if best:
        r = best[1]
        print(f"Pick: **washout = {r[0]:.1f}°**.")
        print(f"  Static margin: {r[6]:.2f} % MAC")
        print(f"  x_CG (aft of apex): {r[4]:.4f} m  (= {r[4]/Planform().mac:.3f}·MAC)")
        print(f"  α_trim: {r[2]:.2f}°,  α_tip_eff: {r[7]:+.2f}°,  margin to stall: {r[8]:.2f}°")
    else:
        print("No washout in the sweep meets both SM and stall-margin gates. Reopen architecture decision #4.")


if __name__ == "__main__":
    main()
