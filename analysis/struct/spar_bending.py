"""
Spar bending analysis for the MANTA wing.

Loads
-----
The spanwise lift distribution L'(y) is taken from the Weissinger lifting-line
solver at the design CL — that captures the actual loading shape (sweep,
taper, washout) rather than approximating with elliptical. Total lift over
both halves is scaled to n × m_total × g.

Decomposition
-------------
Lift acts along the c/4 line of the wing. The two-spar layout shares this
moment between the front and rear spars by their chordwise position:
    front spar at x_front (≈ 0.20·c)
    rear  spar at x_rear  (≈ 0.65·c)
    lift  AC  at x_AC    (≈ 0.25·c, slightly aft for cambered/swept sections)

By force balance, the front-spar load fraction is:
    f_front = (x_rear - x_AC) / (x_rear - x_front)
With x_front = 0.20, x_rear = 0.65, x_AC = 0.25:
    f_front = (0.65 - 0.25) / (0.65 - 0.20) = 0.40 / 0.45 = 0.889

So the front spar carries ~89 % of the bending load when the AC is at 25 %
chord. We treat this as the nominal split; the rear spar carries the
remaining 11 % plus torsion (not analyzed here).

Bending moment + stress
-----------------------
Each spar is treated as a cantilever from the root. The bending moment at
spanwise station y is the integral of (η − y) × q(η) dη from y to b/2,
where q is the per-spar load per unit span.

Stress at any station: σ = M · r_outer / I, with I from the spar model.

Buckling (Brazier)
------------------
Thin-walled circular CFRP tubes under bending fail by local cross-section
flattening (Brazier collapse) at moments well below the simple bending-
strength prediction. Brazier's critical moment for an isotropic thin tube
(Brazier 1927; Calladine 1983):

    M_Brazier = (2·sqrt(2) / 9) · π · E · t · r²  ≈  0.987 · E · t · r²

For laminated CFRP this overpredicts by ~30 % (Cecchini 2005); we apply a
0.70 knockdown.

Outputs
-------
- console: stress and safety-factor table at root, mid, tip for both spars,
  at 1 g, 3 g limit, 4.5 g ultimate.
- out/bending_results.md: same in Markdown.
- out/bending_curves.png: M(y), σ(y).
- out/sensitivity_wall.csv: stress at root vs wall thickness sweep.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.airfoil.polar_analytic import AnalyticPolar  # noqa: E402
from analysis.aero.planform.geometry import Planform  # noqa: E402
from analysis.aero.weissinger.weissinger import WingModel, solve  # noqa: E402

from analysis.struct.materials import CFRPUDTube  # noqa: E402
from analysis.struct.spar_model import (  # noqa: E402
    TelescopingSpar,
    WingSparSet,
    default_front_spar,
    default_rear_spar,
)


# --- Configuration -------------------------------------------------------

DESIGN_CL = 0.50
PILOT_MASS_DESIGN = 95.0    # use the upper end of the BRIEF envelope for sizing
WING_MASS = 15.5
RIG_MASS = 8.0
N_PANELS = 60

LOAD_CASES = {
    "1g cruise":      1.0,
    "3g limit":       3.0,
    "4.5g ultimate":  4.5,
}

# Chord-wise spar positions and AC location (from BRIEF + airfoil polar)
X_FRONT_OVER_C = 0.20
X_REAR_OVER_C  = 0.65
X_AC_OVER_C    = 0.25


@dataclass
class SparBendingResult:
    spar_name: str
    n_load: float
    y_stations: np.ndarray
    M_Nm: np.ndarray            # bending moment at each station
    sigma_MPa: np.ndarray       # max-fiber stress at each station
    M_Brazier_Nm: np.ndarray    # critical buckling moment
    SF_strength: np.ndarray     # σ_design_compression / σ
    SF_buckling: np.ndarray     # M_Brazier / M


def span_load_from_weissinger(p: Planform) -> tuple[np.ndarray, np.ndarray]:
    """Return (y, q_normalized) from Weissinger at design CL.

    q_normalized = cl_section · chord / S_ref so ∫ q · dy / b = ∫ cl·c·dy / (S·b)
    Total lift integral of q · dy across the full span equals 2·b/(2·something)…

    Use the cleaner approach: span-loading shape function L'(y) / L_total
    such that ∫ L'(y) dy = L_total. Convert from cl·c by:
        L'(y) = q · cl(y) · c(y) where q = 1/2 ρ V²
        L_total = q · S · CL
        L'(y) / L_total = cl·c / (S · CL)
    """
    polar = AnalyticPolar()
    wing = WingModel(
        span=p.b,
        chord_at=p.chord_at,
        x_le_at=p.x_le_at,
        twist_deg_at=p.twist_at,
        section_alpha_0_deg=polar.alpha_0_deg,
        section_a0_per_rad=polar.cl_alpha_per_rad,
    )
    # Find α at design CL by linear fit
    r0 = solve(wing, 0.0, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)
    r4 = solve(wing, 4.0, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)
    cla = (r4.CL - r0.CL) / np.deg2rad(4.0)
    cl0 = r0.CL
    alpha_design = (DESIGN_CL - cl0) / cla * 180 / np.pi
    r = solve(wing, alpha_design, n_panels_per_side=N_PANELS, S_ref=p.S, mac_ref=p.mac)
    y = r.y
    L_per_unit_lift = r.cl_section * r.chord / (p.S * r.CL)  # ∫ this dy = 1
    return y, L_per_unit_lift


def per_spar_load_distribution(
    L_total_N: float,
    f_front: float,
    y: np.ndarray,
    L_norm: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return q_front(y), q_rear(y) — load per unit span on each spar [N/m]."""
    L_per_unit_span = L_total_N * L_norm
    return f_front * L_per_unit_span, (1.0 - f_front) * L_per_unit_span


def cantilever_bending_moment(y_half: np.ndarray, q_half: np.ndarray) -> np.ndarray:
    """Cantilever bending moment along a single half-wing.

    y_half : 1D ascending stations 0 → b/2.
    q_half : load per unit span at those stations [N/m].
    Returns M(y) [N·m] — the moment at station y about that station, due
    to all load outboard of y.
    """
    n = len(y_half)
    M = np.zeros(n)
    for i in range(n):
        # Moment at y_i = ∫_{y_i}^{y_max} (η - y_i) · q(η) dη
        eta = y_half[i:]
        q_eta = q_half[i:]
        if len(eta) >= 2:
            M[i] = np.trapezoid((eta - y_half[i]) * q_eta, eta)
    return M


def fold_to_half(y: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Take a full-span y array (negative-to-positive) and fold by symmetry.

    Returns (y_half, q_half) on [0, b/2] with q averaged across the two
    halves (so any small numerical asymmetry is washed out).
    """
    pos_mask = y >= 0
    y_pos = y[pos_mask]
    q_pos = q[pos_mask]

    # Interpolate negative side onto positive y stations
    neg_mask = y < 0
    if neg_mask.any():
        y_neg_mirror = -y[neg_mask][::-1]
        q_neg_mirror = q[neg_mask][::-1]
        # Resample onto y_pos
        q_neg_on_pos = np.interp(y_pos, y_neg_mirror, q_neg_mirror)
        q_avg = 0.5 * (q_pos + q_neg_on_pos)
    else:
        q_avg = q_pos
    return y_pos, q_avg


def stress_along_spar(spar: TelescopingSpar, y_half_axis: np.ndarray, M: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert M(y) into max-fiber stress σ(y) using the active stage at y.

    Returns (sigma_Pa, M_Brazier_Nm).
    """
    n = len(y_half_axis)
    sigma = np.zeros(n)
    M_b = np.zeros(n)
    cfrp = CFRPUDTube()
    knock_buckling = 0.70  # CFRP Brazier knockdown
    for i, y in enumerate(y_half_axis):
        s = spar.section_at(min(y, spar.total_length_m - 1e-6))
        r = s.outer_diameter_m / 2
        sigma[i] = M[i] * r / s.I_m4
        # Brazier critical moment, with CFRP knockdown
        M_brazier_iso = (2.0 * np.sqrt(2.0) / 9.0) * np.pi * cfrp.E_axial * s.wall_thickness_m * r * r
        M_b[i] = knock_buckling * M_brazier_iso
    return sigma, M_b


def analyze_load_case(
    n_load: float,
    p: Planform,
    spars: WingSparSet,
    y_full: np.ndarray,
    L_norm: np.ndarray,
) -> dict[str, SparBendingResult]:
    L_total = (PILOT_MASS_DESIGN + WING_MASS + RIG_MASS) * 9.80665 * n_load

    # Chord-wise lift split between spars
    f_front = (X_REAR_OVER_C - X_AC_OVER_C) / (X_REAR_OVER_C - X_FRONT_OVER_C)

    q_front_full, q_rear_full = per_spar_load_distribution(L_total, f_front, y_full, L_norm)
    y_half, q_front_half = fold_to_half(y_full, q_front_full)
    _,      q_rear_half  = fold_to_half(y_full, q_rear_full)

    M_front = cantilever_bending_moment(y_half, q_front_half)
    M_rear  = cantilever_bending_moment(y_half, q_rear_half)

    sig_front, Mb_front = stress_along_spar(spars.front, y_half, M_front)
    sig_rear,  Mb_rear  = stress_along_spar(spars.rear,  y_half, M_rear)

    cfrp = CFRPUDTube()
    sigma_des_lim = cfrp.sigma_design_compression_limit  # Pa

    sf_strength_front = sigma_des_lim / np.maximum(sig_front, 1.0)
    sf_strength_rear  = sigma_des_lim / np.maximum(sig_rear, 1.0)
    sf_buck_front = Mb_front / np.maximum(M_front, 1.0)
    sf_buck_rear  = Mb_rear  / np.maximum(M_rear, 1.0)

    return {
        "front": SparBendingResult(
            spar_name="front",
            n_load=n_load,
            y_stations=y_half,
            M_Nm=M_front,
            sigma_MPa=sig_front / 1e6,
            M_Brazier_Nm=Mb_front,
            SF_strength=sf_strength_front,
            SF_buckling=sf_buck_front,
        ),
        "rear": SparBendingResult(
            spar_name="rear",
            n_load=n_load,
            y_stations=y_half,
            M_Nm=M_rear,
            sigma_MPa=sig_rear / 1e6,
            M_Brazier_Nm=Mb_rear,
            SF_strength=sf_strength_rear,
            SF_buckling=sf_buck_rear,
        ),
    }


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    p = Planform()
    spars = WingSparSet()
    cfrp = CFRPUDTube()

    y_full, L_norm = span_load_from_weissinger(p)

    print("# Spar bending — MANTA")
    print()
    print(f"Design pilot mass         : {PILOT_MASS_DESIGN} kg")
    print(f"Wing + rig mass           : {WING_MASS + RIG_MASS} kg")
    print(f"Total mass for sizing     : {PILOT_MASS_DESIGN + WING_MASS + RIG_MASS} kg")
    print(f"Material                  : {cfrp.name}")
    print(f"σ_design_compression_limit: {cfrp.sigma_design_compression_limit/1e6:.1f} MPa")
    f_front = (X_REAR_OVER_C - X_AC_OVER_C) / (X_REAR_OVER_C - X_FRONT_OVER_C)
    print(f"Front-spar load fraction  : {f_front:.3f} (rear = {1 - f_front:.3f})")
    print()

    rows = []
    print("## Stress & safety factors at the root station\n")
    print("| Load case      | Spar | M_root (N·m) | σ_root (MPa) | SF_strength | M_Brazier (N·m) | SF_buckling |")
    print("|---|---|---|---|---|---|---|")
    for label, n in LOAD_CASES.items():
        results = analyze_load_case(n, p, spars, y_full, L_norm)
        for spar_name in ("front", "rear"):
            r = results[spar_name]
            M0 = r.M_Nm[0]
            sig0 = r.sigma_MPa[0]
            Mb0 = r.M_Brazier_Nm[0]
            sf_s = r.SF_strength[0]
            sf_b = r.SF_buckling[0]
            print(f"| {label:14s} | {spar_name:5s} | {M0:8.1f}    | {sig0:8.1f}     | {sf_s:5.2f}     "
                  f"| {Mb0:8.1f}      | {sf_b:5.2f}     |")
            rows.append((label, n, spar_name, M0, sig0, Mb0, sf_s, sf_b))

    # CSV: full M(y), σ(y) tables for each (case, spar)
    with (out_dir / "bending_full.csv").open("w") as f:
        f.write("load_case,n_load,spar,y_m,M_Nm,sigma_MPa,SF_strength,SF_buckling\n")
        for label, n in LOAD_CASES.items():
            results = analyze_load_case(n, p, spars, y_full, L_norm)
            for spar_name in ("front", "rear"):
                r = results[spar_name]
                for i in range(len(r.y_stations)):
                    f.write(f"{label},{n},{spar_name},{r.y_stations[i]:.4f},"
                            f"{r.M_Nm[i]:.3f},{r.sigma_MPa[i]:.3f},"
                            f"{r.SF_strength[i]:.3f},{r.SF_buckling[i]:.3f}\n")

    # Wall-thickness sensitivity at 3g limit (BRIEF OD progression)
    print()
    print("## Wall-thickness sensitivity (front spar, 3g limit, root stress)")
    print("(BRIEF OD progression 40 / 32 / 25 mm — only wall varied)\n")
    print("| Wall (mm) | Front mass (kg/side) | σ_root (MPa) | SF_strength | SF_buckling |")
    print("|---|---|---|---|---|")
    for wall_mm in [1.5, 2.0, 2.5, 3.0]:
        wall = wall_mm / 1000
        front_alt = default_front_spar(wall=wall)
        rear_alt  = default_rear_spar(wall=wall)
        spars_alt = WingSparSet(front=front_alt, rear=rear_alt)
        results = analyze_load_case(3.0, p, spars_alt, y_full, L_norm)
        r = results["front"]
        m_kg = front_alt.mass_kg(cfrp)
        print(f"|   {wall_mm:.1f}    |   {m_kg:.4f}      |   {r.sigma_MPa[0]:.1f}      |  "
              f"{r.SF_strength[0]:.2f}      |  {r.SF_buckling[0]:.2f}     |")

    # OD sensitivity at fixed wall = 2.5 mm
    print()
    print("## Front-spar OD sensitivity at wall = 2.5 mm, 3g limit, root stress")
    print("(stage progression OD_root / OD_root·0.7 / 25 mm)\n")
    print("| OD_root (mm) | Front mass (kg/side) | σ_root (MPa) | SF_strength |")
    print("|---|---|---|---|")
    from analysis.struct.spar_model import SparStage, TelescopingSpar
    for od_root_mm in [40, 50, 60, 65, 70, 80]:
        od_root = od_root_mm / 1000
        od_mid = od_root * 0.70
        od_tip = 0.025
        L = 3.7 / 3.0 + 2 * 0.025
        wall = 0.0025
        front_alt = TelescopingSpar(
            name="front",
            stages=(
                SparStage("front_root", od_root, wall, L),
                SparStage("front_mid",  od_mid,  wall, L),
                SparStage("front_tip",  od_tip,  wall, L),
            ),
        )
        spars_alt = WingSparSet(front=front_alt, rear=default_rear_spar())
        results = analyze_load_case(3.0, p, spars_alt, y_full, L_norm)
        r = results["front"]
        m_kg = front_alt.mass_kg(cfrp)
        print(f"|    {od_root_mm:3d}      |    {m_kg:.4f}        |    {r.sigma_MPa[0]:.1f}     |   "
              f"{r.SF_strength[0]:.2f}     |")

    # Recommended sizing: find the smallest OD that gives SF_strength ≥ 1.5
    # at 3g limit with wall = 2.5 mm.
    print()
    print("## Recommended sizing (target SF_strength ≥ 1.5 at 3g limit, wall = 2.5 mm)\n")
    target_sf = 1.5
    for od_root_mm in range(40, 100, 1):
        od_root = od_root_mm / 1000
        od_mid = od_root * 0.70
        od_tip = 0.025
        L = 3.7 / 3.0 + 2 * 0.025
        wall = 0.0025
        front_alt = TelescopingSpar(
            name="front",
            stages=(
                SparStage("front_root", od_root, wall, L),
                SparStage("front_mid",  od_mid,  wall, L),
                SparStage("front_tip",  od_tip,  wall, L),
            ),
        )
        spars_alt = WingSparSet(front=front_alt, rear=default_rear_spar())
        results = analyze_load_case(3.0, p, spars_alt, y_full, L_norm)
        r = results["front"]
        if r.SF_strength[0] >= target_sf:
            m_kg = front_alt.mass_kg(cfrp)
            d_mass = m_kg - default_front_spar().mass_kg(cfrp)
            print(f"  Smallest OD_root meeting SF ≥ {target_sf}: **{od_root_mm} mm** ")
            print(f"  Stage progression: {od_root_mm} / {od_root_mm*0.7:.0f} / 25 mm, wall 2.5 mm")
            print(f"  Front-spar mass per side: {m_kg:.3f} kg  (BRIEF default {default_front_spar().mass_kg(cfrp):.3f} kg, Δ = +{d_mass:.3f} kg)")
            print(f"  σ_root at 3g: {r.sigma_MPa[0]:.1f} MPa, SF_strength = {r.SF_strength[0]:.2f}")
            break
    else:
        print("  No OD ≤ 100 mm meets the target — re-examine architecture.")

    # Plot M(y) and σ(y) at 3g limit
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        results_3g = analyze_load_case(3.0, p, spars, y_full, L_norm)
        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        for spar_name, color in (("front", "tab:blue"), ("rear", "tab:red")):
            r = results_3g[spar_name]
            axes[0].plot(r.y_stations, r.M_Nm, label=f"{spar_name} spar", color=color)
            axes[1].plot(r.y_stations, r.sigma_MPa, label=f"{spar_name} σ", color=color)
        axes[1].axhline(cfrp.sigma_design_compression_limit / 1e6, color="black",
                        linestyle="--", label="σ_design_lim (compression)")
        axes[0].set_xlabel("y from root (m)")
        axes[0].set_ylabel("Bending moment (N·m)")
        axes[0].set_title("Bending moment vs spanwise station — 3 g limit")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].set_xlabel("y from root (m)")
        axes[1].set_ylabel("Max-fiber stress (MPa)")
        axes[1].set_title("Stress vs spanwise station — 3 g limit")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        fig.tight_layout()
        fig.savefig(out_dir / "bending_curves.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
