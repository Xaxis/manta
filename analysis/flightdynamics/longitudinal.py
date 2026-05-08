"""
Linearized longitudinal flight dynamics for the MANTA wing.

State vector  x = [u, w, q, θ]ᵀ  (perturbations from trim).
Control       δ_e  (symmetric flaperon deflection, treated as an "elevator").

Stability derivatives are sourced from the upstream Weissinger run + CD0
build-up. Where the Weissinger code doesn't directly give a number (M_q
in particular), the value is taken from a typical-AR-and-sweep approximation
with the reference cited in-line.

What the script does
--------------------
- Solves trim at design CL (V_bg ≈ 16 m/s by default; override on CLI).
- Builds the dimensional A, B matrices.
- Computes the four longitudinal eigenvalues; identifies short-period and
  phugoid modes; reports ω_n, ζ.
- Sweeps the static margin from −2 % to +10 % MAC (i.e. CG offset) and
  prints how the modes shift — this is the pilot-CG-perturbation scan.
- Outputs a CSV + PNG.

References
----------
- Etkin & Reid, *Dynamics of Flight*, 3rd ed., Ch. 4 (linearization and
  stability derivatives).
- McRuer, Ashkenas, Graham, *Aircraft Dynamics and Automatic Control*,
  §10 (handling-quality reference values for ω_n, ζ).
- Roskam, *Airplane Flight Dynamics and Automatic Flight Controls*, Vol I,
  Tables 5.1 / 5.2 (typical Cm_q for swept wings).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.aero.airfoil.polar_analytic import AnalyticPolar  # noqa: E402
from analysis.aero.lift_drag.cd0 import Cd0Buildup  # noqa: E402
from analysis.aero.planform.geometry import Planform  # noqa: E402
from analysis.aero.weissinger.weissinger import WingModel, alpha_sweep  # noqa: E402


@dataclass
class TrimState:
    V: float          # m/s
    rho: float        # kg/m^3
    m: float          # kg total mass
    g: float          # m/s²
    CL: float         # trim CL
    CD: float         # trim CD
    alpha_deg: float  # trim alpha
    static_margin: float  # fraction of MAC, positive = stable
    Iyy: float        # kg·m²
    S: float          # m²
    MAC: float        # m
    AR: float         # aspect ratio
    cl_alpha_per_rad: float
    cm_q_per_rad: float
    cm_de_per_rad: float  # control derivative, see below


def _solve_alpha_for_cl(wing: WingModel, target_cl: float, p: Planform) -> tuple[float, float]:
    """Linear fit on small-α sweep to invert CL→α; returns (alpha_deg, CL_α)."""
    sweep = alpha_sweep(wing, [0.0, 4.0], n_panels_per_side=40, S_ref=p.S, mac_ref=p.mac)
    cl0, cl1 = sweep[0].CL, sweep[1].CL
    cl_alpha = (cl1 - cl0) / math.radians(4.0)
    alpha_design_deg = (target_cl - cl0) / cl_alpha * 180 / math.pi
    return alpha_design_deg, cl_alpha


def trim(V: float = 16.0, m_total: float = 105.0,
         pilot_mass: float = 82.5, rho: float = 1.225,
         Iyy: float = 25.0, washout_deg: float = 6.0) -> TrimState:
    """Solve trim at the given airspeed and mass."""
    p = Planform(washout_deg=washout_deg)
    polar = AnalyticPolar()
    wing = WingModel(
        span=p.b, chord_at=p.chord_at, x_le_at=p.x_le_at,
        twist_deg_at=p.twist_at,
        section_alpha_0_deg=polar.alpha_0_deg,
        section_a0_per_rad=polar.cl_alpha_per_rad,
    )
    g = 9.80665
    q_bar = 0.5 * rho * V * V
    CL_trim = m_total * g / (q_bar * p.S)
    alpha_design_deg, cl_alpha = _solve_alpha_for_cl(wing, CL_trim, p)
    cd0 = Cd0Buildup().total(0.20)[0]
    CD_trim = cd0 + CL_trim * CL_trim / (math.pi * p.aspect_ratio * 0.95)
    # Static margin from Weissinger NP at 6° washout (~5.4 % MAC at design CL=0.5;
    # scales roughly with CL — see analysis/aero/trim/out for the curve).
    SM_at_CL05 = 0.054
    # Approximate scaling: SM_at_CL = SM_at_CL05 * (0.5 / CL_trim)
    SM = SM_at_CL05 * (0.5 / max(CL_trim, 0.1))

    # Cm_q: for a tailless swept wing AR=6.5, sweep 25°, Roskam table
    # gives Cm_q in the −2 to −5 /rad range. We use −3.5 as nominal.
    cm_q = -3.5

    # Cm_δe: control authority per rad of symmetric flaperon. Rough hand
    # calc: hinge at 0.75c, full-span flap on outer 50 % of half-span,
    # gives Cm_δe ≈ −1.0 to −1.5 /rad (depends on hinge moment and arm
    # to NP). Use −1.2 /rad as nominal.
    cm_de = -1.2

    return TrimState(
        V=V, rho=rho, m=m_total, g=g,
        CL=CL_trim, CD=CD_trim,
        alpha_deg=alpha_design_deg,
        static_margin=SM,
        Iyy=Iyy,
        S=p.S, MAC=p.mac, AR=p.aspect_ratio,
        cl_alpha_per_rad=cl_alpha,
        cm_q_per_rad=cm_q,
        cm_de_per_rad=cm_de,
    )


def state_space(t: TrimState) -> tuple[np.ndarray, np.ndarray]:
    """Return (A, B) for the longitudinal state-space at trim."""
    q_bar = 0.5 * t.rho * t.V * t.V
    qS = q_bar * t.S
    qScm = qS * t.MAC

    # Drag derivative wrt α: CD = CD0 + CL²/(π·AR·e)  →  dCD/dα = 2·k·CL·CL_α
    k_drag = 1.0 / (math.pi * t.AR * 0.95)
    CD_alpha = 2.0 * k_drag * t.CL * t.cl_alpha_per_rad

    # Wrt velocity (small-angle, level trim approximation)
    X_u = -2.0 * qS * t.CD / t.V / t.m
    X_w = -qS * CD_alpha / t.V / t.m         # 1/s
    Z_u = -2.0 * qS * t.CL / t.V / t.m
    Z_w = -qS * t.cl_alpha_per_rad / t.V / t.m
    M_u = 0.0  # Cm = 0 at trim
    M_w = -qScm * t.cl_alpha_per_rad * t.static_margin / t.V / t.Iyy
    M_q = qScm * t.MAC * t.cm_q_per_rad / (2.0 * t.V * t.Iyy)

    # Note: w-equation has +V·q; θ-equation has +q (kinematic).
    A = np.array([
        [X_u,    X_w,           -t.g,    0.0],
        [Z_u,    Z_w,            t.V,    0.0],
        [M_u,    M_w,            M_q,    0.0],
        [0.0,    0.0,            1.0,    0.0],
    ])

    # Control: symmetric flaperon δ_e produces a pitching moment.
    Z_de = 0.0  # neglect lift contribution from elevon for first cut
    M_de = qScm * t.cm_de_per_rad / t.Iyy
    B = np.array([[0.0], [Z_de], [M_de], [0.0]])

    return A, B


def modes(A: np.ndarray) -> dict:
    """Identify short-period and phugoid modes from the eigenvalues of A."""
    eigvals = np.linalg.eigvals(A)
    # Sort by magnitude of imaginary part: short period has higher ω
    pairs = []
    seen = set()
    for i, val in enumerate(eigvals):
        if i in seen:
            continue
        # Find conjugate
        for j in range(i + 1, len(eigvals)):
            if j in seen:
                continue
            if abs(eigvals[j] - np.conj(val)) < 1e-9:
                pairs.append((val, eigvals[j]))
                seen.add(i)
                seen.add(j)
                break
        else:
            pairs.append((val, None))
            seen.add(i)
    # Now categorize
    out = {"raw_eigvals": eigvals.tolist(), "modes": []}
    for pair in pairs:
        v = pair[0]
        omega_n = abs(v)
        sigma = v.real
        if abs(v.imag) < 1e-9:
            zeta = -1.0 if sigma < 0 else 1.0
            mode = {"type": "real", "lambda": v, "tau_s": -1.0 / sigma if abs(sigma) > 1e-9 else float("inf")}
        else:
            zeta = -sigma / omega_n
            mode = {"type": "oscillatory", "lambda": v, "omega_n": omega_n, "zeta": zeta,
                    "period": 2 * math.pi / abs(v.imag) if abs(v.imag) > 1e-9 else float("inf")}
        out["modes"].append(mode)
    # Sort: oscillatory modes by ω_n descending; real modes go after
    osc = sorted([m for m in out["modes"] if m["type"] == "oscillatory"],
                 key=lambda m: m["omega_n"], reverse=True)
    real_modes = [m for m in out["modes"] if m["type"] == "real"]
    out["modes"] = osc + real_modes
    return out


def cg_offset_to_static_margin(t: TrimState, dx_cg: float) -> float:
    """Convert a forward CG offset (positive = forward of trim CG, m) into
    a static margin change in fraction of MAC.
    """
    return t.static_margin + dx_cg / t.MAC


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    t = trim()
    A, B = state_space(t)
    m = modes(A)

    print("# MANTA longitudinal dynamics")
    print()
    print(f"Trim:  V = {t.V:.2f} m/s,  CL = {t.CL:.3f},  CD = {t.CD:.4f}")
    print(f"       α_trim = {t.alpha_deg:.2f}°,  CL_α = {t.cl_alpha_per_rad:.3f}/rad")
    print(f"       static margin = {t.static_margin*100:.2f} % MAC")
    print(f"       Iyy = {t.Iyy} kg·m², MAC = {t.MAC:.3f} m")
    print()
    print("State-space A:")
    np.set_printoptions(precision=4, suppress=True, linewidth=120)
    print(A)
    print()
    print("Eigenvalues + modes:")
    for mm in m["modes"]:
        if mm["type"] == "oscillatory":
            print(f"  oscillatory  λ = {mm['lambda']:+.4f}j   "
                  f"ω_n = {mm['omega_n']:.3f} rad/s ({mm['omega_n']/2/math.pi:.3f} Hz)   "
                  f"ζ = {mm['zeta']:.3f}   period = {mm['period']:.2f} s")
        else:
            print(f"  real         λ = {mm['lambda']:+.4f}    τ = {mm['tau_s']:+.3f} s")
    print()

    # CG offset sweep
    print("## CG offset sweep — pilot head/torso shift")
    print()
    print("| Δx_CG (m) | SM (% MAC) | short-period ω_n (rad/s) | ζ_sp | phugoid ω_n | ζ_ph | Status |")
    print("|---|---|---|---|---|---|---|")

    rows = []
    for dx_mm in (-50, -30, -20, -10, 0, +10, +20, +30, +50):
        dx = dx_mm / 1000.0
        SM_new = cg_offset_to_static_margin(t, dx)
        # Modify trim copy with new SM and rebuild A
        t2 = TrimState(**{**t.__dict__, "static_margin": SM_new})
        A2, _ = state_space(t2)
        m2 = modes(A2)
        osc = [mm for mm in m2["modes"] if mm["type"] == "oscillatory"]
        sp = osc[0] if len(osc) >= 1 else None
        ph = osc[1] if len(osc) >= 2 else None
        sp_str = f"{sp['omega_n']:.3f}" if sp else "n/a"
        sp_z = f"{sp['zeta']:+.3f}" if sp else "n/a"
        ph_str = f"{ph['omega_n']:.3f}" if ph else "n/a"
        ph_z = f"{ph['zeta']:+.3f}" if ph else "n/a"
        status = "STABLE" if SM_new > 0 and (sp and sp['zeta'] > 0) else (
                 "NEUTRAL" if abs(SM_new) < 1e-3 else "UNSTABLE")
        print(f"| {dx:+.3f} | {SM_new*100:+.2f} | {sp_str} | {sp_z} | {ph_str} | {ph_z} | {status} |")
        rows.append((dx_mm, SM_new * 100, sp['omega_n'] if sp else None,
                     sp['zeta'] if sp else None,
                     ph['omega_n'] if ph else None,
                     ph['zeta'] if ph else None,
                     status))

    with (out_dir / "cg_sweep.csv").open("w") as f:
        f.write("dx_cg_mm,SM_pct,sp_omega_n,sp_zeta,ph_omega_n,ph_zeta,status\n")
        for r in rows:
            f.write(",".join(str(x) if x is not None else "" for x in r) + "\n")

    print()
    print("Pilot CG perturbation context:")
    print("  Pilot mass fraction of total: {:.1%}".format(82.5 / 105.0))
    print("  ±50 mm shift of upper-body CG → ±{:.2f} mm shift of vehicle CG".format(50 * 82.5 / 105.0))
    veh_dx_m = 0.050 * 82.5 / 105.0
    print("  → ±{:.2f} % of MAC".format(veh_dx_m / t.MAC * 100))


if __name__ == "__main__":
    main()
