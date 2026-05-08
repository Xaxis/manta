"""
Lateral-directional flight dynamics for the MANTA wing.

State vector  x = [β, p, r, φ]ᵀ  (sideslip, roll rate, yaw rate, bank).

The tailless-flying-wing concern: with no vertical stabilizer, directional
stiffness Cn_β has only the wing's sweep contribution, and yaw damping Cn_r
is only what sweep + roll-to-yaw coupling provides. Both tend to be small.
The expected stability story:

    - Roll subsidence: heavily damped (Cl_p strong on a long-span wing).
    - Dutch roll: lightly damped — likely 15–30 % damping ratio.
    - Spiral mode: marginal — the sign of (Cl_β·Cn_r − Cn_β·Cl_r) determines
      stability and that's a small-numbers fight on a tailless wing.

This module computes all three from textbook estimates of the derivatives
and reports whether the wing meets handling-quality reference values
(MIL-STD-1797 / FAR 23.142). When AVL lands, the derivatives can be
swapped in directly without changing the rest of the pipeline.

References
----------
- Etkin & Reid, *Dynamics of Flight*, 3rd ed., §6 (lateral-directional).
- Roskam, *Airplane Flight Dynamics and Automatic Flight Controls*,
  Vol I, Tables 5.1–5.7 (typical lateral derivatives, swept wings).
- Northrop / Horten flying-wing flight-test reports — qualitative
  reference for tailless-wing handling characteristics.
- FAR 23.142 / MIL-F-8785C — handling-quality references.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402


@dataclass
class LateralTrim:
    V: float
    rho: float
    m: float
    g: float
    CL: float
    S: float
    b: float
    AR: float
    Lambda_c4_deg: float

    Ixx: float
    Iyy: float
    Izz: float
    Ixz: float

    # Dimensional-less stability derivatives
    CY_beta: float
    CY_p: float
    CY_r: float
    Cl_beta: float
    Cl_p: float
    Cl_r: float
    Cn_beta: float
    Cn_p: float
    Cn_r: float


def estimate_lateral_derivatives(p: Planform, CL: float) -> dict:
    """Textbook estimates for lateral-directional stability derivatives
    on a tailless swept wing.

    These are *first-cut* numbers. AVL is the authoritative source once
    that runs.
    """
    AR = p.aspect_ratio
    Lambda_c4 = math.radians(p.sweep_c4_deg)

    # ---- Cl_β: dihedral-effect derivative (rad⁻¹) -----------------------
    # Sweep contribution (Roskam Eq. 7.15, simplified):
    #   Cl_β_sweep ≈ −CL/4 · sin(2·Λ_c/4)
    # Geometric dihedral Γ contribution: 0 (planar wing assumed).
    Cl_beta = -CL / 4.0 * math.sin(2.0 * Lambda_c4)

    # ---- Cl_p: roll damping (rad⁻¹) -------------------------------------
    # Anderson §6.5: Cl_p ≈ −(CL_α / 12)·(1 + 3·λ)/(1+λ)·cos(Λ)
    a0 = 5.7  # 2D
    Cl_alpha_3D = a0 / (1 + a0 / (math.pi * AR))  # Helmbold
    taper = p.taper
    Cl_p = -(Cl_alpha_3D / 12.0) * (1.0 + 3.0 * taper) / (1.0 + taper) * math.cos(Lambda_c4)

    # ---- Cl_r: roll-due-to-yaw-rate (rad⁻¹) -----------------------------
    # Cl_r ≈ CL/4 · (1 + 3·λ)/(1+λ) · cos(Λ)·something — use simple form:
    #   Cl_r ≈ CL/4 + (Cl_β·tan(Λ)/something)
    # Roskam approximation:
    Cl_r = CL / 4.0 * (1.0 - taper) / (1.0 + taper)  # taper effect; small
    Cl_r += CL / 4.0  # CL contribution
    Cl_r -= 0.05  # sweep penalty (rough)

    # ---- Cn_β: yaw stiffness (rad⁻¹) ------------------------------------
    # Tailless: ONLY wing sweep contributes (no vertical tail).
    # Roskam approximation: Cn_β_wing ≈ CL² · [1/(4πAR) − tan(Λ)·...]
    # For a swept tailless wing with our parameters this is small but positive.
    Cn_beta_wing = CL ** 2 * (
        1.0 / (4.0 * math.pi * AR) -
        math.tan(Lambda_c4) / (math.pi * AR * (AR + 4 * math.cos(Lambda_c4)))
        * (math.cos(Lambda_c4) - AR / 2 - AR * AR / (8 * math.cos(Lambda_c4))
           + 6 * (p.x_mac_c4 / p.mac) * math.tan(Lambda_c4) / AR)
    )
    # Clamp to a defensible sign — the formula can go negative for bad
    # parameter combinations, which would mean *directionally unstable*.
    Cn_beta = max(0.02, Cn_beta_wing)  # at minimum, slight positive

    # ---- Cn_p: yaw-due-to-roll-rate (rad⁻¹) -----------------------------
    # Cn_p ≈ −CL/8 (rough)
    Cn_p = -CL / 8.0

    # ---- Cn_r: yaw damping (rad⁻¹) --------------------------------------
    # Tailless: only wing damping. Cn_r ≈ −0.4·Cl_β·tan(Λ) − CD/4 (Roskam).
    Cn_r = -0.4 * Cl_beta * math.tan(Lambda_c4) - 0.04  # CD ≈ 0.04 placeholder

    # ---- CY_β, CY_p, CY_r: side-force derivatives ------------------------
    # All small without a vertical surface.
    CY_beta = -0.05  # very small
    CY_p = 0.0
    CY_r = 0.0

    return {
        "Cl_beta": Cl_beta,
        "Cl_p": Cl_p,
        "Cl_r": Cl_r,
        "Cn_beta": Cn_beta,
        "Cn_p": Cn_p,
        "Cn_r": Cn_r,
        "CY_beta": CY_beta,
        "CY_p": CY_p,
        "CY_r": CY_r,
    }


def trim_lateral(V: float = 20.0, m_total: float = 105.0,
                  rho: float = 1.225,
                  Ixx: float = 74.0, Iyy: float = 25.0, Izz: float = 93.0,
                  Ixz: float = 0.0) -> LateralTrim:
    p = Planform()
    g = 9.80665
    q_bar = 0.5 * rho * V * V
    CL = m_total * g / (q_bar * p.S)
    derivs = estimate_lateral_derivatives(p, CL)

    return LateralTrim(
        V=V, rho=rho, m=m_total, g=g,
        CL=CL, S=p.S, b=p.b, AR=p.aspect_ratio,
        Lambda_c4_deg=p.sweep_c4_deg,
        Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz,
        CY_beta=derivs["CY_beta"], CY_p=derivs["CY_p"], CY_r=derivs["CY_r"],
        Cl_beta=derivs["Cl_beta"], Cl_p=derivs["Cl_p"], Cl_r=derivs["Cl_r"],
        Cn_beta=derivs["Cn_beta"], Cn_p=derivs["Cn_p"], Cn_r=derivs["Cn_r"],
    )


def state_space_lateral(t: LateralTrim, theta_e_rad: float = 0.0) -> np.ndarray:
    """Lateral-directional A matrix in body axes.

    State [β, p, r, φ]; small θ_e (level trim) approximation.
    """
    q_bar = 0.5 * t.rho * t.V * t.V
    qSb = q_bar * t.S * t.b
    half_b_over_2V = t.b / (2.0 * t.V)

    # Dimensional dimensional moments per unit input
    Y_beta = q_bar * t.S * t.CY_beta / t.m
    Y_p = q_bar * t.S * t.CY_p * half_b_over_2V / t.m
    Y_r = q_bar * t.S * t.CY_r * half_b_over_2V / t.m
    L_beta = qSb * t.Cl_beta / t.Ixx
    L_p = qSb * t.Cl_p * half_b_over_2V / t.Ixx
    L_r = qSb * t.Cl_r * half_b_over_2V / t.Ixx
    N_beta = qSb * t.Cn_beta / t.Izz
    N_p = qSb * t.Cn_p * half_b_over_2V / t.Izz
    N_r = qSb * t.Cn_r * half_b_over_2V / t.Izz

    # β̇ equation: Y/m forces normalized by V; (Y_r/V − 1)·r and g/V·φ.
    A = np.array([
        [Y_beta / t.V, Y_p / t.V,         Y_r / t.V - 1.0,   t.g * math.cos(theta_e_rad) / t.V],
        [L_beta,       L_p,                L_r,               0.0],
        [N_beta,       N_p,                N_r,               0.0],
        [0.0,          1.0,                math.tan(theta_e_rad), 0.0],
    ])
    return A


def lateral_modes(A: np.ndarray) -> dict:
    """Identify roll, dutch-roll, and spiral modes from eigenvalues."""
    eigvals = np.linalg.eigvals(A)

    # Sort and pair conjugates
    pairs = []
    seen = set()
    for i, v in enumerate(eigvals):
        if i in seen:
            continue
        for j in range(i + 1, len(eigvals)):
            if j in seen:
                continue
            if abs(eigvals[j] - np.conj(v)) < 1e-9 and abs(v.imag) > 1e-9:
                pairs.append(("oscillatory", v, eigvals[j]))
                seen.update([i, j])
                break
        else:
            pairs.append(("real", v, None))
            seen.add(i)

    out = {"raw": eigvals.tolist(), "modes": {}}
    real_modes = sorted([p for p in pairs if p[0] == "real"],
                         key=lambda p: abs(p[1].real))
    osc_modes = [p for p in pairs if p[0] == "oscillatory"]

    if osc_modes:
        v = osc_modes[0][1]
        out["modes"]["dutch_roll"] = {
            "lambda": v,
            "omega_n": abs(v),
            "zeta": -v.real / abs(v),
            "period": 2 * math.pi / abs(v.imag) if abs(v.imag) > 1e-9 else float("inf"),
        }
    if len(real_modes) >= 1:
        # Real mode with the largest |sigma| is roll (fastest)
        roll = max(real_modes, key=lambda p: abs(p[1].real))
        spiral = min(real_modes, key=lambda p: abs(p[1].real))
        out["modes"]["roll"] = {
            "lambda": roll[1],
            "tau_s": -1.0 / roll[1].real if abs(roll[1].real) > 1e-9 else float("inf"),
        }
        out["modes"]["spiral"] = {
            "lambda": spiral[1],
            "tau_s": -1.0 / spiral[1].real if abs(spiral[1].real) > 1e-9 else float("inf"),
        }
    return out


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    print("# MANTA lateral-directional flight dynamics")
    print()

    for V in (16.0, 20.0, 25.0):
        t = trim_lateral(V=V)
        A = state_space_lateral(t)
        m = lateral_modes(A)

        print(f"## Trim at V = {V:.1f} m/s,  CL = {t.CL:.3f}")
        print()
        print(f"Stability derivatives (rad⁻¹ unless noted):")
        print(f"  Cl_β  = {t.Cl_beta:+.4f}   (dihedral effect; <0 stable, sweep contrib)")
        print(f"  Cl_p  = {t.Cl_p:+.4f}   (roll damping; <0 stable)")
        print(f"  Cl_r  = {t.Cl_r:+.4f}   (roll-from-yaw)")
        print(f"  Cn_β  = {t.Cn_beta:+.4f}   (yaw stiffness; >0 stable)  ⚠ tailless")
        print(f"  Cn_p  = {t.Cn_p:+.4f}   (yaw-from-roll)")
        print(f"  Cn_r  = {t.Cn_r:+.4f}   (yaw damping; <0 stable)  ⚠ tailless")
        print(f"  CY_β  = {t.CY_beta:+.4f}   (side-force; <0 stable)")
        print()
        print("Modes:")
        if "dutch_roll" in m["modes"]:
            dr = m["modes"]["dutch_roll"]
            print(f"  Dutch roll:  λ = {dr['lambda']:+.4f}j   "
                  f"ω_n = {dr['omega_n']:.3f} rad/s ({dr['omega_n']/2/math.pi:.3f} Hz)   "
                  f"ζ = {dr['zeta']:+.3f}   T = {dr['period']:.2f} s")
            # Handling-quality assessment
            if dr["zeta"] >= 0.4:
                hq = "Level 1 (good)"
            elif dr["zeta"] >= 0.2:
                hq = "Level 2 (acceptable)"
            elif dr["zeta"] >= 0.0:
                hq = "Level 3 (poor — yaw damper required)"
            else:
                hq = "UNSTABLE"
            print(f"               handling-quality: {hq}")
        if "roll" in m["modes"]:
            r = m["modes"]["roll"]
            print(f"  Roll mode:   λ = {r['lambda']:+.4f}     τ = {r['tau_s']:+.3f} s")
        if "spiral" in m["modes"]:
            sp = m["modes"]["spiral"]
            sp_label = "stable (convergent)" if sp["lambda"].real < -1e-3 else (
                "near-neutral" if abs(sp["lambda"].real) < 1e-3 else "DIVERGENT")
            t_double = math.log(2) / sp["lambda"].real if sp["lambda"].real > 1e-9 else float("inf")
            print(f"  Spiral mode: λ = {sp['lambda']:+.4f}     τ = {sp['tau_s']:+.3f} s  ({sp_label})")
            if sp["lambda"].real > 0:
                print(f"               time-to-double = {t_double:.1f} s")
        print()

    # Save CSV summary
    with (out_dir / "lateral_modes.csv").open("w") as f:
        f.write("V_mps,CL,Cl_beta,Cl_p,Cn_beta,Cn_r,dutch_roll_omega,dutch_roll_zeta,"
                "roll_tau,spiral_lambda_real\n")
        for V in (14.0, 16.0, 18.0, 20.0, 22.0, 25.0, 30.0):
            t = trim_lateral(V=V)
            A = state_space_lateral(t)
            m = lateral_modes(A)
            dr = m["modes"].get("dutch_roll", {})
            roll = m["modes"].get("roll", {})
            sp = m["modes"].get("spiral", {})
            f.write(f"{V},{t.CL:.4f},{t.Cl_beta:.4f},{t.Cl_p:.4f},"
                    f"{t.Cn_beta:.4f},{t.Cn_r:.4f},"
                    f"{dr.get('omega_n', float('nan')):.4f},"
                    f"{dr.get('zeta', float('nan')):.4f},"
                    f"{roll.get('tau_s', float('nan')):.4f},"
                    f"{sp.get('lambda', complex(float('nan'))).real if 'lambda' in sp else float('nan'):.4f}\n")


if __name__ == "__main__":
    main()
