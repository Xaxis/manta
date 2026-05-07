"""
First-cut analytical 2D airfoil polar for the MH-78-class section.

PURPOSE
    Provide a runnable Cl(α, Re), Cd(Cl, Re), Cm(α) used by the rest of the aero
    pipeline before XFOIL runs are completed. Replace this with table interpolation
    against XFOIL CSV output once `polars/` is populated.

LIMITS
    The numbers here are literature-bracketed for "MH/EH-class reflexed
    flying-wing section, 10 % t/c, fully turbulent boundary layer at Re ≥ 1×10⁶".
    Stall and post-stall behavior are an analytical model, not a fit to data.
    Anything load-bearing on the safety case must be backed by XFOIL or
    wind-tunnel data.

UNCERTAINTY
    - Cm0:        +0.005   ± 0.010   (sensitivity bracket on washout)
    - Cl_max:     1.25     ± 0.10
    - alpha_0:   −1.0°     ± 0.5°    (camber-driven zero-lift angle)
    - dCl/dα:    5.7 /rad  ± 0.2 /rad (slightly below 2π; Re-corrected, fully turb)
    - Cd0:       0.0085    ± 0.0015 at Re=1.5×10⁶, design Cl
    - k (Cl² coefficient in low-drag bucket):  0.006  ± 0.002
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AnalyticPolar:
    """Analytical 2D polar for a reflexed flying-wing section."""

    name: str = "MH-78-class (analytic)"

    # Linear lift curve
    cl_alpha_per_rad: float = 5.7   # ≈ 0.099 / deg; below 2π for finite Re/turb BL
    alpha_0_deg: float = -1.0       # zero-lift angle, deg

    # Stall model
    cl_max: float = 1.25
    alpha_stall_buffer_deg: float = 1.5  # smoothing of post-stall fall-off

    # Drag polar: Cd = Cd0 + k1·(Cl - Cl_min_drag) + k2·(Cl - Cl_min_drag)^2
    cd0: float = 0.0085          # at Cl_min_drag, Re_ref
    cl_min_drag: float = 0.40
    k_quadratic: float = 0.006
    re_ref: float = 1.5e6
    re_drag_exponent: float = -0.20  # Cd0 ~ Re^(-0.2), turbulent friction trend

    # Pitching moment about quarter chord
    cm0: float = 0.005
    dcm_dcl: float = -0.02       # mild stable trend; close to ideal-fluid 0

    # ---- functions ----------------------------------------------------------

    def cl(self, alpha_deg: float, _re: float | None = None) -> float:
        """2D section lift coefficient at angle of attack `alpha_deg`.

        Linear up to (alpha_stall − buffer); soft fall-off thereafter using a
        cosine roll-down so Cl is continuous and bounded for the lifting-line
        solver. Physical post-stall behavior is *not* modeled accurately —
        anything alpha > stall is "we're already departing".
        """
        a = math.radians(alpha_deg - self.alpha_0_deg)
        cl_lin = self.cl_alpha_per_rad * a

        # Smooth saturation toward cl_max in the buffer region
        a_stall = self.alpha_stall_deg
        a_buf = self.alpha_stall_buffer_deg
        if alpha_deg <= a_stall - a_buf:
            return cl_lin
        if alpha_deg >= a_stall + a_buf:
            # crude post-stall: roll off from cl_max past the buffer.
            # Continuous with the blend branch at α = a_stall + a_buf.
            return self.cl_max * (1.0 - 0.08 * (alpha_deg - (a_stall + a_buf)) / a_buf)

        # Cosine blend in [a_stall - a_buf, a_stall + a_buf]
        t = (alpha_deg - (a_stall - a_buf)) / (2.0 * a_buf)
        cl_lin_at_buf = self.cl_alpha_per_rad * math.radians(
            (a_stall - a_buf) - self.alpha_0_deg
        )
        cl_top = self.cl_max
        return cl_lin_at_buf + (cl_top - cl_lin_at_buf) * 0.5 * (1.0 - math.cos(math.pi * t))

    @property
    def alpha_stall_deg(self) -> float:
        """Linear-cl-curve intersection with cl_max."""
        return self.alpha_0_deg + math.degrees(self.cl_max / self.cl_alpha_per_rad)

    def cd(self, cl: float, re: float = 1.5e6) -> float:
        """2D section drag coefficient as a function of section Cl and Re."""
        cd_friction = self.cd0 * (re / self.re_ref) ** self.re_drag_exponent
        delta = cl - self.cl_min_drag
        cd_pressure = self.k_quadratic * delta * delta
        # Mild Re-independent term for laminar/turbulent transition fudge
        return cd_friction + cd_pressure

    def cm(self, cl: float) -> float:
        """2D section pitching moment about c/4."""
        return self.cm0 + self.dcm_dcl * (cl - self.cl_min_drag)

    # ---- sweep helpers ------------------------------------------------------

    def sample_polar(self, alpha_range_deg=(-5.0, 18.0), n: int = 47, re: float = 1.5e6):
        """Return (alphas, cls, cds, cms, l_over_d) numpy arrays."""
        import numpy as np

        alphas = np.linspace(alpha_range_deg[0], alpha_range_deg[1], n)
        cls = np.array([self.cl(a) for a in alphas])
        cds = np.array([self.cd(cl, re=re) for cl in cls])
        cms = np.array([self.cm(cl) for cl in cls])
        with np.errstate(divide="ignore", invalid="ignore"):
            lod = np.where(cds > 0, cls / cds, 0.0)
        return alphas, cls, cds, cms, lod


def main() -> None:
    """Print a summary table of the analytic polar at Re = 1.5e6."""
    p = AnalyticPolar()
    alphas, cls, cds, cms, lod = p.sample_polar()
    print(f"# 2D analytic polar — {p.name}, Re = 1.5e6")
    print()
    print("| α (°) |   Cl   |   Cd   |   Cm   |  L/D  |")
    print("|---|---|---|---|---|")
    for a, cl, cd, cm, ld in zip(alphas, cls, cds, cms, lod):
        print(f"| {a:5.1f} | {cl:6.3f} | {cd:6.4f} | {cm:6.4f} | {ld:5.1f} |")
    print()
    print(f"alpha_0 = {p.alpha_0_deg:.2f}°,  alpha_stall ≈ {p.alpha_stall_deg:.2f}°,  Cl_max = {p.cl_max:.2f}")
    cl_design = 0.5
    print(f"Cd at design Cl={cl_design}: {p.cd(cl_design):.4f}    L/D_2D = {cl_design / p.cd(cl_design):.1f}")
    print(f"Cm0 = {p.cm0:.4f}    dCm/dCl = {p.dcm_dcl:+.3f}")


if __name__ == "__main__":
    main()
