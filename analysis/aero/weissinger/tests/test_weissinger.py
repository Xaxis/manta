"""
Validation tests for the Weissinger solver.

Run with:
    .venv/bin/python -m pytest analysis/aero/weissinger/tests/ -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

# Make the package importable when run from project root
_PROJECT_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_PROJECT_ROOT.parent))  # so 'analysis' is a top-level namespace pkg

from analysis.aero.weissinger.weissinger import WingModel, solve, neutral_point  # noqa: E402


def _rect_wing(b: float, c: float, twist=lambda y: 0.0) -> WingModel:
    return WingModel(
        span=b,
        chord_at=lambda y: c,
        x_le_at=lambda y: 0.0,
        twist_deg_at=twist,
        section_alpha_0_deg=0.0,
        section_a0_per_rad=2.0 * math.pi,
    )


def test_symmetric_loading_for_symmetric_wing() -> None:
    """Untwisted rectangular wing must produce symmetric span loading."""
    wing = _rect_wing(b=8.0, c=1.0)
    r = solve(wing, alpha_deg=4.0, n_panels_per_side=20)
    # Loading at mirror-image stations should match within numerical tolerance
    cl = r.cl_section
    cl_mirror = cl[::-1]
    err = np.max(np.abs(cl - cl_mirror) / np.maximum(np.abs(cl), 1e-9))
    assert err < 1e-9, f"asymmetry {err:.3e} on a symmetric input"


def test_helmbold_AR_correction_unswept_rect() -> None:
    """For unswept rectangular wing with thin-airfoil 2D slope a0 = 2π,
    the Helmbold formula gives:
        CL_α = a0 / (1 + a0/(π·AR))
    For AR = 8, this is 2π / (1 + 2π/(8π)) = 2π / 1.25 = 5.027 / rad ≈ 0.0877 / deg.
    Weissinger should match within ~3 % (slightly underestimates due to wake model
    + single-row chord).
    """
    AR = 8.0
    b = 8.0
    c = b / AR
    a0 = 2.0 * math.pi
    helmbold = a0 / (1.0 + a0 / (math.pi * AR))

    wing = _rect_wing(b=b, c=c)
    r0 = solve(wing, alpha_deg=0.0, n_panels_per_side=40)
    r4 = solve(wing, alpha_deg=4.0, n_panels_per_side=40)
    CL_alpha = (r4.CL - r0.CL) / math.radians(4.0)

    rel_err = abs(CL_alpha - helmbold) / helmbold
    assert rel_err < 0.04, f"Weissinger CL_α={CL_alpha:.3f}/rad vs Helmbold {helmbold:.3f}/rad ({rel_err*100:.1f} %)"


def test_neutral_point_unswept_rectangle() -> None:
    """For an unswept rectangular wing, the wing aerodynamic center is at MAC c/4
    aft of the apex (= 0.25·c since x_LE = 0 here, MAC = c). Weissinger should
    place the neutral point within a few percent of c/4.
    """
    b = 8.0
    c = 1.0
    wing = _rect_wing(b=b, c=c)
    x_np, mac = neutral_point(wing, alphas_deg=(0.0, 4.0), n_panels_per_side=40)
    # Expected: x_NP ≈ 0.25·c = 0.25 m
    expected = 0.25 * c
    assert abs(x_np - expected) / expected < 0.06, f"x_NP={x_np:.4f} vs expected {expected:.4f}"


def test_swept_wing_neutral_point_aft_of_apex_c4() -> None:
    """A swept wing must have its neutral point aft of the root c/4 location.
    This is a sanity check, not a precise number — the AVL run will pin the value.
    """
    b = 7.4
    c_root = 1.6216
    sweep_le_rad = math.radians(25.0)

    def chord_at(y):
        eta = abs(y) / (b / 2)
        return c_root * (1.0 - 0.6 * eta)  # taper 0.4

    def x_le_at(y):
        return abs(y) * math.tan(sweep_le_rad)

    wing = WingModel(
        span=b,
        chord_at=chord_at,
        x_le_at=x_le_at,
        twist_deg_at=lambda y: 0.0,  # untwisted for AC location check
        section_alpha_0_deg=0.0,
        section_a0_per_rad=2.0 * math.pi,
    )
    x_np, mac = neutral_point(wing, alphas_deg=(0.0, 4.0), n_panels_per_side=40)
    # Sanity bounds: x_np is aft of root c/4 (= 0.405 m) and forward of root TE (= 1.62 m).
    assert 0.405 < x_np < c_root, f"x_NP={x_np:.3f} m outside reasonable bounds"
    # Geometric MAC c/4 reference: y_MAC ≈ 1.586 m, x_MAC c/4 ≈ y_MAC·tan(25°) + 0.25·MAC
    # = 0.7394 + 0.25·1.205 ≈ 1.041 m. NP should be near that for an untwisted swept wing.
    geom_mac_c4 = 1.586 * math.tan(sweep_le_rad) + 0.25 * 1.2046
    assert abs(x_np - geom_mac_c4) / geom_mac_c4 < 0.12, (
        f"x_NP={x_np:.3f} m differs from geometric MAC c/4 {geom_mac_c4:.3f} m by >12 %"
    )


def test_zero_alpha_zero_lift_no_twist() -> None:
    """Untwisted wing with α0 = 0 at α = 0 must have CL ≈ 0 to numerical noise."""
    wing = _rect_wing(b=8.0, c=1.0)
    r = solve(wing, alpha_deg=0.0, n_panels_per_side=20)
    assert abs(r.CL) < 1e-9
