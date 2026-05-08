"""
Validation tests for the structural analysis modules.

Run with:
    PYTHONPATH=. .venv/bin/python -m pytest analysis/struct/tests/ -v
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.struct.materials import CFRPUDTube  # noqa: E402
from analysis.struct.spar_model import SparStage, TelescopingSpar  # noqa: E402
from analysis.struct.spar_bending import cantilever_bending_moment  # noqa: E402


# ---------- materials ---------------------------------------------------

def test_cfrp_design_stress_chain():
    """sigma_design_compression_limit = sigma_ult_compression × knockdown / SF."""
    m = CFRPUDTube()
    expected = m.sigma_ult_compression * m.knockdown / m.safety_factor_limit_to_ultimate
    assert math.isclose(m.sigma_design_compression_limit, expected, rel_tol=1e-12)


# ---------- spar section properties -------------------------------------

def test_hollow_tube_I_matches_analytical():
    """I = π/64·(D⁴ − d⁴)."""
    s = SparStage("test", outer_diameter_m=0.040, wall_thickness_m=0.002, length_m=1.0)
    D = 0.040
    d = 0.040 - 2 * 0.002
    I_analytical = math.pi / 64.0 * (D**4 - d**4)
    assert math.isclose(s.I_m4, I_analytical, rel_tol=1e-9)


def test_hollow_tube_area():
    s = SparStage("test", 0.040, 0.002, 1.0)
    A_analytical = math.pi * (0.020**2 - 0.018**2)
    assert math.isclose(s.area_m2, A_analytical, rel_tol=1e-9)


def test_telescoping_spar_total_length():
    """3 stages of L each, with two joints overlapping by O each, gives
    deployed length 3L − 2O."""
    L = 1.0
    O = 0.05
    spar = TelescopingSpar(
        name="t",
        stages=(
            SparStage("a", 0.04, 0.002, L),
            SparStage("b", 0.03, 0.002, L),
            SparStage("c", 0.02, 0.002, L),
        ),
        joint_overlap_m=O,
        joint_hardware_kg_per_joint=0.0,
    )
    assert math.isclose(spar.total_length_m, 3 * L - 2 * O, rel_tol=1e-9)


# ---------- cantilever bending moment integrator ------------------------

def test_cantilever_uniform_load_root_moment():
    """Uniform line load q over a cantilever of length L: M_root = q·L²/2."""
    L = 5.0
    q = 100.0
    n = 200
    y = np.linspace(0.0, L, n)
    q_arr = np.full(n, q)
    M = cantilever_bending_moment(y, q_arr)
    M_root = M[0]
    expected = q * L * L / 2.0
    rel_err = abs(M_root - expected) / expected
    assert rel_err < 0.005, f"Got {M_root}, expected {expected} (rel err {rel_err:.4f})"


def test_cantilever_tip_moment_is_zero():
    """No load outboard of the tip → moment at tip is zero (within ε)."""
    n = 100
    y = np.linspace(0.0, 3.7, n)
    q = np.ones(n) * 50.0
    M = cantilever_bending_moment(y, q)
    assert abs(M[-1]) < 1e-6


def test_cantilever_elliptic_load_root_moment():
    """For elliptical loading L'(y) = (4·L/(π·b))·sqrt(1 − (2y/b)²) on a
    half-wing of length b/2:
        M_root = ∫₀^(b/2) y · L'(y) dy = (2·L·b)/(3·π) · 0.5
    Wait — more carefully: L_total over full span is L; half-span lift is
    L/2; spread over the half-span elliptically with magnitude factor
    (4·(L/2))/(π·(b/2)) = 4·L/(π·b). So:
        M_root = (4·L/(π·b)) · ∫₀^(b/2) y·sqrt(1 − (2y/b)²) dy
              = (4·L/(π·b)) · (b²/12)
              = L·b/(3π)
    """
    b = 7.4
    L = 1000.0  # half-wing total lift
    n = 400
    y = np.linspace(0.0, b / 2, n)
    L_per_span = (4 * L / (math.pi * b)) * np.sqrt(np.maximum(1.0 - (2 * y / b) ** 2, 0.0))
    M = cantilever_bending_moment(y, L_per_span)
    expected = L * b / (3 * math.pi)
    rel_err = abs(M[0] - expected) / expected
    assert rel_err < 0.01, f"Got {M[0]:.3f}, expected {expected:.3f}, rel err {rel_err:.4f}"


# ---------- spar mass ---------------------------------------------------

def test_default_spars_mass_matches_handcalc():
    """Sanity-check the default spar set against an independent hand calc."""
    from analysis.struct.spar_model import default_front_spar, default_rear_spar  # noqa: WPS433

    cfrp = CFRPUDTube()
    front = default_front_spar()
    rear = default_rear_spar()

    def tube_mass(D, t, L):
        A = math.pi * ((D / 2) ** 2 - ((D - 2 * t) / 2) ** 2)
        return cfrp.rho * A * L

    L_stage = 3.7 / 3.0 + 2 * 0.025
    m_front_hand = (
        tube_mass(0.040, 0.002, L_stage)
        + tube_mass(0.032, 0.002, L_stage)
        + tube_mass(0.025, 0.002, L_stage)
        + 2 * 0.04   # joint hardware
    )
    m_rear_hand = (
        tube_mass(0.030, 0.002, L_stage)
        + tube_mass(0.024, 0.002, L_stage)
        + tube_mass(0.018, 0.002, L_stage)
        + 2 * 0.04
    )
    rel_err_front = abs(front.mass_kg(cfrp) - m_front_hand) / m_front_hand
    rel_err_rear = abs(rear.mass_kg(cfrp) - m_rear_hand) / m_rear_hand
    assert rel_err_front < 1e-9
    assert rel_err_rear < 1e-9
