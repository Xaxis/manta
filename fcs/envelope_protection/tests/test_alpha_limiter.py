"""
Validation tests for the alpha limiter.

Run with:
    PYTHONPATH=. .venv/bin/python -m pytest fcs/envelope_protection/tests/ -v
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from fcs.envelope_protection.alpha_limiter import AlphaLimiter, LimiterConfig  # noqa: E402


def test_pilot_below_limit_passes_through():
    lim = AlphaLimiter()
    out = lim.step(alpha_pilot_cmd_deg=4.0, V_indicated_mps=20.0,
                   alpha_sensor_valid=True, alpha_measured_deg=4.0)
    assert out["alpha_cmd_deg"] == 4.0
    assert out["saturated"] is False


def test_pilot_above_limit_clamps():
    lim = AlphaLimiter()
    out = lim.step(alpha_pilot_cmd_deg=15.0, V_indicated_mps=20.0,
                   alpha_sensor_valid=True, alpha_measured_deg=10.0)
    cfg = LimiterConfig()
    expected_limit = cfg.alpha_stall_deg - cfg.margin_to_stall_deg
    assert abs(out["alpha_cmd_deg"] - expected_limit) < 1e-9
    assert out["saturated"] is True


def test_low_airspeed_widens_margin():
    """Below V_low, limit tightens (more margin)."""
    lim = AlphaLimiter()
    a_at_15 = lim.alpha_limit_deg(V=15.0, degraded=False)
    a_at_10 = lim.alpha_limit_deg(V=10.0, degraded=False)
    assert a_at_10 < a_at_15  # tighter at low speed


def test_degraded_mode_tightens_limit():
    lim = AlphaLimiter()
    a_normal = lim.alpha_limit_deg(V=20.0, degraded=False)
    a_degraded = lim.alpha_limit_deg(V=20.0, degraded=True)
    cfg = LimiterConfig()
    assert abs((a_normal - a_degraded) - cfg.degraded_extra_margin_deg) < 1e-9


def test_sensor_dropout_triggers_degraded():
    lim = AlphaLimiter()
    out = lim.step(alpha_pilot_cmd_deg=10.0, V_indicated_mps=20.0,
                   alpha_sensor_valid=False, alpha_measured_deg=None)
    assert out["degraded"] is True
    cfg = LimiterConfig()
    expected = cfg.alpha_stall_deg - cfg.margin_to_stall_deg - cfg.degraded_extra_margin_deg
    assert abs(out["alpha_limit_deg"] - expected) < 1e-9


def test_limit_never_exceeds_alpha_stall():
    """No matter how fast, limit must always remain at least 1° below stall."""
    lim = AlphaLimiter()
    cfg = LimiterConfig()
    for V in (15.0, 25.0, 35.0, 50.0, 80.0):
        a_lim = lim.alpha_limit_deg(V=V, degraded=False)
        assert a_lim <= cfg.alpha_stall_deg - 1.0, f"V={V}: limit {a_lim} too close to stall"


def test_estimate_alpha_for_trim():
    """Coarse trim α from CL = mg/(0.5·ρV²S) at the design pilot mass.

    Resized planform defaults (S = 6.5 m², CL_α = 4.17 /rad, α₀ = +1°); checked
    at the cruise point (the smaller wing's V_bg is ~18 m/s, so use ~20 m/s).
    """
    lim = AlphaLimiter()
    a = lim.estimate_alpha_for_trim(V=20.0, m=106.0, rho=1.225)
    # CL = 106·9.81/(0.5·1.225·400·6.5) = 0.653; α = 1° + 0.653/4.17·57.296 ≈ 10.0°
    assert 8.0 < a < 12.0
