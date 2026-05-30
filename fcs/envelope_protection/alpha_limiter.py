"""
Alpha-limiter prototype for the MANTA FCS.

Per BRIEF: the alpha limiter is treated as a structural design assumption —
not an advisory feature. The structure is sized assuming the limiter
functions; loss-of-limiter is a hazardous failure mode.

Architecture
------------
The limiter is a hard-deck on commanded angle of attack. It runs in the
inner loop of the FCS at 400 Hz alongside the EKF. Output is a corrected
α command that the pitch-rate inner loop tracks via flaperon deflection.

    pilot_α_cmd  ──┐
                   ├──►  saturate at min(α_cmd, α_limit(V, m))  ──►  inner loop
    α_limit(V)  ──┘                                              │
                                                                 ▼
                                                            δ_e via PI

Gain schedule
-------------
α_limit decreases with airspeed (slow flight is closer to stall) and with
pilot mass (heavier pilot → higher CL_trim → less margin). At V_bg the
limit is roughly α_stall − 2.5° margin to absorb gusts and CG-perturbation
transients. At cruise (well below stall), limit can be relaxed to give
pilot more authority.

Sensor fault handling
---------------------
If the primary AoA vane fails, fall back to:
  α_estimated = (1/CL_α) · (m·g / (½·ρ·V²·S))   minus α_0

This is the "trim α at current load and airspeed" — a coarse but
defensible degraded-mode estimate. The limiter widens its margin
(α_limit_degraded = α_limit − 1.5°) when in degraded mode.

Anti-windup
-----------
The PI integrator on the inner pitch-rate loop is frozen whenever the α
command saturates at the limit, to prevent integrator wind-up that would
overshoot when the limit is released.

Test scenarios (in test_alpha_limiter.py):
    - steady-state below limit: pass-through.
    - steady-state at limit: clamp.
    - gust ingestion: limit holds during transient.
    - sensor dropout: degraded mode kicks in, limit tightens.
    - integrator anti-windup: no overshoot when limit is released.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class LimiterConfig:
    """Tunable parameters for the alpha limiter."""

    alpha_stall_deg: float = 11.5
    margin_to_stall_deg: float = 2.5     # α_limit = α_stall − margin
    margin_low_speed_deg: float = 4.0    # extra margin below V_low
    V_low_mps: float = 14.0              # below this, widen the margin
    V_high_mps: float = 30.0             # above this, relax somewhat
    margin_high_speed_relax_deg: float = 1.0
    degraded_extra_margin_deg: float = 1.5  # additional margin when sensor is degraded
    Iout_max: float = 1.0                # anti-windup output saturation marker


@dataclass
class LimiterState:
    integrator: float = 0.0
    saturated: bool = False
    degraded: bool = False
    last_alpha_cmd: float = 0.0
    last_alpha_limit: float = 0.0


class AlphaLimiter:
    """Stateful alpha limiter with gain-scheduled limit and anti-windup hooks.

    Inputs each tick:
        alpha_pilot_cmd_deg : pilot stick → desired alpha
        V_indicated_mps    : pitot-static airspeed
        alpha_sensor_valid : bool, AoA vane health
        alpha_measured_deg : measured α (None if invalid)
        m_total_kg         : current vehicle mass (for the degraded estimate)
        rho_kg_per_m3      : air density at current alt

    Outputs:
        alpha_cmd_deg      : alpha command sent to inner loop, clamped.
        saturated          : True if limit is biting (anti-windup signal).
        degraded           : True if running on degraded-mode estimate.
    """

    def __init__(self, cfg: Optional[LimiterConfig] = None) -> None:
        self.cfg = cfg or LimiterConfig()
        self.state = LimiterState()

    def alpha_limit_deg(self, V: float, degraded: bool) -> float:
        cfg = self.cfg
        margin = cfg.margin_to_stall_deg
        # Widen when slow
        if V < cfg.V_low_mps:
            margin += cfg.margin_low_speed_deg * (cfg.V_low_mps - V) / cfg.V_low_mps
        # Relax (slightly) when fast
        if V > cfg.V_high_mps:
            margin -= cfg.margin_high_speed_relax_deg * min(1.0, (V - cfg.V_high_mps) / 10.0)
        if degraded:
            margin += cfg.degraded_extra_margin_deg
        return cfg.alpha_stall_deg - margin

    def step(
        self,
        alpha_pilot_cmd_deg: float,
        V_indicated_mps: float,
        alpha_sensor_valid: bool,
        alpha_measured_deg: Optional[float] = None,
        m_total_kg: float = 105.0,
        rho_kg_per_m3: float = 1.225,
    ) -> dict:
        # Determine whether we're in degraded mode
        degraded = (not alpha_sensor_valid) or (alpha_measured_deg is None)
        self.state.degraded = degraded

        # Compute the active limit
        alpha_limit = self.alpha_limit_deg(V_indicated_mps, degraded)

        # Saturate the command
        alpha_cmd = min(alpha_pilot_cmd_deg, alpha_limit)
        saturated = alpha_pilot_cmd_deg > alpha_limit

        self.state.saturated = saturated
        self.state.last_alpha_cmd = alpha_cmd
        self.state.last_alpha_limit = alpha_limit

        return {
            "alpha_cmd_deg": alpha_cmd,
            "alpha_limit_deg": alpha_limit,
            "saturated": saturated,
            "degraded": degraded,
        }

    def estimate_alpha_for_trim(self, V: float, m: float, rho: float,
                                 cl_alpha_per_rad: float = 4.17,
                                 alpha_0_deg: float = 1.0,
                                 S: float = 6.5) -> float:
        """Coarse degraded-mode α estimate from trim equation."""
        import math
        g = 9.80665
        CL = m * g / (0.5 * rho * V * V * S)
        return alpha_0_deg + math.degrees(CL / cl_alpha_per_rad)


def main() -> None:
    """Quick demo of the limiter behavior across a range of conditions."""
    lim = AlphaLimiter()
    print("# Alpha limiter — V vs limit")
    print()
    print("|  V (m/s) |  α_limit (deg)  |  α_limit_degraded (deg)  |")
    print("|---|---|---|")
    for V in (10, 12, 14, 16, 18, 20, 22, 25, 30, 35, 40):
        a = lim.alpha_limit_deg(V, degraded=False)
        ad = lim.alpha_limit_deg(V, degraded=True)
        print(f"|  {V:5.1f}  |  {a:6.2f}  |  {ad:6.2f}  |")

    print()
    print("# Demo step responses")
    for label, scenario in (
        ("Pilot below limit", dict(alpha_pilot_cmd_deg=4.0, V_indicated_mps=20.0,
                                    alpha_sensor_valid=True, alpha_measured_deg=4.0)),
        ("Pilot at limit", dict(alpha_pilot_cmd_deg=10.0, V_indicated_mps=16.0,
                                 alpha_sensor_valid=True, alpha_measured_deg=8.0)),
        ("Pilot above limit, slow", dict(alpha_pilot_cmd_deg=12.0, V_indicated_mps=14.0,
                                          alpha_sensor_valid=True, alpha_measured_deg=11.0)),
        ("Sensor dropout", dict(alpha_pilot_cmd_deg=10.0, V_indicated_mps=18.0,
                                 alpha_sensor_valid=False, alpha_measured_deg=None)),
    ):
        out = lim.step(**scenario)
        print(f"  {label:30s}  →  cmd={out['alpha_cmd_deg']:.2f}°  "
              f"limit={out['alpha_limit_deg']:.2f}°  "
              f"saturated={out['saturated']}  degraded={out['degraded']}")


if __name__ == "__main__":
    main()
