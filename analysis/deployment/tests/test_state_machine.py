"""
Validation tests for the deployment state machine.

Run with:
    PYTHONPATH=. .venv/bin/python -m pytest analysis/deployment/tests/ -v
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.deployment.state_machine import (  # noqa: E402
    AbortCause,
    DeploymentStateMachine,
    SensorReading,
    State,
    make_nominal_scenario,
    run_simulation,
)


def test_nominal_scenario_reaches_glide():
    res = run_simulation(make_nominal_scenario(), t_end=10.0, dt=0.005)
    assert res.final_state == State.GLIDE
    assert res.abort_cause == AbortCause.NONE
    assert res.glide_t is not None
    # Total exit-to-glide time well under 10 s
    assert res.glide_t < 10.0


def test_pilot_abort_jettisons_during_freefall():
    """Pilot pulls handle during freefall — must jettison + reserve."""
    def scenario(t):
        if t < 0.5:
            return SensorReading(t=t, accel_mag=9.81, body_rates=(0, 0, 0),
                                 airspeed_mps=0.0, altitude_agl_m=4000.0,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 1.5:
            return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0),
                                 airspeed_mps=20.0, altitude_agl_m=3990.0,
                                 pilot_arm=True, fcs_healthy=True)
        # Pilot abort at 1.5 s
        return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0),
                             airspeed_mps=30.0, altitude_agl_m=3950.0,
                             pilot_arm=True, pilot_abort=True, fcs_healthy=True)
    res = run_simulation(scenario, t_end=3.0, dt=0.01)
    assert res.final_state == State.JETTISON_RESERVE
    assert res.abort_cause == AbortCause.PILOT_MANUAL


def test_aad_trigger_below_alt_threshold_jettisons():
    """AAD fires when altitude < 200 m and still in freefall configuration."""
    def scenario(t):
        if t < 0.5:
            return SensorReading(t=t, accel_mag=9.81, body_rates=(0, 0, 0),
                                 airspeed_mps=0.0, altitude_agl_m=400.0,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 2.0:
            return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0),
                                 airspeed_mps=30.0, altitude_agl_m=300.0,
                                 pilot_arm=True, fcs_healthy=True)
        # AAD fires below 200 m
        return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0),
                             airspeed_mps=50.0, altitude_agl_m=150.0,
                             pilot_arm=True, aad_fire=True, fcs_healthy=True)
    res = run_simulation(scenario, t_end=4.0, dt=0.01)
    assert res.final_state == State.JETTISON_RESERVE
    assert res.abort_cause == AbortCause.AAD_TRIGGER


def test_drogue_failure_jettisons_after_timeout():
    """If drogue load never reaches 50 % within timeout, system jettisons."""
    def scenario(t):
        if t < 0.5:
            return SensorReading(t=t, accel_mag=9.81, body_rates=(0, 0, 0),
                                 airspeed_mps=0.0, altitude_agl_m=4000.0,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 2.5:
            v = min(55.0, 10.0 + 30.0 * (t - 1.0))
            return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0.05),
                                 airspeed_mps=v, altitude_agl_m=3990.0,
                                 pilot_arm=True, fcs_healthy=True)
        # Drogue extracts but never inflates: load stays at 0
        return SensorReading(t=t, accel_mag=15.0, body_rates=(0.5, 0.5, 0.2),
                             airspeed_mps=55.0, altitude_agl_m=3500.0,
                             pilot_arm=True, drogue_load_pct=0.0,
                             fcs_healthy=True)
    res = run_simulation(scenario, t_end=10.0, dt=0.01)
    assert res.final_state == State.JETTISON_RESERVE
    assert res.abort_cause == AbortCause.DROGUE_MAL


def test_asymmetric_deploy_above_threshold_jettisons():
    """Wing locks complete on both sides but Δt > 10 ms — jettison."""
    nominal = make_nominal_scenario()

    def scenario(t):
        s = nominal(t)
        # Override the symmetry timing to make it asymmetric (>10ms)
        if s.deploy_left_t is not None:
            s = SensorReading(
                t=s.t, accel_mag=s.accel_mag, body_rates=s.body_rates,
                airspeed_mps=s.airspeed_mps, altitude_agl_m=s.altitude_agl_m,
                pilot_arm=s.pilot_arm, drogue_load_pct=s.drogue_load_pct,
                spar_lock_left=s.spar_lock_left, spar_lock_right=s.spar_lock_right,
                deploy_left_t=5.020, deploy_right_t=5.060,  # 40 ms gap — too far apart
                fcs_healthy=s.fcs_healthy,
            )
        return s
    res = run_simulation(scenario, t_end=10.0, dt=0.005)
    assert res.final_state == State.JETTISON_RESERVE
    assert res.abort_cause == AbortCause.ASYMMETRIC_DEPLOY


def test_init_state_stays_until_arm():
    sm = DeploymentStateMachine()
    s = SensorReading(t=0.0, accel_mag=9.81, body_rates=(0, 0, 0),
                      airspeed_mps=0.0, altitude_agl_m=4000.0,
                      pilot_arm=False)
    sm.step(s)
    assert sm.state == State.INIT
    s = SensorReading(t=0.1, accel_mag=9.81, body_rates=(0, 0, 0),
                      airspeed_mps=0.0, altitude_agl_m=4000.0,
                      pilot_arm=True)
    sm.step(s)
    assert sm.state == State.ARMED


def test_freefall_dwell_required():
    """Brief sub-g spike doesn't trigger FREEFALL transition; sustained does."""
    sm = DeploymentStateMachine()
    sm.step(SensorReading(t=0.0, accel_mag=9.81, body_rates=(0, 0, 0),
                          airspeed_mps=0.0, altitude_agl_m=4000.0,
                          pilot_arm=True))
    # Brief micro-g spike
    sm.step(SensorReading(t=0.05, accel_mag=1.0, body_rates=(0, 0, 0),
                          airspeed_mps=0.0, altitude_agl_m=4000.0,
                          pilot_arm=True))
    # Back to 1 g
    sm.step(SensorReading(t=0.10, accel_mag=9.81, body_rates=(0, 0, 0),
                          airspeed_mps=0.0, altitude_agl_m=4000.0,
                          pilot_arm=True))
    assert sm.state == State.ARMED  # didn't transition

    # Now sustained sub-g for > 200 ms
    for t in (0.20, 0.30, 0.41, 0.50):
        sm.step(SensorReading(t=t, accel_mag=2.0, body_rates=(0, 0, 0),
                              airspeed_mps=10.0, altitude_agl_m=3990.0,
                              pilot_arm=True))
    assert sm.state == State.FREEFALL
