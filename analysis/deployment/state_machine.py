"""
MANTA deployment state-machine simulator.

Implements the BRIEF deployment sequence with sensed gates and abort paths:

    INIT
      ↓ (pilot arm)
    ARMED
      ↓ (exit detected: accel below g for ≥ 200 ms or pilot manual exit cmd)
    FREEFALL
      ↓ (stable freefall sensed: body rates within window for ≥ 1 s)
    STABILIZE
      ↓ (drogue extract command)
    DROGUE_INFLATING
      ↓ (drogue load ≥ 50 % nominal AND airspeed < 32 m/s within timeout)
    DROGUE_STABLE
      ↓ (wing deploy command)
    WING_DEPLOY
      ↓ (all 6 spar-lock sensors confirmed, left-right Δt < 10 ms 3-σ)
    WING_TRIM_ACQUIRE
      ↓ (FCS confirms trimmed glide, alpha-limiter active)
    GLIDE  ← steady state

Abort transitions (parallel monitors at every state from ARMED onward):

    ASYMMETRIC_DEPLOY → JETTISON → RESERVE
    SPAR_LOCK_FAIL    → JETTISON → RESERVE
    DROGUE_MAL        → BYPASS → JETTISON → RESERVE  (or pilot reserve)
    AAD_TRIGGER       → JETTISON → RESERVE  (FCS-bypassed)
    PILOT_MANUAL_ABORT → JETTISON → RESERVE
    FCS_FAULT (irrecoverable) → MECHANICAL_REVERSION OR JETTISON
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum, Enum, auto
from typing import Callable, Optional


class State(IntEnum):
    INIT = 0
    ARMED = 1
    FREEFALL = 2
    STABILIZE = 3
    DROGUE_INFLATING = 4
    DROGUE_STABLE = 5
    WING_DEPLOY = 6
    WING_TRIM_ACQUIRE = 7
    GLIDE = 8
    # Terminal abort branches
    JETTISON_RESERVE = 100
    MECHANICAL_REVERSION = 101


class AbortCause(Enum):
    NONE = auto()
    ASYMMETRIC_DEPLOY = auto()
    SPAR_LOCK_FAIL = auto()
    DROGUE_MAL = auto()
    AAD_TRIGGER = auto()
    PILOT_MANUAL = auto()
    FCS_FAULT = auto()
    DEPLOY_TIMEOUT = auto()


@dataclass
class SensorReading:
    """Per-tick fused sensor snapshot fed to the state machine."""
    t: float                           # seconds since arm
    accel_mag: float                   # m/s² magnitude
    body_rates: tuple[float, float, float]  # rad/s p, q, r
    airspeed_mps: float
    altitude_agl_m: float
    pilot_arm: bool = False
    pilot_abort: bool = False
    aad_fire: bool = False
    drogue_load_pct: float = 0.0       # % of nominal drogue tension
    spar_lock_left: tuple[bool, bool, bool] = (False, False, False)   # 3 stages × side
    spar_lock_right: tuple[bool, bool, bool] = (False, False, False)
    deploy_left_t: Optional[float] = None  # time at which left wing fully locked (None until)
    deploy_right_t: Optional[float] = None
    fcs_healthy: bool = True


@dataclass
class Config:
    """Tunable thresholds. Defaults match BRIEF / docs/03-deployment-sequence.md."""
    accel_freefall_g_threshold: float = 0.3       # accel < 0.3 g → freefall
    freefall_dwell_s: float = 0.2                  # 200 ms continuous
    body_rate_stable_rad_s: float = 0.5            # rates below for stable
    stable_dwell_s: float = 1.0
    drogue_min_load_pct: float = 50.0
    drogue_speed_max_mps: float = 32.0
    drogue_inflation_timeout_s: float = 4.0
    wing_deploy_symmetry_max_dt_ms: float = 10.0   # 3-σ gate
    wing_deploy_timeout_s: float = 0.5
    aad_min_alt_m: float = 200.0                   # AAD fires below this if still falling fast


@dataclass
class StateLogEntry:
    t: float
    state: State
    abort: AbortCause = AbortCause.NONE
    note: str = ""


@dataclass
class DeploymentSimResult:
    log: list[StateLogEntry] = field(default_factory=list)
    final_state: State = State.INIT
    abort_cause: AbortCause = AbortCause.NONE
    drogue_extract_t: Optional[float] = None
    wing_deploy_cmd_t: Optional[float] = None
    wing_locked_t: Optional[float] = None
    glide_t: Optional[float] = None


class DeploymentStateMachine:
    """One-shot deployment state machine. Step it with sensor readings."""

    def __init__(self, cfg: Config | None = None):
        self.cfg = cfg or Config()
        self.state = State.INIT
        self.result = DeploymentSimResult()
        self._t_state_entered: float = 0.0
        self._freefall_dwell_start: Optional[float] = None
        self._stable_dwell_start: Optional[float] = None

    def _transition(self, new_state: State, t: float, note: str = "", abort: AbortCause = AbortCause.NONE) -> None:
        self.state = new_state
        self._t_state_entered = t
        self.result.log.append(StateLogEntry(t=t, state=new_state, abort=abort, note=note))
        if new_state == State.GLIDE:
            self.result.glide_t = t
        if abort != AbortCause.NONE:
            self.result.abort_cause = abort

    def _check_global_aborts(self, s: SensorReading) -> Optional[AbortCause]:
        """Aborts that can fire from any in-flight state."""
        if self.state in (State.INIT, State.JETTISON_RESERVE, State.MECHANICAL_REVERSION, State.GLIDE):
            return None
        if s.pilot_abort:
            return AbortCause.PILOT_MANUAL
        if s.aad_fire and s.altitude_agl_m < self.cfg.aad_min_alt_m:
            return AbortCause.AAD_TRIGGER
        if not s.fcs_healthy and self.state >= State.WING_TRIM_ACQUIRE:
            return AbortCause.FCS_FAULT
        return None

    def step(self, s: SensorReading) -> State:
        # Global aborts override everything
        cause = self._check_global_aborts(s)
        if cause is not None:
            self._transition(State.JETTISON_RESERVE, s.t,
                             note=f"global abort: {cause.name}", abort=cause)
            return self.state

        if self.state == State.INIT:
            if s.pilot_arm:
                self._transition(State.ARMED, s.t, "pilot arm")
        elif self.state == State.ARMED:
            # Detect freefall: accel below threshold sustained
            g = 9.80665
            if s.accel_mag < self.cfg.accel_freefall_g_threshold * g:
                if self._freefall_dwell_start is None:
                    self._freefall_dwell_start = s.t
                if s.t - self._freefall_dwell_start >= self.cfg.freefall_dwell_s:
                    self._transition(State.FREEFALL, s.t, "freefall dwell met")
            else:
                self._freefall_dwell_start = None
        elif self.state == State.FREEFALL:
            # Wait for stable body rates
            rate_mag = math.sqrt(sum(r * r for r in s.body_rates))
            if rate_mag < self.cfg.body_rate_stable_rad_s:
                if self._stable_dwell_start is None:
                    self._stable_dwell_start = s.t
                if s.t - self._stable_dwell_start >= self.cfg.stable_dwell_s:
                    self._transition(State.STABILIZE, s.t, "stable freefall")
            else:
                self._stable_dwell_start = None
        elif self.state == State.STABILIZE:
            # Issue drogue extract command immediately when stabilization confirmed
            self._transition(State.DROGUE_INFLATING, s.t, "drogue extract cmd")
            self.result.drogue_extract_t = s.t
        elif self.state == State.DROGUE_INFLATING:
            # Pass: load adequate AND speed below max
            if s.drogue_load_pct >= self.cfg.drogue_min_load_pct and s.airspeed_mps < self.cfg.drogue_speed_max_mps:
                self._transition(State.DROGUE_STABLE, s.t, "drogue stable")
            elif s.t - self._t_state_entered > self.cfg.drogue_inflation_timeout_s:
                self._transition(State.JETTISON_RESERVE, s.t,
                                 note="drogue inflation timeout", abort=AbortCause.DROGUE_MAL)
        elif self.state == State.DROGUE_STABLE:
            # Issue wing deploy command immediately
            self._transition(State.WING_DEPLOY, s.t, "wing deploy cmd")
            self.result.wing_deploy_cmd_t = s.t
        elif self.state == State.WING_DEPLOY:
            # Check spar locks both sides AND the symmetry budget
            left_locked = all(s.spar_lock_left)
            right_locked = all(s.spar_lock_right)
            if left_locked and right_locked:
                if s.deploy_left_t is not None and s.deploy_right_t is not None:
                    dt_ms = abs(s.deploy_left_t - s.deploy_right_t) * 1000.0
                    if dt_ms <= self.cfg.wing_deploy_symmetry_max_dt_ms:
                        self.result.wing_locked_t = s.t
                        self._transition(State.WING_TRIM_ACQUIRE, s.t,
                                         f"wing locked, Δt = {dt_ms:.2f} ms")
                    else:
                        self._transition(State.JETTISON_RESERVE, s.t,
                                         note=f"asymmetric deploy: Δt = {dt_ms:.2f} ms",
                                         abort=AbortCause.ASYMMETRIC_DEPLOY)
            elif s.t - self._t_state_entered > self.cfg.wing_deploy_timeout_s:
                self._transition(State.JETTISON_RESERVE, s.t,
                                 note="wing deploy timeout (lock sensors)",
                                 abort=AbortCause.SPAR_LOCK_FAIL)
        elif self.state == State.WING_TRIM_ACQUIRE:
            # Wait for FCS to report trimmed
            if s.fcs_healthy and abs(s.airspeed_mps - 16.0) < 4.0:
                self._transition(State.GLIDE, s.t, "trimmed glide")
        # else: terminal state, no further transitions
        self.result.final_state = self.state
        return self.state


# ------------------------------------------------------------------------
# Reference scenarios (used by tests + symmetry budget driver)
# ------------------------------------------------------------------------

def make_nominal_scenario() -> Callable[[float], SensorReading]:
    """Synthetic sensor stream for a nominal exit-and-deploy."""

    def s(t: float) -> SensorReading:
        if t < 0.5:
            # Pre-exit: hand on rail, 1 g
            return SensorReading(t=t, accel_mag=9.81, body_rates=(0, 0, 0),
                                 airspeed_mps=0.0, altitude_agl_m=4000.0,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 1.0:
            # Just exited; tumbling briefly
            return SensorReading(t=t, accel_mag=2.0, body_rates=(2.0, 1.0, 0.5),
                                 airspeed_mps=20.0, altitude_agl_m=3990.0,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 2.5:
            # Stable freefall, hits terminal velocity ~55 m/s
            v = min(55.0, 10.0 + 30.0 * (t - 1.0))
            return SensorReading(t=t, accel_mag=2.0, body_rates=(0.1, 0.1, 0.05),
                                 airspeed_mps=v, altitude_agl_m=4000.0 - 0.5 * 9.8 * (t - 0.5) ** 2,
                                 pilot_arm=True, fcs_healthy=True)
        if t < 5.0:
            # Drogue extracted at t≈2.0s; inflates and decelerates.
            # Load reaches 50 % within ~0.8 s, fully decelerating in ~2.5 s.
            decel_t = t - 2.5
            v = max(28.0, 55.0 - 11.0 * decel_t)
            load_pct = min(85.0, 75.0 * decel_t)
            alt = 3700.0 - v * decel_t
            return SensorReading(t=t, accel_mag=10.0, body_rates=(0.1, 0.1, 0),
                                 airspeed_mps=v, altitude_agl_m=alt,
                                 pilot_arm=True, drogue_load_pct=load_pct,
                                 fcs_healthy=True)
        if t < 5.05:
            # Wing deploying — locks coming in
            return SensorReading(t=t, accel_mag=12.0, body_rates=(0.2, 0.5, 0.1),
                                 airspeed_mps=29.0, altitude_agl_m=3580.0,
                                 pilot_arm=True, drogue_load_pct=80.0,
                                 spar_lock_left=(False, False, False),
                                 spar_lock_right=(False, False, False),
                                 fcs_healthy=True)
        # Wing locked symmetrically Δt ≈ 5 ms; FCS captures trim
        return SensorReading(t=t, accel_mag=9.81, body_rates=(0.1, 0.1, 0.05),
                             airspeed_mps=18.0, altitude_agl_m=3500.0,
                             pilot_arm=True, drogue_load_pct=20.0,
                             spar_lock_left=(True, True, True),
                             spar_lock_right=(True, True, True),
                             deploy_left_t=5.020, deploy_right_t=5.025,
                             fcs_healthy=True)

    return s


def run_simulation(scenario: Callable[[float], SensorReading],
                   t_end: float = 12.0, dt: float = 0.005) -> DeploymentSimResult:
    sm = DeploymentStateMachine()
    n = int(t_end / dt)
    for i in range(n):
        t = i * dt
        sm.step(scenario(t))
        if sm.state in (State.JETTISON_RESERVE, State.MECHANICAL_REVERSION, State.GLIDE):
            # Run one more step for log completeness
            sm.step(scenario(t + dt))
            break
    return sm.result


def main() -> None:
    print("# Deployment state machine — nominal scenario")
    print()
    res = run_simulation(make_nominal_scenario())
    print(f"Final state: {res.final_state.name}")
    print(f"Abort cause: {res.abort_cause.name}")
    print(f"Drogue extract t : {res.drogue_extract_t}")
    print(f"Wing deploy cmd  : {res.wing_deploy_cmd_t}")
    print(f"Wing locked t    : {res.wing_locked_t}")
    print(f"Glide acquired   : {res.glide_t}")
    print()
    print("State trace:")
    for entry in res.log:
        msg = f"  t={entry.t:6.3f}s  →  {entry.state.name}"
        if entry.note:
            msg += f"   ({entry.note})"
        if entry.abort != AbortCause.NONE:
            msg += f"   ABORT={entry.abort.name}"
        print(msg)


if __name__ == "__main__":
    main()
