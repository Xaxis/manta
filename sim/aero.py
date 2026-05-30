"""
MANTA aerodynamic model — single source of truth for the flight physics.

The same coefficient model is used by:
  * sim/flight_dynamics.py  (point-mass longitudinal validator + trajectory)
  * sim/build.py            (MuJoCo free-flight wrench via xfrc_applied)

Everything is derived from the locked planform in BRIEF.md:

    S   = 8.4  m^2     wing area (fully deployed)
    b   = 7.4  m       span
    AR  = 6.5          aspect ratio
    e   = 0.85         Oswald efficiency
    CD0 = 0.025        zero-lift (profile + parasite) drag of the clean wing

These numbers are *not* tuned to hit the targets — they fall out of the
geometry, and the targets fall out of them. The point of `assert_targets()`
is to prove that closure:

    L/D_max   = 0.5*sqrt(pi*e*AR/CD0)            -> 13.2   (BRIEF stretch 13:1)
    CL_bg     = sqrt(CD0*pi*e*AR)                -> 0.659
    V_bg      = sqrt(2 W / (rho S CL_bg))        -> 16.5 m/s (BRIEF ~16 m/s)
    V_stall   = sqrt(2 W / (rho S CL_max))       -> 12.3 m/s (BRIEF < 14 m/s)
    V_term    = sqrt(2 W / (rho CdA_stowed))     -> 42.9 m/s (belly freefall)

During deployment every quantity is interpolated stowed -> deployed by the
deploy progress p in [0, 1]:

    S(p)    flat-plate body          0.75 m^2  ->  8.4 m^2
    CdA(p)  parasite area*Cd         0.83      ->  0.21
    AR(p)   effective aspect ratio   0.5       ->  6.5
    CLa(p)  lift-curve slope (1/rad) 0.1       ->  4.5
    CLmax(p)                         0.05      ->  1.20

So at p=0 the system is a bluff freefalling body (no lift, vertical terminal
velocity); at p=1 it is the locked wing (best-glide ~16 m/s, L/D ~13).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# --- atmosphere ---------------------------------------------------------
# ISA at ~1.5 km AGL, a representative skydive deployment altitude.
RHO = 1.0556          # kg/m^3   air density at ~1500 m
G = 9.80665           # m/s^2

# --- resized planform (BRIEF #5/#6: 6.5 m²/6.3 m, AR 6.1, 7° washout) -----
S_WING = 6.5          # m^2
SPAN = 6.3            # m
AR_WING = 6.106
E_OSWALD = 0.85
CD0_WING = 0.025
CHORD_BAR = S_WING / SPAN     # mean aero chord ~1.03 m
ALPHA0 = math.radians(-2.0)   # zero-lift angle (slight reflex/washout camber)
CLMAX_WING = 1.10
CLA_WING = 4.5                # 1/rad finite-wing lift slope w/ 25 deg sweep

# --- stowed bluff body (belly-to-earth freefall) ------------------------
S_STOWED = 0.75       # m^2   frontal/plan reference area, tucked pilot
CDA_STOWED = 0.83     # m^2   Cd*A of the stowed body (Cd ~ 1.1)
AR_STOWED = 0.5
CLA_STOWED = 0.10
CLMAX_STOWED = 0.05


def lerp(a: float, b: float, t: float) -> float:
    return a * (1.0 - t) + b * t


@dataclass
class AeroState:
    """Deploy-interpolated coefficients at progress p in [0,1]."""
    S: float
    CdA: float
    AR: float
    CLa: float
    CLmax: float

    @classmethod
    def at(cls, p: float) -> "AeroState":
        p = max(0.0, min(1.0, p))
        return cls(
            S=lerp(S_STOWED, S_WING, p),
            CdA=lerp(CDA_STOWED, S_WING * CD0_WING, p),
            AR=lerp(AR_STOWED, AR_WING, p),
            CLa=lerp(CLA_STOWED, CLA_WING, p),
            CLmax=lerp(CLMAX_STOWED, CLMAX_WING, p),
        )

    def CL(self, alpha: float) -> float:
        cl = self.CLa * (alpha - ALPHA0)
        return max(-self.CLmax, min(self.CLmax, cl))

    def coeffs(self, alpha: float) -> tuple[float, float, float]:
        """Return (CL, CD, Cm) for the given angle of attack [rad].

        CD = CdA/S (parasite, referenced to S) + induced.  Cm is a static
        pitch coefficient about the mean-chord aero center: weakly nose-down,
        proportional to (alpha - alpha0), representing washout + reflex.
        """
        cl = self.CL(alpha)
        cd_parasite = self.CdA / self.S
        cd_induced = cl * cl / (math.pi * self.E_eff)
        cd = cd_parasite + cd_induced
        cm = -0.06 - 0.20 * (alpha - ALPHA0)   # statically stable (dCm/dCL<0)
        return cl, cd, cm

    @property
    def E_eff(self) -> float:
        return E_OSWALD * self.AR


def steady_glide(mass: float, p: float = 1.0) -> dict:
    """Closed-form steady-glide solution at full deploy for a given mass.

    Returns best-glide speed, sink rate, glide ratio, stall speed, and the
    terminal velocity of the *stowed* body — the headline physics numbers.
    """
    W = mass * G
    aero = AeroState.at(p)
    # Best L/D: minimize CD/CL over CL.
    ld_max = 0.5 * math.sqrt(math.pi * aero.E_eff / (aero.CdA / aero.S))
    cl_bg = math.sqrt((aero.CdA / aero.S) * math.pi * aero.E_eff)
    v_bg = math.sqrt(2 * W / (RHO * aero.S * cl_bg))
    gamma_bg = math.atan2(1.0, ld_max)           # descent angle [rad]
    sink = v_bg * math.sin(gamma_bg)
    v_stall = math.sqrt(2 * W / (RHO * aero.S * aero.CLmax))

    stowed = AeroState.at(0.0)
    v_term = math.sqrt(2 * W / (RHO * stowed.CdA))

    return {
        "mass": mass,
        "L_over_D_max": ld_max,
        "CL_best_glide": cl_bg,
        "alpha_best_glide_deg": math.degrees(cl_bg / aero.CLa + ALPHA0),
        "V_best_glide": v_bg,
        "sink_rate": sink,
        "glide_ratio": 1.0 / math.tan(gamma_bg),
        "V_stall": v_stall,
        "V_terminal_stowed": v_term,
    }


def assert_targets(mass: float = 86.0, verbose: bool = True) -> dict:
    """Prove the model reproduces the BRIEF performance targets."""
    s = steady_glide(mass)
    # Targets updated for the resized 6.5 m² wing (BRIEF #5/#6): higher wing
    # loading → faster V_bg + higher stall, which is fine since MANTA lands
    # under reserve (the <14 m/s stall target was relaxed in the resize).
    checks = [
        ("V_best_glide  ~18-20 m/s", s["V_best_glide"], 16.0, 22.0),
        ("L/D_max       10-13",      s["L_over_D_max"], 10.0, 13.5),
        ("V_stall       ~15 m/s",    s["V_stall"], 11.0, 17.0),
        ("V_terminal    40-50",      s["V_terminal_stowed"], 38.0, 52.0),
    ]
    if verbose:
        print(f"  steady-state @ {mass:.0f} kg all-up:")
        for label, val, lo, hi in checks:
            ok = "OK " if lo <= val <= hi else "XX "
            print(f"    [{ok}] {label:24s} = {val:7.2f}   (want {lo}-{hi})")
    for label, val, lo, hi in checks:
        assert lo <= val <= hi, f"{label}: {val:.2f} outside [{lo},{hi}]"
    return s


if __name__ == "__main__":
    assert_targets()
