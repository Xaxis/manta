"""
MANTA longitudinal flight dynamics — freefall -> deploy -> pull-out -> glide.

A 3-DOF point-mass-plus-pitch model integrated in the vertical plane using the
shared aero model in `sim/aero.py`.  This is the rigorous, verifiable physics
deliverable: it starts the pilot in belly-to-earth freefall at terminal
velocity, deploys the wing over the BRIEF's 0.6 s mechanical sequence, and
shows the aerodynamic pull-out into steady best-glide.

State:
    x      downrange position        [m]
    h      altitude                  [m]
    vx,vz  inertial velocity         [m/s]   (vz<0 = descending)
    theta  pitch attitude            [rad]
    q      pitch rate                [rad/s]

Pitch is closed by an FCS alpha-hold law (BRIEF decision #8 fly-by-wire +
alpha limiter): the controller commands best-glide angle of attack once the
wing is deploying.  Before deploy the body is a passively-stable freefaller.

Run:
    PYTHONPATH=. .venv/bin/python sim/flight_dynamics.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim.aero import (
    RHO, G, AeroState, ALPHA0, steady_glide, assert_targets,
)

_HERE = Path(__file__).parent
OUT_DIR = _HERE / "out"
OUT_DIR.mkdir(parents=True, exist_ok=True)

# --- scenario (faithful to the docs/03 state machine) -------------------
MASS = 86.0                # kg all-up (pilot + gear + 16.5 kg wing system)
T_END = 32.0               # s
DT = 0.001                 # s integration step
H0 = 2200.0                # m  start altitude

T_DROGUE = 2.0             # s  drogue extract (FREEFALL -> STABILIZE)
T_DROGUE_DUR = 0.4         # s  drogue inflation ramp
CDA_DROGUE = 2.4           # m^2 added Cd*A: brings 44 -> ~22 m/s before deploy

T_DEPLOY_START = 6.0       # s  wing deploy cmd once drogue-stable (<32 m/s)
T_DEPLOY_DUR = 0.6         # s  BRIEF mechanical sequence (Phase A+B+C)
T_DROGUE_RELEASE = 6.15    # s  drogue cut shortly after wing starts to load

# FCS longitudinal autopilot (BRIEF #8 fly-by-wire + alpha limiter).
# A load-factor-limited path tracker: command a flight-path angle (biased by
# airspeed error so the energy state is regulated — climb if fast, dive if
# slow), pull toward it at a capped load factor, and back out the AoA that
# produces the required lift.  This is how a real recovery autopilot pulls out
# of a dive without looping or over-g, then captures best-glide.
FCS_TAU = 0.25             # s    AoA actuation time constant
FCS_KG = 3.0               # commanded maneuver-g per rad of path error
FCS_KSPD = 0.022           # rad of path bias per (m/s) airspeed error
N_MAX = 2.5                # structural pull-out load-factor limit
ALPHA_LIMIT = math.radians(12.0)   # alpha limiter (structural assumption)
LD_TARGET = steady_glide(MASS, p=1.0)["L_over_D_max"]   # best-glide L/D ~13.2


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def deploy_progress(t: float) -> float:
    return smoothstep((t - T_DEPLOY_START) / T_DEPLOY_DUR)


def drogue_cda(t: float) -> float:
    if t < T_DROGUE or t >= T_DROGUE_RELEASE:
        return 0.0
    return CDA_DROGUE * smoothstep((t - T_DROGUE) / T_DROGUE_DUR)


def alpha_best_glide(p: float) -> float:
    """Commanded AoA: the best-glide CL of the current (deploying) wing."""
    aero = AeroState.at(p)
    cl_bg = math.sqrt((aero.CdA / aero.S) * math.pi * aero.E_eff)
    cl_bg = min(cl_bg, 0.9 * aero.CLmax)
    return cl_bg / aero.CLa + ALPHA0


def simulate() -> dict:
    # initial belly-to-earth freefall: nearly vertical descent at terminal
    s0 = steady_glide(MASS, p=0.0)
    v_term = s0["V_terminal_stowed"]
    glide = steady_glide(MASS, p=1.0)
    v_bg = glide["V_best_glide"]
    vx = 2.0
    vz = -math.sqrt(max(v_term ** 2 - vx ** 2, 1.0))
    x, h = 0.0, H0
    alpha = math.radians(2.0)               # small positive AoA, tracking flight path

    rec = {k: [] for k in
           ("t", "x", "h", "V", "gamma_deg", "alpha_deg", "theta_deg",
            "n_load", "glide_ratio", "deploy", "CL", "CD", "sink")}
    steps = int(T_END / DT)
    rec_every = max(1, steps // 900)

    for i in range(steps):
        t = i * DT
        p = deploy_progress(t)
        aero = AeroState.at(p)

        V = math.hypot(vx, vz)
        gamma = math.atan2(vz, vx)
        theta = gamma + alpha

        cl, cd, _ = aero.coeffs(alpha)  # Cm unused: pitch is FCS-closed below
        q_dyn = 0.5 * RHO * V * V
        L = q_dyn * aero.S * cl
        D = q_dyn * aero.S * cd + q_dyn * drogue_cda(t)   # drogue adds pure drag

        cg, sg = math.cos(gamma), math.sin(gamma)
        # drag along -velocity, lift +90deg from velocity
        ax = (-D * cg - L * sg) / MASS
        az = (-D * sg + L * cg) / MASS - G

        # --- pitch loop: FCS alpha-hold (first-order lag toward commanded AoA) ---
        if p > 0.02:
            # --- load-factor-limited path tracker (longitudinal autopilot) ---
            gamma_glide = -math.atan2(1.0, LD_TARGET)
            # bias target path by speed error: climb when fast, dive when slow
            gamma_cmd = gamma_glide + FCS_KSPD * (V - v_bg)
            gamma_cmd = max(-1.4, min(0.4, gamma_cmd))
            # maneuver g to pull toward commanded path, capped at the limit
            n_man = max(-(N_MAX - 1.0),
                        min(N_MAX - 1.0, FCS_KG * (gamma_cmd - gamma)))
            n_cmd = math.cos(gamma) + n_man          # support weight + maneuver
            W = MASS * G
            cl_cmd = n_cmd * W / max(q_dyn * aero.S, 1e-3)
            cl_cmd = max(-0.9 * aero.CLmax, min(0.9 * aero.CLmax, cl_cmd))
            a_cmd = cl_cmd / aero.CLa + ALPHA0
            a_cmd = max(-ALPHA_LIMIT, min(ALPHA_LIMIT, a_cmd))
            alpha += (a_cmd - alpha) * (DT / FCS_TAU)
        else:
            # freefall: body weathercocks toward the relative wind (alpha -> ~0)
            alpha += (math.radians(2.0) - alpha) * (DT / 0.5)

        # integrate (semi-implicit Euler)
        vx += ax * DT
        vz += az * DT
        x += vx * DT
        h += vz * DT

        if i % rec_every == 0:
            n_total = math.hypot(ax, az + G) / G    # total aero load factor
            gr = (vx / -vz) if vz < -1e-3 else float("nan")
            rec["t"].append(t)
            rec["x"].append(x)
            rec["h"].append(h)
            rec["V"].append(V)
            rec["gamma_deg"].append(math.degrees(gamma))
            rec["alpha_deg"].append(math.degrees(alpha))
            rec["theta_deg"].append(math.degrees(theta))
            rec["n_load"].append(n_total)
            rec["glide_ratio"].append(gr)
            rec["deploy"].append(p)
            rec["CL"].append(cl)
            rec["CD"].append(cd)
            rec["sink"].append(-vz)

        if h <= 0:
            break

    return rec


def settle_metrics(rec: dict) -> dict:
    """Steady-state values from the last 1 s of the trajectory."""
    n = len(rec["t"])
    tail = slice(int(n * 0.9), n)
    avg = lambda k: sum(rec[k][tail]) / max(1, len(rec[k][tail]))
    return {
        "V_glide": avg("V"),
        "gamma_glide_deg": avg("gamma_deg"),
        "glide_ratio": avg("glide_ratio"),
        "alpha_glide_deg": avg("alpha_deg"),
        "sink_rate": avg("sink"),
        "peak_load": max(rec["n_load"]),
    }


def plot(rec: dict, settle: dict):
    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("MANTA — freefall → deployment → glide (3-DOF longitudinal)",
                 fontsize=14, fontweight="bold")
    t = rec["t"]

    def band(a):
        a.axvspan(T_DEPLOY_START, T_DEPLOY_START + T_DEPLOY_DUR,
                  color="orange", alpha=0.15, label="deploy")

    ax[0][0].plot(rec["x"], rec["h"], lw=2, color="#1a3a8e")
    ax[0][0].set_title("Trajectory (range vs altitude)")
    ax[0][0].set_xlabel("downrange x [m]"); ax[0][0].set_ylabel("altitude h [m]")
    ax[0][0].grid(alpha=0.3)

    ax[0][1].plot(t, rec["V"], lw=2, color="#c2410c"); band(ax[0][1])
    ax[0][1].axhline(settle["V_glide"], ls="--", color="gray",
                     label=f"glide {settle['V_glide']:.1f} m/s")
    ax[0][1].set_title("Airspeed"); ax[0][1].set_xlabel("t [s]")
    ax[0][1].set_ylabel("V [m/s]"); ax[0][1].legend(); ax[0][1].grid(alpha=0.3)

    ax[0][2].plot(t, rec["gamma_deg"], lw=2, label="flight path γ")
    ax[0][2].plot(t, rec["alpha_deg"], lw=2, label="AoA α")
    ax[0][2].plot(t, rec["theta_deg"], lw=1, ls=":", label="pitch θ")
    band(ax[0][2])
    ax[0][2].set_title("Angles"); ax[0][2].set_xlabel("t [s]")
    ax[0][2].set_ylabel("deg"); ax[0][2].legend(); ax[0][2].grid(alpha=0.3)

    ax[1][0].plot(t, rec["n_load"], lw=2, color="#7c3aed"); band(ax[1][0])
    ax[1][0].set_title("Aerodynamic load factor (pull-out)")
    ax[1][0].set_xlabel("t [s]"); ax[1][0].set_ylabel("n [g]")
    ax[1][0].grid(alpha=0.3)

    gr = [g if g == g and g < 30 else None for g in rec["glide_ratio"]]
    ax[1][1].plot(t, gr, lw=2, color="#0891b2"); band(ax[1][1])
    ax[1][1].axhline(settle["glide_ratio"], ls="--", color="gray",
                     label=f"{settle['glide_ratio']:.1f}:1")
    ax[1][1].set_title("Instantaneous glide ratio")
    ax[1][1].set_xlabel("t [s]"); ax[1][1].set_ylabel("L/D"); ax[1][1].set_ylim(0, 16)
    ax[1][1].legend(); ax[1][1].grid(alpha=0.3)

    ax[1][2].plot(t, rec["deploy"], lw=2, color="#16a34a"); band(ax[1][2])
    ax[1][2].set_title("Deploy progress"); ax[1][2].set_xlabel("t [s]")
    ax[1][2].set_ylabel("p"); ax[1][2].grid(alpha=0.3)

    fig.tight_layout()
    out = OUT_DIR / "flight_dynamics.png"
    fig.savefig(out, dpi=110)
    print(f"    wrote {out}")
    # also publish to the site
    site = _HERE.parent / "site" / "public" / "img" / "flight_dynamics.png"
    if site.parent.exists():
        fig.savefig(site, dpi=110)
        print(f"    wrote {site}")
    plt.close(fig)


def main():
    print("(0) Aero model closure vs BRIEF targets:")
    target = assert_targets(MASS)
    print("(1) Integrating freefall -> deploy -> glide ...")
    rec = simulate()
    settle = settle_metrics(rec)
    print("    steady glide reached:")
    print(f"      V        = {settle['V_glide']:.2f} m/s   (target V_bg {target['V_best_glide']:.2f})")
    print(f"      γ        = {settle['gamma_glide_deg']:.2f} deg")
    print(f"      L/D      = {settle['glide_ratio']:.2f}      (target {target['L_over_D_max']:.2f})")
    print(f"      sink     = {settle['sink_rate']:.2f} m/s")
    print(f"      peak n   = {settle['peak_load']:.2f} g")
    plot(rec, settle)

    # publish per-frame telemetry for the web viewer overlay.
    # NaN/Inf are invalid JSON (browsers reject `NaN`); replace with null.
    def clean(v):
        return v if isinstance(v, float) and math.isfinite(v) else (
            v if not isinstance(v, float) else None)
    telem = {"dt_record": rec["t"][1] - rec["t"][0],
             "deploy_start": T_DEPLOY_START, "deploy_dur": T_DEPLOY_DUR,
             "settle": settle, "target": target,
             "series": {k: [clean(x) for x in rec[k]] for k in
                        ("t", "deploy", "V", "gamma_deg", "alpha_deg",
                         "n_load", "glide_ratio", "h")}}
    p = OUT_DIR / "telemetry.json"
    p.write_text(json.dumps(telem))
    print(f"    wrote {p}")
    site = _HERE.parent / "site" / "public" / "models" / "v3" / "telemetry.json"
    if site.parent.exists():
        site.write_text(json.dumps(telem))
        print(f"    wrote {site}")


if __name__ == "__main__":
    main()
