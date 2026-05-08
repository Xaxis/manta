"""
Drogue dynamics — inflation profile, snatch load, deceleration timeline.

The drogue is a small ringslot canopy that decelerates the pilot+rig from
freefall terminal velocity (~55 m/s) to the wing-deploy airspeed (~30 m/s)
before the main wing snaps open. Loading on the drogue during inflation
sets one of the harness-mount design loads and one of the timing inputs to
the deployment state machine.

What this script computes
-------------------------
1. Drogue size required to produce equilibrium at V_target.
2. Inflation profile (drag area vs. time, ringslot empirical model).
3. Bridle tension transient: peak snatch load including the dynamic
   amplification factor over steady-state.
4. Deceleration trajectory: V(t) from terminal to V_target after drogue
   extract command, integrating the equation of motion in 1-DOF vertical.
5. Comparison of snatch load to flight-load cases (1 g, 3 g, 4.5 g).
6. Statement on whether the spar root sees this load (it does not, in
   nominal sequence; harness mount takes it).

References
----------
- Knacke, T. W., *Parachute Recovery Systems Design Manual*, NWC TP 6575,
  1985, §5 (drogues, snatch loads, inflation profiles).
- Cockrell, D. J., *Aerodynamics of Parachutes*, AGARD-AG-295, 1987.
- Lingard, J. S., *Ram-Air Parachute Design*, AIAA Aerodynamic
  Decelerator Systems Conference 1995 — for reference on inflation
  amplification factors.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))


@dataclass(frozen=True)
class DrogueConfig:
    # Mass + atmosphere
    m_total_kg: float = 105.0          # 82.5 pilot + 15.5 wing + 8 rig (sized config)
    rho: float = 1.225
    g: float = 9.80665

    # Pilot CdA in prone freefall (drag area without drogue)
    CdA_pilot: float = 0.40            # m², standard prone skydiver
    V_terminal_target: float = 55.0    # m/s, BRIEF reference

    # Drogue / target
    V_target_after_drogue: float = 30.0  # m/s, wing-deploy condition
    CD_drogue: float = 0.55            # ringslot, partial-porosity (Knacke)
    inflation_time_s: float = 0.45     # ringslot empirical (Knacke §5.4)
    snatch_dynamic_factor: float = 1.7 # peak/steady ratio during inflation
                                        # (Knacke fig. 5-8, conservative)


def required_drogue(cfg: DrogueConfig) -> dict:
    """Size the drogue to produce equilibrium descent at V_target."""
    # Steady descent: m g = ½ρ V² (CdA_pilot + CdA_drogue)
    CdA_total_required = cfg.m_total_kg * cfg.g / (0.5 * cfg.rho * cfg.V_target_after_drogue ** 2)
    CdA_drogue = CdA_total_required - cfg.CdA_pilot
    A_drogue = CdA_drogue / cfg.CD_drogue
    diameter = math.sqrt(4.0 * A_drogue / math.pi)
    return {
        "CdA_total_required": CdA_total_required,
        "CdA_drogue_required": CdA_drogue,
        "A_drogue_m2": A_drogue,
        "diameter_m": diameter,
    }


def inflation_profile_cda(t: float, cda_full: float, t_inflate: float) -> float:
    """Drogue CdA buildup vs. time (linear from 0 → CdA_full over inflation_time)."""
    if t < 0:
        return 0.0
    if t >= t_inflate:
        return cda_full
    return cda_full * (t / t_inflate)


def integrate_descent(cfg: DrogueConfig, dt: float = 0.005, t_end: float = 6.0) -> dict:
    """Integrate 1-DOF vertical descent through drogue extract → stable.

    Phase 1 (t < 0): equilibrium at V_terminal (CdA_pilot only).
    Phase 2 (t = 0): drogue extract command.
    Phase 3 (t > 0): drogue inflates linearly; system decelerates.

    Returns time series of V, drogue CdA, total drag, bridle tension.
    """
    sized = required_drogue(cfg)
    cda_drogue_full = sized["CdA_drogue_required"]

    n = int(t_end / dt)
    t_arr = np.linspace(0, t_end, n)
    V = np.full(n, cfg.V_terminal_target)
    cda_d = np.zeros(n)
    F_bridle = np.zeros(n)
    F_drag_total = np.zeros(n)

    for i in range(1, n):
        t = t_arr[i]
        cda_d[i] = inflation_profile_cda(t, cda_drogue_full, cfg.inflation_time_s)
        cda_total = cfg.CdA_pilot + cda_d[i]
        F_drag_total[i] = 0.5 * cfg.rho * V[i - 1] ** 2 * cda_total
        F_drogue_only = 0.5 * cfg.rho * V[i - 1] ** 2 * cda_d[i]
        # Dynamic amplification during inflation transient (during rising
        # CdA the canopy snaps open faster than the airframe responds)
        if t < cfg.inflation_time_s:
            F_bridle[i] = F_drogue_only * cfg.snatch_dynamic_factor
        else:
            F_bridle[i] = F_drogue_only
        # Equation of motion (1-DOF vertical)
        a = cfg.g - F_drag_total[i] / cfg.m_total_kg
        V[i] = V[i - 1] + a * dt

    return {
        "t": t_arr,
        "V": V,
        "cda_drogue": cda_d,
        "F_drag_total": F_drag_total,
        "F_bridle": F_bridle,
        "sized": sized,
    }


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    cfg = DrogueConfig()
    print("# MANTA drogue dynamics")
    print()
    print(f"  Total mass for sizing: {cfg.m_total_kg} kg (sized-config wing)")
    print(f"  Terminal V (no drogue): {cfg.V_terminal_target} m/s (BRIEF reference)")
    print(f"  Target V after drogue:  {cfg.V_target_after_drogue} m/s")
    print(f"  Pilot CdA (prone):      {cfg.CdA_pilot} m²")
    print(f"  Drogue CD (ringslot):   {cfg.CD_drogue}")
    print()

    # Sizing
    sized = required_drogue(cfg)
    print("## Sizing — drogue area required to produce target equilibrium descent")
    print()
    print(f"  CdA_total required at V = {cfg.V_target_after_drogue} m/s:  {sized['CdA_total_required']:.3f} m²")
    print(f"  CdA_drogue (subtracting pilot):                 {sized['CdA_drogue_required']:.3f} m²")
    print(f"  Drogue area (A = CdA / C_D):                    {sized['A_drogue_m2']:.3f} m²")
    print(f"  Drogue diameter (round canopy):                 {sized['diameter_m']:.3f} m")
    print()

    # Integrate descent
    res = integrate_descent(cfg)
    F_peak = res["F_bridle"].max()
    F_peak_t = res["t"][res["F_bridle"].argmax()]
    V_at_peak = res["V"][res["F_bridle"].argmax()]
    # Time to reach within 1 m/s of target
    target_idx = next((i for i, v in enumerate(res["V"]) if v <= cfg.V_target_after_drogue + 1.0), -1)
    target_t = res["t"][target_idx] if target_idx > 0 else None

    print("## Snatch load + deceleration")
    print()
    print(f"  Peak bridle tension:    {F_peak:.0f} N  ({F_peak/1000:.2f} kN)")
    print(f"    occurs at t = {F_peak_t:.3f} s, V = {V_at_peak:.1f} m/s")
    print(f"    dynamic amplification factor: {cfg.snatch_dynamic_factor}")
    print()
    print(f"  Time to V ≤ {cfg.V_target_after_drogue+1:.0f} m/s: "
          f"{target_t:.2f} s" if target_t else "(not reached in window)")
    print()

    # Compare to flight loads
    g_peak = F_peak / (cfg.m_total_kg * cfg.g)
    print("## Snatch vs. flight loads")
    print()
    print(f"  Pilot weight (1 g):           {cfg.m_total_kg * cfg.g:.0f} N  ({1.0:.2f} g)")
    print(f"  3 g limit flight:             {3 * cfg.m_total_kg * cfg.g:.0f} N  (3.00 g)")
    print(f"  4.5 g ultimate flight:        {4.5 * cfg.m_total_kg * cfg.g:.0f} N  (4.50 g)")
    print(f"  **Drogue snatch peak**:       {F_peak:.0f} N  ({g_peak:.2f} g)")
    print()
    if g_peak > 4.5:
        print("  ⚠ Drogue snatch exceeds 4.5 g ultimate flight load. "
              "Either reduce snatch (reefed inflation, slower extract) or "
              "size the harness mount to this case explicitly.")
    elif g_peak > 3.0:
        print("  ⚠ Drogue snatch exceeds 3 g limit flight load. The harness "
              "mount has to be sized to this case rather than 3 g flight.")
    else:
        print("  ✓ Drogue snatch within the flight-load envelope. "
              "Harness mount sized for flight loads is adequate for the snatch case.")
    print()

    # Where does this load go?
    print("## Where does this load go?")
    print()
    print("  Drogue bridle attaches to the **harness**, NOT the spar roots.")
    print("  The wing is still stowed when the drogue is loaded; the spars")
    print("  do not see the snatch load directly in the nominal sequence.")
    print()
    print("  Coupling cases that DO put drogue tension on the spars:")
    print("  - Asymmetric deploy with drogue still attached: brief")
    print("    cross-coupling via the bound wing-harness interface.")
    print("    Bounded by the ground-rig data once that exists.")
    print("  - Drogue-cut-release with one cutter not firing: drogue")
    print("    drag asymmetry into the deployed wing. Mitigated by")
    print("    redundant drogue release.")

    # Save CSV time series
    with (out_dir / "drogue_descent.csv").open("w") as f:
        f.write("t_s,V_mps,cda_drogue_m2,F_drag_total_N,F_bridle_N\n")
        for i in range(0, len(res["t"]), 4):  # decimate
            f.write(f"{res['t'][i]:.4f},{res['V'][i]:.3f},"
                    f"{res['cda_drogue'][i]:.4f},{res['F_drag_total'][i]:.1f},"
                    f"{res['F_bridle'][i]:.1f}\n")

    # Plot
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        axes[0].plot(res["t"], res["V"], color="tab:blue")
        axes[0].axhline(cfg.V_target_after_drogue, color="grey", linestyle="--",
                          label=f"target V = {cfg.V_target_after_drogue} m/s")
        axes[0].axhline(cfg.V_terminal_target, color="grey", linestyle=":",
                          label=f"terminal V = {cfg.V_terminal_target} m/s")
        axes[0].set_ylabel("V (m/s)")
        axes[0].set_title("MANTA drogue descent — V vs. t")
        axes[0].legend(fontsize=9)
        axes[0].grid(True, alpha=0.3)

        axes[1].plot(res["t"], res["cda_drogue"], color="tab:green")
        axes[1].set_ylabel("drogue CdA (m²)")
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(res["t"], res["F_bridle"] / 1000, color="tab:red", label="bridle tension")
        axes[2].axhline(F_peak / 1000, color="black", linestyle="--",
                          label=f"peak {F_peak/1000:.2f} kN ({g_peak:.2f} g)")
        axes[2].axhline(4.5 * cfg.m_total_kg * cfg.g / 1000, color="purple", linestyle=":",
                          label="4.5 g ultimate flight")
        axes[2].set_ylabel("bridle tension (kN)")
        axes[2].set_xlabel("t after drogue extract cmd (s)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(fontsize=9)

        fig.tight_layout()
        fig.savefig(out_dir / "drogue_descent.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
