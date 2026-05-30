"""
Glide polar: L/D vs airspeed for the MANTA wing across the pilot mass envelope.

Drag model:
    CD(CL)  =  CD0  +  CL² / (π · AR · e)

Span efficiency e is taken from a Weissinger-derived running estimate
(typical value 0.95–1.0 for the swept/twisted MANTA planform; we adopt
e = 0.95 here as a slightly conservative working value pending AVL).

Best-glide airspeed:
    CL_bg  =  sqrt(CD0 · π · AR · e)        (CDi = CD0)
    V_bg   =  sqrt(2·m·g / (ρ · S · CL_bg))

Run as a script for the curves and a Markdown summary table. Outputs to
out/.

References:
    Anderson, *Fundamentals of Aerodynamics*, §6.3 (cruise / best-glide
    derivation). Pope & Goin, *High-Speed Wind Tunnel Testing*, ch. 5
    (induced drag e for swept-tapered planforms).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(_ROOT))

from analysis.aero.lift_drag.cd0 import Cd0Buildup  # noqa: E402
from analysis.aero.planform.geometry import Planform  # noqa: E402

_PLAN = Planform()


@dataclass
class GlideConfig:
    S: float = _PLAN.S                # from the single source of truth
    AR: float = _PLAN.aspect_ratio
    e: float = 0.95
    rho: float = 1.225          # ISA sea level
    g: float = 9.80665
    wing_mass_kg: float = 15.5
    rig_mass_kg: float = 8.0    # avionics + harness + drogue + cutters

    pilot_masses_kg: tuple[float, ...] = (70.0, 82.5, 95.0)

    # CD0 brackets — will be filled from cd0.Cd0Buildup
    cd0_optimistic: float = 0.0
    cd0_nominal: float = 0.0
    cd0_pessimistic: float = 0.0

    @classmethod
    def with_cd0(cls, b: Cd0Buildup) -> "GlideConfig":
        rows = b.sensitivity_table()
        cd0_o, cd0_n, cd0_p = (r[2] for r in rows)
        return cls(cd0_optimistic=cd0_o, cd0_nominal=cd0_n, cd0_pessimistic=cd0_p)


def cl_for_steady_glide(m_kg: float, V_mps: float, cfg: GlideConfig) -> float:
    return m_kg * cfg.g / (0.5 * cfg.rho * V_mps * V_mps * cfg.S)


def cd_for_cl(cl: float, cd0: float, cfg: GlideConfig) -> float:
    return cd0 + cl * cl / (math.pi * cfg.AR * cfg.e)


def best_glide(m_kg: float, cd0: float, cfg: GlideConfig) -> tuple[float, float, float]:
    """Return (V_bg, CL_bg, L/D_bg) at best-glide condition."""
    cl_bg = math.sqrt(cd0 * math.pi * cfg.AR * cfg.e)
    v_bg = math.sqrt(2.0 * m_kg * cfg.g / (cfg.rho * cfg.S * cl_bg))
    cd_bg = 2.0 * cd0  # CDi = CD0 at best L/D
    lod_bg = cl_bg / cd_bg
    return v_bg, cl_bg, lod_bg


def glide_curve(m_kg: float, cd0: float, cfg: GlideConfig, V_min=10.0, V_max=45.0, n=200):
    V = np.linspace(V_min, V_max, n)
    cl = m_kg * cfg.g / (0.5 * cfg.rho * V * V * cfg.S)
    cd = cd0 + cl * cl / (math.pi * cfg.AR * cfg.e)
    lod = cl / cd
    sink = V / lod
    return V, cl, cd, lod, sink


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    b = Cd0Buildup()
    cfg = GlideConfig.with_cd0(b)

    print("# Glide polar — MANTA")
    print()
    print(f"S = {cfg.S} m², AR = {cfg.AR:.3f}, e = {cfg.e}")
    print(f"Wing mass = {cfg.wing_mass_kg} kg, rig allowance = {cfg.rig_mass_kg} kg")
    print()
    print("Best-glide condition across pilot mass × CD0 brackets:")
    print()
    print("|  Pilot   |  Bracket  | CD0    | V_bg (m/s) | CL_bg | (L/D)_max | Sink V/(L/D) (m/s) |")
    print("|---|---|---|---|---|---|---|")
    rows = []
    for m_p in cfg.pilot_masses_kg:
        m_total = m_p + cfg.wing_mass_kg + cfg.rig_mass_kg
        for label, cd0 in [("optimistic", cfg.cd0_optimistic),
                           ("nominal", cfg.cd0_nominal),
                           ("pessimistic", cfg.cd0_pessimistic)]:
            v_bg, cl_bg, lod_bg = best_glide(m_total, cd0, cfg)
            sink = v_bg / lod_bg
            rows.append((m_p, m_total, label, cd0, v_bg, cl_bg, lod_bg, sink))
            print(f"|  {m_p:5.1f} kg | {label:11s} | {cd0:.4f} |   {v_bg:5.1f}    | {cl_bg:.3f} |   {lod_bg:5.2f}   |       {sink:.2f}        |")

    # Also: L/D at the BRIEF target V_bg = 25 m/s, design pilot 82.5 kg
    print()
    print("L/D at BRIEF target V = 25 m/s (pilot 82.5 kg, m_total = 106 kg):")
    m_total = 82.5 + cfg.wing_mass_kg + cfg.rig_mass_kg
    cl_25 = cl_for_steady_glide(m_total, 25.0, cfg)
    for label, cd0 in [("optimistic", cfg.cd0_optimistic),
                       ("nominal", cfg.cd0_nominal),
                       ("pessimistic", cfg.cd0_pessimistic)]:
        cd_25 = cd_for_cl(cl_25, cd0, cfg)
        lod_25 = cl_25 / cd_25
        print(f"  {label:11s}: CL = {cl_25:.3f}, CD = {cd_25:.4f}, L/D = {lod_25:.2f}")

    # Save sensitivity table to CSV
    with (out_dir / "best_glide.csv").open("w") as f:
        f.write("pilot_kg,total_kg,bracket,CD0,V_bg,CL_bg,LD_max,sink\n")
        for r in rows:
            f.write(",".join(f"{x:.5f}" if isinstance(x, float) else str(x) for x in r) + "\n")

    # Plot glide polars (V, L/D) for the design pilot 82.5 kg, all brackets.
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        ax_lod, ax_sink = axes

        m_total = 82.5 + cfg.wing_mass_kg + cfg.rig_mass_kg
        for label, cd0, color in [
            ("optimistic", cfg.cd0_optimistic, "tab:green"),
            ("nominal",    cfg.cd0_nominal,    "tab:blue"),
            ("pessimistic", cfg.cd0_pessimistic, "tab:red"),
        ]:
            V, _cl, _cd, lod, sink = glide_curve(m_total, cd0, cfg)
            ax_lod.plot(V, lod, label=f"{label}  (CD0={cd0:.4f})", color=color)
            ax_sink.plot(V, sink, label=f"{label}", color=color)

        ax_lod.axhline(10.0, color="k", linestyle="--", alpha=0.5, label="BRIEF 10:1 target")
        ax_lod.axvline(25.0, color="grey", linestyle=":", alpha=0.5, label="BRIEF V_bg = 25 m/s")
        ax_lod.axvline(14.0, color="brown", linestyle=":", alpha=0.5, label="BRIEF V_stall = 14 m/s")
        ax_lod.set_xlabel("airspeed V (m/s)")
        ax_lod.set_ylabel("L/D")
        ax_lod.set_title(f"Glide polar — pilot 82.5 kg, m_total = {m_total} kg")
        ax_lod.set_ylim(0, 14)
        ax_lod.grid(True, alpha=0.3)
        ax_lod.legend(loc="lower right", fontsize=8)

        ax_sink.set_xlabel("airspeed V (m/s)")
        ax_sink.set_ylabel("sink rate (m/s)")
        ax_sink.set_title("Sink rate")
        ax_sink.set_ylim(0, 8)
        ax_sink.grid(True, alpha=0.3)
        ax_sink.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        fig.savefig(out_dir / "glide_polar.png", dpi=140)
        plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
