"""
MANTA wing-system mass budget — top-level rollup with sensitivities.

Rolls up:
  - Spar set (front + rear, both sides) using `spar_model.WingSparSet`
  - Ribs, skin, root fittings, pneumatics, FCS, actuators, drogue,
    harness from `components`
  - Margin (% of allocated)

Compares against the BRIEF 15.5 kg target and surfaces:
  - Pass / fail vs. budget
  - Each component's share of budget
  - Sensitivity to key knobs (spar OD/wall, rib count, skin g/m²)

Outputs:
  out/budget_default.csv
  out/budget_sized_spar.csv
  out/sensitivity.md
  out/budget_pie.png
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

from analysis.struct.components import (  # noqa: E402
    Actuators,
    Drogue,
    FlightControl,
    Harness,
    PneumaticSystem,
    RootFittingSet,
    rib_set_mass_kg,
    skin_mass_kg,
)
from analysis.struct.materials import CFRPUDTube  # noqa: E402
from analysis.struct.spar_model import (  # noqa: E402
    SparStage,
    TelescopingSpar,
    WingSparSet,
    default_front_spar,
    default_rear_spar,
)


WING_MASS_TARGET = 15.5  # BRIEF
MARGIN_FRACTION = 0.10   # 10% of allocated for unaccounted growth

CFRP = CFRPUDTube()


def sized_front_spar(od_root_mm: float = 73, wall_mm: float = 2.5) -> TelescopingSpar:
    """The bending-analysis-recommended front spar."""
    od_root = od_root_mm / 1000
    od_mid = od_root * 0.70
    od_tip = 0.025
    L = 3.7 / 3.0 + 2 * 0.025
    wall = wall_mm / 1000
    return TelescopingSpar(
        name="front",
        stages=(
            SparStage("front_root", od_root, wall, L),
            SparStage("front_mid",  od_mid,  wall, L),
            SparStage("front_tip",  od_tip,  wall, L),
        ),
    )


def build_budget(spars: WingSparSet, n_ribs_per_side: int = 9, skin_gpm2: float = 50.0):
    p = Planform()
    rows = [
        ("Spars (4 spars, both sides)",       spars.total_mass_kg(CFRP)),
        ("Ribs (tape-spring × per side)",     rib_set_mass_kg(p, n_per_side=n_ribs_per_side)),
        ("Skin (DCF + bond overhead)",        skin_mass_kg(p, areal_density_kg_per_m2=skin_gpm2 / 1000)),
        ("Root fittings + cutters",           RootFittingSet().total_kg()),
        ("Pneumatic deployment",              PneumaticSystem().total_kg()),
        ("Flight control system",             FlightControl().total_kg()),
        ("Actuators + reversion",             Actuators().total_kg()),
        ("Drogue + bridle",                   Drogue().total_kg()),
        ("Harness shell + interface",         Harness().total_kg()),
    ]
    subtotal = sum(m for _, m in rows)
    margin = subtotal * MARGIN_FRACTION
    rows.append(("Margin (10 % of allocated)", margin))
    total = subtotal + margin
    return rows, total


def print_budget(label: str, rows: list[tuple[str, float]], total: float) -> None:
    print(f"## {label}\n")
    print(f"| Component | Mass (kg) | % of budget |")
    print(f"|---|---|---|")
    for name, m in rows:
        print(f"| {name:35s} | {m:6.3f} | {m / WING_MASS_TARGET * 100:5.1f} % |")
    print(f"| **Total** | **{total:.3f}** | **{total / WING_MASS_TARGET * 100:.1f} %** |")
    delta = total - WING_MASS_TARGET
    status = "WITHIN BUDGET" if total <= WING_MASS_TARGET else "OVER BUDGET"
    print(f"\n**vs BRIEF target {WING_MASS_TARGET} kg: {delta:+.3f} kg → {status}**")
    print()


def sensitivity_sweep() -> list[tuple]:
    """Sweep three knobs, see total mass + budget margin."""
    p = Planform()
    sweeps = []
    for n_ribs in (7, 9, 11):
        for skin_g in (40, 50, 60):
            for which_front in ("BRIEF", "SIZED"):
                spars = WingSparSet(
                    front=default_front_spar() if which_front == "BRIEF" else sized_front_spar(),
                    rear=default_rear_spar(),
                )
                rows, total = build_budget(spars, n_ribs_per_side=n_ribs, skin_gpm2=skin_g)
                sweeps.append((which_front, n_ribs, skin_g, total))
    return sweeps


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    print("# MANTA wing-system mass budget\n")
    # 1) BRIEF defaults
    spars_brief = WingSparSet()
    rows_brief, total_brief = build_budget(spars_brief)
    print_budget("Mass roll-up — BRIEF spar dimensions (40/2 mm front, 30/2 mm rear)", rows_brief, total_brief)

    # 2) Sized spar from bending analysis
    spars_sized = WingSparSet(front=sized_front_spar(), rear=default_rear_spar())
    rows_sized, total_sized = build_budget(spars_sized)
    print_budget("Mass roll-up — bending-sized front spar (73/2.5 mm)", rows_sized, total_sized)

    # CSV outputs
    for label_short, rows, total in (("default", rows_brief, total_brief),
                                      ("sized_spar", rows_sized, total_sized)):
        with (out_dir / f"budget_{label_short}.csv").open("w") as f:
            f.write("component,mass_kg\n")
            for name, m in rows:
                f.write(f"\"{name}\",{m:.4f}\n")
            f.write(f"\"TOTAL\",{total:.4f}\n")

    # Sensitivity sweep
    sweeps = sensitivity_sweep()
    print("## Sensitivity sweep\n")
    print("| Front spar | Ribs/side | Skin (g/m²) | Total (kg) | Δ vs target |")
    print("|---|---|---|---|---|")
    with (out_dir / "sensitivity.md").open("w") as f:
        f.write("# Mass budget — sensitivity sweep\n\n")
        f.write("| Front spar | Ribs/side | Skin (g/m²) | Total (kg) | Δ vs target |\n")
        f.write("|---|---|---|---|---|\n")
        for which_front, n_ribs, skin_g, total in sweeps:
            line = f"| {which_front:5s} | {n_ribs:2d} | {skin_g:2d} | {total:5.3f} | {total - WING_MASS_TARGET:+.3f} |"
            print(line)
            f.write(line + "\n")

    # Pie chart for default budget
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        for label, rows, total in (("BRIEF", rows_brief, total_brief),
                                     ("SIZED", rows_sized, total_sized)):
            fig, ax = plt.subplots(figsize=(8, 8))
            labels = [n for n, _ in rows]
            sizes = [m for _, m in rows]
            ax.pie(sizes, labels=labels, autopct="%1.1f%%", startangle=90)
            ax.set_title(f"MANTA wing-system mass: {label} ({total:.2f} kg total, target {WING_MASS_TARGET} kg)")
            fig.tight_layout()
            fig.savefig(out_dir / f"budget_pie_{label.lower()}.png", dpi=140)
            plt.close(fig)
    except ImportError:
        pass


if __name__ == "__main__":
    main()
