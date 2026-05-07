"""
Generate a quick top-view + side-view planform PNG for inclusion in docs.
Independent of CadQuery — pure matplotlib + numpy. Reads the same Planform.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402


def main() -> None:
    p = Planform()
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    # Sample y stations symmetrically
    n = 41
    y_half = np.linspace(-p.half_span, p.half_span, n)
    chord = np.array([p.chord_at(yi) for yi in y_half])
    x_le = np.array([p.x_le_at(yi) for yi in y_half])
    x_te = x_le + chord
    x_c4 = x_le + 0.25 * chord

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(11, 5.5))

    # Planform outline
    ax.plot(np.concatenate([y_half, y_half[::-1]]),
            np.concatenate([x_le, x_te[::-1]]),
            color="black", linewidth=1.4)
    # Quarter chord line
    ax.plot(y_half, x_c4, color="C1", linestyle="--", linewidth=1.0, label="c/4 line")
    # Mean aerodynamic chord
    ax.plot([p.y_mac, p.y_mac], [p.x_mac_le, p.x_mac_le + p.mac],
            color="C2", linewidth=2.0, label=f"MAC ({p.mac:.3f} m) at y={p.y_mac:.3f} m")
    ax.plot([-p.y_mac, -p.y_mac], [p.x_mac_le, p.x_mac_le + p.mac],
            color="C2", linewidth=2.0)
    # MAC c/4 marker
    ax.plot([0], [p.x_mac_c4], marker="x", color="C3", markersize=10,
            markeredgewidth=2, label=f"MAC c/4 (x={p.x_mac_c4:.3f} m)")

    # Annotations
    ax.invert_yaxis()  # so +x (aft) goes downward in the plot, matching top-view convention
    ax.set_xlabel("y — span (m, +y starboard)")
    ax.set_ylabel("x — aft (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(
        f"MANTA wing planform\n"
        f"S = {p.S} m², b = {p.b} m, AR = {p.aspect_ratio:.3f}, "
        f"sweep_LE = {p.sweep_le_deg:.1f}°, taper = {p.taper}, washout = {p.washout_deg:.1f}°"
    )
    ax.legend(loc="lower center", framealpha=0.9)

    fig.tight_layout()
    fig.savefig(out_dir / "planform_top.png", dpi=160)
    plt.close(fig)
    print(f"Wrote {out_dir / 'planform_top.png'}")


if __name__ == "__main__":
    main()
