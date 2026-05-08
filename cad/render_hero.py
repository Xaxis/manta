"""
Generate a hero render for the README — 3D iso view of the wing OML plus
spars plus cutters, all from the same parametric source.

Pure matplotlib (mpl_toolkits.mplot3d) so it doesn't require external
renderers; resolution is fine for README embedding.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402


def _airfoil_xy(n: int = 40, t_c: float = 0.10, camber: float = 0.012):
    beta = np.linspace(0, np.pi, n)
    x = (1 - np.cos(beta)) * 0.5
    yt = 5 * t_c * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 +
                    0.2843 * x ** 3 - 0.1036 * x ** 4)
    yc = camber * 4 * x * (1 - x) * (1 - 2 * x)
    return x, yc + yt, yc - yt


def _section_3d(x_af, y_up, y_lo, y_station: float, chord: float,
                 x_le: float, twist_deg: float):
    twist = math.radians(twist_deg)
    cos_t, sin_t = math.cos(twist), math.sin(twist)

    def _pt(xn, yn):
        xr, zr = xn - 0.25, yn
        x_rot = cos_t * xr - sin_t * zr
        z_rot = sin_t * xr + cos_t * zr
        return ((x_rot + 0.25) * chord + x_le,
                y_station,
                z_rot * chord)
    upper = [_pt(xi, yi) for xi, yi in zip(x_af, y_up)]
    lower = [_pt(xi, yi) for xi, yi in zip(x_af[::-1], y_lo[::-1])]
    return upper + lower


def main() -> None:
    p = Planform()
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    n_stations = 16
    n_af = 40
    x_af, y_up, y_lo = _airfoil_xy(n_af)

    # Build sections per side
    eta_arr = np.linspace(0, 1, n_stations)
    sections = {"left": [], "right": []}
    for side, sign in (("right", +1), ("left", -1)):
        for eta in eta_arr:
            y = sign * eta * p.half_span
            chord = p.chord_at(y)
            x_le = p.x_le_at(y)
            twist = p.twist_at(y)
            sections[side].append(_section_3d(x_af, y_up, y_lo, y, chord, x_le, twist))

    fig = plt.figure(figsize=(16, 9))
    ax = fig.add_subplot(111, projection="3d")

    # Skin: triangulate between adjacent sections
    skin_polys = []
    for side in ("right", "left"):
        secs = sections[side]
        for i in range(len(secs) - 1):
            s0 = secs[i]
            s1 = secs[i + 1]
            n = len(s0) // 2
            # connect upper surface
            for k in range(n - 1):
                # Quad:  s0[k], s0[k+1], s1[k+1], s1[k]
                poly = [s0[k], s0[k + 1], s1[k + 1], s1[k]]
                skin_polys.append(poly)
            # connect lower surface
            for k in range(n, 2 * n - 1):
                poly = [s0[k], s0[k + 1], s1[k + 1], s1[k]]
                skin_polys.append(poly)

    skin_coll = Poly3DCollection(skin_polys, alpha=0.45, facecolor="#79a8ff",
                                  edgecolor="none", linewidths=0)
    ax.add_collection3d(skin_coll)

    # Section ribs (wireframe, every nth section)
    for side in ("right", "left"):
        for i, sec in enumerate(sections[side]):
            if i % 3 != 0:
                continue
            xs = [pt[0] for pt in sec] + [sec[0][0]]
            ys = [pt[1] for pt in sec] + [sec[0][1]]
            zs = [pt[2] for pt in sec] + [sec[0][2]]
            ax.plot(xs, ys, zs, color="#1a3a8e", linewidth=0.6, alpha=0.5)

    # Spars — front and rear at sized-config dimensions, stitched as thin lines
    for x_chord, label, lw, color in [
        (0.20 * p.chord_root, "front spar", 3.0, "#222"),
        (0.65 * p.chord_root, "rear spar", 2.0, "#555"),
    ]:
        ys = np.linspace(-p.half_span, p.half_span, 80)
        xs_s = np.full_like(ys, x_chord)
        zs_s = np.zeros_like(ys)
        ax.plot(xs_s, ys, zs_s, color=color, linewidth=lw, label=label, alpha=0.85)

    # Cutter / root fitting markers
    for x_chord in (0.20 * p.chord_root, 0.65 * p.chord_root):
        ax.scatter([x_chord], [0], [0], color="red", s=80, marker="o",
                    label=None, zorder=5)
    ax.scatter([], [], [], color="red", s=80, marker="o", label="cutter / root fitting (×4)")

    # Wing apex marker
    ax.scatter([0], [0], [0], color="black", s=40, marker="x", label="wing apex (origin)")

    # MAC c/4 marker
    ax.scatter([p.x_mac_c4], [0], [0], color="orange", s=120, marker="*",
                label=f"MAC c/4 ({p.x_mac_c4:.3f} m)")

    ax.set_xlabel("x — aft (m)", fontsize=11)
    ax.set_ylabel("y — span (m)", fontsize=11)
    ax.set_zlabel("z — up (m)", fontsize=11)
    ax.set_title(
        f"MANTA — deployed wing\n"
        f"S = {p.S} m², b = {p.b} m, AR = {p.aspect_ratio:.2f}, "
        f"sweep_LE = {p.sweep_le_deg:.1f}°, taper = {p.taper}, washout = {p.washout_deg:.1f}°",
        fontsize=12,
    )
    ax.legend(loc="upper left", fontsize=10, bbox_to_anchor=(0.0, 0.95))

    # Iso-ish view
    ax.view_init(elev=22, azim=-55)
    xspan = (-0.3, 2.5)
    yspan = (-4.0, 4.0)
    zspan = (-0.4, 1.0)
    ax.set_xlim(xspan)
    ax.set_ylim(yspan)
    ax.set_zlim(zspan)
    ax.set_box_aspect((xspan[1] - xspan[0], yspan[1] - yspan[0], zspan[1] - zspan[0]))

    fig.savefig(out_dir / "hero.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_dir / 'hero.png'}")

    # Also a clean top-down companion
    fig2, ax2 = plt.subplots(figsize=(13, 5))
    ys = np.linspace(-p.half_span, p.half_span, 60)
    chord = np.array([p.chord_at(yi) for yi in ys])
    x_le = np.array([p.x_le_at(yi) for yi in ys])
    x_te = x_le + chord
    ax2.fill_between(ys, x_le, x_te, color="#79a8ff", alpha=0.4, label="wing planform")
    ax2.plot(ys, x_le + 0.25 * chord, "--", color="#1a3a8e", label="c/4 line")
    for x_chord, lab, lw in [(0.20 * p.chord_root, "front spar", 3),
                               (0.65 * p.chord_root, "rear spar", 2)]:
        ax2.plot(ys, np.full_like(ys, x_chord), "-", color="black", linewidth=lw, label=lab)
    ax2.plot([0], [p.x_mac_c4], "*", color="orange", markersize=18, label=f"MAC c/4 ({p.x_mac_c4:.3f} m)")
    for xc in (0.20 * p.chord_root, 0.65 * p.chord_root):
        ax2.plot([+0.030, -0.030], [xc, xc], "ro", markersize=10)
    ax2.plot([], [], "ro", markersize=10, label="cutter / root fitting")
    ax2.invert_yaxis()
    ax2.set_xlabel("y — span (m)")
    ax2.set_ylabel("x — aft (m)")
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="lower center", framealpha=0.95)
    ax2.set_title("MANTA wing — top view (planform + spars + cutters)")
    fig2.tight_layout()
    fig2.savefig(out_dir / "topview.png", dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print(f"wrote {out_dir / 'topview.png'}")


if __name__ == "__main__":
    main()
