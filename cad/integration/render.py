"""
Generate iso + side + top renderings of the integrated assembly to make the
"how does this affix to the human body" question answerable visually.

Matplotlib 3D — no external renderer required, runs in CI.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

from cad.integration.build import (  # noqa: E402
    PILOT_BACK_Z, PILOT_HEAD_X, PILOT_LENGTH, PILOT_THICKNESS, PILOT_WIDTH,
    MAIN_LENGTH, MAIN_WIDTH, MAIN_THICKNESS, MAIN_X_CENTER,
    RES_LENGTH, RES_WIDTH, RES_THICKNESS, RES_X_CENTER,
    SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS, SUBFRAME_X_CENTER,
    STUB_HEIGHT, FRONT_SPAR_X_OVER_C, REAR_SPAR_X_OVER_C, STUB_Y_OFFSET,
    CONE_HALF_ANGLE_DEG, CONE_HEIGHT,
    _spar_x_at_y, _airfoil_xy, _section_pts3d,
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402


# -----------------------------------------------------------
# Helpers — build mesh primitives for matplotlib
# -----------------------------------------------------------

def _box_polys(cx, cy, cz, dx, dy, dz):
    """Six faces of an axis-aligned box centered at (cx, cy, cz)."""
    x0, x1 = cx - dx / 2, cx + dx / 2
    y0, y1 = cy - dy / 2, cy + dy / 2
    z0, z1 = cz - dz / 2, cz + dz / 2
    v = np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],  # bottom
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],  # top
    ])
    return [
        [v[0], v[1], v[2], v[3]],   # bottom
        [v[4], v[5], v[6], v[7]],   # top
        [v[0], v[1], v[5], v[4]],   # -y
        [v[2], v[3], v[7], v[6]],   # +y
        [v[0], v[3], v[7], v[4]],   # -x
        [v[1], v[2], v[6], v[5]],   # +x
    ]


def _swept_tube_polys(P_in, P_out, od, n_around: int = 16):
    """Approximate a swept tube as a band of triangles between two end circles."""
    P_in = np.array(P_in)
    P_out = np.array(P_out)
    axis = P_out - P_in
    L = np.linalg.norm(axis)
    if L < 1e-9:
        return []
    a = axis / L
    # Find a perpendicular vector
    if abs(a[2]) < 0.9:
        u = np.array([0.0, 0.0, 1.0])
    else:
        u = np.array([1.0, 0.0, 0.0])
    e1 = u - np.dot(u, a) * a
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)

    r = od / 2
    polys = []
    angles = np.linspace(0, 2 * np.pi, n_around, endpoint=False)
    pts_in = [P_in + r * (math.cos(t) * e1 + math.sin(t) * e2) for t in angles]
    pts_out = [P_out + r * (math.cos(t) * e1 + math.sin(t) * e2) for t in angles]
    for i in range(n_around):
        i_next = (i + 1) % n_around
        polys.append([pts_in[i], pts_in[i_next], pts_out[i_next], pts_out[i]])
    return polys


def _wing_skin_polys(p: Planform, n_stations: int = 14, n_af: int = 30):
    xs2d, ys2d = _airfoil_xy(n=n_af)
    sections = []
    eta = np.linspace(0, 1, n_stations)
    for side_sign in (+1, -1):
        section_set = []
        for e in eta:
            y = side_sign * e * p.half_span
            chord = p.chord_at(y)
            x_le = p.x_le_at(y)
            twist = p.twist_at(y)
            pts = _section_pts3d(xs2d, ys2d, y, chord, x_le, twist)
            # shift up so wing chord plane is at z = sub-frame top
            z_offset = SUBFRAME_THICKNESS / 2 + 0.005 + (
                max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS / 2
            )
            pts = [(pt[0], pt[1], pt[2] + z_offset) for pt in pts]
            section_set.append(pts)
        sections.append(section_set)

    polys = []
    for section_set in sections:
        for i in range(len(section_set) - 1):
            s0, s1 = section_set[i], section_set[i + 1]
            n = len(s0)
            for k in range(n - 1):
                polys.append([s0[k], s0[k + 1], s1[k + 1], s1[k]])
    return polys


def _swept_spar_polys(p: Planform, x_over_c: float,
                       od_root: float, od_mid: float, od_tip: float):
    """Build polygons for the three-stage swept spar (per side)."""
    sweep_deg = p.sweep_at_chord_fraction_deg(x_over_c)
    cos_sweep = math.cos(math.radians(sweep_deg))
    half_b = p.half_span
    L_stage_axial = half_b / 3.0 / cos_sweep + 2 * 0.025
    joint_overlap_axial = 0.05

    z_spar = SUBFRAME_THICKNESS / 2 + 0.005 + (
        max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS / 2
    )

    polys = []
    for side in (+1, -1):
        y_cursor = 0.0
        for od in (od_root, od_mid, od_tip):
            y_extent = L_stage_axial * cos_sweep
            y_in = y_cursor
            y_out = min(y_cursor + y_extent, half_b)
            P_in = (_spar_x_at_y(p, x_over_c, side * y_in), side * y_in, z_spar)
            P_out = (_spar_x_at_y(p, x_over_c, side * y_out), side * y_out, z_spar)
            polys.extend(_swept_tube_polys(P_in, P_out, od))
            y_cursor += y_extent - joint_overlap_axial * cos_sweep
    return polys


# -----------------------------------------------------------
# Driver
# -----------------------------------------------------------

def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    # Pilot, rig, sub-frame
    pilot_polys = _box_polys(
        PILOT_HEAD_X + PILOT_LENGTH / 2, 0, PILOT_BACK_Z - PILOT_THICKNESS / 2,
        PILOT_LENGTH, PILOT_WIDTH, PILOT_THICKNESS,
    )
    main_polys = _box_polys(
        MAIN_X_CENTER, 0, PILOT_BACK_Z + MAIN_THICKNESS / 2,
        MAIN_LENGTH, MAIN_WIDTH, MAIN_THICKNESS,
    )
    res_polys = _box_polys(
        RES_X_CENTER, 0, PILOT_BACK_Z + RES_THICKNESS / 2,
        RES_LENGTH, RES_WIDTH, RES_THICKNESS,
    )
    sub_z = max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS / 2
    sub_polys = _box_polys(
        SUBFRAME_X_CENTER, 0, sub_z,
        SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS,
    )

    # Wing skin
    skin_polys = _wing_skin_polys(p)

    # Spars
    front_polys = _swept_spar_polys(p, FRONT_SPAR_X_OVER_C, 0.073, 0.051, 0.025)
    rear_polys = _swept_spar_polys(p, REAR_SPAR_X_OVER_C, 0.030, 0.024, 0.018)

    # Stubs (small cylinders)
    z_stub_top = sub_z + SUBFRAME_THICKNESS / 2 + STUB_HEIGHT
    stub_polys = []
    for x_over_c, od in [(FRONT_SPAR_X_OVER_C, 0.073), (REAR_SPAR_X_OVER_C, 0.030)]:
        x_spar = x_over_c * p.chord_root
        for y_sign in (+1, -1):
            P_in = (x_spar, y_sign * STUB_Y_OFFSET, sub_z + SUBFRAME_THICKNESS / 2)
            P_out = (x_spar, y_sign * STUB_Y_OFFSET, z_stub_top)
            stub_polys.extend(_swept_tube_polys(P_in, P_out, od, n_around=20))

    # Reserve cone (wireframe)
    apex_x = RES_X_CENTER
    apex_z = PILOT_BACK_Z + RES_THICKNESS + 0.02

    fig = plt.figure(figsize=(16, 11))

    # ---- iso view ----
    ax_iso = fig.add_subplot(2, 2, (1, 3), projection="3d")

    def add_polys(ax, polys, **kw):
        coll = Poly3DCollection(polys, **kw)
        ax.add_collection3d(coll)

    add_polys(ax_iso, pilot_polys, facecolor="#a87d52", alpha=0.55, edgecolor="#5d3c20", linewidths=0.3, label="pilot")
    add_polys(ax_iso, main_polys, facecolor="#444", alpha=0.85, edgecolor="black", linewidths=0.3)
    add_polys(ax_iso, res_polys, facecolor="#b03030", alpha=0.85, edgecolor="black", linewidths=0.3)
    add_polys(ax_iso, sub_polys, facecolor="#888", alpha=0.95, edgecolor="black", linewidths=0.3)
    add_polys(ax_iso, skin_polys, facecolor="#79a8ff", alpha=0.30, edgecolor="#1a3a8e", linewidths=0.2)
    add_polys(ax_iso, front_polys, facecolor="#222", alpha=0.95, edgecolor="black", linewidths=0.2)
    add_polys(ax_iso, rear_polys, facecolor="#444", alpha=0.95, edgecolor="black", linewidths=0.2)
    add_polys(ax_iso, stub_polys, facecolor="red", alpha=0.95, edgecolor="black", linewidths=0.2)

    # Cone wireframe
    cone_t = np.linspace(0, 2 * np.pi, 32)
    cone_h = np.linspace(0, CONE_HEIGHT, 8)
    for h in cone_h:
        r = h * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
        ax_iso.plot(apex_x + r * np.cos(cone_t),
                     r * np.sin(cone_t),
                     apex_z + h, color="orange", alpha=0.25, linewidth=0.7)

    ax_iso.set_xlabel("x — aft (m)")
    ax_iso.set_ylabel("y — span (m)")
    ax_iso.set_zlabel("z — up (m)")
    ax_iso.set_title("MANTA — integrated assembly (iso)\nWing on pilot's back, swept spars in OML, reserve cone clear of stubs")
    ax_iso.view_init(elev=22, azim=-55)
    xspan = (-0.3, 2.5)
    yspan = (-4.0, 4.0)
    zspan = (-0.6, 0.8)
    ax_iso.set_xlim(xspan)
    ax_iso.set_ylim(yspan)
    ax_iso.set_zlim(zspan)
    ax_iso.set_box_aspect((xspan[1] - xspan[0], yspan[1] - yspan[0], zspan[1] - zspan[0]))

    # ---- side view (x-z plane) ----
    ax_side = fig.add_subplot(2, 2, 2)
    # pilot
    ax_side.add_patch(plt.Rectangle(
        (PILOT_HEAD_X, PILOT_BACK_Z - PILOT_THICKNESS),
        PILOT_LENGTH, PILOT_THICKNESS,
        facecolor="#a87d52", edgecolor="#5d3c20", alpha=0.5, label="pilot"))
    # main
    ax_side.add_patch(plt.Rectangle(
        (MAIN_X_CENTER - MAIN_LENGTH / 2, PILOT_BACK_Z),
        MAIN_LENGTH, MAIN_THICKNESS,
        facecolor="#444", edgecolor="black", alpha=0.85, label="main"))
    # reserve
    ax_side.add_patch(plt.Rectangle(
        (RES_X_CENTER - RES_LENGTH / 2, PILOT_BACK_Z),
        RES_LENGTH, RES_THICKNESS,
        facecolor="#b03030", edgecolor="black", alpha=0.85, label="reserve"))
    # subframe
    ax_side.add_patch(plt.Rectangle(
        (SUBFRAME_X_CENTER - SUBFRAME_LENGTH / 2, sub_z - SUBFRAME_THICKNESS / 2),
        SUBFRAME_LENGTH, SUBFRAME_THICKNESS,
        facecolor="#888", edgecolor="black", alpha=0.95, label="sub-frame"))
    # wing root chord profile
    z_w = sub_z + SUBFRAME_THICKNESS / 2 + 0.005
    xs2d, ys2d = _airfoil_xy(50)
    chord = p.chord_root
    ax_side.fill(xs2d * chord, ys2d * chord + z_w,
                  facecolor="#79a8ff", edgecolor="#1a3a8e", alpha=0.35,
                  label="wing root section")
    # spars (front + rear) projected onto side view (their x at root)
    for x_over_c, od, color, name in [(FRONT_SPAR_X_OVER_C, 0.073, "#222", "front spar"),
                                       (REAR_SPAR_X_OVER_C, 0.030, "#444", "rear spar")]:
        x_root = x_over_c * chord
        ax_side.add_patch(plt.Circle((x_root, z_w), od / 2,
                                      facecolor=color, edgecolor="black", label=name))
    # cone
    cone_zs = np.linspace(0, CONE_HEIGHT * 0.6, 30)
    cone_rs = cone_zs * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
    ax_side.fill_betweenx(apex_z + cone_zs, apex_x - cone_rs, apex_x + cone_rs,
                            color="orange", alpha=0.10, label="reserve cone")
    ax_side.set_xlim(-0.2, 2.5)
    ax_side.set_ylim(-0.6, 0.8)
    ax_side.set_aspect("equal")
    ax_side.set_xlabel("x — aft (m)")
    ax_side.set_ylabel("z — up (m)")
    ax_side.set_title("Side view — pilot, rig stack, sub-frame, wing root section")
    ax_side.legend(loc="upper right", fontsize=7, framealpha=0.95)
    ax_side.grid(True, alpha=0.3)

    # ---- top view (x-y plane) ----
    ax_top = fig.add_subplot(2, 2, 4)
    # planform outline
    ys = np.linspace(-p.half_span, p.half_span, 80)
    x_le = np.array([p.x_le_at(yi) for yi in ys])
    chord_arr = np.array([p.chord_at(yi) for yi in ys])
    x_te = x_le + chord_arr
    ax_top.fill_between(ys, x_le, x_te, color="#79a8ff", alpha=0.30, label="wing OML")
    # spars
    for x_over_c, color, name in [(FRONT_SPAR_X_OVER_C, "#222", "front spar"),
                                   (REAR_SPAR_X_OVER_C, "#666", "rear spar")]:
        x_arr = np.array([_spar_x_at_y(p, x_over_c, yi) for yi in ys])
        ax_top.plot(ys, x_arr, color=color, linewidth=2, label=name)
    # pilot footprint
    ax_top.add_patch(plt.Rectangle(
        (-PILOT_WIDTH / 2, PILOT_HEAD_X),
        PILOT_WIDTH, PILOT_LENGTH,
        facecolor="#a87d52", edgecolor="#5d3c20", alpha=0.50, label="pilot footprint"))
    # main
    ax_top.add_patch(plt.Rectangle(
        (-MAIN_WIDTH / 2, MAIN_X_CENTER - MAIN_LENGTH / 2),
        MAIN_WIDTH, MAIN_LENGTH,
        facecolor="#444", edgecolor="black", alpha=0.85, label="main"))
    # reserve
    ax_top.add_patch(plt.Rectangle(
        (-RES_WIDTH / 2, RES_X_CENTER - RES_LENGTH / 2),
        RES_WIDTH, RES_LENGTH,
        facecolor="#b03030", edgecolor="black", alpha=0.85, label="reserve"))
    # cutter stubs
    for x_over_c, od in [(FRONT_SPAR_X_OVER_C, 0.073), (REAR_SPAR_X_OVER_C, 0.030)]:
        x_spar = x_over_c * p.chord_root
        for y_sign in (+1, -1):
            ax_top.add_patch(plt.Circle(
                (y_sign * STUB_Y_OFFSET, x_spar), od / 2,
                facecolor="red", edgecolor="black", zorder=5))
    ax_top.add_patch(plt.Circle((0, RES_X_CENTER), 0.05,
                                  facecolor="orange", edgecolor="black", alpha=0.7,
                                  label="reserve PC apex"))
    ax_top.invert_yaxis()
    ax_top.set_xlabel("y — span (m)")
    ax_top.set_ylabel("x — aft (m)")
    ax_top.set_title("Top view — pilot under the wing, swept spars, cutter stub locations")
    ax_top.set_aspect("equal")
    ax_top.legend(loc="lower center", fontsize=7, framealpha=0.95)
    ax_top.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "iso_render.png", dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_dir / 'iso_render.png'}")


if __name__ == "__main__":
    main()
