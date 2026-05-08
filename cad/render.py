"""
Multi-view renders of the corrected architecture for the project site.

Three views: top, side, iso. Both stowed and deployed.
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


# -------------------------------------------------------------
# Geometry — same parameters as cad/build.py
# -------------------------------------------------------------

# Anthropometry
TORSO_LEN = 0.55
TORSO_W = 0.36
TORSO_T = 0.22
SHOULDER_Y = 0.18
HIP_Y = 0.12
UPPER_ARM = 0.30
FOREARM = 0.27
HAND = 0.20
UPPER_LEG = 0.45
LOWER_LEG = 0.42

# Architecture
SHOULDER_X = 0.30
SHOULDER_Z = 0.10
HIP_X = -0.20
HIP_Z = -0.02
TE_HUB_X = -0.55
TE_HUB_Z = 0.05

ARM_DEPLOY_SWEEP = math.radians(25.0)
ARM_DEPLOY_DIVE = 0.0
LEG_DEPLOY_SPREAD = math.radians(25.0)

# LE / TE half-spans
HALF_B = 3.7
LE_SWEEP = math.radians(25.0)
TE_SWEEP = math.radians(11.5)


# -------------------------------------------------------------
# Helpers
# -------------------------------------------------------------

def _tube_pts(P_in, P_out, od, n_around=10):
    """Triangulate a tube between two endpoints."""
    P_in = np.array(P_in)
    P_out = np.array(P_out)
    axis = P_out - P_in
    L = np.linalg.norm(axis)
    if L < 1e-9:
        return []
    a = axis / L
    if abs(a[2]) < 0.9:
        u = np.array([0.0, 0.0, 1.0])
    else:
        u = np.array([1.0, 0.0, 0.0])
    e1 = u - np.dot(u, a) * a
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    r = od / 2
    angles = np.linspace(0, 2 * np.pi, n_around, endpoint=False)
    pts_in = [P_in + r * (math.cos(t) * e1 + math.sin(t) * e2) for t in angles]
    pts_out = [P_out + r * (math.cos(t) * e1 + math.sin(t) * e2) for t in angles]
    polys = []
    for i in range(n_around):
        j = (i + 1) % n_around
        polys.append([pts_in[i], pts_in[j], pts_out[j], pts_out[i]])
    return polys


def _box_polys(cx, cy, cz, dx, dy, dz):
    x0, x1 = cx - dx / 2, cx + dx / 2
    y0, y1 = cy - dy / 2, cy + dy / 2
    z0, z1 = cz - dz / 2, cz + dz / 2
    v = np.array([
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ])
    return [
        [v[0], v[1], v[2], v[3]], [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]], [v[2], v[3], v[7], v[6]],
        [v[0], v[3], v[7], v[4]], [v[1], v[2], v[6], v[5]],
    ]


def arm_endpoint(side: int, deploy: bool):
    """Return wrist position for a given side at given state."""
    if not deploy:
        # Stowed: arms hanging close to torso, slightly down
        sh_x, sh_y, sh_z = SHOULDER_X, side * SHOULDER_Y, SHOULDER_Z
        # Pointing -z (down) slightly
        arm_dir = (0.0, side * 0.15, -0.95)
    else:
        # Deployed: spread out laterally, swept aft per LE sweep
        sh_x, sh_y, sh_z = SHOULDER_X, side * SHOULDER_Y, SHOULDER_Z
        sweep = ARM_DEPLOY_SWEEP
        arm_dir = (-math.sin(sweep), side * math.cos(sweep), 0.0)
    arm_total = UPPER_ARM + FOREARM + HAND * 0.5
    wr = (
        sh_x + arm_total * arm_dir[0],
        sh_y + arm_total * arm_dir[1],
        sh_z + arm_total * arm_dir[2],
    )
    return (sh_x, sh_y, sh_z), wr, arm_dir


def le_tip(side: int, deploy: bool):
    """End of telescoping wrist tip extension."""
    sh, wr, arm_dir = arm_endpoint(side, deploy)
    ext_total = 3.30 if deploy else 0.30
    tip = (
        wr[0] + ext_total * arm_dir[0],
        wr[1] + ext_total * arm_dir[1],
        wr[2] + ext_total * arm_dir[2],
    )
    return wr, tip, arm_dir


def te_tip(side: int, deploy: bool):
    """End of TE spar."""
    hub = (TE_HUB_X, 0, TE_HUB_Z)
    if not deploy:
        L = 0.40
    else:
        L = HALF_B / math.cos(TE_SWEEP)
    direction = (-math.sin(TE_SWEEP), side * math.cos(TE_SWEEP), 0.0) if deploy else (0.0, side * 0.95, 0.0)
    if not deploy:
        L = 0.30
    tip = (
        hub[0] + L * direction[0],
        hub[1] + L * direction[1],
        hub[2] + L * direction[2],
    )
    return hub, tip


def collect_polys(deploy: bool):
    """Return a list of (color, alpha, polys) tuples for the architecture."""
    layers = []

    # Pilot torso (with simple head)
    torso_x = (SHOULDER_X + HIP_X) / 2
    torso = _box_polys(torso_x, 0, 0, TORSO_LEN, TORSO_W, TORSO_T)
    layers.append(("#a87d52", 0.85, torso))

    # Head
    head_x = SHOULDER_X + 0.12
    head = _box_polys(head_x, 0, 0.05, 0.18, 0.18, 0.20)
    layers.append(("#c4956a", 0.95, head))

    # Spine yoke
    spine = _tube_pts((SHOULDER_X - 0.05, 0, TORSO_T / 2 + 0.012),
                      (TE_HUB_X + 0.05, 0, TORSO_T / 2 + 0.012), 0.045)
    layers.append(("#1a1a1a", 0.98, spine))

    # Arms + LE spars + tip extensions
    for side in (+1, -1):
        sh, wr, arm_dir = arm_endpoint(side, deploy)
        # Upper arm + forearm + hand as cylinders
        e1 = (sh[0] + UPPER_ARM * arm_dir[0],
              sh[1] + UPPER_ARM * arm_dir[1],
              sh[2] + UPPER_ARM * arm_dir[2])
        e2 = (e1[0] + FOREARM * arm_dir[0],
              e1[1] + FOREARM * arm_dir[1],
              e1[2] + FOREARM * arm_dir[2])
        e3 = (e2[0] + HAND * 0.5 * arm_dir[0],
              e2[1] + HAND * 0.5 * arm_dir[1],
              e2[2] + HAND * 0.5 * arm_dir[2])
        layers.append(("#a87d52", 0.85, _tube_pts(sh, e1, 0.10)))
        layers.append(("#a87d52", 0.85, _tube_pts(e1, e2, 0.08)))
        layers.append(("#a87d52", 0.85, _tube_pts(e2, e3, 0.06)))
        # LE spar runs alongside arm, offset in +z
        offset_z = 0.10 / 2 + 0.062 / 2 + 0.005
        spar_in = (sh[0], sh[1], sh[2] + offset_z)
        spar_out = (wr[0], wr[1], wr[2] + offset_z)
        layers.append(("#222", 0.98, _tube_pts(spar_in, spar_out, 0.062)))
        # Wrist tip extension
        _, tip, _ = le_tip(side, deploy)
        # Three stages with decreasing diameter
        n_stages = 3
        stage_lens = [
            (tip[0] - wr[0]) / n_stages,
            (tip[1] - wr[1]) / n_stages,
            (tip[2] - wr[2]) / n_stages,
        ]
        cur = (wr[0] + offset_z * 0, wr[1], wr[2] + offset_z)  # start at wrist height of LE spar
        for i in range(n_stages):
            od = 0.046 + (0.020 - 0.046) * (i + 0.5) / n_stages
            stg_out = (
                cur[0] + stage_lens[0],
                cur[1] + stage_lens[1],
                cur[2] + stage_lens[2],
            )
            layers.append(("#1a1a1a", 0.95, _tube_pts(cur, stg_out, od)))
            cur = stg_out

    # Legs
    for side in (+1, -1):
        if deploy:
            spread = LEG_DEPLOY_SPREAD
            leg_dir = (-math.cos(spread), side * math.sin(spread), 0.0)
        else:
            leg_dir = (-1.0, 0.0, 0.0)
        hip = (HIP_X, side * HIP_Y, HIP_Z)
        knee = (hip[0] + UPPER_LEG * leg_dir[0],
                hip[1] + UPPER_LEG * leg_dir[1],
                hip[2] + UPPER_LEG * leg_dir[2])
        ankle = (knee[0] + LOWER_LEG * leg_dir[0],
                 knee[1] + LOWER_LEG * leg_dir[1],
                 knee[2] + LOWER_LEG * leg_dir[2])
        layers.append(("#a87d52", 0.85, _tube_pts(hip, knee, 0.15)))
        layers.append(("#a87d52", 0.85, _tube_pts(knee, ankle, 0.10)))

    # TE spar
    for side in (+1, -1):
        hub, tip = te_tip(side, deploy)
        n_stages = 3
        stage_lens = [
            (tip[0] - hub[0]) / n_stages,
            (tip[1] - hub[1]) / n_stages,
            (tip[2] - hub[2]) / n_stages,
        ]
        cur = hub
        for i in range(n_stages):
            od = 0.040 + (0.018 - 0.040) * (i + 0.5) / n_stages
            stg_out = (
                cur[0] + stage_lens[0],
                cur[1] + stage_lens[1],
                cur[2] + stage_lens[2],
            )
            layers.append(("#444", 0.95, _tube_pts(cur, stg_out, od)))
            cur = stg_out

    # Wing skin (deployed only)
    if deploy:
        for side in (+1, -1):
            sh, wr, arm_dir = arm_endpoint(side, deploy)
            _, le_tip_pt, _ = le_tip(side, deploy)
            hub, te_tip_pt = te_tip(side, deploy)
            # Quad: shoulder → wrist → wingtip LE → wingtip TE → te_hub → ...
            # Build a swept quad of the wing planform
            le_root = (SHOULDER_X, 0, SHOULDER_Z)
            te_root = (TE_HUB_X, 0, TE_HUB_Z)
            n = 8
            sections = []
            for i in range(n + 1):
                eta = i / n
                le_pt = (
                    le_root[0] * (1 - eta) + le_tip_pt[0] * eta,
                    le_root[1] * (1 - eta) + le_tip_pt[1] * eta,
                    le_root[2] * (1 - eta) + le_tip_pt[2] * eta,
                )
                te_pt = (
                    te_root[0] * (1 - eta) + te_tip_pt[0] * eta,
                    te_root[1] * (1 - eta) + te_tip_pt[1] * eta,
                    te_root[2] * (1 - eta) + te_tip_pt[2] * eta,
                )
                sections.append((le_pt, te_pt))
            # Triangulate
            polys = []
            for i in range(n):
                le0, te0 = sections[i]
                le1, te1 = sections[i + 1]
                polys.append([le0, le1, te1, te0])
            layers.append(("#79a8ff", 0.30, polys))

    return layers


# -------------------------------------------------------------
# Render
# -------------------------------------------------------------

def render(deploy: bool, ax_iso, ax_top, ax_side):
    layers = collect_polys(deploy)

    # Iso
    for color, alpha, polys in layers:
        coll = Poly3DCollection(polys, facecolor=color, alpha=alpha,
                                 edgecolor="black", linewidths=0.15)
        ax_iso.add_collection3d(coll)
    ax_iso.set_xlim(-3.0, 1.0)
    ax_iso.set_ylim(-4.0, 4.0)
    ax_iso.set_zlim(-0.8, 0.8)
    ax_iso.set_box_aspect((4.0, 8.0, 1.6))
    ax_iso.view_init(elev=22, azim=-60)
    ax_iso.set_xlabel("x — fwd (m)")
    ax_iso.set_ylabel("y — span (m)")
    ax_iso.set_zlabel("z — up (m)")

    # Top view: project polys onto xy plane
    for color, alpha, polys in layers:
        for poly in polys:
            xs = [p[0] for p in poly] + [poly[0][0]]
            ys = [p[1] for p in poly] + [poly[0][1]]
            ax_top.fill(ys, xs, color=color, alpha=alpha * 0.6,
                         edgecolor="black", linewidth=0.15)
    ax_top.set_xlabel("y — span (m)")
    ax_top.set_ylabel("x — fwd (m)")
    ax_top.set_aspect("equal")
    ax_top.invert_yaxis()
    ax_top.grid(True, alpha=0.3)

    # Side view: project polys onto xz plane (max-y projection)
    for color, alpha, polys in layers:
        for poly in polys:
            xs = [p[0] for p in poly] + [poly[0][0]]
            zs = [p[2] for p in poly] + [poly[0][2]]
            # Skip far-side polys (y > 0 mirror) for clarity in side view
            if all(p[1] > 0.5 for p in poly):
                continue
            ax_side.fill(xs, zs, color=color, alpha=alpha * 0.5,
                          edgecolor="black", linewidth=0.15)
    ax_side.set_xlabel("x — fwd (m)")
    ax_side.set_ylabel("z — up (m)")
    ax_side.set_aspect("equal")
    ax_side.grid(True, alpha=0.3)


def main():
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    for deploy, label in [(True, "deployed"), (False, "stowed")]:
        fig = plt.figure(figsize=(16, 9))
        ax_iso = fig.add_subplot(2, 2, (1, 3), projection="3d")
        ax_top = fig.add_subplot(2, 2, 2)
        ax_side = fig.add_subplot(2, 2, 4)
        render(deploy, ax_iso, ax_top, ax_side)
        title = "DEPLOYED" if deploy else "STOWED"
        fig.suptitle(
            f"MANTA — {title}: arm-braced wingsuit-extension wing\n"
            "spine yoke + along-arm LE spar + telescoping wrist+ankle tips + body-mounted TE spar",
            fontsize=13,
        )
        fig.tight_layout()
        fname = f"hero_{label}.png"
        fig.savefig(out_dir / fname, dpi=140, bbox_inches="tight")
        plt.close(fig)
        print(f"  wrote {out_dir / fname}")


if __name__ == "__main__":
    main()
