"""
Integrated assembly — pilot + skydiving rig + wing-mount sub-frame +
swept spars + root fittings + reserve canopy deployment cone.

This is the model that explains how the wing actually affixes to the
human body. Earlier subsystem CAD models (cad/wing, cad/spars,
cad/harness, cad/jettison) each lived in their own coordinate
frames and were never combined; that left the obvious geometric
question — "where exactly does this thing sit on the pilot?" —
unanswered. This build answers it.

Frame
-----
World origin at the wing apex (root LE), x aft, y starboard, z up.

Pilot lies prone with body axis along +x. The pilot's CG is positioned
to coincide with the design CG (x_CG ≈ 1.05 m aft of root LE), so:

    Pilot head     at x ≈ +0.22 m   (slightly aft of wing apex)
    Pilot CG       at x ≈ +1.07 m
    Pilot feet     at x ≈ +1.92 m   (aft of wing root TE at x = 1.62 m)

The skydiving rig sits on the pilot's back; the wing-mount sub-frame
sits on top of the rig; the wing root sits on top of the sub-frame.
The reserve container is at the upper back of the rig (forward, near
shoulder blades) so the reserve PC launches upward through the gap
where the wing used to be after jettison.

Outputs
-------
    cad/integration/out/full_assembly.{step,stl}        complete scene
    cad/integration/out/full_assembly_no_skin.{step,stl}  skin hidden so internal structure is visible
    cad/integration/out/iso_render.png                  matplotlib iso render
    cad/integration/out/clearance_report.md             stub-vs-cone numbers

Verification gates (all must pass):
    - All four post-jettison stubs are outside the reserve canopy
      deployment cone (30° half-angle from PC apex).
    - Spar tubes lie inside the wing OML at every span station
      (already verified upstream in cad/spars/build.py).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

import cadquery as cq  # noqa: E402


# ---------------------------------------------------------------------------
# Anchoring decisions
# ---------------------------------------------------------------------------

PILOT_LENGTH = 1.70
PILOT_WIDTH = 0.45
PILOT_THICKNESS = 0.30
PILOT_HEAD_X = 0.22                 # head at slightly aft of wing apex
PILOT_BACK_Z = -0.05                # back surface; pilot belly at PILOT_BACK_Z - PILOT_THICKNESS

RIG_LENGTH = 0.60                   # along pilot back
RIG_WIDTH = 0.40
RIG_THICKNESS = 0.18
RIG_X_CENTER = 0.85                 # rig centered roughly mid-torso (over upper back -> mid)

# Reserve container occupies the FORWARD half of the rig (over upper back / shoulder blades)
RES_LENGTH = 0.28
RES_WIDTH = 0.36
RES_THICKNESS = 0.16
RES_X_CENTER = 0.55                 # reserve over upper back / shoulders

# Main container occupies the AFT half of the rig
MAIN_LENGTH = 0.30
MAIN_WIDTH = 0.36
MAIN_THICKNESS = 0.16
MAIN_X_CENTER = 1.05                # main over mid/lower torso

SUBFRAME_LENGTH = 1.10              # extends from over reserve to over front of main
SUBFRAME_WIDTH = 0.32
SUBFRAME_THICKNESS = 0.012
SUBFRAME_X_CENTER = 0.70            # spans from x ≈ 0.15 to 1.25

# Stub geometry (post-jettison)
STUB_HEIGHT = 0.060
FRONT_SPAR_X_OVER_C = 0.20
REAR_SPAR_X_OVER_C = 0.65
STUB_Y_OFFSET = 0.030               # ±30 mm from centerline (left/right)

# Reserve cone
CONE_HALF_ANGLE_DEG = 30.0
CONE_HEIGHT = 4.0


# ---------------------------------------------------------------------------
# Wing OML (parametric airfoil lofted through spanwise stations)
# ---------------------------------------------------------------------------

def _airfoil_xy(n: int = 50, t_c: float = 0.10, camber: float = 0.012):
    beta = np.linspace(0.0, np.pi, n)
    x = (1.0 - np.cos(beta)) * 0.5
    yt = 5.0 * t_c * (
        0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4
    )
    yc = camber * 4.0 * x * (1.0 - x) * (1.0 - 2.0 * x)
    upper_x = x
    upper_y = yc + yt
    lower_x = x[::-1]
    lower_y = (yc - yt)[::-1]
    xs = np.concatenate([upper_x[::-1], lower_x[1:]])
    ys = np.concatenate([upper_y[::-1], lower_y[1:]])
    return xs, ys


def _section_pts3d(xs2d, ys2d, y_station, chord, x_le, twist_deg):
    twist = math.radians(twist_deg)
    cos_t, sin_t = math.cos(twist), math.sin(twist)
    pts = []
    for xi, ti in zip(xs2d, ys2d):
        xr = xi - 0.25
        x_rot = cos_t * xr - sin_t * ti
        z_rot = sin_t * xr + cos_t * ti
        x_world = (x_rot + 0.25) * chord + x_le
        z_world = z_rot * chord
        pts.append((x_world, y_station, z_world))
    return pts


def _build_wing_OML(p: Planform, n_stations: int = 14) -> cq.Workplane:
    xs2d, ys2d = _airfoil_xy()
    wires = []
    for i in range(n_stations):
        eta = i / (n_stations - 1)
        y = eta * p.half_span
        chord = p.chord_at(y)
        x_le = p.x_le_at(y)
        twist = p.twist_at(y)
        pts3d = _section_pts3d(xs2d, ys2d, y, chord, x_le, twist)
        vecs = [cq.Vector(*pt) for pt in pts3d]
        wires.append(cq.Wire.makePolygon(vecs, forConstruction=False, close=False))
    right_solid = cq.Solid.makeLoft(wires, ruled=False)
    right = cq.Workplane(obj=right_solid)
    left = right.mirror("XZ")
    return right.union(left)


# ---------------------------------------------------------------------------
# Swept spars — copied from cad/spars/build.py
# ---------------------------------------------------------------------------

def _spar_x_at_y(p: Planform, x_over_c: float, y: float) -> float:
    return p.x_le_at(y) + x_over_c * p.chord_at(y)


def _swept_tube(od: float, id_: float, P_in, P_out) -> cq.Workplane:
    dx = P_out[0] - P_in[0]
    dy = P_out[1] - P_in[1]
    dz = P_out[2] - P_in[2]
    L_axial = math.sqrt(dx * dx + dy * dy + dz * dz)
    axis_dir = (dx / L_axial, dy / L_axial, dz / L_axial)
    plane = cq.Plane(origin=cq.Vector(*P_in), xDir=cq.Vector(0, 0, 1),
                     normal=cq.Vector(*axis_dir))
    outer = cq.Workplane(plane).circle(od / 2).extrude(L_axial)
    inner = cq.Workplane(plane).circle(id_ / 2).extrude(L_axial)
    return outer.cut(inner)


def _build_swept_spar(p: Planform, x_over_c: float,
                       od_root: float, od_mid: float, od_tip: float,
                       wall: float, side: int = +1) -> cq.Workplane:
    """Three-stage telescoping spar swept along the constant-x/c locus."""
    sweep_deg = p.sweep_at_chord_fraction_deg(x_over_c)
    cos_sweep = math.cos(math.radians(sweep_deg))
    half_b = p.half_span

    # Match the stage-length convention from cad/spars/build.py
    L_stage_axial = half_b / 3.0 / cos_sweep + 2 * 0.025
    joint_overlap_axial = 0.05

    stages = [
        (od_root, od_root - 2 * wall),
        (od_mid, od_mid - 2 * wall),
        (od_tip, od_tip - 2 * wall),
    ]

    # Anchor spar at z = sub-frame top (so spar is flush with the wing root underside)
    z_spar = SUBFRAME_THICKNESS / 2 + 0.005   # tiny gap

    parts = []
    y_cursor = 0.0
    for od_outer, od_inner in stages:
        y_extent = L_stage_axial * cos_sweep
        y_in = y_cursor
        y_out = min(y_cursor + y_extent, half_b)
        P_in = (_spar_x_at_y(p, x_over_c, side * y_in), side * y_in, z_spar)
        P_out = (_spar_x_at_y(p, x_over_c, side * y_out), side * y_out, z_spar)
        parts.append(_swept_tube(od_outer, od_inner, P_in, P_out))
        y_cursor += y_extent - joint_overlap_axial * cos_sweep

    result = parts[0]
    for body in parts[1:]:
        result = result.union(body)
    return result


# ---------------------------------------------------------------------------
# Pilot + rig + sub-frame + stubs + cone
# ---------------------------------------------------------------------------

def _pilot_box() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PILOT_LENGTH, PILOT_WIDTH, PILOT_THICKNESS)
        .translate((PILOT_HEAD_X + PILOT_LENGTH / 2, 0, PILOT_BACK_Z - PILOT_THICKNESS / 2))
    )


def _rig_main() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(MAIN_LENGTH, MAIN_WIDTH, MAIN_THICKNESS)
        .translate((MAIN_X_CENTER, 0, PILOT_BACK_Z + MAIN_THICKNESS / 2))
    )


def _rig_reserve() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(RES_LENGTH, RES_WIDTH, RES_THICKNESS)
        .translate((RES_X_CENTER, 0, PILOT_BACK_Z + RES_THICKNESS / 2))
    )


def _subframe() -> cq.Workplane:
    z_top = max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z
    return (
        cq.Workplane("XY")
        .box(SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS)
        .translate((SUBFRAME_X_CENTER, 0, z_top + SUBFRAME_THICKNESS / 2))
    )


def _post_jettison_stubs(p: Planform) -> tuple[cq.Workplane, list[tuple[float, float, float]]]:
    """4 stubs (front-left, front-right, rear-left, rear-right) + their centers.

    Each stub is a short cylinder rising above the sub-frame at the
    spar-root x position (x = ξ·c_root) at small ±y offsets.
    """
    z_subframe_top = (
        max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS
    )
    stubs = []
    centers = []
    for x_over_c, od in [(FRONT_SPAR_X_OVER_C, 0.073),
                          (REAR_SPAR_X_OVER_C, 0.030)]:
        x_spar = x_over_c * p.chord_root
        for y_sign in (+1, -1):
            y = y_sign * STUB_Y_OFFSET
            stub = (
                cq.Workplane("XY")
                .circle(od / 2)
                .extrude(STUB_HEIGHT)
                .translate((x_spar, y, z_subframe_top))
            )
            stubs.append(stub)
            centers.append((x_spar, y, z_subframe_top + STUB_HEIGHT / 2))

    body = stubs[0]
    for s in stubs[1:]:
        body = body.union(s)
    return body, centers


def _reserve_cone() -> tuple[cq.Workplane, tuple[float, float, float]]:
    """Reserve canopy deployment cone — apex at the reserve PC launch
    point above the reserve container, opening upward."""
    apex_x = RES_X_CENTER
    apex_z = PILOT_BACK_Z + RES_THICKNESS + 0.02   # PC just above reserve
    apex = (apex_x, 0.0, apex_z)
    bottom_r = 0.05
    top_r = CONE_HEIGHT * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
    cone = (
        cq.Workplane("XY")
        .circle(bottom_r)
        .workplane(offset=CONE_HEIGHT)
        .circle(top_r)
        .loft(combine=True)
        .translate(apex)
    )
    return cone, apex


def _stub_inside_cone(stub_center, cone_apex):
    sx, sy, sz = stub_center
    ax, ay, az = cone_apex
    if sz < az:
        return False, math.hypot(sx - ax, sy - ay), 0.0
    h = sz - az
    cone_r = h * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
    lateral = math.hypot(sx - ax, sy - ay)
    return lateral < cone_r, lateral, cone_r


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    print("# MANTA integrated assembly")
    print()

    print("  Building pilot, rig, sub-frame...")
    pilot = _pilot_box()
    main_can = _rig_main()
    reserve_can = _rig_reserve()
    subframe = _subframe()

    print("  Building wing OML...")
    wing = _build_wing_OML(p)

    print("  Building swept spars (sized config: 73/51/25 mm front, BRIEF rear)...")
    front_r = _build_swept_spar(p, FRONT_SPAR_X_OVER_C,
                                 od_root=0.073, od_mid=0.051, od_tip=0.025,
                                 wall=0.0025, side=+1)
    front_l = _build_swept_spar(p, FRONT_SPAR_X_OVER_C,
                                 od_root=0.073, od_mid=0.051, od_tip=0.025,
                                 wall=0.0025, side=-1)
    rear_r = _build_swept_spar(p, REAR_SPAR_X_OVER_C,
                                od_root=0.030, od_mid=0.024, od_tip=0.018,
                                wall=0.002, side=+1)
    rear_l = _build_swept_spar(p, REAR_SPAR_X_OVER_C,
                                od_root=0.030, od_mid=0.024, od_tip=0.018,
                                wall=0.002, side=-1)

    print("  Building post-jettison stubs + reserve cone (verification only)...")
    stubs, stub_centers = _post_jettison_stubs(p)
    cone, cone_apex = _reserve_cone()

    print()
    print("Body / rig / harness layout (m):")
    print(f"  Wing apex        x = +0.000  z = +0.000")
    print(f"  Pilot head       x = {PILOT_HEAD_X:+.3f}  body extends to x = {PILOT_HEAD_X + PILOT_LENGTH:+.3f}")
    print(f"  Pilot back top   z = {PILOT_BACK_Z:+.3f}  (belly at z = {PILOT_BACK_Z - PILOT_THICKNESS:+.3f})")
    print(f"  Reserve container x = {RES_X_CENTER:+.3f},  z range [{PILOT_BACK_Z:+.3f}, {PILOT_BACK_Z + RES_THICKNESS:+.3f}]")
    print(f"  Main container    x = {MAIN_X_CENTER:+.3f},  z range [{PILOT_BACK_Z:+.3f}, {PILOT_BACK_Z + MAIN_THICKNESS:+.3f}]")
    print(f"  Sub-frame top    z = {PILOT_BACK_Z + max(MAIN_THICKNESS, RES_THICKNESS) + SUBFRAME_THICKNESS:+.3f}")
    print(f"  Wing root LE     x = +0.000, root TE x = {p.chord_root:+.3f}")
    print(f"  Wing tip LE      x = {p.half_span * math.tan(math.radians(p.sweep_le_deg)):+.3f}")
    print()

    print("Reserve cone clearance vs post-jettison stubs:")
    print()
    print(f"  Cone apex: ({cone_apex[0]:+.3f}, 0.000, {cone_apex[2]:+.3f}) m,  "
          f"half-angle {CONE_HALF_ANGLE_DEG}°")
    print()
    print("| Stub | x (m) | y (m) | z (m) | lateral (m) | cone-r @ z (m) | margin | status |")
    print("|---|---|---|---|---|---|---|---|")
    rows = []
    labels = ["front +y", "front -y", "rear +y", "rear -y"]
    all_clear = True
    for label, c in zip(labels, stub_centers):
        inside, lateral, cr = _stub_inside_cone(c, cone_apex)
        margin = lateral - cr
        if inside:
            all_clear = False
        status = "INSIDE — fail" if inside else "clear"
        print(f"| {label:8s} | {c[0]:+.3f} | {c[1]:+.3f} | {c[2]:+.3f} | "
              f"{lateral:.3f} | {cr:.3f} | {margin:+.3f} m | {status} |")
        rows.append((label, c[0], c[1], c[2], lateral, cr, margin, inside))

    print()
    print(f"Overall reserve clearance: {'PASS' if all_clear else 'FAIL'}")
    print()

    # Combined export — use cq.Compound rather than chained Workplane.union,
    # which fails when any intermediate Workplane has an empty stack.
    components = [
        ("pilot", pilot),
        ("main", main_can),
        ("reserve", reserve_can),
        ("subframe", subframe),
        ("wing", wing),
        ("front_r", front_r),
        ("front_l", front_l),
        ("rear_r", rear_r),
        ("rear_l", rear_l),
        ("stubs", stubs),
    ]
    component_solids = []
    for name, wp in components:
        try:
            for s in wp.solids().vals():
                component_solids.append(s)
        except Exception as exc:
            print(f"    skip {name}: {exc}")

    full_no_cone = cq.Compound.makeCompound(component_solids)
    cone_solid_list = list(cone.solids().vals())
    full_with_cone = cq.Compound.makeCompound(component_solids + cone_solid_list)

    print("  Exporting full_assembly (no cone) ...")
    cq.exporters.export(full_no_cone, str(out_dir / "full_assembly.step"))
    cq.exporters.export(full_no_cone, str(out_dir / "full_assembly.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    print("  Exporting full_assembly_with_cone ...")
    cq.exporters.export(full_with_cone, str(out_dir / "full_assembly_with_cone.step"))
    cq.exporters.export(full_with_cone, str(out_dir / "full_assembly_with_cone.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    bb = full_no_cone.BoundingBox()
    print()
    print(f"Full assembly bbox (m): "
          f"x [{bb.xmin:+.3f},{bb.xmax:+.3f}], "
          f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}], "
          f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}]")

    # Markdown clearance report
    md = ["# Integrated assembly — clearance report\n"]
    md.append(f"Cone apex: ({cone_apex[0]:+.3f}, 0.000, {cone_apex[2]:+.3f}) m,  half-angle {CONE_HALF_ANGLE_DEG}°\n")
    md.append("| Stub | x (m) | y (m) | z (m) | lateral (m) | cone-r @ z (m) | margin | status |")
    md.append("|---|---|---|---|---|---|---|---|")
    for r in rows:
        md.append(f"| {r[0]:8s} | {r[1]:+.3f} | {r[2]:+.3f} | {r[3]:+.3f} | "
                   f"{r[4]:.3f} | {r[5]:.3f} | {r[6]:+.3f} | {'INSIDE — fail' if r[7] else 'clear'} |")
    md.append(f"\n**Overall:** {'PASS' if all_clear else 'FAIL'}\n")
    (out_dir / "clearance_report.md").write_text("\n".join(md))


if __name__ == "__main__":
    main()
