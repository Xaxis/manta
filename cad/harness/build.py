"""
Harness + integration CAD — placeholder geometry that captures the
volumetric envelope of the deployed-and-stowed pilot+rig+wing-harness
stack, plus an explicit reserve-canopy deployment-cone clearance check.

This is the closure of the BRIEF reserve-compatibility hard constraint.
Detailed harness mechanical design comes later; what this builds is:

    - pilot torso (representative volume, 1.70 × 0.45 × 0.30 m prone)
    - skydiving rig (main + reserve container stack on the back)
    - wing-mount sub-frame on top of the rig (the bridge that the
      MANTA harness bolts to)
    - 4 spar-root fitting stubs (post-jettison configuration — stubs
      are what's left after the wing departs)
    - reserve canopy deployment cone, projected from the reserve PC
      launch point

Outputs:
    cad/harness/out/integration.{step,stl}    — assembly
    cad/harness/out/reserve_cone.{step,stl}   — cone alone
    cad/harness/out/clearance_report.md       — stub-vs-cone numbers

Reserve cone geometry (skydiving industry consensus, conservative):
    apex at PC launch point (top of reserve container)
    opening upward (+z)
    half-angle: 30° (industry-common)
    height: 4 m above launch (reaches line-stretch)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

import cadquery as cq  # noqa: E402


# Frame: pilot lies prone, head at +x (forward direction), back at +z.
# Wing apex is at (1.0, 0, 0.30) m (forward of pilot CG by ~0.35 m,
# mounted on top of the back rig by 0.30 m).
PILOT_LENGTH = 1.70
PILOT_WIDTH = 0.45
PILOT_THICKNESS = 0.30  # belly-to-back

RIG_LENGTH = 0.55       # along pilot back (head-to-foot)
RIG_WIDTH = 0.40        # cross-back
RIG_THICKNESS = 0.18    # protrusion above the back

SUBFRAME_THICKNESS = 0.012  # 12 mm aluminum / CFRP bridge plate
SUBFRAME_LENGTH = 0.65
SUBFRAME_WIDTH = 0.30

# Cone parameters
CONE_HALF_ANGLE_DEG = 30.0
CONE_HEIGHT = 4.0


def _pilot_torso() -> cq.Workplane:
    """Box approximation of the prone pilot. Origin at pilot CG."""
    return (
        cq.Workplane("XY")
        .box(PILOT_LENGTH, PILOT_WIDTH, PILOT_THICKNESS)
    )


def _skydiving_rig() -> cq.Workplane:
    """The main + reserve container stack on the pilot's back."""
    return (
        cq.Workplane("XY")
        .box(RIG_LENGTH, RIG_WIDTH, RIG_THICKNESS)
        .translate((0.0, 0.0, PILOT_THICKNESS / 2 + RIG_THICKNESS / 2))
    )


def _wing_mount_subframe(spar_x_positions: list[float],
                         cutter_y_offset: float = 0.030) -> tuple[cq.Workplane, list[tuple[float, float, float]]]:
    """The sub-frame plate that the MANTA harness bolts to. Returns the
    plate plus a list of (x, y, z) post-jettison stub centerpoints."""
    plate = (
        cq.Workplane("XY")
        .box(SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS)
        .translate((0.10, 0.0,
                    PILOT_THICKNESS / 2 + RIG_THICKNESS + SUBFRAME_THICKNESS / 2))
    )

    # Post-jettison stub geometry: short cylinders (60 mm tall, sized OD)
    stubs = []
    stub_centers = []
    stub_z = PILOT_THICKNESS / 2 + RIG_THICKNESS + SUBFRAME_THICKNESS

    for x in spar_x_positions:
        # Front spar OD = 73 mm sized; rear = 30 mm
        od = 0.073 if x == spar_x_positions[0] else 0.030
        for y_sign in (+1, -1):
            y = y_sign * cutter_y_offset
            stub = (
                cq.Workplane("XY")
                .circle(od / 2)
                .extrude(0.060)   # 60 mm tall stub
                .translate((x, y, stub_z))
            )
            stubs.append(stub)
            stub_centers.append((x, y, stub_z + 0.030))  # mid-height of stub

    full_plate = plate
    for s in stubs:
        full_plate = full_plate.union(s)
    return full_plate, stub_centers


def _reserve_cone() -> cq.Workplane:
    """Reserve canopy deployment cone, apex at PC launch point above the
    reserve container, opening upward."""
    apex_x = -0.10  # slightly aft of pilot CG (where reserve container sits)
    apex_z = PILOT_THICKNESS / 2 + RIG_THICKNESS + 0.03   # PC just above container

    # Cone: bottom radius small, top radius = h·tan(half_angle)
    top_radius = CONE_HEIGHT * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
    bottom_radius = 0.05    # 50 mm at the apex (PC pack)

    cone = (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=CONE_HEIGHT)
        .circle(top_radius)
        .loft(combine=True)
        .translate((apex_x, 0.0, apex_z))
    )
    return cone


def stub_inside_cone(stub_center: tuple[float, float, float],
                       cone_apex: tuple[float, float, float]) -> tuple[bool, float, float]:
    """Return (is_inside, perp_distance_to_cone_axis, half_angle_at_z).

    The cone has apex at cone_apex, axis along +z.  A point is inside if
    its lateral distance from the cone axis (measured at the same z) is
    less than the cone's radius at that z.
    """
    sx, sy, sz = stub_center
    ax, ay, az = cone_apex
    if sz < az:
        return False, math.hypot(sx - ax, sy - ay), 0.0
    h = sz - az
    cone_radius_at_z = h * math.tan(math.radians(CONE_HALF_ANGLE_DEG))
    lateral = math.hypot(sx - ax, sy - ay)
    return lateral < cone_radius_at_z, lateral, cone_radius_at_z


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    pilot = _pilot_torso()
    rig = _skydiving_rig()

    spar_x = [0.20 * p.chord_root, 0.65 * p.chord_root]
    subframe_with_stubs, stub_centers = _wing_mount_subframe(spar_x)
    cone = _reserve_cone()

    # Combined STEP for inspection
    assembly = pilot.union(rig).union(subframe_with_stubs)

    print("  Exporting integration assembly (without cone)...")
    cq.exporters.export(assembly, str(out_dir / "integration.step"))
    cq.exporters.export(assembly, str(out_dir / "integration.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    print("  Exporting reserve cone alone...")
    cq.exporters.export(cone, str(out_dir / "reserve_cone.step"))
    cq.exporters.export(cone, str(out_dir / "reserve_cone.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    # Combined assembly with cone for visualization
    full = assembly.union(cone)
    cq.exporters.export(full, str(out_dir / "integration_with_cone.step"))
    cq.exporters.export(full, str(out_dir / "integration_with_cone.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    # Clearance report
    apex_x = -0.10
    apex_z = PILOT_THICKNESS / 2 + RIG_THICKNESS + 0.03
    cone_apex = (apex_x, 0.0, apex_z)

    print()
    print("## Clearance check — post-jettison stubs vs reserve canopy cone")
    print()
    print(f"Cone apex: ({apex_x:+.3f}, 0.000, {apex_z:+.3f}) m")
    print(f"Cone half-angle: {CONE_HALF_ANGLE_DEG}°  (industry conservative)")
    print()
    print("| Stub | x (m) | y (m) | z (m) | lateral (m) | cone-radius @ z (m) | Status |")
    print("|---|---|---|---|---|---|---|")
    rows = []
    for i, c in enumerate(stub_centers):
        inside, lateral, cone_r = stub_inside_cone(c, cone_apex)
        # Margin reads positive when the stub is OUTSIDE the cone (good).
        margin = lateral - cone_r
        status = "INSIDE CONE — fail" if inside else f"clear (margin {margin:+.3f} m outboard)"
        spar_label = "front" if i < 2 else "rear"
        side = "+y" if c[1] > 0 else "-y"
        print(f"| {spar_label} {side} | {c[0]:+.3f} | {c[1]:+.3f} | {c[2]:+.3f} | "
              f"{lateral:.3f} | {cone_r:.3f} | {status} |")
        rows.append((spar_label, side, c[0], c[1], c[2], lateral, cone_r, inside))

    md_lines = [
        "# Reserve canopy clearance — placeholder geometry",
        "",
        f"Cone apex: ({apex_x:+.3f}, 0.000, {apex_z:+.3f}) m",
        f"Cone half-angle: {CONE_HALF_ANGLE_DEG}°",
        "",
        "| Stub | x (m) | y (m) | z (m) | lateral (m) | cone-r @ z (m) | inside cone? |",
        "|---|---|---|---|---|---|---|",
    ]
    for sl, side, x, y, z, lat, cr, ins in rows:
        md_lines.append(f"| {sl} {side} | {x:+.3f} | {y:+.3f} | {z:+.3f} | "
                          f"{lat:.3f} | {cr:.3f} | {'YES (FAIL)' if ins else 'no'} |")
    md_lines.append("")
    md_lines.append("**This is placeholder geometry.** Real harness mounting positions")
    md_lines.append("will move the spar roots; this analysis demonstrates the *check*,")
    md_lines.append("not the final verdict. The verdict is when the actual harness CAD")
    md_lines.append("is finalized AND a physical mock-up confirms.")
    (out_dir / "clearance_report.md").write_text("\n".join(md_lines) + "\n")

    bb = full.val().BoundingBox()
    print()
    print(f"Full bounding box (m): "
          f"x [{bb.xmin:+.2f}, {bb.xmax:+.2f}], "
          f"y [{bb.ymin:+.2f}, {bb.ymax:+.2f}], "
          f"z [{bb.zmin:+.2f}, {bb.zmax:+.2f}]")


if __name__ == "__main__":
    main()
