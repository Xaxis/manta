"""
Tape-spring rib CAD — bistable boom in both stable states.

Builds two configurations of a single rib:
    - DEPLOYED: open shell (curved arc cross-section) extruded along the
      rib chord. This is the rigid-flying configuration.
    - STOWED:   thin strip wrapped helically around the spar at a tight
      radius. This approximates the coiled configuration that gives the
      stowed-package thickness.

The bistable physics (snap-through energy and stability) is NOT modeled
here — that's deformation analysis that needs FEA. This is geometric
visualization for packaging and integration purposes.

Output:
    cad/ribs/out/rib_deployed.{step,stl}
    cad/ribs/out/rib_stowed.{step,stl}
    cad/ribs/out/rib_compare.{step,stl}    — both in one scene
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

import cadquery as cq  # noqa: E402


# Tape spring dimensions (from components.py)
TAPE_WIDTH = 0.040          # 40 mm
TAPE_THICKNESS = 0.0006     # 0.6 mm
ARC_HALF_ANGLE_DEG = 60.0   # subtended half-angle of the open-shell cross-section
ARC_RADIUS = 0.018          # radius of curvature of the open shell

# Stowed state — coil radius
COIL_INNER_RADIUS = 0.025   # 25 mm inner radius of the coil
COIL_TURNS_MAX = 12.0       # number of turns when fully stowed


def _deployed_section_polyline(n: int = 40) -> list[tuple[float, float]]:
    """Cross-section of the open-shell tape spring in deployed state.

    Returns a list of (s, t) coordinates in the chordwise (s) /
    thickness (t) plane forming a closed polygon: outer arc, inner arc,
    closed at the ends.
    """
    half_angle = math.radians(ARC_HALF_ANGLE_DEG)
    R_outer = ARC_RADIUS
    R_inner = ARC_RADIUS - TAPE_THICKNESS

    angles = [-half_angle + 2 * half_angle * i / (n - 1) for i in range(n)]

    outer = [(R_outer * math.sin(a), R_outer * (1 - math.cos(a))) for a in angles]
    inner = [(R_inner * math.sin(a), R_inner * (1 - math.cos(a))) for a in angles[::-1]]
    return outer + inner


def _build_deployed_rib(rib_length_m: float = 1.0) -> cq.Workplane:
    """Extrude the open-shell cross-section along the rib's length."""
    pts = _deployed_section_polyline()
    # Build wire in YZ plane (width-thickness), extrude along X (chord)
    w = cq.Workplane("YZ").polyline(pts).close()
    return w.extrude(rib_length_m)


def _build_stowed_rib(strip_length_m: float = 1.0,
                       width: float = TAPE_WIDTH,
                       thickness: float = TAPE_THICKNESS,
                       inner_R: float = COIL_INNER_RADIUS) -> cq.Workplane:
    """Helical-wrap representation of the coiled tape.

    Approximates the stowed coil as a thin annular spiral. The actual
    bistable shape has the strip curling about an axis perpendicular to
    its width, so the coil is "flat" — i.e. the coil axis is parallel
    to the tape width direction, and the spiral is in the chord-thickness
    plane.

    For visualization we approximate the coil as a thick washer (a tube
    with multiple internal turns). True bistable kinematics would
    require an actual spiral solid, but for packaging-thickness numbers
    a simple washer of the right OD is sufficient.
    """
    # Number of turns to fit the strip length at the given inner radius
    n_turns = strip_length_m / (2 * math.pi * inner_R)
    # As coil grows, OD increases — approximate as cumulative wrap
    # OD = inner_R + n_turns × thickness
    coil_thickness_radial = n_turns * thickness
    outer_R = inner_R + coil_thickness_radial

    # Solid washer
    washer = (
        cq.Workplane("XY")
        .circle(outer_R)
        .circle(inner_R)
        .extrude(width)
    )
    return washer


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    # Mid-span rib geometry for a representative example
    y_mid = p.half_span * 0.5
    chord_mid = p.chord_at(y_mid)
    rib_length = chord_mid * 0.95   # rib runs LE→TE, slightly inboard of full chord

    print("# Tape-spring rib geometry")
    print()
    print(f"  Rib station (y):      {y_mid:.3f} m")
    print(f"  Local chord:          {chord_mid:.3f} m")
    print(f"  Rib length (deployed):{rib_length:.3f} m")
    print(f"  Tape width:           {TAPE_WIDTH*1000:.0f} mm")
    print(f"  Tape thickness:       {TAPE_THICKNESS*1000:.1f} mm")
    print(f"  Open-shell radius:    {ARC_RADIUS*1000:.0f} mm, half-angle {ARC_HALF_ANGLE_DEG}°")
    print()

    deployed = _build_deployed_rib(rib_length)
    stowed = _build_stowed_rib(strip_length_m=rib_length)

    # Bounding-box dimensions
    bb_d = deployed.val().BoundingBox()
    bb_s = stowed.val().BoundingBox()
    print("Deployed bbox (m):  "
          f"chord {bb_d.xmax-bb_d.xmin:.3f}, "
          f"width {bb_d.ymax-bb_d.ymin:.3f}, "
          f"thickness {bb_d.zmax-bb_d.zmin:.3f}")
    print("Stowed bbox (m):    "
          f"x {bb_s.xmax-bb_s.xmin:.3f}, "
          f"y {bb_s.ymax-bb_s.ymin:.3f}, "
          f"z {bb_s.zmax-bb_s.zmin:.3f}  (this is the *coil OD* in the rib's wide direction)")
    coil_radial_thickness = (bb_s.xmax - bb_s.xmin) / 2 - COIL_INNER_RADIUS
    print(f"  → coil radial thickness (added to stowed package): {coil_radial_thickness*1000:.1f} mm")
    print(f"  → BRIEF stowed-package thickness budget: < 150 mm off body. "
          f"Margin to budget: {(0.150 - coil_radial_thickness*2)*1000:.1f} mm "
          "(per rib-axis dimension)")
    print()

    # Side-by-side comparison: stowed at x = 0, deployed offset by 0.4 m in x
    deployed_for_compare = deployed.translate((0.4, 0, 0))
    compare = stowed.union(deployed_for_compare)

    print("  Exporting rib_deployed...")
    cq.exporters.export(deployed, str(out_dir / "rib_deployed.step"))
    cq.exporters.export(deployed, str(out_dir / "rib_deployed.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    print("  Exporting rib_stowed...")
    cq.exporters.export(stowed, str(out_dir / "rib_stowed.step"))
    cq.exporters.export(stowed, str(out_dir / "rib_stowed.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    print("  Exporting rib_compare (side by side)...")
    cq.exporters.export(compare, str(out_dir / "rib_compare.step"))
    cq.exporters.export(compare, str(out_dir / "rib_compare.stl"),
                        tolerance=0.0005, angularTolerance=0.5)


if __name__ == "__main__":
    main()
