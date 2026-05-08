"""
Pyrotechnic spar-root cutter + root fitting assembly — placeholder CAD.

This is intentionally low-fidelity: dimensions are representative, not
production-ready. Detailed mechanical design comes after the vendor pick
and follows the safety analysis in `safety/failure-modes/cutter-no-fire.md`
and `safety/failure-modes/cutter-inadvertent-fire.md`.

Each of the 4 root fittings (front-left, front-right, rear-left, rear-right)
gets:
  - An aluminum cup that bonds to the spar root
  - A flange with bolt pattern that mounts to the harness sub-frame
  - A linear-shaped-charge (LSC) cutter housing wrapped around the spar
    root, with a redundant-initiator interface

The cutter severs the spar through a circumferential cut just outboard of
the bond region. After firing, the wing assembly separates cleanly from
the bonded fitting (which stays with the harness).

Output: cad/jettison/out/{root_fitting,cutter_assembly,full_set}.{step,stl}
"""

from __future__ import annotations

import sys
from pathlib import Path

import cadquery as cq

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402


# Sized-spar dimensions for the front spar; rear spar reuses BRIEF dims.
FRONT_OD_ROOT_M = 0.073    # see analysis/struct/spar_bending.py recommendation
REAR_OD_ROOT_M = 0.030
FITTING_WALL_M = 0.005
FITTING_LENGTH_M = 0.080   # bond region length
FLANGE_OD_M_FRONT = 0.110
FLANGE_OD_M_REAR  = 0.060
FLANGE_THICK_M = 0.008
BOLT_CIRCLE_DIA_FRONT = 0.085
BOLT_CIRCLE_DIA_REAR  = 0.045
BOLT_HOLE_DIA = 0.0065     # M6 clearance
N_BOLTS = 6

CUTTER_OUTER_M_FRONT = FRONT_OD_ROOT_M + 2 * 0.012  # ~12 mm thick housing
CUTTER_OUTER_M_REAR  = REAR_OD_ROOT_M + 2 * 0.012
CUTTER_LENGTH_M = 0.025
CUTTER_AXIAL_OFFSET_M = FITTING_LENGTH_M + 0.005    # placed just outboard of fitting


def _root_fitting(spar_od: float, flange_od: float, bolt_circle_dia: float) -> cq.Workplane:
    """Aluminum cup + flange that bonds the spar root to the harness mount."""
    # Cup body
    cup_outer_od = spar_od + 2 * FITTING_WALL_M
    cup = (
        cq.Workplane("XY")
        .circle(cup_outer_od / 2)
        .circle(spar_od / 2)
        .extrude(FITTING_LENGTH_M)
    )
    # Flange at the inboard end (z = 0)
    flange = (
        cq.Workplane("XY")
        .circle(flange_od / 2)
        .circle(spar_od / 2)
        .extrude(-FLANGE_THICK_M)
    )
    # Bolt holes through the flange (visualization only; not to spec)
    holes = (
        cq.Workplane("XY")
        .polarArray(radius=bolt_circle_dia / 2, startAngle=0, angle=360, count=N_BOLTS)
        .circle(BOLT_HOLE_DIA / 2)
        .extrude(-FLANGE_THICK_M - 0.001)
    )
    return cup.union(flange).cut(holes)


def _cutter_housing(spar_od: float, outer_od: float) -> cq.Workplane:
    """Annular housing wrapping the spar root, holds LSC and dual initiators."""
    body = (
        cq.Workplane("XY")
        .circle(outer_od / 2)
        .circle(spar_od / 2 + 0.0005)  # 0.5 mm clearance to spar
        .extrude(CUTTER_LENGTH_M)
        .translate((0, 0, CUTTER_AXIAL_OFFSET_M))
    )
    # Two initiator ports (raised cylindrical bosses, opposite sides)
    init_boss_dia = 0.012
    init_boss_h = 0.006
    init_y = outer_od / 2 + init_boss_h / 2
    init_z = CUTTER_AXIAL_OFFSET_M + CUTTER_LENGTH_M / 2
    boss1 = cq.Workplane("XZ").circle(init_boss_dia / 2).extrude(init_boss_h).translate((0, init_y, init_z))
    boss2 = cq.Workplane("XZ").circle(init_boss_dia / 2).extrude(init_boss_h).translate((0, -init_y, init_z))
    return body.union(boss1).union(boss2)


def root_fitting_with_cutter(spar_od: float, flange_od: float,
                              bolt_circle_dia: float) -> cq.Workplane:
    fit = _root_fitting(spar_od, flange_od, bolt_circle_dia)
    cutter = _cutter_housing(spar_od, spar_od + 0.024)
    return fit.union(cutter)


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    # Single fitting + cutter assemblies (front and rear)
    fit_front = root_fitting_with_cutter(FRONT_OD_ROOT_M, FLANGE_OD_M_FRONT, BOLT_CIRCLE_DIA_FRONT)
    fit_rear = root_fitting_with_cutter(REAR_OD_ROOT_M, FLANGE_OD_M_REAR, BOLT_CIRCLE_DIA_REAR)

    print("  Exporting single front fitting+cutter...")
    cq.exporters.export(fit_front, str(out_dir / "front_fitting_cutter.step"))
    cq.exporters.export(fit_front, str(out_dir / "front_fitting_cutter.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    print("  Exporting single rear fitting+cutter...")
    cq.exporters.export(fit_rear, str(out_dir / "rear_fitting_cutter.step"))
    cq.exporters.export(fit_rear, str(out_dir / "rear_fitting_cutter.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    # Full set: 4 fittings placed at each spar root, both sides.
    # Spar roots are at (x_spar, ±y_root, 0) where y_root = 0 (centerline) and
    # x_spar is the chordwise position of each spar in the harness mount.
    x_front = 0.20 * p.chord_root
    x_rear = 0.65 * p.chord_root
    # Spacing between front-left and front-right at the centerline harness:
    # they meet at the wing apex with a small structural cross-tie, but the
    # mounting plate is symmetric — place them ±0.030 m from centerline so
    # they don't overlap visually.
    y_off = 0.030

    # Front-left, front-right
    fl = fit_front.translate((x_front, +y_off, 0))
    fr = fit_front.translate((x_front, -y_off, 0))
    # Rear-left, rear-right
    rl = fit_rear.translate((x_rear, +y_off, 0))
    rr = fit_rear.translate((x_rear, -y_off, 0))
    full = fl.union(fr).union(rl).union(rr)

    print("  Exporting full set (4 fittings)...")
    cq.exporters.export(full, str(out_dir / "full_set.step"))
    cq.exporters.export(full, str(out_dir / "full_set.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    bb = full.val().BoundingBox()
    print(f"  Full set bbox (m): x [{bb.xmin:+.3f}, {bb.xmax:+.3f}], "
          f"y [{bb.ymin:+.3f}, {bb.ymax:+.3f}], z [{bb.zmin:+.3f}, {bb.zmax:+.3f}]")


if __name__ == "__main__":
    main()
