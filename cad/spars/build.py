"""
Parametric spar set CAD — front + rear telescoping CFRP spars, both sides.

Generates STEP and STL for:
    front spar (right side, 3 stages telescoped end-to-end)
    rear  spar (right side, 3 stages telescoped end-to-end)
    front spar (left side, mirrored)
    rear  spar (left side, mirrored)

Each stage is a hollow circular tube of length and OD set by the parametric
spar model. Joint overlap regions show the larger-OD stage extending over
the smaller; the actual joint hardware (locking pins, inner sleeve) is
not modeled here — placeholder only.

Two configurations are exported:
    spars_brief/   BRIEF dimensions (40/32/25 mm front; 30/24/18 mm rear)
    spars_sized/   bending-analysis sized front spar (73/51/25 mm)

Coordinate frame: same as analysis/aero (x aft, y starboard, z up).
The front spar's c/4-line position vs. the wing OML is taken from the
nominal 0.20·c front-spar location.
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

from analysis.struct.spar_model import (  # noqa: E402
    SparStage,
    TelescopingSpar,
    WingSparSet,
    default_rear_spar,
)

import cadquery as cq  # noqa: E402


X_FRONT_OVER_C = 0.20
X_REAR_OVER_C = 0.65


def sized_front_spar() -> TelescopingSpar:
    od_root = 0.073
    od_mid = od_root * 0.70
    od_tip = 0.025
    L = 3.7 / 3.0 + 2 * 0.025
    wall = 0.0025
    return TelescopingSpar(
        name="front_sized",
        stages=(
            SparStage("front_root", od_root, wall, L),
            SparStage("front_mid",  od_mid,  wall, L),
            SparStage("front_tip",  od_tip,  wall, L),
        ),
    )


def stage_solid(s: SparStage) -> cq.Workplane:
    """A hollow tube of the stage. Axis along +y; root face at y=0, tip at y=length.

    The XZ workplane's default extrude direction is −y, so we extrude with a
    negative length to get +y orientation.
    """
    outer = (
        cq.Workplane("XZ")
        .circle(s.outer_diameter_m / 2)
        .extrude(-s.length_m)
    )
    inner = (
        cq.Workplane("XZ")
        .circle(s.inner_diameter_m / 2)
        .extrude(-s.length_m)
    )
    return outer.cut(inner)


def build_telescoping_solid(spar: TelescopingSpar, x_chord: float) -> cq.Workplane:
    """Place 3 stages tip-to-tail along +y, with joint overlap.

    The overall spar runs from y = 0 (root) to y = total_length (tip).
    """
    parts = []
    y_cursor = 0.0
    for s in spar.stages:
        # Place the stage at its current y_cursor, accounting for joint overlap
        body = stage_solid(s).translate((x_chord, y_cursor, 0))
        parts.append(body)
        # Next stage starts inboard of this stage's outer end by joint_overlap
        y_cursor += s.length_m - spar.joint_overlap_m
    # Combine
    result = parts[0]
    for body in parts[1:]:
        result = result.union(body)
    return result


def build_full_set(set_: WingSparSet, p: Planform) -> cq.Workplane:
    # x-positions per BRIEF chordwise spar location, referenced to root chord
    x_front = X_FRONT_OVER_C * p.chord_root
    x_rear = X_REAR_OVER_C * p.chord_root

    front_right = build_telescoping_solid(set_.front, x_front)
    rear_right = build_telescoping_solid(set_.rear, x_rear)
    front_left = front_right.mirror("XZ")
    rear_left = rear_right.mirror("XZ")
    return front_right.union(rear_right).union(front_left).union(rear_left)


def export_set(set_: WingSparSet, p: Planform, label: str, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"  Building {label} spar set...")
    full = build_full_set(set_, p)
    step_path = out_dir / f"{label}.step"
    stl_path = out_dir / f"{label}.stl"
    print(f"  Exporting {step_path}")
    cq.exporters.export(full, str(step_path))
    print(f"  Exporting {stl_path}")
    cq.exporters.export(full, str(stl_path), tolerance=0.001, angularTolerance=0.5)
    bb = full.val().BoundingBox()
    print(f"  Bounding box: x={bb.xmin:+.3f}→{bb.xmax:+.3f}, "
          f"y={bb.ymin:+.3f}→{bb.ymax:+.3f}, z={bb.zmin:+.3f}→{bb.zmax:+.3f} (m)")


def main() -> None:
    p = Planform()
    out_dir = Path(__file__).parent / "out"

    # BRIEF dimensions
    brief_set = WingSparSet()
    export_set(brief_set, p, "spars_brief", out_dir)

    # Bending-analysis-sized
    sized_set = WingSparSet(front=sized_front_spar(), rear=default_rear_spar())
    export_set(sized_set, p, "spars_sized", out_dir)


if __name__ == "__main__":
    main()
