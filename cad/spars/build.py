"""
Parametric spar set CAD — front + rear telescoping CFRP spars, both sides.

The spars sweep with the wing along constant-x/c loci, NOT along world-frame
constant-x lines. The previous version had spars at constant x in world
frame, which exits the wing OML before reaching the tip on a swept wing —
a real geometric bug. This version places each stage along the swept x/c
locus and includes a numerical in-wing verification check.

Reference frame
---------------
Wing apex (root LE) at the origin. x aft, y starboard, z up.
Root chord runs from x=0 to x=c_root along the centerline.
Wing LE: x_LE(y) = |y|·tan(Λ_LE).
Wing TE: x_TE(y) = x_LE(y) + c(y) where c(y) = c_root·(1 − (1−λ)·|y|/half_b).
A "constant x/c = ξ" spar is at x_spar(y) = x_LE(y) + ξ·c(y).
"""

from __future__ import annotations

import math
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


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def spar_x_at_y(p: Planform, x_over_c: float, y: float) -> float:
    """Return the x-coordinate of the spar at spanwise station y for a
    spar that runs at fractional chord x/c = x_over_c.
    """
    return p.x_le_at(y) + x_over_c * p.chord_at(y)


def stage_endpoints(p: Planform, x_over_c: float, side: int,
                     y_in_y_axis: float, y_out_y_axis: float):
    """Return ((x_in, y_in, 0), (x_out, y_out, 0)) for a stage that spans
    [y_in_y_axis, y_out_y_axis] on the +y or -y wing (per `side`).

    y_in_y_axis < y_out_y_axis are *positive* y values in the right-wing
    convention; if side is -1, mirror them.
    """
    y_in = side * y_in_y_axis
    y_out = side * y_out_y_axis
    return (
        (spar_x_at_y(p, x_over_c, y_in), y_in, 0.0),
        (spar_x_at_y(p, x_over_c, y_out), y_out, 0.0),
    )


def _swept_tube(stage: SparStage, P_in, P_out) -> cq.Workplane:
    """Build a hollow tube of the given OD/ID with axis from P_in to P_out.

    P_in, P_out are 3-tuples of world coordinates (x, y, z).
    """
    dx = P_out[0] - P_in[0]
    dy = P_out[1] - P_in[1]
    dz = P_out[2] - P_in[2]
    L_axial = math.sqrt(dx * dx + dy * dy + dz * dz)

    # Axis direction in world frame
    axis_dir = (dx / L_axial, dy / L_axial, dz / L_axial)

    # Build a Plane whose normal is the axis direction, with xDir chosen
    # to be a perpendicular reference (the world +z, projected). For a
    # mostly-horizontal axis, +z is well-separated from the axis.
    z_world = (0.0, 0.0, 1.0)
    # x reference direction = z_world projected perpendicular to axis_dir
    # = z_world − (z_world·axis)·axis. For our case (axis in xy-plane),
    # z_world·axis = 0, so xDir = z_world. That works perfectly.
    if abs(dz) > 0.99 * L_axial:
        # Axis is essentially +z; pick +x as the perp reference
        x_ref = (1.0, 0.0, 0.0)
    else:
        x_ref = z_world

    plane = cq.Plane(
        origin=cq.Vector(*P_in),
        xDir=cq.Vector(*x_ref),
        normal=cq.Vector(*axis_dir),
    )
    outer = (
        cq.Workplane(plane)
        .circle(stage.outer_diameter_m / 2)
        .extrude(L_axial)
    )
    inner = (
        cq.Workplane(plane)
        .circle(stage.inner_diameter_m / 2)
        .extrude(L_axial)
    )
    return outer.cut(inner)


def build_swept_spar(spar: TelescopingSpar, x_over_c: float, p: Planform,
                      side: int = +1) -> cq.Workplane:
    """Build all 3 stages of `spar` swept along the constant-x/c locus."""
    # Map spar-axis-length stage_length back to a y-extent.
    # Stage axis length L_stage corresponds to y-span L_stage·cos(Λ_xc).
    # Use the actual sweep at this fractional chord.
    sweep_xc_deg = p.sweep_at_chord_fraction_deg(x_over_c)
    cos_sweep = math.cos(math.radians(sweep_xc_deg))

    parts = []
    y_cursor = 0.0
    for stage in spar.stages:
        y_extent = stage.length_m * cos_sweep   # span (y) extent of this stage
        y_in = y_cursor
        y_out = y_cursor + y_extent
        if y_out > p.half_span:
            y_out = p.half_span
        P_in, P_out = stage_endpoints(p, x_over_c, side, y_in, y_out)
        parts.append(_swept_tube(stage, P_in, P_out))
        # Next stage starts inboard of this stage's outboard end by joint_overlap
        y_cursor += y_extent - spar.joint_overlap_m * cos_sweep

    result = parts[0]
    for body in parts[1:]:
        result = result.union(body)
    return result


# ---------------------------------------------------------------------------
# In-wing verification
# ---------------------------------------------------------------------------

def verify_spar_in_wing(spar: TelescopingSpar, x_over_c: float, p: Planform,
                        n: int = 60) -> dict:
    """Sample y across the half-span; for each sample, verify that the
    spar's tube cross-section at the local OD lies within the wing's
    chord-direction extent (LE to TE). Returns a dict with min margins
    and a pass/fail flag.

    This catches the bug where a constant-x spar exits the wing on a
    swept planform: every sampled spar-x must satisfy
    LE(y) ≤ spar_x − r_outer AND spar_x + r_outer ≤ TE(y).
    """
    # Build a list of y, OD pairs by walking the stages
    sweep_xc_deg = p.sweep_at_chord_fraction_deg(x_over_c)
    cos_sweep = math.cos(math.radians(sweep_xc_deg))
    stage_segments = []
    y_cursor = 0.0
    for stage in spar.stages:
        y_extent = stage.length_m * cos_sweep
        y_in = y_cursor
        y_out = min(y_cursor + y_extent, p.half_span)
        stage_segments.append((y_in, y_out, stage.outer_diameter_m / 2))
        y_cursor += y_extent - spar.joint_overlap_m * cos_sweep

    min_le_margin = float("inf")
    min_te_margin = float("inf")
    worst_y_le = None
    worst_y_te = None

    for (y_in, y_out, r_outer) in stage_segments:
        for i in range(n):
            t = i / max(n - 1, 1)
            y = y_in * (1 - t) + y_out * t
            x_LE = p.x_le_at(y)
            x_TE = x_LE + p.chord_at(y)
            x_spar = spar_x_at_y(p, x_over_c, y)
            le_margin = (x_spar - r_outer) - x_LE
            te_margin = x_TE - (x_spar + r_outer)
            if le_margin < min_le_margin:
                min_le_margin = le_margin
                worst_y_le = y
            if te_margin < min_te_margin:
                min_te_margin = te_margin
                worst_y_te = y

    return {
        "min_le_margin_m": min_le_margin,
        "min_te_margin_m": min_te_margin,
        "worst_y_le": worst_y_le,
        "worst_y_te": worst_y_te,
        "in_wing": min_le_margin >= 0.0 and min_te_margin >= 0.0,
    }


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def build_full_set(set_: WingSparSet, p: Planform) -> cq.Workplane:
    front_right = build_swept_spar(set_.front, X_FRONT_OVER_C, p, side=+1)
    front_left = build_swept_spar(set_.front, X_FRONT_OVER_C, p, side=-1)
    rear_right = build_swept_spar(set_.rear, X_REAR_OVER_C, p, side=+1)
    rear_left = build_swept_spar(set_.rear, X_REAR_OVER_C, p, side=-1)
    return front_right.union(front_left).union(rear_right).union(rear_left)


def export_set(set_: WingSparSet, p: Planform, label: str, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"  Building {label} spar set...")
    full = build_full_set(set_, p)
    step_path = out_dir / f"{label}.step"
    stl_path = out_dir / f"{label}.stl"
    print(f"  Exporting {step_path}")
    cq.exporters.export(full, str(step_path))
    cq.exporters.export(full, str(stl_path), tolerance=0.001, angularTolerance=0.5)
    bb = full.val().BoundingBox()
    print(f"  Bounding box: x [{bb.xmin:+.3f},{bb.xmax:+.3f}]  "
          f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}]  "
          f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}] (m)")

    # In-wing verification
    print(f"  In-wing check (front, x/c={X_FRONT_OVER_C}):")
    vf = verify_spar_in_wing(set_.front, X_FRONT_OVER_C, p)
    print(f"    LE margin min: {vf['min_le_margin_m']*1000:+.1f} mm at y = {vf['worst_y_le']:.3f} m")
    print(f"    TE margin min: {vf['min_te_margin_m']*1000:+.1f} mm at y = {vf['worst_y_te']:.3f} m")
    print(f"    Status: {'IN WING ✓' if vf['in_wing'] else 'EXITS WING ✗'}")
    print(f"  In-wing check (rear, x/c={X_REAR_OVER_C}):")
    vr = verify_spar_in_wing(set_.rear, X_REAR_OVER_C, p)
    print(f"    LE margin min: {vr['min_le_margin_m']*1000:+.1f} mm at y = {vr['worst_y_le']:.3f} m")
    print(f"    TE margin min: {vr['min_te_margin_m']*1000:+.1f} mm at y = {vr['worst_y_te']:.3f} m")
    print(f"    Status: {'IN WING ✓' if vr['in_wing'] else 'EXITS WING ✗'}")


def main() -> None:
    p = Planform()
    out_dir = Path(__file__).parent / "out"

    # BRIEF dimensions
    brief_set = WingSparSet()
    export_set(brief_set, p, "spars_brief", out_dir)
    print()

    # Bending-analysis-sized
    sized_set = WingSparSet(front=sized_front_spar(), rear=default_rear_spar())
    export_set(sized_set, p, "spars_sized", out_dir)


if __name__ == "__main__":
    main()
