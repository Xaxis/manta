"""
Stowed-configuration CAD — wing collapsed against the pilot's body.

Architecture decisions (first cut, propose for BRIEF amendment):

1. **Spar pivot at the root** — each telescoping spar pivots ~68° about a
   z-axis hinge at the cutter location, from the deployed swept direction
   (≈22° aft of +y) to a stowed orientation along +x (pilot body axis).
   Stowed position lies above the rig along the pilot's spine, extending
   AFT from the cutter location.

2. **Spar telescoping retracted** — each 3-stage spar collapses to its
   root-stage length (~1.27 m). Outer stage (root) stays put; mid + tip
   stages telescope inside it.

3. **Ribs coiled** — each bistable tape-spring rib is in its coiled
   stable state, stacked along the stowed spar at uniform intervals.
   Coil OD per rib ≈ 58 mm (washer of inner R 25 mm + ~4 mm of strip
   thickness wrap).

4. **Skin gathered** — DCF skin folds accordion-style between consecutive
   rib coils. Not modeled geometrically here (skin volume is a small
   fraction of the package); accounted as ~5 mm radial allowance on the
   rib coil OD.

Package thickness budget (BRIEF: stowed package thickness < 150 mm off
body profile). Verified numerically below.

Output:
    cad/stowed/out/stowed_assembly.{step,stl}
    cad/stowed/out/thickness_report.md
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import cadquery as cq

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from analysis.aero.planform.geometry import Planform  # noqa: E402

# Shared geometry constants from the integration build
from cad.integration.build import (  # noqa: E402
    PILOT_BACK_Z, PILOT_HEAD_X, PILOT_LENGTH, PILOT_THICKNESS, PILOT_WIDTH,
    MAIN_LENGTH, MAIN_WIDTH, MAIN_THICKNESS, MAIN_X_CENTER,
    RES_LENGTH, RES_WIDTH, RES_THICKNESS, RES_X_CENTER,
    SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS, SUBFRAME_X_CENTER,
    FRONT_SPAR_X_OVER_C, REAR_SPAR_X_OVER_C, STUB_Y_OFFSET,
)


# ---------------------------------------------------------------------------
# Stowed-configuration parameters
# ---------------------------------------------------------------------------

# Retracted spar length: 3-stage telescoping collapses to root-stage length
RETRACTED_SPAR_LEN = 1.27

# Per-side stowed offset: right wing's spars stow at +y_offset, left at −y_offset
SPAR_STOWED_Y_OFFSET = 0.060

# Rib geometry (coiled state)
RIB_COIL_OD = 0.058               # 58 mm — per cad/ribs/build.py
RIB_COIL_LENGTH = 0.040           # rib WIDTH, 40 mm
N_RIBS_PER_SIDE = 9               # BRIEF
RIB_GAP = 0.005                   # 5 mm spacing between rib coils

# Skin allowance
SKIN_RADIAL_ALLOWANCE = 0.005     # 5 mm

# Package geometry — what we'll measure
PACKAGE_TARGET_OFF_BODY_M = 0.150  # BRIEF


# ---------------------------------------------------------------------------
# Builders
# ---------------------------------------------------------------------------

def _box(dims, pos):
    return cq.Workplane("XY").box(*dims).translate(pos)


def _stowed_spar(x_root_world: float, side: int,
                  od_root: float, od_mid: float, od_tip: float,
                  wall: float) -> cq.Workplane:
    """A retracted-and-rotated spar.

    Stowed orientation: spar axis along +x, starting at the cutter location
    on the sub-frame and extending aft. Stages are concentric (telescoped):
    outer stage is the root OD, full length; the mid and tip stages are
    inside it (not visible from outside, so we just model the root).

    Position: at (x_root_world, side·SPAR_STOWED_Y_OFFSET, sub-frame top + r_outer)
    so the spar lies on TOP of the sub-frame.
    """
    z_subframe_top = (
        max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS
    )
    z_spar_axis = z_subframe_top + od_root / 2 + 0.005   # 5 mm gap

    # Outer tube (visible). Inner stages are concentric inside — we don't
    # model them since they're hidden.
    outer = (
        cq.Workplane("YZ")
        .circle(od_root / 2)
        .circle(od_root / 2 - wall)
        .extrude(RETRACTED_SPAR_LEN)
        .translate((x_root_world, side * SPAR_STOWED_Y_OFFSET, z_spar_axis))
    )
    return outer


def _stowed_rib_coils(x_root_world: float, side: int,
                       od_root_spar: float) -> tuple[cq.Workplane, float]:
    """Stack rib coils along the stowed spar.

    Each coil is a thin washer wrapped around the spar (inner R = spar OD/2 +
    small clearance). Stacked along x with RIB_GAP between coils.
    Returns (workplane, max_radial_extent_above_subframe).
    """
    z_subframe_top = (
        max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS
    )
    z_spar_axis = z_subframe_top + od_root_spar / 2 + 0.005

    coil_inner_r = od_root_spar / 2 + 0.002   # 2 mm clearance to spar
    coil_outer_r = coil_inner_r + (RIB_COIL_OD - od_root_spar) / 2
    if coil_outer_r < coil_inner_r + 0.010:
        coil_outer_r = coil_inner_r + 0.010   # min 10 mm radial thickness

    coils = []
    # Stack along x starting at x_root_world + 0.05 and ending at
    # x_root_world + RETRACTED_SPAR_LEN - 0.05
    x0 = x_root_world + 0.05
    x_max = x_root_world + RETRACTED_SPAR_LEN - 0.05
    spacing = (x_max - x0) / max(N_RIBS_PER_SIDE - 1, 1)

    for i in range(N_RIBS_PER_SIDE):
        x_center = x0 + i * spacing
        coil = (
            cq.Workplane("YZ")
            .circle(coil_outer_r)
            .circle(coil_inner_r)
            .extrude(RIB_COIL_LENGTH)
            .translate((x_center - RIB_COIL_LENGTH / 2,
                        side * SPAR_STOWED_Y_OFFSET,
                        z_spar_axis))
        )
        coils.append(coil)

    body = coils[0]
    for c in coils[1:]:
        body = body.union(c)

    # Max radial extent above sub-frame top
    max_z_top = z_spar_axis + coil_outer_r
    extent_above_subframe = max_z_top - z_subframe_top
    return body, extent_above_subframe


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)
    p = Planform()

    print("# MANTA — STOWED configuration")
    print()

    # Pilot + rig + sub-frame (same as integrated build)
    pilot = _box((PILOT_LENGTH, PILOT_WIDTH, PILOT_THICKNESS),
                  (PILOT_HEAD_X + PILOT_LENGTH / 2, 0, PILOT_BACK_Z - PILOT_THICKNESS / 2))
    main_can = _box((MAIN_LENGTH, MAIN_WIDTH, MAIN_THICKNESS),
                     (MAIN_X_CENTER, 0, PILOT_BACK_Z + MAIN_THICKNESS / 2))
    res_can = _box((RES_LENGTH, RES_WIDTH, RES_THICKNESS),
                    (RES_X_CENTER, 0, PILOT_BACK_Z + RES_THICKNESS / 2))
    z_sub_center = max(MAIN_THICKNESS, RES_THICKNESS) + PILOT_BACK_Z + SUBFRAME_THICKNESS / 2
    subframe = _box((SUBFRAME_LENGTH, SUBFRAME_WIDTH, SUBFRAME_THICKNESS),
                     (SUBFRAME_X_CENTER, 0, z_sub_center))

    # Stowed spars + rib coils (front + rear, both sides)
    stowed_parts = []
    coil_parts = []
    max_extents = []
    for x_over_c, od_root, od_mid, od_tip, wall, label in [
        (FRONT_SPAR_X_OVER_C, 0.073, 0.051, 0.025, 0.0025, "front"),
        (REAR_SPAR_X_OVER_C, 0.030, 0.024, 0.018, 0.002, "rear"),
    ]:
        x_root_world = x_over_c * p.chord_root
        for side in (+1, -1):
            spar = _stowed_spar(x_root_world, side, od_root, od_mid, od_tip, wall)
            coils, extent = _stowed_rib_coils(x_root_world, side, od_root)
            stowed_parts.append(spar)
            coil_parts.append(coils)
            max_extents.append(extent)
        print(f"  {label} spar: stowed extent above sub-frame "
              f"= {max_extents[-1] * 1000:.1f} mm")

    package_thickness_off_back = (
        # rig + subframe + stowed extent above sub-frame
        max(MAIN_THICKNESS, RES_THICKNESS) + SUBFRAME_THICKNESS + max(max_extents)
    ) + SKIN_RADIAL_ALLOWANCE

    print()
    print(f"  Package thickness off body (z above pilot back):")
    print(f"    rig stack       = {max(MAIN_THICKNESS, RES_THICKNESS) * 1000:.1f} mm")
    print(f"    sub-frame       = {SUBFRAME_THICKNESS * 1000:.1f} mm")
    print(f"    largest stowed bundle (spar + ribs) = {max(max_extents) * 1000:.1f} mm")
    print(f"    skin allowance  = {SKIN_RADIAL_ALLOWANCE * 1000:.1f} mm")
    print(f"    TOTAL off back  = {package_thickness_off_back * 1000:.1f} mm")
    print(f"    BRIEF target    < {PACKAGE_TARGET_OFF_BODY_M * 1000:.0f} mm")
    pass_fail = "PASS" if package_thickness_off_back <= PACKAGE_TARGET_OFF_BODY_M else "FAIL"
    print(f"    Status: {pass_fail}")
    print()

    # Compound export
    components = [pilot, main_can, res_can, subframe] + stowed_parts + coil_parts
    solids = []
    for c in components:
        for s in c.solids().vals():
            solids.append(s)
    full = cq.Compound.makeCompound(solids)

    print("  Exporting stowed_assembly.step / .stl ...")
    cq.exporters.export(full, str(out_dir / "stowed_assembly.step"))
    cq.exporters.export(full, str(out_dir / "stowed_assembly.stl"),
                        tolerance=0.001, angularTolerance=0.5)

    # Per-subsystem (for the viewer)
    parts_dir = out_dir / "parts"
    parts_dir.mkdir(exist_ok=True)
    subsystems = {
        "pilot": [pilot],
        "rig_main": [main_can],
        "rig_reserve": [res_can],
        "subframe": [subframe],
        "stowed_spars": stowed_parts,
        "stowed_ribs": coil_parts,
    }
    for name, wps in subsystems.items():
        ss_solids = []
        for wp in wps:
            for s in wp.solids().vals():
                ss_solids.append(s)
        if not ss_solids:
            continue
        cq.exporters.export(cq.Compound.makeCompound(ss_solids),
                            str(parts_dir / f"{name}.stl"),
                            tolerance=0.001, angularTolerance=0.5)

    bb = full.BoundingBox()
    print()
    print(f"Stowed bbox (m): "
          f"x [{bb.xmin:+.3f},{bb.xmax:+.3f}], "
          f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}], "
          f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}]")

    md = ["# Stowed package thickness — verification\n",
          f"Off-body z (above pilot back): **{package_thickness_off_back * 1000:.1f} mm**\n",
          f"BRIEF target: **< {PACKAGE_TARGET_OFF_BODY_M * 1000:.0f} mm**\n",
          f"Status: **{pass_fail}**\n",
          "",
          "| Component | Thickness (mm) |",
          "|---|---|",
          f"| Rig stack (main / reserve) | {max(MAIN_THICKNESS, RES_THICKNESS) * 1000:.1f} |",
          f"| Sub-frame plate | {SUBFRAME_THICKNESS * 1000:.1f} |",
          f"| Largest stowed bundle (spar + rib coils) | {max(max_extents) * 1000:.1f} |",
          f"| Skin gathered allowance | {SKIN_RADIAL_ALLOWANCE * 1000:.1f} |",
          f"| **Total** | **{package_thickness_off_back * 1000:.1f}** |\n"]
    (out_dir / "thickness_report.md").write_text("\n".join(md))


if __name__ == "__main__":
    main()
