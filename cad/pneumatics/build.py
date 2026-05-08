"""
Pneumatics CAD — placeholder geometry for the deployment plumbing.

Two configurations:

    BRIEF / option-A:   shared CO2 reservoir + central valve + matched-
                        impedance manifold + lines to per-side actuators.
                        (Closes much of the common-mode CO2 variance per
                        the symmetry-budget recommendation.)

    Option-B (recommended): same shared reservoir + per-side ACTIVE valves
                        instead of a matched-impedance manifold. Adds an
                        extra component but lets the FCS modulate per-side
                        flow based on stage-lock progress.

Outputs:
    cad/pneumatics/out/option_a.{step,stl}
    cad/pneumatics/out/option_b.{step,stl}

Detailed mechanical design (orifice sizing, regulator selection, fitting
spec) comes after the symmetry budget tightens with bench data.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cadquery as cq

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))


# ----------------------------------------------------------------------
# Component dimensions (representative)
# ----------------------------------------------------------------------

# 88 g CO2 cartridge (standard sporting goods)
CART_OD = 0.019         # 19 mm
CART_LEN = 0.090        # 90 mm
CART_NECK_OD = 0.011
CART_NECK_LEN = 0.012

# Regulator block (CO2 → 30 bar working pressure)
REG_BLOCK = (0.060, 0.040, 0.040)   # X, Y, Z dimensions

# Central solenoid valve (option-A) — single 3-way valve
VALVE_BLOCK = (0.050, 0.050, 0.060)

# Manifold (matched-impedance T-piece, option-A)
MANI_LEN = 0.140        # along Y, span direction
MANI_OD = 0.020

# Per-side proportional valve (option-B)
PER_SIDE_VALVE = (0.040, 0.030, 0.045)

# Tubing
TUBE_OD = 0.006         # 6 mm OD
TUBE_LENGTH = 0.300     # to per-side actuator


def _co2_cartridge() -> cq.Workplane:
    """88 g CO2 cartridge with a small neck, axis along +y."""
    body = (
        cq.Workplane("XZ")
        .circle(CART_OD / 2)
        .extrude(-CART_LEN)
    )
    neck = (
        cq.Workplane("XZ")
        .circle(CART_NECK_OD / 2)
        .extrude(CART_NECK_LEN)
    )
    return body.union(neck)


def _box(dims: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(dims[0], dims[1], dims[2])


def _tube(length: float, od: float = TUBE_OD) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(od / 2)
        .extrude(-length)
    )


def build_option_a() -> cq.Workplane:
    """Shared reservoir, central valve, matched manifold, lines."""
    cart = _co2_cartridge().translate((0, 0, 0))   # cartridge points +y
    reg = _box(REG_BLOCK).translate((0, 0.005 + REG_BLOCK[1] / 2, 0))
    valve = _box(VALVE_BLOCK).translate((0, 0.005 + REG_BLOCK[1] + VALVE_BLOCK[1] / 2 + 0.005, 0))

    # Manifold is a long horizontal tube (along y) downstream of valve
    manifold_y = 0.005 + REG_BLOCK[1] + VALVE_BLOCK[1] + 0.020
    manifold = (
        cq.Workplane("XZ")
        .circle(MANI_OD / 2)
        .extrude(MANI_LEN)
        .translate((0, manifold_y, 0))
    )
    # Per-side outlet tubes from manifold tips, going outboard in y
    out_left = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(TUBE_LENGTH)
        .translate((0, manifold_y + MANI_LEN, 0))
    )
    out_right = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(-TUBE_LENGTH)
        .translate((0, manifold_y, 0))
    )
    return cart.union(reg).union(valve).union(manifold).union(out_left).union(out_right)


def build_option_b() -> cq.Workplane:
    """Shared reservoir + central regulator, then per-side active valves."""
    cart = _co2_cartridge().translate((0, 0, 0))
    reg = _box(REG_BLOCK).translate((0, 0.005 + REG_BLOCK[1] / 2, 0))

    # Common feed line out of regulator splits at a small T into two per-side
    # active proportional valves placed slightly outboard.
    feed_y = 0.005 + REG_BLOCK[1] + 0.010
    feed_tube = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(0.080)
        .translate((0, feed_y, 0))
    )
    # Per-side valves at ±0.080 in y, both at the same x
    valve_left = _box(PER_SIDE_VALVE).translate((0, feed_y + 0.110, 0))
    # Two-port T at end of the feed → two short lines to each valve
    feed_T_left = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(0.030)
        .translate((PER_SIDE_VALVE[0] / 2, feed_y + 0.080, 0))
    )
    valve_right = _box(PER_SIDE_VALVE).translate((0, feed_y + 0.110, 0))
    # Place the two valves apart laterally
    valve_left = valve_left.translate((+(PER_SIDE_VALVE[0] / 2 + 0.030), 0, 0))
    valve_right = valve_right.translate((-(PER_SIDE_VALVE[0] / 2 + 0.030), 0, 0))
    feed_T_left = feed_T_left.translate((0, 0, 0))
    # Outlet tubes from each valve outboard
    out_left = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(TUBE_LENGTH)
        .translate((+(PER_SIDE_VALVE[0] / 2 + 0.030), feed_y + 0.110 + PER_SIDE_VALVE[1] / 2 + 0.005, 0))
    )
    out_right = (
        cq.Workplane("XZ")
        .circle(TUBE_OD / 2)
        .extrude(-TUBE_LENGTH)
        .translate((-(PER_SIDE_VALVE[0] / 2 + 0.030), feed_y + 0.110 + PER_SIDE_VALVE[1] / 2 + 0.005, 0))
    )

    return (cart.union(reg).union(feed_tube)
            .union(valve_left).union(valve_right)
            .union(feed_T_left)
            .union(out_left).union(out_right))


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    print("# MANTA pneumatics CAD")
    print()
    print(f"  CO2 cartridge:  {CART_OD*1000:.0f} mm OD × {CART_LEN*1000:.0f} mm (88 g standard)")
    print(f"  Regulator:      {[d*1000 for d in REG_BLOCK]} mm")
    print(f"  Central valve:  {[d*1000 for d in VALVE_BLOCK]} mm  (option A)")
    print(f"  Manifold:       {MANI_OD*1000:.0f} mm OD × {MANI_LEN*1000:.0f} mm  (option A)")
    print(f"  Per-side valve: {[d*1000 for d in PER_SIDE_VALVE]} mm  (option B)")
    print()

    print("  Building option_a (shared reservoir + central valve + matched manifold)...")
    a = build_option_a()
    cq.exporters.export(a, str(out_dir / "option_a.step"))
    cq.exporters.export(a, str(out_dir / "option_a.stl"),
                        tolerance=0.0005, angularTolerance=0.5)
    bb = a.val().BoundingBox()
    print(f"    bbox: x [{bb.xmin*1000:+.0f},{bb.xmax*1000:+.0f}] mm, "
          f"y [{bb.ymin*1000:+.0f},{bb.ymax*1000:+.0f}] mm, "
          f"z [{bb.zmin*1000:+.0f},{bb.zmax*1000:+.0f}] mm")

    print("  Building option_b (shared reservoir + per-side active valves)...")
    b = build_option_b()
    cq.exporters.export(b, str(out_dir / "option_b.step"))
    cq.exporters.export(b, str(out_dir / "option_b.stl"),
                        tolerance=0.0005, angularTolerance=0.5)
    bb = b.val().BoundingBox()
    print(f"    bbox: x [{bb.xmin*1000:+.0f},{bb.xmax*1000:+.0f}] mm, "
          f"y [{bb.ymin*1000:+.0f},{bb.ymax*1000:+.0f}] mm, "
          f"z [{bb.zmin*1000:+.0f},{bb.zmax*1000:+.0f}] mm")


if __name__ == "__main__":
    main()
