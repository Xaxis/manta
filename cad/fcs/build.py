"""
FCS bay CAD — placeholder geometry for the flight-control-system
hardware: FCS-A + FCS-B + aux IMU in spatially-separated bays,
batteries, sensors (pitot boom, AoA vane, magnetometer), wiring
routing.

Spatial separation is the safety architecture per
[`docs/04-fcs-architecture.md`](../../docs/04-fcs-architecture.md):
FCS-A in the forward bay, FCS-B in the aft bay, aux IMU on FCS-B side
but its own thermal envelope — independent thermal management is one
of the listed correlated-fault mitigations.

Output:
    cad/fcs/out/fcs_bay.{step,stl}
    cad/fcs/out/fcs_with_sensors.{step,stl}
"""

from __future__ import annotations

import sys
from pathlib import Path

import cadquery as cq

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))


# Component dimensions (representative — Pixhawk 6X form factor + batteries)
PIXHAWK = (0.085, 0.045, 0.020)        # 85 × 45 × 20 mm
AUX_IMU = (0.040, 0.030, 0.012)        # standalone IMU board
BATTERY = (0.090, 0.045, 0.025)        # 4S2P 18650 pack, ~600 g

# Sensor protrusions
PITOT_BOOM_LEN = 0.220                  # 220 mm forward of LE
PITOT_BOOM_OD = 0.012
AOA_VANE_LEN = 0.080
AOA_VANE_BLADE = (0.060, 0.002, 0.020)  # blade L × t × span

# Bay geometry
BAY_LEN = 0.200
BAY_WIDTH = 0.150
BAY_DEPTH = 0.040

# Layout in vehicle frame (x aft, y span, z up)
# Pilot CG at origin; bays are forward of CG on the harness shell
BAY_A_POS = (-0.40, 0.0, 0.20)         # FCS-A: forward, centerline
BAY_B_POS = (-0.10, 0.0, 0.20)         # FCS-B: aft of FCS-A
PITOT_POS = (-0.55, +0.30, 0.18)       # right wingtip-ish boom (placeholder)


def _box(dims: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(dims[0], dims[1], dims[2])


def _bay_assembly(pos: tuple[float, float, float], label: str,
                  battery: bool = True, aux_imu: bool = False) -> cq.Workplane:
    """A bay shell + Pixhawk + (optional) battery + (optional) aux IMU."""
    shell = _box((BAY_LEN, BAY_WIDTH, BAY_DEPTH)).translate(pos)
    pixhawk = (
        _box(PIXHAWK)
        .translate((pos[0] - 0.04, pos[1], pos[2] + (BAY_DEPTH - PIXHAWK[2]) / 2 - 0.005))
    )
    parts = [shell, pixhawk]
    if battery:
        bat = _box(BATTERY).translate(
            (pos[0] + 0.04, pos[1], pos[2] - (BAY_DEPTH - BATTERY[2]) / 2 + 0.005)
        )
        parts.append(bat)
    if aux_imu:
        imu = _box(AUX_IMU).translate(
            (pos[0] - 0.07, pos[1] + 0.04, pos[2] + (BAY_DEPTH - AUX_IMU[2]) / 2 - 0.005)
        )
        parts.append(imu)
    out = parts[0]
    for p in parts[1:]:
        out = out.union(p)
    return out


def _pitot_boom() -> cq.Workplane:
    """Pitot tube on a boom protruding forward (in -x direction)."""
    boom = (
        cq.Workplane("YZ")
        .circle(PITOT_BOOM_OD / 2)
        .extrude(PITOT_BOOM_LEN)
        .translate(PITOT_POS)
    )
    # Pitot probe end
    probe = (
        cq.Workplane("YZ")
        .circle(PITOT_BOOM_OD / 2 * 1.4)
        .extrude(0.020)
        .translate((PITOT_POS[0] + PITOT_BOOM_LEN, PITOT_POS[1], PITOT_POS[2]))
    )
    return boom.union(probe)


def _aoa_vane() -> cq.Workplane:
    """AoA vane mounted on a small post off the harness shell."""
    post = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(AOA_VANE_LEN)
        .translate((-0.50, -0.20, 0.20))
    )
    blade = (
        _box(AOA_VANE_BLADE)
        .translate((-0.50, -0.20, 0.20 + AOA_VANE_LEN + AOA_VANE_BLADE[2] / 2))
    )
    return post.union(blade)


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    print("# FCS bay layout")
    print()
    print(f"  Bay A (FCS-A + battery):       {BAY_A_POS}")
    print(f"  Bay B (FCS-B + battery + aux): {BAY_B_POS}")
    print(f"  Pitot boom forward at:         {PITOT_POS}, length {PITOT_BOOM_LEN*1000:.0f} mm")
    print(f"  AoA vane:                       on bottom of harness")
    print()

    bay_a = _bay_assembly(BAY_A_POS, "FCS-A", battery=True, aux_imu=False)
    bay_b = _bay_assembly(BAY_B_POS, "FCS-B", battery=True, aux_imu=True)
    bays = bay_a.union(bay_b)

    print("  Exporting fcs_bay (bays only)...")
    cq.exporters.export(bays, str(out_dir / "fcs_bay.step"))
    cq.exporters.export(bays, str(out_dir / "fcs_bay.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    pitot = _pitot_boom()
    aoa = _aoa_vane()
    full = bays.union(pitot).union(aoa)

    print("  Exporting fcs_with_sensors...")
    cq.exporters.export(full, str(out_dir / "fcs_with_sensors.step"))
    cq.exporters.export(full, str(out_dir / "fcs_with_sensors.stl"),
                        tolerance=0.0005, angularTolerance=0.5)

    bb = full.val().BoundingBox()
    print()
    print(f"  Full FCS+sensors bbox (m): "
          f"x [{bb.xmin:+.3f},{bb.xmax:+.3f}], "
          f"y [{bb.ymin:+.3f},{bb.ymax:+.3f}], "
          f"z [{bb.zmin:+.3f},{bb.zmax:+.3f}]")
    print()
    print("Notes:")
    print("  - FCS-A and FCS-B are 0.30 m apart (longitudinal) so a single")
    print("    impact / thermal event can't take both.")
    print("  - Each bay has its own battery; rails are routed on opposite")
    print("    sides of the harness shell.")
    print("  - Aux IMU is in bay B but uses a separate connector and bus,")
    print("    so a bay-B board fault that takes FCS-B doesn't necessarily")
    print("    take the aux.")
    print("  - Pitot boom: protrudes forward, well clear of the wing wake")
    print("    during deploy. Mounting boom needs to fold/protect during stow.")
    print("  - AoA vane: on the bottom of the harness shell, in clean flow")
    print("    when prone.")


if __name__ == "__main__":
    main()
