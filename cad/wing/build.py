"""
Parametric MANTA wing CAD — generates STEP + STL from the locked planform.

Sources its geometry from analysis/aero/planform/geometry.py so the model
tracks any planform parameter changes downstream of analysis updates.

Section profile is a parametric reflexed airfoil (~10 % t/c with mild
S-camber) standing in for MH 78 until the real .dat file is dropped at
cad/wing/airfoils/MH78.dat. The script auto-detects that file and uses it
when present.

Run:
    PYTHONPATH=. .venv/bin/python cad/wing/build.py

Output:
    cad/wing/out/wing.step
    cad/wing/out/wing.stl

References:
    NACA 4-digit thickness equation: NACA TR 460 (Jacobs, Ward, Pinkerton, 1933).
    Loft and Boolean operations: CadQuery 2.7 user guide.
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


# -----------------------------------------------------------------------
# Airfoil section
# -----------------------------------------------------------------------

def parametric_reflexed_airfoil(n: int = 60, t_c: float = 0.10, camber: float = 0.012):
    """A parametric MH-78-class reflexed airfoil.

    Uses NACA 4-digit thickness distribution for the symmetric thickness
    component and a cubic S-camber line: y_c = 4·camber·x·(1−x)·(1−2x),
    which produces a small forward camber bump and a reflexed (negative)
    aft section, the qualitative shape needed for tailless flying-wing
    sections. Not a substitute for actual MH 78 coordinates but close
    enough for first-cut CAD and visualization.

    Returns (xs, ys) numpy arrays describing a closed contour traversing
    the upper surface from TE to LE then the lower surface from LE to TE.
    """
    # Cosine-spaced x for tighter LE resolution
    beta = np.linspace(0.0, np.pi, n)
    x = (1.0 - np.cos(beta)) * 0.5

    # Symmetric thickness (NACA 4-digit)
    yt = 5.0 * t_c * (
        0.2969 * np.sqrt(x)
        - 0.1260 * x
        - 0.3516 * x ** 2
        + 0.2843 * x ** 3
        - 0.1036 * x ** 4
    )

    # S-camber line
    yc = camber * 4.0 * x * (1.0 - x) * (1.0 - 2.0 * x)

    upper_x = x
    upper_y = yc + yt
    lower_x = x[::-1]
    lower_y = (yc - yt)[::-1]

    # Closed contour: upper TE→LE then lower LE→TE
    xs = np.concatenate([upper_x[::-1], lower_x[1:]])
    ys = np.concatenate([upper_y[::-1], lower_y[1:]])
    return xs, ys


def load_dat_airfoil(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load a Selig-format .dat airfoil (one (x, y) pair per line).

    Skips a single header line (the airfoil name).
    """
    pts = []
    with path.open() as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 2:
                continue
            try:
                pts.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    arr = np.array(pts)
    return arr[:, 0], arr[:, 1]


# -----------------------------------------------------------------------
# 3D section construction
# -----------------------------------------------------------------------

def section_points_3d(
    xs2d: np.ndarray,
    ys2d: np.ndarray,
    y_station: float,
    chord: float,
    x_le: float,
    twist_deg: float,
):
    """Return a list of (x, y, z) tuples describing the airfoil contour at
    spanwise station y_station, scaled by `chord`, with leading edge at
    `x_le`, rotated by `twist_deg` about the quarter-chord (y axis through
    the c/4 point).

    Coordinate convention (matches the rest of the project):
        x: aft (chord direction)
        y: span (right wing positive)
        z: up (thickness direction)
    """
    twist = math.radians(twist_deg)
    cos_t, sin_t = math.cos(twist), math.sin(twist)

    pts3d = []
    for xi, ti in zip(xs2d, ys2d):
        # Translate so c/4 is at origin, rotate, translate back
        xr = xi - 0.25
        x_rot = cos_t * xr - sin_t * ti
        z_rot = sin_t * xr + cos_t * ti
        x_chord = (x_rot + 0.25) * chord
        x_world = x_chord + x_le
        y_world = y_station
        z_world = z_rot * chord
        pts3d.append((x_world, y_world, z_world))
    return pts3d


def build_half_wing(p: Planform, xs2d: np.ndarray, ys2d: np.ndarray, n_stations: int = 14):
    """Build the right half-wing as a CadQuery solid via loft."""
    wires = []
    for i in range(n_stations):
        eta = i / (n_stations - 1)
        y_station = eta * p.half_span
        chord = p.chord_at(y_station)
        x_le = p.x_le_at(y_station)
        twist = p.twist_at(y_station)
        pts3d = section_points_3d(xs2d, ys2d, y_station, chord, x_le, twist)
        # CadQuery wire from polyline (closed)
        vecs = [cq.Vector(*pt) for pt in pts3d]
        # Ensure the contour closes by adding the first point at the end
        if vecs[0] != vecs[-1]:
            vecs.append(vecs[0])
        wire = cq.Wire.makePolygon(vecs, forConstruction=False, close=False)
        wires.append(wire)

    # Loft through wires
    solid = cq.Solid.makeLoft(wires, ruled=False)
    return cq.Workplane(obj=solid)


def main() -> None:
    out_dir = Path(__file__).parent / "out"
    out_dir.mkdir(exist_ok=True)

    p = Planform()

    # Pick airfoil source: MH78.dat if present, otherwise parametric
    af_path = Path(__file__).parent / "airfoils" / "MH78.dat"
    if af_path.exists():
        xs2d, ys2d = load_dat_airfoil(af_path)
        af_label = "MH78.dat"
    else:
        xs2d, ys2d = parametric_reflexed_airfoil(n=60, t_c=p.section_t_c, camber=0.012)
        af_label = "parametric reflexed (placeholder for MH78)"

    print(f"  Airfoil source : {af_label}")
    print(f"  Wing area      : {p.S} m²")
    print(f"  Span           : {p.b} m   (half-span {p.half_span:.3f} m)")
    print(f"  Chord root/tip : {p.chord_root:.4f} / {p.chord_tip:.4f} m")
    print(f"  Sweep LE       : {p.sweep_le_deg:.2f}°  (c/4 {p.sweep_c4_deg:.2f}°)")
    print(f"  Washout        : {p.washout_deg:.2f}°")

    print("  Building right half...")
    right = build_half_wing(p, xs2d, ys2d, n_stations=16)

    print("  Mirroring to left half + union...")
    left = right.mirror("XZ")
    full = right.union(left)

    step_out = out_dir / "wing.step"
    stl_out = out_dir / "wing.stl"

    print(f"  Exporting {step_out}")
    cq.exporters.export(full, str(step_out))
    print(f"  Exporting {stl_out}")
    cq.exporters.export(full, str(stl_out), tolerance=0.001, angularTolerance=0.5)

    # Stats
    bb = full.val().BoundingBox()
    print()
    print(f"  Bounding box (m):")
    print(f"    x:  {bb.xmin:+.4f}  →  {bb.xmax:+.4f}   (span of x-extent {bb.xmax - bb.xmin:.4f} m)")
    print(f"    y:  {bb.ymin:+.4f}  →  {bb.ymax:+.4f}   (full span         {bb.ymax - bb.ymin:.4f} m)")
    print(f"    z:  {bb.zmin:+.4f}  →  {bb.zmax:+.4f}   (max thickness     {bb.zmax - bb.zmin:.4f} m)")


if __name__ == "__main__":
    main()
