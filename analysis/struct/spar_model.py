"""
Parametric telescoping CFRP spar model.

Single source of truth for spar geometry, mass, and section properties (A, I)
as a function of spanwise station. Used by:
  - spar_bending.py  (stress / safety-factor check)
  - mass_budget.py   (component mass roll-up)
  - cad/spars/build.py  (3D model)

Architecture (locked from BRIEF, parameters here can be revised):
  - 2 spars per side (front + rear), 4 spars total in the deployed wing.
  - Each spar is 3-stage telescoping: stage 1 = root, stage 3 = tip.
  - Each stage is a constant-OD CFRP tube; OD steps down at each stage.
  - Stages overlap at the joints by ~1 chord-MAC of the local tube to
    carry shear; that overlap region adds mass via inner sleeve and
    locking pin hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from analysis.struct.materials import CFRPUDTube


@dataclass(frozen=True)
class SparStage:
    """One telescoping stage: a constant-OD hollow circular CFRP tube."""

    name: str
    outer_diameter_m: float
    wall_thickness_m: float
    length_m: float

    @property
    def inner_diameter_m(self) -> float:
        return self.outer_diameter_m - 2.0 * self.wall_thickness_m

    @property
    def area_m2(self) -> float:
        ro = self.outer_diameter_m / 2
        ri = self.inner_diameter_m / 2
        return math.pi * (ro * ro - ri * ri)

    @property
    def I_m4(self) -> float:
        """Second moment of area for hollow circular cross-section."""
        ro = self.outer_diameter_m / 2
        ri = self.inner_diameter_m / 2
        return (math.pi / 4.0) * (ro ** 4 - ri ** 4)

    @property
    def J_m4(self) -> float:
        """Polar second moment (torsion)."""
        return 2.0 * self.I_m4

    def mass_kg(self, rho: float) -> float:
        return rho * self.area_m2 * self.length_m


@dataclass(frozen=True)
class TelescopingSpar:
    """Three-stage telescoping spar from root (stage 1) to tip (stage 3).

    The stages telescope INSIDE one another in the stowed configuration —
    the smallest (tip) stage retracts inside the next, etc. The deployed
    geometry has them extended end-to-end with an overlap region at each
    joint where the smaller tube fits inside the larger one.
    """

    name: str
    stages: tuple[SparStage, SparStage, SparStage]
    joint_overlap_m: float = 0.05            # 50 mm overlap at each of 2 joints
    joint_hardware_kg_per_joint: float = 0.04  # locking pin + inner sleeve + glue

    @property
    def total_length_m(self) -> float:
        # Deployed end-to-end length. Subtract one joint_overlap per joint
        # so that the effective deployed half-span is the sum of stages
        # minus 2 × joint_overlap.
        return sum(s.length_m for s in self.stages) - 2 * self.joint_overlap_m

    def mass_kg(self, material: CFRPUDTube) -> float:
        m_tubes = sum(s.mass_kg(material.rho) for s in self.stages)
        m_joints = 2 * self.joint_hardware_kg_per_joint
        return m_tubes + m_joints

    def section_at(self, y_along: float) -> SparStage:
        """Return the stage active at axial coordinate `y_along` ∈ [0, total_length].

        Coordinates measured outboard from the root end of stage 1.
        """
        if y_along <= self.stages[0].length_m - self.joint_overlap_m:
            return self.stages[0]
        if y_along <= (self.stages[0].length_m + self.stages[1].length_m
                       - 2 * self.joint_overlap_m):
            return self.stages[1]
        return self.stages[2]


# ------------------------------------------------------------------------
# BRIEF-locked default spar specifications
# ------------------------------------------------------------------------

def default_front_spar(half_span: float = 3.7, wall: float = 0.002) -> TelescopingSpar:
    """Front spar per BRIEF: 40 mm OD root, 25 mm OD tip, 2 mm wall, 3 stages.

    Stage OD progression: 40 → 32 → 25 mm (three steps with smooth taper).
    Each stage is half_span / 3 long plus the joint overlap on each end.
    """
    L = half_span / 3.0 + 2 * 0.025  # add a small allowance for joint overlap
    return TelescopingSpar(
        name="front",
        stages=(
            SparStage("front_root", 0.040, wall, L),
            SparStage("front_mid",  0.032, wall, L),
            SparStage("front_tip",  0.025, wall, L),
        ),
    )


def default_rear_spar(half_span: float = 3.7, wall: float = 0.002) -> TelescopingSpar:
    """Rear spar per BRIEF: 30 mm OD root, 18 mm OD tip, 2 mm wall, 3 stages.

    Stage OD progression: 30 → 24 → 18 mm.
    """
    L = half_span / 3.0 + 2 * 0.025
    return TelescopingSpar(
        name="rear",
        stages=(
            SparStage("rear_root", 0.030, wall, L),
            SparStage("rear_mid",  0.024, wall, L),
            SparStage("rear_tip",  0.018, wall, L),
        ),
    )


@dataclass(frozen=True)
class WingSparSet:
    """All four spars (front + rear, both sides). Mass and section info."""

    front: TelescopingSpar = field(default_factory=default_front_spar)
    rear: TelescopingSpar = field(default_factory=default_rear_spar)

    def total_mass_kg(self, material: CFRPUDTube | None = None) -> float:
        m = material if material is not None else CFRPUDTube()
        # Two sides of each spar
        return 2 * (self.front.mass_kg(m) + self.rear.mass_kg(m))


def main() -> None:
    print("# Spar parametric model — MANTA defaults\n")
    set_ = WingSparSet()
    cfrp = CFRPUDTube()

    for spar in (set_.front, set_.rear):
        print(f"## {spar.name} spar")
        print(f"   total deployed length per side: {spar.total_length_m:.3f} m")
        print(f"   per-side mass: {spar.mass_kg(cfrp):.4f} kg")
        for s in spar.stages:
            print(f"   stage {s.name:12s}  OD={s.outer_diameter_m*1000:.1f}mm  "
                  f"t={s.wall_thickness_m*1000:.1f}mm  L={s.length_m:.3f} m  "
                  f"A={s.area_m2*1e6:.2f} mm²  I={s.I_m4*1e12:.3e} mm⁴  "
                  f"m={s.mass_kg(cfrp.rho)*1000:.1f} g")
        print()

    print(f"Total spar set mass (4 spars): {set_.total_mass_kg(cfrp):.3f} kg")


if __name__ == "__main__":
    main()
