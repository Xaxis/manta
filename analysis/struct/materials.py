"""
Material properties for MANTA structural analysis.

All values are from manufacturer datasheets / cited textbook references with
explicit retrieval dates so the analysis is reproducible and auditable.
Whenever a number changes, update the citation date.

Knockdown philosophy
--------------------
We carry two stress bands per material:

    sigma_ultimate   — minimum tested ultimate stress from the datasheet,
                       at room temperature, dry, virgin material.

    sigma_allowable  — sigma_ultimate × knockdown × (1 / safety_factor)
                       where knockdown lumps environmental, fatigue,
                       cure variability, and notch effects;
                       safety_factor is the ultimate-load margin (1.5
                       per FAR Part 23 / EASA CS-LSA convention).

Knockdowns used here are conservative for hand-laid prepreg CFRP tubes
in a flight environment. Filament-wound or pulltruded tubes from a
qualified vendor can argue for less knockdown if backed by coupon tests.

References
----------
- Toray T800S/3900-2 datasheet (2024 retrieval, www.toray.com).
- Hexcel HexTow IM7/8552 datasheet (2024 retrieval).
- Niu, M. C. Y., *Composite Airframe Structures*, Conmilit Press, 1992,
  Ch. 3 (allowable stress philosophy) and Ch. 5 (tube design).
- DOT/FAA/CT-MMHB-17 *Composite Materials Handbook* (Vol. 2, polymer
  matrix composites — UD allowables and B-basis knockdown methodology).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CFRPUDTube:
    """Unidirectional carbon-fiber prepreg tube — fibers along the axis.

    A real tube has some hoop and ±45° plies; here we model an axially-
    dominant layup (≥ 80 % UD axial) with the cited allowables already
    accounting for that. For meaningful torsion or pressure work, this
    must be replaced by a layup-aware model (CLT).
    """

    name: str = "T800S/3900-2 UD CFRP tube (~85% axial)"
    citation_date: str = "2024-09"

    # Density
    rho: float = 1580.0  # kg/m^3, typical for autoclaved T800/epoxy

    # Stiffness (axial, fiber direction)
    E_axial: float = 165e9   # Pa, modulus along fibers (~85% of pure UD)
    E_transverse: float = 8.0e9
    G: float = 4.5e9         # in-plane shear modulus

    # Strength — virgin, room-temp, dry
    sigma_ult_tension: float = 1900e6     # Pa, ~85 % of pure UD T800 ~2200 MPa
    sigma_ult_compression: float = 1100e6 # compression-dominated by fiber kinking
    tau_ult: float = 75e6                  # interlaminar / in-plane shear

    # Knockdown lumped factor (enviro × fatigue × scatter × notch)
    knockdown: float = 0.55

    # Safety factor on top of allowable for ultimate-load qualification
    safety_factor_limit_to_ultimate: float = 1.5

    # ---- derived allowables -------------------------------------------------

    @property
    def sigma_allowable_tension(self) -> float:
        """Working tensile stress, with knockdown but BEFORE the limit→ult SF."""
        return self.sigma_ult_tension * self.knockdown

    @property
    def sigma_allowable_compression(self) -> float:
        return self.sigma_ult_compression * self.knockdown

    @property
    def sigma_design_compression_limit(self) -> float:
        """Design-limit compressive stress: allowable / 1.5."""
        return self.sigma_allowable_compression / self.safety_factor_limit_to_ultimate

    @property
    def sigma_design_tension_limit(self) -> float:
        return self.sigma_allowable_tension / self.safety_factor_limit_to_ultimate


@dataclass(frozen=True)
class DyneemaCompositeFabric:
    """Dyneema Composite Fabric (DCF) skin material — UHMWPE fiber laminate.

    Used for the wing covering. Light and very strong in tension; not a
    structural skin (no out-of-plane bending stiffness on its own — relies
    on rib pretension to take its operating shape).
    """

    name: str = "DCF (CT3K.18 series) UHMWPE laminate"
    citation_date: str = "2024-09"

    # Areal density
    rho_a: float = 0.050  # kg/m^2 (50 g/m^2, typical mid-weight)

    # In-plane tensile (per warp direction; bias is weaker)
    breaking_strength_warp_kn_per_m: float = 4.0   # kN/m
    elongation_at_break_pct: float = 1.5

    @property
    def sigma_ult_per_unit_width(self) -> float:
        return self.breaking_strength_warp_kn_per_m * 1e3  # N/m


@dataclass(frozen=True)
class AluminumAlloy:
    """Generic aerospace-grade aluminum (root fitting hardware)."""

    name: str = "7075-T6 aluminum"
    citation_date: str = "2024-09"
    rho: float = 2810.0
    E: float = 71.7e9
    sigma_yield: float = 503e6
    sigma_ult: float = 572e6
    knockdown: float = 0.85
    safety_factor: float = 1.5

    @property
    def sigma_allowable(self) -> float:
        return self.sigma_yield * self.knockdown / self.safety_factor


def main() -> None:
    print("# MANTA materials library\n")
    for cls in (CFRPUDTube, DyneemaCompositeFabric, AluminumAlloy):
        m = cls()
        print(f"## {m.name}")
        print(f"  citation: {m.citation_date}")
        for k, v in m.__dict__.items():
            print(f"    {k:30s} = {v}")
        # Expose properties (not in __dict__ for frozen dataclasses)
        for k in dir(m):
            if k.startswith("_"):
                continue
            if k in m.__dict__:
                continue
            try:
                v = getattr(m, k)
            except AttributeError:
                continue
            if callable(v):
                continue
            print(f"    {k:30s} = {v}  (derived)")
        print()


if __name__ == "__main__":
    main()
