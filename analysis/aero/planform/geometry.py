"""
MANTA planform geometry — single source of truth.

All downstream aero analyses (Weissinger, AVL deck generation, trim, glide polar)
and the parametric wing CAD should import from this module rather than re-deriving
chord/sweep/area numbers from BRIEF parameters.

Inputs are the locked planform decisions in BRIEF.md; outputs are the derived
quantities needed by the rest of the pipeline.

Run as a script to print a Markdown summary table:

    python analysis/aero/planform/geometry.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Planform:
    """Tapered, swept, planar half-wing referenced to total (both-sides) area.

    Sign and frame convention:
        - x: aft (drag direction)
        - y: starboard (right wing)
        - z: up
        - Wing apex (root LE) is the origin.
        - Sweep angles are positive aft.
        - Twist (washout) is positive geometric leading-edge-down at the tip
          relative to the root, i.e. the tip flies at lower local angle of
          attack than the root.

    Inputs are the locked BRIEF parameters. Don't edit these without amending
    BRIEF.md and updating docs/00-design-rationale.md.
    """

    S: float = 6.5         # total wing reference area, m^2 (both sides)
    b: float = 6.3         # tip-to-tip span, m
    taper: float = 0.4     # tip chord / root chord
    sweep_le_deg: float = 25.0  # leading-edge sweep, degrees
    washout_deg: float = 6.0    # geometric twist, root → tip, degrees (top of BRIEF range)
    section_t_c: float = 0.12   # representative airfoil thickness ratio for wetted area

    # ---- Derived quantities -------------------------------------------------

    @property
    def aspect_ratio(self) -> float:
        return self.b**2 / self.S

    @property
    def half_span(self) -> float:
        return self.b / 2.0

    @property
    def chord_root(self) -> float:
        # S = (b/2)·(c_root + c_tip) per side → 2S/b across both sides;
        # S = b·(c_root + c_tip)/2 = b·c_root·(1+λ)/2  →  c_root = 2S / [b·(1+λ)]
        return 2.0 * self.S / (self.b * (1.0 + self.taper))

    @property
    def chord_tip(self) -> float:
        return self.taper * self.chord_root

    @property
    def mac(self) -> float:
        """Mean aerodynamic chord."""
        cr, t = self.chord_root, self.taper
        return (2.0 / 3.0) * cr * (1.0 + t + t**2) / (1.0 + t)

    @property
    def y_mac(self) -> float:
        """Spanwise station of the MAC, measured from root chord (y=0) toward the tip."""
        return (self.b / 6.0) * (1.0 + 2.0 * self.taper) / (1.0 + self.taper)

    @property
    def x_mac_le(self) -> float:
        """Streamwise position of the MAC's leading edge, aft of root LE."""
        return self.y_mac * math.tan(math.radians(self.sweep_le_deg))

    @property
    def x_mac_c4(self) -> float:
        """Streamwise position of the MAC's quarter chord, aft of root LE."""
        return self.x_mac_le + 0.25 * self.mac

    def sweep_at_chord_fraction_deg(self, x_over_c: float) -> float:
        """Sweep angle (degrees) of the constant x/c locus.

        For a linearly tapered planform, the locus of constant fractional chord
        is a straight line; its slope is tan(Λ_LE) − 2·(x/c)·c_root·(1−λ)/b.
        """
        tan_le = math.tan(math.radians(self.sweep_le_deg))
        slope = tan_le - 2.0 * x_over_c * self.chord_root * (1.0 - self.taper) / self.b
        return math.degrees(math.atan(slope))

    @property
    def sweep_c4_deg(self) -> float:
        return self.sweep_at_chord_fraction_deg(0.25)

    @property
    def sweep_te_deg(self) -> float:
        return self.sweep_at_chord_fraction_deg(1.0)

    @property
    def wetted_area(self) -> float:
        """Wing wetted area (both sides), Raymer Eq. 12.43 form factor.

        S_wet ≈ 2·S·(1 + 0.25·t/c)  for thin sections (t/c < 0.2).
        Slightly conservative for cambered/reflexed sections.
        """
        return 2.0 * self.S * (1.0 + 0.25 * self.section_t_c)

    @property
    def wing_loading_design(self) -> float:
        """Wing loading at the design pilot+system mass (median 82.5 kg pilot + 15.5 kg wing
        + 10 kg rig allowance), N/m^2. For sanity-check display only.
        """
        m_total = 82.5 + 15.5 + 10.0
        g = 9.80665
        return m_total * g / self.S

    # ---- Geometry helpers (called by CAD + Weissinger) ----------------------

    def chord_at(self, y: float) -> float:
        """Chord length at spanwise station y (m), where 0 ≤ y ≤ b/2."""
        eta = abs(y) / self.half_span  # 0 root, 1 tip
        return self.chord_root * (1.0 - (1.0 - self.taper) * eta)

    def x_le_at(self, y: float) -> float:
        """Leading-edge x-coordinate at spanwise station y."""
        return abs(y) * math.tan(math.radians(self.sweep_le_deg))

    def twist_at(self, y: float) -> float:
        """Geometric twist (degrees, washout positive) at station y.

        Linear distribution from 0 at root to washout_deg at tip, by convention.
        """
        eta = abs(y) / self.half_span
        return -self.washout_deg * eta  # negative = nose down at tip = washout


def summary_markdown(p: Planform) -> str:
    """Markdown table of derived geometry — used by the docs and by humans."""
    rows = [
        ("Wing area  S",                f"{p.S:.3f} m²"),
        ("Span       b",                f"{p.b:.3f} m"),
        ("Aspect ratio",                f"{p.aspect_ratio:.3f}"),
        ("Taper      λ",                f"{p.taper:.3f}"),
        ("Root chord c_root",           f"{p.chord_root:.4f} m"),
        ("Tip chord  c_tip",            f"{p.chord_tip:.4f} m"),
        ("MAC",                         f"{p.mac:.4f} m"),
        ("y_MAC (from root)",           f"{p.y_mac:.4f} m"),
        ("x_MAC LE (aft of root LE)",   f"{p.x_mac_le:.4f} m"),
        ("x_MAC c/4 (aft of root LE)",  f"{p.x_mac_c4:.4f} m"),
        ("Sweep LE",                    f"{p.sweep_le_deg:.2f}°"),
        ("Sweep c/4",                   f"{p.sweep_c4_deg:.2f}°"),
        ("Sweep TE",                    f"{p.sweep_te_deg:.2f}°"),
        ("Geometric twist (washout)",   f"{p.washout_deg:.2f}°"),
        ("Wetted area S_wet",           f"{p.wetted_area:.3f} m²"),
        ("Wing loading W/S (design)",   f"{p.wing_loading_design:.2f} N/m²"),
    ]
    width = max(len(name) for name, _ in rows)
    lines = ["| Quantity | Value |", "|---|---|"]
    for name, value in rows:
        lines.append(f"| {name.ljust(width)} | {value} |")
    return "\n".join(lines)


if __name__ == "__main__":
    p = Planform()
    print(summary_markdown(p))
