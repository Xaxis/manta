"""
CD0 (zero-lift drag) estimate for the MANTA configuration.

Component buildup with literature-cited inputs and explicit uncertainty
brackets. The body-fairing CdA dominates and is the biggest first-cut
unknown — the 10:1 BRIEF target is gated on closing it ≤ 0.20 m².

References
----------
- Raymer, D. P., *Aircraft Design: A Conceptual Approach*, AIAA 6th ed. 2018,
  Ch. 12 (Aerodynamics): friction Cf, form factor FF, interference factor Q.
- Hoerner, S. F., *Fluid-Dynamic Drag*, self-published 1965, §3 (Skin friction)
  and §13 (Bodies of revolution / streamlined bodies).
- Anderson, J. D., *Fundamentals of Aerodynamics*, McGraw-Hill 6th ed., §5.10
  (Drag breakdown, parasite drag for low-speed configurations).
- Skydiving terminal velocity: prone-stable freefaller in a standard suit,
  CdA ≈ 0.40–0.50 m², from V_term ≈ 55 m/s at 80 kg (mg = ½ρV²·CdA).
- Faired prone-pilot ultralight reference: ARV Super2, Goldwing, Quicksilver
  GT. CdA in the 0.10–0.18 m² range with proper fairings, harness, faired feet.

Run as a script for a sensitivity table:

    PYTHONPATH=. .venv/bin/python analysis/aero/lift-drag/cd0.py
"""

from __future__ import annotations

from dataclasses import dataclass, field

import math


@dataclass
class Cd0Buildup:
    """Component build-up for parasite-drag coefficient (referenced to wing area)."""

    # ---- Wing -----------------------------------------------------------
    # Skin-friction Cf and form factor FF give: CD0_wing = Cf · FF · S_wet/S
    # Cf for fully turbulent flat plate (Schlichting):
    #     Cf = 0.074 / Re^0.2     (turb)
    # Cf for laminar:
    #     Cf = 1.328 / sqrt(Re)   (laminar)
    # We assume mostly-turbulent BL aft of an x_tr ≈ 30 % transition.
    # MAC-based Re at design speed is ~2×10⁶ (see airfoil/README).
    re_wing: float = 2.0e6
    transition_x_over_c: float = 0.30   # fraction laminar before transition
    section_t_c: float = 0.12           # for FF
    s_wet_over_s_ref: float = 2.06      # both sides; from planform.wetted_area / S
    wing_interference_factor: float = 1.05   # wing-body junction add-on

    # ---- Body fairing (the dominant unknown) ----------------------------
    # CdA of the prone, faired pilot+rig assembly. Three brackets:
    #   optimistic = 0.15 m²  (well-faired, low-drag belly fairing)
    #   nominal    = 0.20 m²  (representative v1 build, prone harness fairing)
    #   pessimistic= 0.30 m²  (rough surfaces, exposed lines, leg drag, etc.)
    body_cda_optimistic_m2: float = 0.15
    body_cda_nominal_m2: float = 0.20
    body_cda_pessimistic_m2: float = 0.30
    s_ref: float = 6.5      # resized planform (BRIEF #5)

    # ---- Miscellaneous (drogue stowage tether, sensors, gaps) ----------
    # Raymer recommends 5–10 % of clean CD0 as roughness/excrescence.
    misc_fraction_of_clean: float = 0.07

    # Optional explicit additions
    additional_components: list[tuple[str, float]] = field(default_factory=list)

    # ---- Derived helpers ------------------------------------------------

    @staticmethod
    def cf_turb(re: float) -> float:
        """Flat-plate fully-turbulent skin-friction coefficient (Schlichting)."""
        return 0.074 / re ** 0.2

    @staticmethod
    def cf_lam(re: float) -> float:
        """Flat-plate fully-laminar (Blasius) skin-friction coefficient."""
        return 1.328 / math.sqrt(re)

    def cf_mixed(self) -> float:
        """Mixed laminar/turbulent Cf with transition at x_tr/c."""
        re = self.re_wing
        x_tr = self.transition_x_over_c
        # Laminar contribution up to transition
        cf_lam_part = self.cf_lam(re) * x_tr
        # Turbulent contribution from transition to x/c = 1 (subtract the
        # virtual turbulent starting from leading edge for the laminar region)
        cf_turb_full = self.cf_turb(re)
        cf_turb_lam_region = self.cf_turb(re * x_tr) * x_tr
        cf_turb_part = cf_turb_full - cf_turb_lam_region
        return cf_lam_part + cf_turb_part

    @staticmethod
    def form_factor(t_c: float, x_c_max_t: float = 0.30) -> float:
        """Raymer Eq. 12.30 form factor for a wing section."""
        # FF = (1 + 0.6/(x/c)_max·(t/c) + 100·(t/c)^4) · 1.34·M^0.18·(cos Λ)^0.28
        # We are subsonic, M ≈ 0; sweep correction handled separately.
        return 1.0 + 0.6 / x_c_max_t * t_c + 100.0 * t_c ** 4

    # ---- Component contributions to CD0 referenced to S_ref ------------

    def cd0_wing(self) -> float:
        cf = self.cf_mixed()
        ff = self.form_factor(self.section_t_c)
        return cf * ff * self.s_wet_over_s_ref * self.wing_interference_factor

    def cd0_body(self, cda_m2: float) -> float:
        return cda_m2 / self.s_ref

    def cd0_misc(self, cd0_clean: float) -> float:
        return self.misc_fraction_of_clean * cd0_clean

    def total(self, body_cda_m2: float) -> tuple[float, dict[str, float]]:
        """Return (CD0_total, breakdown) for a given body CdA value."""
        cd_w = self.cd0_wing()
        cd_b = self.cd0_body(body_cda_m2)
        cd_extra = sum(c for _, c in self.additional_components) / self.s_ref
        cd_clean = cd_w + cd_b + cd_extra
        cd_misc = self.cd0_misc(cd_clean)
        cd_total = cd_clean + cd_misc
        breakdown = {
            "wing": cd_w,
            "body_fairing": cd_b,
            "misc/excrescence": cd_misc,
        }
        for name, cda in self.additional_components:
            breakdown[name] = cda / self.s_ref
        return cd_total, breakdown

    def sensitivity_table(self) -> list[tuple[str, float, float, dict]]:
        """Three brackets: optimistic / nominal / pessimistic. Returns
        list of (label, body_cda_m2, cd0_total, breakdown).
        """
        rows = []
        for label, cda in [
            ("optimistic", self.body_cda_optimistic_m2),
            ("nominal", self.body_cda_nominal_m2),
            ("pessimistic", self.body_cda_pessimistic_m2),
        ]:
            total, br = self.total(cda)
            rows.append((label, cda, total, br))
        return rows


def main() -> None:
    b = Cd0Buildup()
    print("# CD0 component build-up — MANTA")
    print()
    print(f"Wing Reynolds number   : {b.re_wing:.2e}")
    print(f"Mixed Cf               : {b.cf_mixed():.5f}  (laminar fraction {b.transition_x_over_c:.2f})")
    print(f"Form factor (t/c={b.section_t_c}) : {b.form_factor(b.section_t_c):.3f}")
    print(f"S_wet/S_ref            : {b.s_wet_over_s_ref:.3f}")
    print(f"CD0_wing               : {b.cd0_wing():.5f}   (referenced to S = {b.s_ref} m²)")
    print()
    print("Body-fairing CdA brackets (m²) and resulting CD0_body:")
    for label, cda in [("optimistic", b.body_cda_optimistic_m2),
                       ("nominal", b.body_cda_nominal_m2),
                       ("pessimistic", b.body_cda_pessimistic_m2)]:
        print(f"  {label:12s} CdA = {cda:.3f} m²   →  CD0_body = {b.cd0_body(cda):.5f}")
    print()
    print("Sensitivity table (CL = 0.5, e = 0.85, AR = 6.5):")
    print(f"  CDi at design CL  : {0.5**2 / (math.pi * 6.5 * 0.85):.4f}")
    print()
    print("| Bracket     | CdA (m²) |  CD0   |  CD     |  L/D  |")
    print("|---|---|---|---|---|")
    cdi_design = 0.5 ** 2 / (math.pi * 6.5 * 0.85)
    for label, cda, cd0, br in b.sensitivity_table():
        cd_total = cd0 + cdi_design
        lod = 0.5 / cd_total
        print(f"| {label:11s} |   {cda:.3f}   | {cd0:.4f} | {cd_total:.4f}  | {lod:5.2f} |")
    print()
    print("Breakdown for nominal:")
    _, br = b.total(b.body_cda_nominal_m2)
    for k, v in br.items():
        print(f"  {k:25s} CD0 = {v:.5f}")


if __name__ == "__main__":
    main()
