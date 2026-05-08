"""
Mass models for the non-spar wing components.

Each component has explicit assumptions, parameter knobs for sensitivity
sweeps, and citations where applicable. Whenever possible, the geometry
or sizing parameter (rib chord, skin area, etc.) is sourced from the
planform module so a planform change updates all downstream masses.
"""

from __future__ import annotations

from dataclasses import dataclass

from analysis.aero.planform.geometry import Planform
from analysis.struct.materials import CFRPUDTube


# ------------------------------------------------------------------------
# Bistable tape-spring rib
# ------------------------------------------------------------------------

@dataclass(frozen=True)
class TapeSpringRib:
    """A single bistable CFRP tape-spring rib boom.

    The rib is a thin curved strip (radius R_curve) of CFRP that snaps
    between a stable rolled state and a stable extended (open-shell)
    state. In the open state it acts as a stiff cantilever defining
    the airfoil section. Mass is dominated by the strip itself plus a
    small allowance for end fittings (bond pads to the spars at the LE
    and TE attachment points).
    """

    chord_length_m: float            # extended length along chord
    width_m: float = 0.040           # tape width (fixed)
    thickness_m: float = 0.0006      # 0.6 mm, typical bistable layup
    end_fitting_kg: float = 0.020    # 20 g of bond pad / spar attachment hardware
    cfrp: CFRPUDTube = CFRPUDTube()

    @property
    def strip_volume_m3(self) -> float:
        return self.chord_length_m * self.width_m * self.thickness_m

    def mass_kg(self) -> float:
        return self.strip_volume_m3 * self.cfrp.rho + self.end_fitting_kg


def rib_set_mass_kg(p: Planform, n_per_side: int = 9) -> float:
    """Total rib mass for both wings.

    Ribs are evenly distributed along the half-span and run from the LE
    to the TE at each station, so each rib's chord = local wing chord.
    """
    half_b = p.half_span
    total = 0.0
    for side in (-1, +1):
        for i in range(n_per_side):
            eta = (i + 0.5) / n_per_side
            y = side * eta * half_b
            chord = p.chord_at(y)
            rib = TapeSpringRib(chord_length_m=chord)
            total += rib.mass_kg()
    return total


# ------------------------------------------------------------------------
# DCF skin
# ------------------------------------------------------------------------

def skin_mass_kg(p: Planform, areal_density_kg_per_m2: float = 0.050,
                  bond_overhead_factor: float = 1.10) -> float:
    """Wing skin mass.

    `areal_density_kg_per_m2` is the bare DCF; the bond_overhead_factor
    accounts for adhesive at rib bonds, edge tape, sealing patches.
    """
    return p.wetted_area * areal_density_kg_per_m2 * bond_overhead_factor


# ------------------------------------------------------------------------
# Root fittings + pyrotechnic cutters
# ------------------------------------------------------------------------

@dataclass(frozen=True)
class RootFittingSet:
    """4 root fittings (one per spar root: 2 front + 2 rear) + 4 cutters.

    Each fitting is a machined 7075-T6 aluminum cup bonded to the spar
    root and bolted to the harness mount. Cutter is the linear-shaped-
    charge (LSC) cartridge or a guillotine-style mechanical cutter that
    severs the spar at the joint.
    """
    fitting_mass_kg: float = 0.180   # per fitting
    cutter_mass_kg: float = 0.220    # per cutter (initiator + LSC + housing)

    def total_kg(self) -> float:
        return 4 * self.fitting_mass_kg + 4 * self.cutter_mass_kg


# ------------------------------------------------------------------------
# Pneumatic deployment
# ------------------------------------------------------------------------

@dataclass(frozen=True)
class PneumaticSystem:
    """CO2-driven deployment plumbing per side.

    BRIEF specifies one CO2 cartridge per side, but a redundant-cartridge
    architecture is under study. This represents the single-cartridge
    baseline. Numbers from common sporting-goods CO2 hardware (88 g
    cartridges) plus rough estimates for lines and manifold.
    """
    cartridge_kg_per_side: float = 0.130   # 88 g CO2 + ~40 g bottle
    valve_kg: float = 0.150                 # central solenoid valve
    manifold_kg: float = 0.200              # impedance-matched manifold both sides
    lines_kg: float = 0.120                 # tubing, fittings

    def total_kg(self) -> float:
        return 2 * self.cartridge_kg_per_side + self.valve_kg + self.manifold_kg + self.lines_kg


# ------------------------------------------------------------------------
# FCS, servos, drogue, harness
# ------------------------------------------------------------------------

@dataclass(frozen=True)
class FlightControl:
    """Flight controller hardware."""
    pixhawk_kg: float = 0.090           # primary FCS
    pixhawk_redundant_kg: float = 0.090 # second FCS
    imu_aux_kg: float = 0.040           # backup standalone IMU
    pitot_static_kg: float = 0.060      # boom + sensor + tubing
    aoa_vane_kg: float = 0.040
    spar_lock_sensors_kg: float = 0.120 # microswitches × 6 + harness
    skin_tension_load_cells_kg: float = 0.120
    wiring_kg: float = 0.300

    def total_kg(self) -> float:
        return (self.pixhawk_kg + self.pixhawk_redundant_kg + self.imu_aux_kg
                + self.pitot_static_kg + self.aoa_vane_kg
                + self.spar_lock_sensors_kg + self.skin_tension_load_cells_kg
                + self.wiring_kg)


@dataclass(frozen=True)
class Actuators:
    """Brushless waterproof flaperon servos."""
    servo_kg: float = 0.150        # high-output waterproof servo
    n_servos: int = 4              # 2 per side (inner + outer flaperon segment)
    linkage_hardware_kg: float = 0.120
    mech_reversion_cable_kg: float = 0.250  # backup cable system

    def total_kg(self) -> float:
        return self.n_servos * self.servo_kg + self.linkage_hardware_kg + self.mech_reversion_cable_kg


@dataclass(frozen=True)
class Drogue:
    """Drogue + bridle + reefing line + pilot chute extractor."""
    drogue_canopy_kg: float = 0.180
    bridle_kg: float = 0.080
    pilot_chute_kg: float = 0.090

    def total_kg(self) -> float:
        return self.drogue_canopy_kg + self.bridle_kg + self.pilot_chute_kg


@dataclass(frozen=True)
class Harness:
    """Wing-harness shell + interface mount to skydiving rig."""
    shell_kg: float = 1.400          # CFRP/foam sandwich shell
    interface_plate_kg: float = 0.350 # bolts to the rig main lift webs
    pad_and_strap_kg: float = 0.250

    def total_kg(self) -> float:
        return self.shell_kg + self.interface_plate_kg + self.pad_and_strap_kg


def main() -> None:
    p = Planform()
    print("# Component masses (defaults)")
    print()
    print(f"Ribs (9 per side, 18 total): {rib_set_mass_kg(p, 9):.3f} kg")
    print(f"Skin (50 g/m²):              {skin_mass_kg(p):.3f} kg")
    print(f"Root fittings + cutters:     {RootFittingSet().total_kg():.3f} kg")
    print(f"Pneumatic deployment:        {PneumaticSystem().total_kg():.3f} kg")
    print(f"FCS:                         {FlightControl().total_kg():.3f} kg")
    print(f"Actuators:                   {Actuators().total_kg():.3f} kg")
    print(f"Drogue:                      {Drogue().total_kg():.3f} kg")
    print(f"Harness:                     {Harness().total_kg():.3f} kg")


if __name__ == "__main__":
    main()
