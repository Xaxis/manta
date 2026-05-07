# 04 — FCS Architecture

**Status:** stub.

## Scope

Flight control system: hardware redundancy, sensor suite, state estimation, control laws, envelope protection, mechanical reversion.

## Locked from BRIEF

- Redundant Pixhawk-class FCS.
- EKF at 400 Hz.
- MEMS IMUs (count, vendor, voting strategy TBD here).
- Flaperons on outer trailing edge each side, brushless waterproof servos.
- Mechanical cable reversion as last-resort backup — must be airworthy with FCS fully unpowered.
- Alpha limiter is mandatory and treated as a structural design assumption (i.e. the structure is sized assuming the limiter functions; loss-of-limiter is in the FMEA but stall departure is the dominant unrecoverable case for this airframe class).

## Topics

- Sensor suite: IMU(s), magnetometer, pitot-static, GNSS, AoA vane(s) or estimated AoA, spar-lock microswitches, skin tension load cells.
- FCS topology: dual or triplex? Voter or hot-standby? Where are independent failure boundaries?
- Bus / power: redundant rails, separable failure domains.
- EKF inputs and tuning notes; behavior under sensor dropout.
- Inner loop: rate damping, attitude.
- Outer loop: alpha hold during stall guard, airspeed, flightpath/glide.
- Envelope protection: alpha limiter, beta limiter, q limit, roll-rate limit. All hard-deck, not advisories.
- Mechanical reversion: which surfaces, which cable runs, force feedback to pilot, transition logic.
- Pilot input: how — yoke? body-tilt sensor? something else? Must work with arms inside the airframe profile during flight.

## Deliverables

- This doc.
- Block diagrams under `fcs/`.
- SITL configuration under `fcs/sim/` for envelope-protection development.
- `fcs/envelope-protection/` — alpha-limiter prototype with unit tests, gain scheduling vs. airspeed.
- 3D model of FCS bay, sensor placement, servo locations under `cad/fcs/`.
