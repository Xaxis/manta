# fcs/sim/

Software-in-the-loop (SITL) configuration. Wires the firmware against a 6-DOF vehicle model from `analysis/flightdynamics/` so envelope protection and the deployment state machine can be developed and regression-tested without hardware.

To be populated alongside `analysis/flightdynamics/nonlinear-6dof/`.

## Scenarios to maintain

- Nominal deploy → trim acquisition → glide.
- Asymmetric deploy → jettison → reserve.
- Stall guard activation under gust + CG perturbation.
- Sensor dropout (each channel, individually).
- AAD trigger at altitude.
