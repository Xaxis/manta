# fcs/envelope-protection/

Envelope protection logic. The **alpha limiter is mandatory and treated as a structural design assumption** (BRIEF). Other limiters (beta, q, roll rate) are also hard-deck, not advisories.

Standalone development environment: implementation + unit tests + parameter studies + HIL fixtures live here, then the validated code is pulled into `fcs/firmware/`.

## To populate

- `alpha_limiter/` — design, gain schedule vs. airspeed, anti-windup, sensor-fault behavior, unit tests.
- `beta_limiter/`, `q_limiter/`, `p_limiter/` — same structure, lower priority than alpha.
- `tests/` — pytest suite + scenario library (gust, CG perturbation, sensor dropout) running against the linearized + nonlinear models from `analysis/flightdynamics/`.

## Why this is its own directory

- The alpha limiter is on the critical path for human safety.
- It must be testable without a full firmware build.
- It will be cited in the safety case and reviewed independently.
