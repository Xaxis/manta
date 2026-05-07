# fcs/firmware/

Flight controller firmware. Likely a PX4 fork; ArduPilot remains the reference implementation for cross-checking.

To be populated after `docs/04-fcs-architecture.md` is drafted and the topology is pinned.

## Likely modules (provisional)

- `deploy_sm/` — deployment state machine (per `docs/03-deployment-sequence.md`).
- `envelope/` — alpha, beta, q, roll-rate limiters (sourced from `fcs/envelope-protection/`).
- `jettison/` — pyrotechnic cutter command, AAD interface, asymmetric-deploy detector.
- `sensors/` — spar-lock microswitch debouncer, skin tension load cells, AoA front-end.
- `reversion/` — mechanical-reversion mode entry/exit, surface-loss detection.
