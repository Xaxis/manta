# safety/

Safety case artifacts. The bar:

> Would this analysis hold up if a coroner's office asked for it?

If no, it isn't done.

## Contents

| Path | Purpose |
|---|---|
| [`fmea.md`](fmea.md) | Failure mode and effects analysis. Living document — updated as design and test evidence evolves. |
| [`reserve-compat.md`](reserve-compat.md) | Reserve parachute compatibility analysis. The hard-constraint document — the rig must function for canopy flight and landing after wing jettison, and the reserve must deploy clean over the head with no wing-structure occlusion. |
| [`failure-modes/`](failure-modes/) | Per-failure-mode write-ups. Each one: detection, mitigation, response, residual risk, test evidence. |

## Process rules

- Every failure mode listed in `fmea.md` either has a mitigation that closes it to acceptable residual risk, or is a stop-work item until it does.
- Every safety-critical claim cites the test that verifies it — no test, the claim is unverified.
- Failure investigations during test must complete and be reviewed before progression. No "ran it again, it worked."
- Asymmetric deployment is the dominant unrecoverable failure mode (BRIEF). Its file under `failure-modes/` is the one that gets the most attention.
