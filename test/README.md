# test/

Test article specifications. Pairs with `docs/06-test-plan.md`.

## Progression (no skipping)

| Stage | Path | Gate to pass |
|---|---|---|
| Bench / component | (within `analysis/`) | Component data feeding into the rig design. |
| Ground deployment rig | [`ground/`](ground/) | **200 cycles symmetric within 10 ms 3-σ, no intervention, full thermal/humidity envelope.** |
| Tow article | [`tow/`](tow/) | Trim, stability, control authority, envelope protection demonstrated, deployed-on-tow only. |
| Drop article | [`drop/`](drop/) | End-to-end deploy + jettison + reserve verified, no human aboard. |
| Manned tow | (test plan extension) | Handling qualities acceptable, recovery procedures rehearsed. |
| Manned exit, deployed | (test plan extension) | Towed exit at altitude, released into known glide. |
| Manned exit, deploy-in-flight | (test plan extension) | Full operational profile. After all prior gates + separate go/no-go review. |

## Per-article README

Each test article subdirectory carries its own `README.md` with:

- Article description and scope.
- Instrumentation list.
- Test cards (one per run condition).
- Success and abort criteria.
- Site requirements.
- Linked safety review.
