# docs/

Numbered design documents. Read in order.

| # | Document | What it covers |
|---|---|---|
| 00 | [design-rationale](00-design-rationale.md) | Why each locked architecture decision in BRIEF.md was made. The trade studies and rejected alternatives. |
| 01 | [aero-sizing](01-aero-sizing.md) | Planform sizing, airfoil selection, sweep/taper/washout, AVL trim and stability derivatives, L/D verification. |
| 02 | [structural-budget](02-structural-budget.md) | Mass budget, load cases, spar sizing, root-joint design, safety factors. |
| 03 | [deployment-sequence](03-deployment-sequence.md) | Full deployment timeline, sensed handshakes, abort logic. The doc the safety case is built around. |
| 04 | [fcs-architecture](04-fcs-architecture.md) | FCS hardware redundancy, sensor suite, EKF, control laws, envelope protection, mechanical reversion. |
| 05 | [emergency-systems](05-emergency-systems.md) | Spar-root cutters, jettison logic, AAD integration, reserve compatibility, asymmetric-deployment detection. |
| 06 | [test-plan](06-test-plan.md) | Ground rig → tow article → drop article → flight test progression. Gates, instrumentation, success criteria. |

## Conventions

- Every quantitative claim cites its source (analysis file, vendor datasheet, textbook + page).
- Every assumption is called out explicitly, with a justification or a TODO to retire it.
- Every safety-critical claim has a corresponding test in the test plan.
- Diagrams live alongside the doc that references them.
- Treat docs as code — review changes.
