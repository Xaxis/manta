# Failure mode: mechanical reversion path blocked or fouled

**FMEA ID:** `FM-RVR-001`
**Severity:** Hazardous (loss of last-resort manual control)
**Pre-mitigation likelihood:** ~10⁻³ per flight (uncharacterized cable routing)
**Post-mitigation likelihood:** < 10⁻⁵ per flight

## What it means

The mechanical-reversion path (per [`docs/04`](../../docs/04-fcs-architecture.md))
is the last-line manual control mode for MANTA. When both FCS-A and
FCS-B are unrecoverable, the pilot can engage a direct mechanical
linkage from a stick input to a single set of flaperon surfaces.

Reversion-fault means that path is unavailable when needed:

| Sub-case | Cause |
|---|---|
| Cable run chafed through and broken | Long-cycle operation rubbing against deployed wing structure |
| Reversion cable fouled in deployed wing | Cable gets tangled with rib, spar, or skin on deployment |
| Reversion clutch fails to engage | Mechanical mechanism stuck or jammed |
| Cable end-fitting failure | Swage joint pull-out or fatigue crack |
| Reversion-mode entry inhibited by software | FCS health monitor incorrectly believes a unit is healthy |

In any of those cases, the FCS-failure scenario that prompted reversion
becomes catastrophic: pilot has no envelope protection AND no manual
control. Procedure becomes immediate jettison + reserve.

## Detection

- **Pre-flight functional check** of the reversion path: pilot or
  ground crew engages reversion mode (with FCS power off) and verifies
  the surfaces respond to stick input across the full deflection range.
  Refuses to fly if any surface fails to track.
- **Cable tension sensors** (one per cable): report cable tension, slack,
  and over-tension. Loss of tension indicates a broken cable or end
  fitting.
- **Clutch-engaged microswitch**: confirms mechanical clutch is in the
  reversion-engaged position when commanded.
- **In-flight pilot perception**: stick force feedback. Spongy or
  asymmetric resistance indicates a routing or cable issue.

## Mitigation chain

1. **Routing analysis in CAD.** [`cad/fcs/`](../../cad/fcs/) (TBD content)
   documents the cable routing through the wing root, spar bay, and to
   the flaperon hinges. No path that crosses a deployed structure
   element (no cable lying in the path of an unfurling rib, no cable
   crossing a telescoping joint, etc.).
2. **Cable redundancy.** Two parallel cables to each flaperon surface,
   independently swaged. Single-cable break does not lose the surface.
3. **Cable end-fitting inspection.** Pre-flight visual + tactile check
   of each swage; any cracked / pulled fitting → no fly. Cable life
   limit specified per cycle count, retired well before fatigue
   failure.
4. **Clutch design with positive engagement.** Mechanical clutch fully
   engages with a 50 N pilot stick force; clutch position is sensed
   and reported; cannot enter reversion mode if clutch reports failed
   engagement.
5. **Software interlock.** FCS will engage reversion if EITHER FCS unit
   is faulted AND the other reports degraded EKF. Over-conservative
   triggering is acceptable; under-conservative is not.
6. **Pilot training.** Reversion-mode operation is rehearsed in the
   tow article (`test/tow/`) before manned-deploy flights. Pilot is
   familiar with stick force and surface response in reversion.

## Residual risk

After mitigation:

- **Both parallel cables fail simultaneously** (correlated chafe,
  manufacturing batch defect): ~10⁻⁵ per flight. Mitigated by routing
  separation and per-shipset swage cert.
- **Clutch fault not caught by pre-flight self-test**: ~10⁻⁵ per
  flight. Mitigated by built-in test that exercises the full
  engagement chain.
- **Cable cut during a deployment event** (unlikely but credible —
  e.g. asymmetric deploy that whips a cable): ~10⁻⁶ per deploy.
  Mitigated by routing the cable away from any structure that moves
  during deploy.
- **Pilot can't fly in reversion mode** (Cooper-Harper rating worse
  than expected): not yet measured. If tow-article HQR comes in worse
  than Level 4, reversion becomes a "jettison-only" mode — pilot
  cannot maintain control but the surfaces still move under stick
  input, allowing some attempt at controlled descent before reserve.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| Cable routing CAD review | Path clear of all deployed structure motion | `cad/fcs/` |
| Cable cycle test | Single cable survives 10,000 deflection cycles with no failure | Bench |
| Swage strength test | Each end-fitting holds 2× max expected pilot stick load | Bench |
| Reversion-mode pre-flight self-test | Built-in test verifies clutch engagement and surface tracking | Bench HIL |
| Tow-article reversion test | Pilot flies the tow article in reversion mode for at least 5 minutes; Cooper-Harper rated | [`test/tow/`](../../test/tow/) |
| Manned-deploy preconditioning | Pilot has rehearsed reversion-mode entry from cruise + recovery before first manned-deploy flight | Pilot training program |

## Open issues

- Cable routing is not yet detailed. `cad/fcs/build.py` (TBD) needs
  to add the cable runs alongside the FCS hardware placement.
- Reversion-mode pilot input device is unsettled — yoke, stick, or
  body-tilt sensor. Constrained by the deployed-wing pilot ergonomics.
- The tow article is the first place we'll measure handling qualities
  in reversion mode. If HQR is poor, the architecture might need to
  add a low-bandwidth pitch damper that operates even with FCS-A and
  FCS-B faulted (a third independent control authority). That would
  be a substantial design change.
