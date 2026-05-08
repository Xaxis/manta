# Failure mode: reserve canopy fouling on residual structure

**FMEA ID:** `FM-RSV-001`
**Severity:** Catastrophic
**Pre-mitigation likelihood:** ~10⁻³ per jettison (uncharacterized stub geometry)
**Post-mitigation likelihood:** < 10⁻⁴ per jettison (CAD-verified geometry, drop-article confirmed)

## What it means

After the four pyrotechnic cutters fire and the wing assembly departs,
short stubs remain at each of the four root fittings (per
[`safety/reserve-compat.md`](../reserve-compat.md)). The reserve
canopy's pilot chute, bag, lines, or the canopy itself can:

- **Snag on a stub** during line-stretch — line wraps an exposed
  spar end with severed-fiber edge.
- **Inflate INTO a stub** — canopy fabric finds itself trapped
  between a stub and clear air.
- **Foul with the bridle** — drogue bridle, if not cleanly released,
  trails through the reserve deployment path.
- **Inflate asymmetrically** because part of the canopy contacts
  residual structure during inflation, producing a streamer or
  partial inflation that does not provide a survivable descent.

Any of those produces a partially-inflated reserve canopy that may not
control descent rate to a survivable landing.

## Detection

This is a **post-failure** mode — by the time fouling is detected, the
canopy is already mis-inflated. Detection is for the program (root
cause analysis after a survivable event) and for the test campaign
(drop article instrumented for it):

- **Pilot perception** under the reserve: hard to characterize at
  ~150–200 m AGL; pilot may have only 5–10 s before ground impact.
- **Drop article telemetry** + on-board video: characterize each
  drop event for inflation cleanliness.
- **Post-flight rigging inspection**: any reserve event that resulted
  in a survivable but non-nominal descent gets a full rigging
  forensic to identify whether residual structure contributed.

## Mitigation chain — preventive

This is dominantly a *preventive* mitigation problem (you don't recover
from fouling once inflation is already going badly):

1. **Stub geometry verified clear of the reserve cone in CAD.** Per
   `cad/harness/build.py` — all 4 stubs are 0.42–1.15 m outboard of
   the 30°-half-angle reserve cone at the apex level. Geometric
   margin documented per shipset.
2. **Rounded ferrule end on each cut spar.** Internal ferrule
   captures the spar end during the LSC cut, leaving a smooth
   metallic terminus rather than a frayed CFRP fiber tip. Vendor
   verification per cutter design.
3. **Drogue bridle cleanly releases with the wing.** The bridle is
   attached at the wing root and cut by a dedicated mechanism (line
   cutter or pyrotechnic) at the wing-stable signal. If the wing
   departs, the bridle goes with it.
4. **Stub orientation in the chord plane only.** No vertical
   projection above the harness shell: stubs are mounted on the
   sub-frame plate, axes parallel to body x-axis. Reserve cone
   opens upward; stubs sit horizontally, far from the cone.
5. **Drop-article verification across the credible attitude
   envelope.** The drop article exercises symmetric, asymmetric, and
   pilot-pulled-tilted reserve deployments. Each one is logged.
6. **Pre-flight visual inspection of the stubs** to verify no damage,
   debris accumulation, or foreign hardware that could change the
   geometry from the verified configuration.

## Mitigation chain — recovery

If a reserve fouling does occur and the pilot is conscious enough to
respond:

- **Cutaway the reserve risers** is NOT possible — most modern reserves
  have no cutaway capability. (Fouling is the reason main-canopy
  cutaway exists; reserves don't have it because the reserve is the
  last canopy.)
- **Manual canopy clearing** — if the pilot can identify a snag,
  pulling the affected line(s) sometimes clears the fouling. This is
  a skill that is rehearsed in skydiving but is not reliable.
- **Ride the partial inflation to terrain** with whatever descent rate
  results. Pilot trains for high-impact landing in this case.

In summary: recovery from reserve fouling is unreliable. The mitigation
strategy is overwhelmingly weighted toward prevention.

## Residual risk

After mitigation:

- **Stub geometry verified but a new shipset has a manufacturing
  error** that pushes a stub into the cone: ~10⁻⁴ per shipset.
  Mitigated by per-shipset CAD-vs-as-built geometry verification.
- **Drogue bridle release fails AND wing fails to depart cleanly**:
  rare combined failure; mitigated by independent failure-paths in
  the bridle release and the cutter system.
- **Reserve canopy itself has a malfunction** (line twist, partial
  inflation) that's NOT caused by MANTA-specific structure: this is
  baseline skydiving residual risk (~10⁻⁴ per skydive), unaffected
  by MANTA mitigation.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| CAD clearance check (per shipset) | Stub geometry vs. reserve cone with ≥ 3× margin in all dimensions | [`cad/harness/build.py`](../../cad/harness/build.py) |
| Bench mock-up | Physical geometry agrees with CAD; rigger sign-off | TBD |
| Drop article — symmetric jettison reserve deployment | Reserve clears stubs cleanly at design speed | [`test/drop/`](../../test/drop/) |
| Drop article — asymmetric jettison | Reserve clears stubs in worst-credible attitude | Same |
| Drop article — pilot-induced unusual attitude | Reserve clears stubs from inverted, banked, tumbling | Same |
| Drogue release verification | Bridle clears the wing-departure path | Bench + drop |
| Per-shipset stub geometry inspection | As-built stubs match the CAD-verified configuration to within tolerance | Acceptance |

## Open issues

- The rounded-ferrule cutter design is conceptual — vendor selection
  pending.
- Reserve-line-clearing pilot training has not been formalized into
  the MANTA pilot transition syllabus (`docs/07-pilot-training.md`
  TBD).
- Drop-article instrumentation for fouling characterization is heavier
  than the standard drop article — needs additional cameras and on-
  board recording. Spec'd as part of [`test/drop/spec.md`](../../test/drop/) when written.
