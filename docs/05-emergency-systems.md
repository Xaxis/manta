# 05 — Emergency Systems

**Status:** First-cut closed. Pulls together jettison logic, AAD
integration, reserve compatibility, asymmetric-deployment detection.
References the per-failure-mode write-ups for the binding analysis.

## Locked from BRIEF

- Four pyrotechnic cutters at the spar roots fully jettison the wing
  assembly.
- Triggers: pilot manual command, AAD activation, FCS detection of
  asymmetric deployment.
- After jettison, the underlying piggyback skydiving rig functions
  normally for canopy flight and landing.
- Reserve canopy must deploy clean over the head with no wing-structure
  occlusion.

## Cutter system

Four pyrotechnic spar-root cutters, one per spar root (front-left,
front-right, rear-left, rear-right).

| Property | Spec |
|---|---|
| Cutter type | Linear shaped charge (LSC), aluminum housing wrapping the spar root just outboard of the bonded fitting |
| Cut diameter | Front: 67 mm sized spar OD + 5 mm wall ferrule clearance; rear: 30 mm |
| Initiator | NASA-style standard initiator (NSI) or equivalent, dual per cutter (independent pyrotechnic + electrical paths) |
| Cut time | < 5 ms after fire signal received (LSC propagation) |
| No-fire margin | 1 W / 1 A for 5 minutes (B-basis, 95/95 confidence — defended by vendor coupon tests) |
| All-fire margin | 3.5 A for 50 ms |
| Operating environment | −20 °C to +50 °C, 95 % RH, sealed against rain/spray |
| Lockout | Mechanical pin (manual remove pre-flight) + electrical arm enabled by FCS state machine |

Per-cutter sourced from a qualified aerospace pyrotechnic vendor (e.g.
PacSci, Special Devices, or Ensign-Bickford). Selection criteria
documented in `cad/jettison/` after vendor consultation.

## Trigger sources

```
                       ┌── pilot manual abort handle ──┐
                       │                                │
                       ├── deployment SM                │
                       │     • asymmetric Δt > 10 ms    │
                       │     • spar-lock fail timeout   ├──► cutter
                       │     • drogue mal timeout       │      firing
                       │     • combined-mode aborts     │      circuit
                       │                                │
                       ├── FCS-bypass: AAD trigger      │
                       │     (independent power rail)   │
                       │                                │
                       └── ground-test override         ┘
                            (lockout disable cmd, ground only)
```

### Pilot manual abort

Mechanical handle on the harness, reachable in the deployed-wing
configuration. **Two-action requirement** to prevent inadvertent fire:
either two simultaneous switches (one mechanical, one electrical) or a
guarded handle requiring a deliberate two-handed pull. Reach geometry
verified in [`cad/harness/`](../cad/harness/) once the harness CAD is
built.

### FCS-detected aborts

Per [`docs/03-deployment-sequence.md`](03-deployment-sequence.md):

| Trigger | When |
|---|---|
| Asymmetric deploy: Δt_LR > 10 ms once both sides locked | < 1 ms after second-side lock |
| Spar-lock fail: any-channel-incomplete by 0.5 s after deploy cmd | 500 ms timeout |
| Drogue mal: load < 50 % at 4 s after extract | 4 s timeout |
| Roll-rate divergence: > 1.5 rad/s during deploy window | continuous monitor, 200 ms window |
| FCS irrecoverable fault during deploy | continuous |

### AAD trigger (FCS-bypassed)

The AAD has TWO outputs:
1. **Standard skydiving function** — fires the reserve container's
   built-in cutter (severs the closing loop). This works whether or not
   the wing is attached.
2. **MANTA-specific output** — a GPIO that drives the wing-cutter firing
   circuit DIRECTLY, bypassing the FCS. Independent power rail (the
   AAD's own battery), independent wiring, independent timing.

Sequence on AAD trigger:
- t=0: AAD detects altitude < threshold AND descent rate > threshold
- t=0: AAD GPIO fires → wing cutters fire
- t=+5 ms: cutters complete cut
- t=+30 ms: wing assembly mechanically separates
- t=+100 ms (programmed delay in AAD): standard reserve cutter fires
- t=+200 ms: reserve canopy bag launches

The 100 ms delay between wing-cutter fire and reserve-cutter fire is
the critical interlock — the wing must be CLEAR before the reserve
launches.

## Detected asymmetric-deployment response

This is the dominant case (per `safety/failure-modes/asymmetric-
deployment.md`). The full response is:

1. State machine detects Δt_LR > 10 ms or roll-rate > 1.5 rad/s.
2. Cutter firing circuit fires all 4 cutters simultaneously (≤ 5 ms after detection).
3. Wing assembly separates from the harness within 30 ms of cut.
4. State machine commands reserve deployment (or pilot does manually if
   FCS unable).
5. Reserve canopy inflates over the next 1.5–3 s.

Time from detection to fully-inflated reserve: < 4 s. Altitude loss in
that window at 30 m/s descent rate: ~120 m. Combined with the AAD
threshold of 200 m AGL, this leaves no margin for AAD-initiated abort
below ~320 m AGL — practically, the operational deployment altitude
floor is **~600 m AGL** to give margin to the AAD case if the BRIEF
asymmetric-detect path fails.

## Reserve compatibility

Detailed in [`safety/reserve-compat.md`](../safety/reserve-compat.md).
Summary of geometric constraints:

- Pre-jettison: harness does not interfere with reserve container,
  ripcord, RSL, or AAD function.
- Post-jettison: stubs at the four root fittings are < 80 mm long, in
  the chord plane only (no vertical projection into the reserve
  deployment cone), with rounded ferrule ends to prevent line snags.
- Reserve canopy launches in the +z hemisphere; clear of any
  post-jettison structure.
- Standard skydiving rig main canopy is unobstructed.
- Pilot can fly canopy and land normally with the harness mount in
  place (the bolted-down sub-frame stays with the rig; the wing
  assembly leaves on jettison).

## Verification gates

| Gate | Status | Where verified |
|---|---|---|
| Cutter no-fire / all-fire margins (per cutter) | Open | Vendor coupon tests + acceptance test per shipset |
| Cutter EMI immunity (HIRF / lightning per RTCA DO-160 G H) | Open | Bench EMI test |
| Lockout cycle test (mechanical + electrical) | Open | Bench |
| First firing on a complete wing assembly | Open | Ground rig (`test/ground/`) — the FIRST live cutter event in the program |
| Asymmetric-deploy firing on instrumented drop article | Open | Drop article (`test/drop/`) |
| Reserve clearance: symmetric + asymmetric jettison | Open | Drop article + bench mock-up |
| AAD interface integration | Open | Bench integration with selected AAD vendor |
| Pilot-handle reachability in deployed-wing configuration | Open | `cad/harness/` + bench mock-up |
| 200-cycle ground-rig gate (no lockout failures, no inadvertent fire) | Open | `test/ground/spec.md` |

Until all gates close, manned-deployment flight is not authorized. The
no-fire / all-fire and the lockout-cycle gates close BEFORE the first
ground-rig firing.

## What is *not* an emergency-system feature

By design:

- **No active stabilization during reserve descent.** Once the wing is
  jettisoned, the FCS is not in the loop — it's a normal skydiving
  reserve event.
- **No re-deployment of the wing in flight.** A jettison is one-way.
  No procedure for re-stowing in flight; the next deployment is on a
  re-packed assembly on the ground.
- **No partial jettison.** All four cutters fire together. A single-side
  jettison would leave a flapping half-wing and is not a valid mode.

## Open issues

1. **AAD vendor integration sign-off.** Need to confirm that the chosen
   AAD's GPIO supports the timing and electrical isolation MANTA
   requires.
2. **Cutter vendor selection.** Down-select from PacSci / SDI /
   Ensign-Bickford based on cost, lead time, NSI availability, and
   ability to deliver ≤ 5 ms cut on the 67 mm front-spar size.
3. **Stub geometry confirmation.** Once `cad/harness/` is built,
   confirm the stub-vs-reserve-cone clearance with a real harness
   placement, not the placeholder used here.
4. **Re-pack procedure documentation.** Once the cutters are integrated,
   write a rigger SOP for re-arming the cutters after a jettison event
   (post-incident).
