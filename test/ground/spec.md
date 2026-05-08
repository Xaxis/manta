# Ground deployment rig — specification

**Status:** Specification draft. Build is gated on adopting Option B from
the symmetry budget (see `analysis/deployment/symmetry-budget.md`) and
finalizing the cutter / fitting interface (`cad/jettison/`).

This is the **first piece of MANTA hardware to actually build** per BRIEF.
It exists to:

1. Measure each contributor in the deployment-symmetry budget.
2. Demonstrate sub-10 ms 3-σ symmetric deployment with the chosen architecture.
3. Run the **200-cycle reliability gate** that gates progression to flight-relevant test articles.

The rig is *intentionally not flightworthy*. Wing assembly is held in a
fixture; there are no flight loads, no dynamic pressure, no pilot.
The rig isolates and measures the deployment kinematics and timing.

## Architecture

```
                    ┌─────────────────────────────────────────────┐
                    │    Stiff steel-tube test stand               │
                    │                                              │
                    │    ┌─────────────────────────────────┐       │
                    │    │  Wing assembly (front + rear    │       │
                    │    │  spars, 18 ribs, full skin)     │       │
                    │    │  fixed at root mount points     │       │
                    │    └─────────────────────────────────┘       │
                    │                                              │
                    │    Environmental enclosure (separate):       │
                    │      thermal chamber                         │
                    │      humidity / spray rig                    │
                    │      ice conditioning bath                   │
                    └─────────────────────────────────────────────┘

   Instrumentation chain:
     spar-lock microswitches  ──►  GPIO (1 kHz)
     valve actuation feedback ──►  ADC (10 kHz)
     manifold pressure (per port) ──► ADC (10 kHz)
     CO2 bottle T + P  ──────►  serial probe
     skin tension load cells (4) ──► ADC (1 kHz)
     ambient T, RH ──────────► serial probe
     high-speed video (≥ 1000 fps, both wings, two angles) ──► storage
```

### Mechanical fixture

- 80/20 extrusion frame, ~2 m × 4 m footprint, leveling feet.
- Wing root fixture: bolts to a sub-frame matching the harness mount pattern
  (BRIEF "wing harness sits on top of the piggyback rig"). Sub-frame is
  rigid enough to prevent root spar motion during deploy (>10 kN root
  reaction over ~50 ms — sized in `analysis/struct/spar_bending.py`).
- Wing tips: free. The tips swing through ~3.7 m arcs during deploy and
  must clear the test floor and walls. Side clearance ≥ 4.5 m total to
  the nearest obstruction.
- Restow station: tooling fixture that re-stows the wing into its packed
  configuration between cycles. Designed for ≤ 5 minute restow cycle so
  that 200 cycles fits in a working week.

### Environmental conditioning

Per BRIEF: cold + wet are the binding cases. The rig must condition the
wing assembly to the operational envelope:

| Condition | Temperature | Humidity / wet state | Soak time | Cycles |
|---|---|---|---|---|
| Hot/dry | +50 °C | < 20 % RH | 4 h | 25 |
| Hot/humid | +35 °C | 95 % RH | 4 h | 25 |
| Nominal | +20 °C | 50 % RH | — | 50 |
| Cold | −10 °C | dry | 4 h | 25 |
| Cold/wet | −5 °C | spray-soaked then drained | 4 h | 25 |
| Cold/iced | −5 °C | spray-soaked, then frozen | 6 h | 25 |
| Re-test in nominal | +20 °C | 50 % RH | — | 25 |
| **Total** | | | | **200** |

The conditioning enclosure can be a separate walk-in chamber; the
deployment fixture is on a roll-in cart.

## Instrumentation

### Per-side stage-lock sensors

Six microswitches per wing assembly (3 stages × 2 sides) — one at each
telescoping joint, latching when the inner stage is fully extended and the
locking pin engages. Required:

- Latency from physical close to FCS GPIO read: **≤ 1 ms** (drives microswitch
  selection — many parts have 5–10 ms software-debounce by default; we want
  hardware contact only).
- Two independent contacts per switch, cross-checked.
- Voltage levels and connector compatible with the FCS GPIO bus.

Verification of latency is a bench step in itself: drop test of a switch
into a calibrated impactor, capture both the impact event (accelerometer)
and the switch close (GPIO) at 100 kHz, verify ≤ 1 ms.

### Manifold pressure sensors

Pressure transducer in each port of the CO2 manifold (2 sides + supply line):
- Range 0–100 bar
- 10 kHz sample rate
- Time-aligned to all other channels

These are the primary diagnostic for manifold balance issues and CO2
cartridge variance.

### Skin tension load cells

4 load cells (2 per side) at the chord stations specified in
`docs/02-structural-budget.md`. Used for:
- Skin attachment and tension diagnostic during deploy
- Detection of skin failure or asymmetric tension (advisory only)

Range 0–500 N each, 1 kHz sample rate.

### Drogue load cell

Not strictly needed on the ground rig (no drogue sequence here), but the
mounting interface should be present so the same fixture can support
later drop-test article verification.

### High-speed video

Two cameras at ≥ 1000 fps, each with a clean view of one wing's full deploy
trajectory. Synchronized to the DAQ (LED tally light or external trigger
on every frame). Used for:
- Per-rib unfurl time measurement (frame extraction)
- Visual confirmation of skin behavior
- Failure-mode diagnostics when something breaks

A single fixed studio-style lighting setup is acceptable; outdoor / sun
not required.

### Ambient + assembly conditioning probes

- Ambient T / RH at fixture
- T probe in the wing assembly itself (between ribs, mid-span)
- T + P at each CO2 cartridge

## Data acquisition

- Centralized DAQ with hardware-time alignment across all channels.
- Per-cycle, write a single HDF5 file with all channels indexed by cycle
  number, environmental condition, and timestamp.
- File naming: `cycle_{NNN}_{condition}.h5`.
- Retention: every cycle, no exceptions.

## Test cards

A single test card is one trial: condition → cycle → data file. The card
captures:

- Condition label
- Wing assembly serial / build number
- Restow notes (any anomalies during pack)
- Environmental probe values immediately before fire
- CO2 cartridge serial(s) and pre-fire mass / temperature
- Operator
- Timestamp
- Pass / fail per channel
- Free-text observation

200 cards per gate run. Card template lives in `test/ground/cards/`.

## Pass / fail criteria — the 200-cycle gate

Per BRIEF:
> Do not move to flight-relevant test articles until the ground deployment
> rig has demonstrated reliable, symmetric, sensed deployment over the full
> thermal and humidity envelope, in at least 200 cycles without intervention.

Concretely:

1. **Symmetry:** measured |Δt_LR| from the spar-lock channels, across all
   200 cycles, has 3-σ ≤ 10 ms. (See `analysis/deployment/symmetry-budget.md`
   for what makes this realistic.)
2. **Lock confirmation:** all 6 lock channels confirm within 50 ms of cmd
   on every cycle. No "no-lock" events.
3. **Skin tension:** load cells reach > 80 % of nominal within 200 ms on
   every cycle.
4. **No intervention:** no operator touched the wing during a cycle other
   than restow between cycles. No fixed/repaired components mid-run; if
   anything breaks, the cycle stops, the failure is investigated and
   written up, and the count restarts after the fix.
5. **Cross-condition:** at least 25 cycles in each of the conditions in
   the table above.

## Failure-investigation protocol

When something breaks:

1. Stop, do not attempt a recovery deploy or a "let's see if it works
   the next time".
2. Capture state: photograph fixture, save the cycle's HDF5 file, save the
   high-speed video, note operator's last actions, note environmental probe
   values.
3. Open a failure-investigation entry in `safety/failure-modes/`.
4. Write up: what happened, what data shows, root cause, remedy.
5. Verify remedy: 5 cycles of the same condition before resuming the gate
   sequence.
6. Restart the 200-cycle count after the fix is verified — no "credit" for
   pre-fix cycles.

This is the BRIEF rule: "no 'ran it again, it worked' closures."

## Hardware deliverables (in build order)

1. Steel-tube fixture frame + sub-frame.
2. Restow station tooling.
3. Lock sensor selection + latency-verified bench test.
4. Manifold pressure transducer integration.
5. Skin-tension load cell integration.
6. DAQ + storage chain.
7. High-speed video setup + sync.
8. Environmental chamber rental / build, plus spray rig and ice bath.
9. First wing assembly built specifically for the rig (designated R-1).
10. Test cards printed, run procedure rehearsed dry.
11. First condition run (nominal × 50 cycles) for sensor and procedure
    debugging — *not* counted toward the gate.
12. Gate run (200 cycles).

## What's deferred

- Pyrotechnic cutter firings on the ground rig: only after the deploy gate
  is closed, in a separate test campaign at a range with appropriate
  controls. Cutter firings are destructive to the root fittings, so each
  is a single-shot.
- Drogue sequence: lives in the drop article (`test/drop/`), not here.
- Flight loads: lives in tow article (`test/tow/`), not here.

## Reproducibility / cost note

The rig is fixture and instrumentation only — no flight hardware in it
that isn't a serviceable wing assembly. Estimated build cost is dominated
by the environmental chamber (rent vs. build trade) and the high-speed
video. Both are reusable for downstream test articles. Order-of-magnitude
$100k – $250k depending on chamber choice. Time-to-first-fire: 3–6 months
from spec lock.
