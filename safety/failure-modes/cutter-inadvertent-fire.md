# Failure mode: pyrotechnic cutter inadvertent fire

**FMEA ID:** `FM-CUT-002`
**Severity:** Catastrophic if airborne; major if on the ground
**Pre-mitigation likelihood:** ~10⁻⁴ per cutter operating hour (no lockout, ungated arm)
**Post-mitigation likelihood:** < 10⁻⁷ per cutter operating hour

## What it means

A pyrotechnic cutter fires when it should not. Possible triggers
(in roughly increasing order of difficulty to defend against):

| Trigger | Mechanism |
|---|---|
| Static discharge into initiator | Body or atmospheric ESD couples into firing leads |
| EMI / lightning | Radio-frequency or lightning-induced current in firing leads |
| Software bug | FCS issues a fire command in a state where it shouldn't |
| Hardware fault | Latch-up, stuck-on transistor, shorted firing capacitor |
| Mishandling | Lockout pin removed too early; arm switch closed at wrong time |
| Ground-handling event | Drop, impact, vehicle crash with wing on top |

Consequence depends on flight phase:

- **On the ground**: cutter fires on a still-bonded fitting; aluminum
  fitting deformed, possible hand injury, definite shipset write-off.
  Major property damage but no fatality if procedures followed.
- **In flight, pre-deploy** (wing stowed): one cutter firing on a
  stowed wing severs that root early; if the wing then deploys the
  asymmetric-deploy detector fires the other 3 cutters, jettison + reserve.
  Catastrophic only if the firing event happens at altitude where
  reserve event is not survivable.
- **In flight, post-deploy** (wing flying): one cutter firing severs
  one spar at the root; immediate roll departure; FCS detects roll
  rate divergence and fires the other 3 cutters → jettison + reserve.
  Catastrophic if cutters fail or pilot below reserve altitude.

## Detection

Pre-fire (the only useful detection — once it's fired, the cutter is
done):

- **Initiator continuity monitoring** — FCS checks the firing-circuit
  loop continuity at 10 Hz. A short or unexplained current flow flags
  inadvertent-fire risk; the system commands jettison-inhibit and pilot
  manual abort.
- **Lockout state monitoring** — the mechanical lockout pin has a
  microswitch that reports "engaged" or "removed". Should be engaged
  in any pre-flight state and removed only as part of the deliberate
  arm sequence.

## Mitigation chain

The architecture has **three independent layers** that must all be
defeated for an inadvertent fire to occur:

1. **Mechanical lockout pin.** A physical pin blocks the LSC from
   propagating into the spar even if the initiator fires. The pin is
   removed manually as the *first* step of the arm sequence (pilot or
   rigger physically pulls a lanyard). The lockout pin engaged
   condition is monitored by a microswitch.

2. **Electrical arm relay.** The firing-circuit power is routed
   through a relay that is OPEN by default. The relay closes only
   when the FCS state machine transitions out of the pre-deploy
   states (i.e. the system is committed to deployment). Loss of FCS
   power = relay opens = no firing power.

3. **Two-stage fire command.** The actual fire command requires TWO
   simultaneous signals on independent buses: a timing pulse from
   FCS-A and a permission gate from FCS-B (or vice versa). A single-
   FCS software bug cannot fire the cutter alone.

In addition:

4. **EMI hardening per RTCA DO-160 G H.** Firing-circuit shielding,
   filtered penetrations, twisted-pair leads with shield drain.
   Tested to 200 V/m HIRF per Category L (severe environment).

5. **No-fire / all-fire margins.** B-basis-tested LSC + initiator
   such that the no-fire current (1 W / 1 A for 5 minutes) is well
   below any conceivable EMI-induced current at the initiator pad.

6. **Ground-handling protocols.** No live initiators installed in the
   cutters until the aircraft is on the dispatch line. Initiators
   ship in static-shielded packaging and are installed by a qualified
   rigger as part of the pre-flight sequence.

7. **Pre-flight built-in test** of the firing-circuit continuity and
   the lockout / arm chain before initiators are installed.

## Residual risk

After all layers:

- **Common-mode hardware fault** (latch-up that closes both the arm
  relay AND the dual-fire-command gates simultaneously): ~10⁻⁸ per
  flight. Mitigated by independent relay technology (one
  electromechanical, one solid-state) so the same fault doesn't take
  both.
- **Software bug that fires both gates at the wrong state**:
  ~10⁻⁷ per flight. Mitigated by code review, scenario-based testing,
  and the fact that FCS-A and FCS-B run independently-developed
  logic at the gate level.
- **Lightning strike or extreme HIRF event**: not credible at
  skydiving altitudes; for high-altitude deployment scenarios
  outside MANTA v1, a separate lightning analysis is required.
- **Mishandling**: residual is procedural-discipline, not hardware.
  Pre-flight checklist + dual rigger sign-off is the mitigation;
  residual ~10⁻⁵ per flight assumes properly trained operators.

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| LSC + initiator no-fire/all-fire B-basis | Firing thresholds well-separated from EMI environment | Vendor lab |
| EMI immunity test (DO-160 G H) | No fire under 200 V/m HIRF, lightning-induced transients, ESD | EMI test lab |
| Lockout cycle test (×100) | Pin engages/disengages reliably; microswitch reports correctly | Bench |
| Arm-relay independence test | Relay opens when FCS-A or FCS-B is unpowered; remains open when both unpowered | Bench |
| Two-stage fire-command logic | Single-source fire command does NOT fire the cutter | Bench HIL |
| Software regression suite | Scenario tests prove no software-side path issues fire on pre-deploy states | CI / SITL |
| Ground-handling drop test | Cutter assembly survives 1.5 m drop onto concrete with no inadvertent fire | Bench |
| Operations sign-off | Pre-flight checklist signed by rigger AND pilot before initiators installed | Operational doc |

## Open issues

- Initiator vendor selection. Need NSI or equivalent with documented
  ESD insensitivity (5 kV / 500 pF) and a clear no-fire margin.
- Lockout pin geometry — must NOT be defeatable by vibration alone but
  must be removable by hand without tools.
- Two-stage fire-command implementation: the timing window between
  FCS-A pulse and FCS-B gate is a tradeoff between false-firing
  protection (longer = better protection, more chance of mismatch)
  and intentional-fire latency. Default 5 ms; tune from bench data.
- Static ground prior to initiator install — every aircraft needs a
  documented static-ground point and a wrist strap protocol for
  riggers handling initiators.
