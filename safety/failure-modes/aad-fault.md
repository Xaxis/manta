# Failure mode: AAD fault

**FMEA ID:** `FM-AAD-001`
**Severity:** Hazardous (loss of last-resort altitude-triggered abort)
**Pre-mitigation likelihood:** ~10⁻⁵ per flight (skydiving baseline; production AAD with current battery)

## What it means

The Automatic Activation Device (AAD) is the FCS-bypassed altitude-and-
descent-rate-triggered abort. On MANTA it has TWO outputs (per
[`docs/05-emergency-systems.md`](../../docs/05-emergency-systems.md)):

1. **Reserve cutter (standard skydiving function)** — fires the
   reserve container's existing cutter if the pilot is below threshold
   altitude AND descending faster than threshold rate.
2. **MANTA wing-cutter trigger** — auxiliary GPIO output that fires
   the wing's pyrotechnic cutters BEFORE the reserve, with a programmed
   100 ms interlock so the wing departs before the reserve canopy
   inflates.

AAD fault means one or both outputs do not fire when they should, OR
fire when they shouldn't.

| Sub-case | Consequence |
|---|---|
| AAD doesn't fire when it should (true emergency) | Pilot impacts terrain at high V — fatal |
| AAD fires when it shouldn't (no emergency) | Surprise reserve deployment in normal flight; pilot must cutaway main and land under reserve |
| Standard reserve cutter fires but MANTA wing trigger doesn't | Reserve deploys with wing still attached — fouling risk |
| MANTA trigger fires but standard reserve doesn't | Wing departs but no reserve — pilot manual reserve required (slow) |

## Detection

- **Pre-flight built-in test** — AAD reports armed status, battery
  voltage, last-pressure-zero, expected firing thresholds. Aircraft
  does NOT fly with a failed AAD self-test.
- **In-flight monitoring** — AAD has an internal supervisor; produces
  a heartbeat on a serial line that the FCS observes. Loss of
  heartbeat → cockpit advisory, but flight continues (AAD is a
  backup, not primary).
- **Post-flight log review** — AAD records altitude/V profile of every
  jump; review confirms it would have fired in any close-call event.

## Mitigation chain

1. **Production-grade AAD with current cert.** Cypres 2 / Vigil 2 / M2
   are fielded skydiving AADs with multi-decade reliability records
   (~10⁻⁶ to 10⁻⁵ per-flight failure-to-fire rate when properly
   maintained).
2. **Independence from FCS.** AAD has its own battery, its own pressure
   sensor, its own logic. FCS-A and FCS-B can both be dead and the AAD
   still functions for the standard reserve fire.
3. **Wing-cutter trigger output isolation.** AAD's GPIO drives the wing
   cutter firing circuit through an independent power rail (separate
   from FCS-A and FCS-B rails), so a power-system fault that takes both
   FCS units does not also take the AAD trigger path.
4. **Two-path firing of the wing cutters** — even if AAD's trigger
   output fails, FCS-A and FCS-B both have independent fire-cutter
   paths. AAD path is the *third* independent route, used when the
   FCS units are themselves unable to issue the command.
5. **Programmed wing-then-reserve interlock.** AAD fires its standard
   reserve cutter 100 ms AFTER the wing-cutter trigger. If the wing
   cutter trigger somehow fails but the reserve cutter still fires,
   the reserve canopy inflates with the wing still attached — the
   reserve-compatibility analysis ([`safety/reserve-compat.md`](../reserve-compat.md))
   has a residual-risk entry for this.
6. **Pre-flight inspection requirement.** Rigger or pilot verifies AAD
   is armed, battery date current, last-pressure-zero done within 24 h
   (or per AAD manufacturer spec). Failed inspection = no fly.

## Residual risk

After mitigation:

- **AAD complete failure to fire**: ~10⁻⁵ per flight (skydiving
  baseline; FCS-driven abort paths catch most credible MANTA
  emergencies, so AAD's role is genuinely "last resort"). For MANTA
  specifically, the FCS ABORT paths (`docs/03`) cover ~99 % of
  credible failure modes; AAD adds another order of magnitude of
  defense for the remaining ~1 %.
- **AAD inadvertent fire**: ~10⁻⁶ per flight. Same as standard
  skydiving — the failure mode is not specific to MANTA. Recovery is
  a normal cutaway-and-reserve descent.
- **AAD fires standard reserve but not MANTA wing trigger**: ~10⁻⁶
  per flight. This is the binding case for the reserve-compatibility
  analysis — reserve canopy attempting to inflate with wing still
  attached. Residual catastrophic risk requires that the wing
  jettison stub geometry be clear of the reserve cone (verified in
  `cad/harness/build.py` clearance check).

## Verification gates

| Gate | What it shows | Where |
|---|---|---|
| AAD vendor cert | AAD passes manufacturer reliability and field-failure data | Vendor documentation |
| AAD-to-cutter wiring loop test | GPIO output drives the wing cutter firing circuit at the required current | Bench |
| AAD power-rail independence | AAD-bypass path operates with FCS-A and FCS-B both unpowered | Bench |
| AAD pre-flight self-test | Armed-status, battery, pressure-zero readouts agree with manufacturer expectations | Pre-flight checklist |
| AAD timing interlock | 100 ms delay between wing-cutter trigger and standard reserve cutter is correctly programmed and verified | Bench |
| Drop article AAD-trigger scenario | Forced AAD trigger fires wing cutters, then reserve, in the right order | [`test/drop/`](../../test/drop/) |

## Open issues

- AAD vendor selection. Down-select from Cypres / Vigil / M2 based on
  GPIO output capability (the auxiliary output for the wing cutter
  trigger), serial heartbeat support, and operating-temperature
  envelope match to MANTA's −10 °C floor.
- Custom firmware load on the AAD if the vendor doesn't ship the
  required interlock timing — most production AADs allow some
  programmability but the interface is vendor-proprietary.
- AAD altitude-threshold setting. Standard skydiving AADs fire at
  ~225 m AGL. With MANTA's 100 ms wing-then-reserve interlock plus
  drogue / wing-clear time, the practical altitude floor for safe
  AAD operation is ~325 m AGL. Recommend setting AAD threshold to
  match this.
