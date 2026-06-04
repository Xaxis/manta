# Bistable rib deployment — reduced-order physics

Strain-energy-driven unroll of a bistable rolled-composite rib (thin-ply HSC carbon, lenticular section). Backs the `rib_unroll` geometry in `sim/build.py`.

| Quantity | Value |
|---|---|
| Shell wall t | 0.14 mm |
| Coil radius r_coil | 11.0 mm |
| Coil strain ε = t/2R | 0.64 % (< ~1 % HSC allowable) |
| Shell rigidity D | 18.09 mN·m |
| Propagation moment M\* | 2.14 mN·m |
| Driving force F_drive = M\*/r | 195 mN |
| Rib chord (representative) | 1.00 m |
| Rib mass ρ·L | 22 g |
| **Snap time t_snap (s→0.99)** | **692 ms** |
| Peak deploy velocity | 1.98 m/s |
| **Latch-contact velocity** | **1.96 m/s** (soft) |
| End-latch shock | 27 N |
| Passive friction hold / drive | 0.13× (insufficient) |

**Deployment.** Each ~1.0 m rib snaps from packed coil to latched airfoil in ~692 ms, driven by the constant Seffen–Pellegrino propagation force F = M\*/r_coil = 195 mN. The 9 ribs/side fire on a root→tip stagger, so the unfurl front sweeps outboard across Phase C — this is the schedule the `rib_unroll` animation renders (`RIB_STAGGER`, `RIB_SNAP_DUR`).

**Soft latch.** A rotary viscous damper on the hub bleeds the stroke so the rib reaches its end-stop at only 1.96 m/s, keeping the deployment-shock load to ~27 N — well under the 0.2 N the latched root carries — so there is no destructive snap.

**Blossoming guard (design driver).** Inter-layer friction alone holds only 0.13× the steady tip force, i.e. it is **insufficient**: a free friction coil WOULD blossom (unwind loosely inside the deployer). That is exactly why the hub is a **rate-controlled spool** (the same viscous damper): it meters payout so the coil feeds the front under tension instead of blooming. Bistable end-detents then hold both the stowed and the deployed states without a restraint band.