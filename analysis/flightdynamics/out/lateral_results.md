# MANTA lateral-directional flight dynamics

## Trim at V = 16.0 m/s,  CL = 0.782

Stability derivatives (rad⁻¹ unless noted):
  Cl_β  = -0.1349   (dihedral effect; <0 stable, sweep contrib)
  Cl_p  = -0.5420   (roll damping; <0 stable)
  Cl_r  = +0.2292   (roll-from-yaw)
  Cn_β  = +0.0200   (yaw stiffness; >0 stable)  ⚠ tailless
  Cn_p  = -0.0977   (yaw-from-roll)
  Cn_r  = -0.0184   (yaw damping; <0 stable)  ⚠ tailless
  CY_β  = -0.0500   (side-force; <0 stable)

Modes:
  Dutch roll:  λ = -0.6930+2.2924jj   ω_n = 2.395 rad/s (0.381 Hz)   ζ = +0.289   T = 2.74 s
               handling-quality: Level 2 (acceptable)
  Roll mode:   λ = -15.6544+0.0000j     τ = +0.064 s
  Spiral mode: λ = +0.0459+0.0000j     τ = -21.810 s  (DIVERGENT)
               time-to-double = 15.1 s

## Trim at V = 20.0 m/s,  CL = 0.500

Stability derivatives (rad⁻¹ unless noted):
  Cl_β  = -0.0864   (dihedral effect; <0 stable, sweep contrib)
  Cl_p  = -0.5420   (roll damping; <0 stable)
  Cl_r  = +0.1287   (roll-from-yaw)
  Cn_β  = +0.0200   (yaw stiffness; >0 stable)  ⚠ tailless
  Cn_p  = -0.0625   (yaw-from-roll)
  Cn_r  = -0.0262   (yaw damping; <0 stable)  ⚠ tailless
  CY_β  = -0.0500   (side-force; <0 stable)

Modes:
  Dutch roll:  λ = -0.6080+2.2650jj   ω_n = 2.345 rad/s (0.373 Hz)   ζ = +0.259   T = 2.77 s
               handling-quality: Level 2 (acceptable)
  Roll mode:   λ = -20.2715+0.0000j     τ = +0.049 s
  Spiral mode: λ = +0.0086+0.0000j     τ = -115.961 s  (DIVERGENT)
               time-to-double = 80.4 s

## Trim at V = 25.0 m/s,  CL = 0.320

Stability derivatives (rad⁻¹ unless noted):
  Cl_β  = -0.0553   (dihedral effect; <0 stable, sweep contrib)
  Cl_p  = -0.5420   (roll damping; <0 stable)
  Cl_r  = +0.0644   (roll-from-yaw)
  Cn_β  = +0.0200   (yaw stiffness; >0 stable)  ⚠ tailless
  Cn_p  = -0.0400   (yaw-from-roll)
  Cn_r  = -0.0311   (yaw damping; <0 stable)  ⚠ tailless
  CY_β  = -0.0500   (side-force; <0 stable)

Modes:
  Dutch roll:  λ = -0.6814+2.4627jj   ω_n = 2.555 rad/s (0.407 Hz)   ζ = +0.267   T = 2.55 s
               handling-quality: Level 2 (acceptable)
  Roll mode:   λ = -25.6620+0.0000j     τ = +0.039 s
  Spiral mode: λ = -0.0124+0.0000j     τ = +80.824 s  (stable (convergent))

