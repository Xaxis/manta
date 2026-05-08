# MANTA longitudinal dynamics

Trim:  V = 16.00 m/s,  CL = 0.782,  CD = 0.0650
       α_trim = 11.74°,  CL_α = 4.285/rad
       static margin = 3.45 % MAC
       Iyy = 25.0 kg·m², MAC = 1.205 m

State-space A:
[[-0.102  -0.27   -9.8066  0.    ]
 [-1.2258 -3.3598 16.      0.    ]
 [ 0.     -0.5871 -8.3621  0.    ]
 [ 0.      0.      1.      0.    ]]

Eigenvalues + modes:
  oscillatory  λ = -5.7984+1.4483jj   ω_n = 5.976 rad/s (0.951 Hz)   ζ = 0.970   period = 4.34 s
  real         λ = +0.0000+0.0000j    τ = +inf s
  real         λ = -0.2271+0.0000j    τ = +4.403 s

## CG offset sweep — pilot head/torso shift

| Δx_CG (m) | SM (% MAC) | short-period ω_n (rad/s) | ζ_sp | phugoid ω_n | ζ_ph | Status |
|---|---|---|---|---|---|---|
| -0.050 | -0.70 | n/a | n/a | n/a | n/a | UNSTABLE |
| -0.030 | +0.96 | n/a | n/a | n/a | n/a | UNSTABLE |
| -0.020 | +1.79 | n/a | n/a | n/a | n/a | UNSTABLE |
| -0.010 | +2.62 | 5.829 | +0.999 | n/a | n/a | STABLE |
| +0.000 | +3.45 | 5.976 | +0.970 | n/a | n/a | STABLE |
| +0.010 | +4.28 | 6.125 | +0.943 | n/a | n/a | STABLE |
| +0.020 | +5.11 | 6.274 | +0.918 | n/a | n/a | STABLE |
| +0.030 | +5.94 | 6.423 | +0.894 | n/a | n/a | STABLE |
| +0.050 | +7.60 | 6.719 | +0.851 | n/a | n/a | STABLE |

Pilot CG perturbation context:
  Pilot mass fraction of total: 78.6%
  ±50 mm shift of upper-body CG → ±39.29 mm shift of vehicle CG
  → ±3.26 % of MAC
