# Spar bending — MANTA

Design pilot mass         : 95.0 kg
Wing + rig mass           : 23.5 kg
Total mass for sizing     : 118.5 kg
Material                  : T800S/3900-2 UD CFRP tube (~85% axial)
σ_design_compression_limit: 403.3 MPa
Front-spar load fraction  : 0.889 (rear = 0.111)

## Stress & safety factors at the root station

| Load case      | Spar | M_root (N·m) | σ_root (MPa) | SF_strength | M_Brazier (N·m) | SF_buckling |
|---|---|---|---|---|---|---|
| 1g cruise      | front |    830.5    |    384.3     |  1.05     |  91227.2      | 109.85     |
| 1g cruise      | rear  |    103.8    |     89.9     |  4.49     |  51315.3      | 494.33     |
| 3g limit       | front |   2491.4    |   1153.0     |  0.35     |  91227.2      | 36.62     |
| 3g limit       | rear  |    311.4    |    269.6     |  1.50     |  51315.3      | 164.78     |
| 4.5g ultimate  | front |   3737.1    |   1729.5     |  0.23     |  91227.2      | 24.41     |
| 4.5g ultimate  | rear  |    467.1    |    404.4     |  1.00     |  51315.3      | 109.85     |

## Wall-thickness sensitivity (front spar, 3g limit, root stress)
(BRIEF OD progression 40 / 32 / 25 mm — only wall varied)

| Wall (mm) | Front mass (kg/side) | σ_root (MPa) | SF_strength | SF_buckling |
|---|---|---|---|---|
|   1.5    |   0.9639      |   1480.1      |  0.27      |  27.46     |
|   2.0    |   1.2394      |   1153.0      |  0.35      |  36.62     |
|   2.5    |   1.5053      |   958.2      |  0.42      |  45.77     |
|   3.0    |   1.7617      |   829.5      |  0.49      |  54.93     |

## Front-spar OD sensitivity at wall = 2.5 mm, 3g limit, root stress
(stage progression OD_root / OD_root·0.7 / 25 mm)

| OD_root (mm) | Front mass (kg/side) | σ_root (MPa) | SF_strength |
|---|---|---|---|
|     40      |    1.4416        |    958.2     |   0.42     |
|     50      |    1.7123        |    590.3     |   0.68     |
|     60      |    1.9831        |    399.7     |   1.01     |
|     65      |    2.1184        |    337.3     |   1.20     |
|     70      |    2.2538        |    288.4     |   1.40     |
|     80      |    2.5245        |    217.8     |   1.85     |

## Recommended sizing (target SF_strength ≥ 1.5 at 3g limit, wall = 2.5 mm)

  Smallest OD_root meeting SF ≥ 1.5: **73 mm** 
  Stage progression: 73 / 51 / 25 mm, wall 2.5 mm
  Front-spar mass per side: 2.335 kg  (BRIEF default 1.239 kg, Δ = +1.096 kg)
  σ_root at 3g: 264.0 MPa, SF_strength = 1.53
