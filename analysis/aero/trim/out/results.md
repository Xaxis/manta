# Trim + washout iteration — MANTA

Section: MH-78-class (analytic), a0 = 5.700/rad, α₀ = -1.00°, α_stall = 11.56°, Cm0 = 0.0050

## Trim at design CL = 0.5

| Washout | α_trim | Cm_apex_trim | x_cg_trim (m) | x_NP (m) | SM (% MAC) | α_tip_eff (°) | Tip stall margin (°) |
|---|---|---|---|---|---|---|---|
|  3.0°   |  6.90° | -0.4507     |  1.0858    |  1.1184 |    2.71   |    +1.88    |      9.69        |
|  4.0°   |  7.28° | -0.4462     |  1.0749    |  1.1184 |    3.61   |    +1.75    |      9.81        |
|  5.0°   |  7.66° | -0.4417     |  1.0641    |  1.1184 |    4.51   |    +1.63    |      9.93        |
|  6.0°   |  8.04° | -0.4371     |  1.0532    |  1.1184 |    5.42   |    +1.51    |     10.05        |
|  7.0°   |  8.42° | -0.4326     |  1.0423    |  1.1184 |    6.32   |    +1.39    |     10.18        |

## Trim sensitivity at washout = 5° across design CL (cruise vs slow vs fast)

| Design CL | α_trim | x_cg_trim | SM (% MAC) | α_tip_eff |
|---|---|---|---|---|
| 0.30      |  4.96° | 1.0278    |    7.52   |   +0.73   |
| 0.40      |  6.31° | 1.0505    |    5.64   |   +1.18   |
| 0.50      |  7.66° | 1.0641    |    4.51   |   +1.63   |
| 0.60      |  9.01° | 1.0731    |    3.76   |   +2.08   |
| 0.70      | 10.36° | 1.0796    |    3.22   |   +2.53   |

## Recommended washout

Pick: **washout = 7.0°**.
  Static margin: 6.32 % MAC
  x_CG (aft of apex): 1.0423 m  (= 0.865·MAC)
  α_trim: 8.42°,  α_tip_eff: +1.39°,  margin to stall: 10.18°
