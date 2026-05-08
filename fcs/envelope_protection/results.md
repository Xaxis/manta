# Alpha limiter — V vs limit

|  V (m/s) |  α_limit (deg)  |  α_limit_degraded (deg)  |
|---|---|---|
|   10.0  |    7.86  |    6.36  |
|   12.0  |    8.43  |    6.93  |
|   14.0  |    9.00  |    7.50  |
|   16.0  |    9.00  |    7.50  |
|   18.0  |    9.00  |    7.50  |
|   20.0  |    9.00  |    7.50  |
|   22.0  |    9.00  |    7.50  |
|   25.0  |    9.00  |    7.50  |
|   30.0  |    9.00  |    7.50  |
|   35.0  |    9.50  |    8.00  |
|   40.0  |   10.00  |    8.50  |

# Demo step responses
  Pilot below limit               →  cmd=4.00°  limit=9.00°  saturated=False  degraded=False
  Pilot at limit                  →  cmd=9.00°  limit=9.00°  saturated=True  degraded=False
  Pilot above limit, slow         →  cmd=9.00°  limit=9.00°  saturated=True  degraded=False
  Sensor dropout                  →  cmd=7.50°  limit=7.50°  saturated=True  degraded=True
