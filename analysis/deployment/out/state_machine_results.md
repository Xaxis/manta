# Deployment state machine — nominal scenario

Final state: GLIDE
Abort cause: NONE
Drogue extract t : 2.005
Wing deploy cmd  : 4.6000000000000005
Wing locked t    : 5.05
Glide acquired   : 5.055

State trace:
  t= 0.000s  →  ARMED   (pilot arm)
  t= 0.700s  →  FREEFALL   (freefall dwell met)
  t= 2.000s  →  STABILIZE   (stable freefall)
  t= 2.005s  →  DROGUE_INFLATING   (drogue extract cmd)
  t= 4.595s  →  DROGUE_STABLE   (drogue stable)
  t= 4.600s  →  WING_DEPLOY   (wing deploy cmd)
  t= 5.050s  →  WING_TRIM_ACQUIRE   (wing locked, Δt = 5.00 ms)
  t= 5.055s  →  GLIDE   (trimmed glide)
