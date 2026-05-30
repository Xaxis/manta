"""
MANTA — full mechanical build + aero-flow pipeline: MuJoCo deploy schedule ->
a detailed, fully-articulated GLB (morph-animated deployment) + a baked aero
field (surface pressure + streamlines) + Blender hero renders.

This version builds the resized planform (BRIEF #5, finding #5: S=6.5 m^2,
b=6.3 m, AR 6.1, 25 deg LE sweep, taper 0.4, 6 deg washout — downsized from
8.4/7.4 since MANTA lands under reserve) as a DOUBLE-SURFACE cambered wing
tip-to-tip (upper skin + lower wingsuit), continuous across the body and the
region between the legs, like a rigid wing or paraglider canopy.  The pilot is
the fuselage, centred in the airfoil thickness, with the spars inside the fabric
and the legs forming the rear tail.  A control-demo animation drives the
flaperons + pilot weight-shift from the real stability/control derivatives.
Earlier notes below describe the build.  The pilot is embedded under the
translucent tensioned skin, with arms running along the leading-edge spar and
legs along the trailing-edge spar (BRIEF #2).

Pieces modelled (per BRIEF.md), each as a named material slot:
  suit      pilot / harness garment body
  cfrp      spine yoke (box beam), shoulder + hip pivot hubs, LE spar (along
            the arm out to the wrist boom), TE spar (along the leg out to the
            ankle boom), 3-stage telescoping tip booms
  skin      cambered NACA-4412 wing surface (camber + thickness + washout +
            billow scallop between ribs -- the paraglider/ram look)
  rib       9 bistable airfoil-profile ribs per side
  reserve   reserve container on the back, above the spine yoke
  fcs       flight-control bay on the spine
  flaperon  trailing-edge flaperons (deflect down on deploy)
  metal     CO2 canisters + tip-hub fittings

Aero field (built once from the deployed frame, as two SEPARATE static objects
so they never contaminate the morph mesh materials):
  PRESSURE  the wing surface coloured by surface pressure coefficient
  FLOW      streamlines (upwash -> suction acceleration -> downwash)
Both are derived from the real Weissinger span-loading
(analysis/aero/weissinger/out/span_loading.csv) + a thin-airfoil Cp model.
This is an ILLUSTRATIVE field for intuition, NOT a CFD solution -- SU2/OpenFOAM
is the rigorous follow-up per the BRIEF tool stack.

Deployment is ONE mesh with FIXED topology on every frame; the 60 frames are
morph targets (shape keys) -> a single continuously-interpolated animation.
The deploy schedule (phase A spread, phase B tip-telescope, phase C rib snap)
is integrated in MuJoCo (run_simulation) and drives the wing's open fraction.

Run:  PYTHONPATH=. .venv/bin/python sim/build.py            (GLB only)
      PYTHONPATH=. .venv/bin/python sim/build.py --render   (+ hero stills)
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bpy
import mujoco

_HERE = Path(__file__).parent
OUT_DIR = _HERE / "out"
OUT_DIR.mkdir(parents=True, exist_ok=True)
SITE_MODELS = _HERE.parent / "site" / "public" / "models" / "v3"
SPAN_LOADING_CSV = (_HERE.parent / "analysis" / "aero" / "weissinger" /
                    "out" / "span_loading.csv")

# ---- Planform — resized to the moderate B option (6.3 m / 6.5 m^2, AR 6.1) --
# Downsized from the BRIEF v2 8.4 m^2 / 7.4 m because MANTA lands under reserve
# (low stall speed not required), so a smaller wing holds the glide target with
# a far more feasible ~2.4 m telescoping boom. Source of truth: geometry.py.
PLAN_S = 6.5
PLAN_B = 6.3
PLAN_TAPER = 0.4
PLAN_SWEEP_DEG = 25.0
PLAN_WASHOUT_DEG = 7.0          # raised 6->7 to recover static margin (finding #6)
HALF_SPAN_FULL = PLAN_B / 2.0                                  # 3.15 m
CHORD_ROOT = 2.0 * PLAN_S / (PLAN_B * (1.0 + PLAN_TAPER))      # 1.474 m
CHORD_TIP = PLAN_TAPER * CHORD_ROOT                            # 0.590 m
TAN_SWEEP = math.tan(math.radians(PLAN_SWEEP_DEG))
X_ROOT_LE = 0.95          # root leading-edge x (forward = +x)
Z_WING = 0.10             # wing reference plane height
AERO_ALPHA = 6.0          # nearest tabulated case to the settled glide alpha


# =============================================================================
# (1) MuJoCo deployment mechanism — embedded MJCF + kinematic run
# =============================================================================

MJCF_XML = """\
<mujoco model="manta_deployment">
  <option timestep="0.0005" gravity="0 0 0" integrator="RK4"/>
  <compiler angle="radian" coordinate="local"/>
  <default>
    <joint armature="0.001" damping="0.5" limited="true"/>
    <geom contype="0" conaffinity="0"/>
    <position kp="80" kv="6"/>
  </default>
  <worldbody>
    <body name="torso" pos="0 0 0">
      <inertial pos="0 0 0" mass="55" diaginertia="2.5 2.0 1.5"/>
      <body name="shoulder_R" pos="0.30 0.18 0.10">
        <inertial pos="0 0.15 0" mass="2.5" diaginertia="0.05 0.005 0.05"/>
        <joint name="sh_R_yaw" type="hinge" axis="0 0 1" range="-0.05 1.6" damping="2"/>
        <body name="wrist_R" pos="0 0.57 0">
          <inertial pos="0 0 0" mass="0.4" diaginertia="0.001 0.001 0.001"/>
          <body name="tip_R" pos="0 0 0.07">
            <joint name="tip_R_slide" type="slide" axis="0 1 0" range="0 2.30" damping="2"/>
            <inertial pos="0 -1.1 0" mass="0.9" diaginertia="0.03 0.001 0.03"/>
          </body>
        </body>
      </body>
      <body name="shoulder_L" pos="0.30 -0.18 0.10">
        <inertial pos="0 -0.15 0" mass="2.5" diaginertia="0.05 0.005 0.05"/>
        <joint name="sh_L_yaw" type="hinge" axis="0 0 1" range="-1.6 0.05" damping="2"/>
        <body name="wrist_L" pos="0 -0.57 0">
          <inertial pos="0 0 0" mass="0.4" diaginertia="0.001 0.001 0.001"/>
          <body name="tip_L" pos="0 0 0.07">
            <joint name="tip_L_slide" type="slide" axis="0 -1 0" range="0 2.30" damping="2"/>
            <inertial pos="0 1.1 0" mass="0.9" diaginertia="0.03 0.001 0.03"/>
          </body>
        </body>
      </body>
      <body name="hip_R" pos="-0.20 0.12 0">
        <inertial pos="-0.4 0 0" mass="9.0" diaginertia="0.10 0.008 0.10"/>
        <joint name="hip_R_yaw" type="hinge" axis="0 0 1" range="-0.6 0.05" damping="2"/>
      </body>
      <body name="hip_L" pos="-0.20 -0.12 0">
        <inertial pos="-0.4 0 0" mass="9.0" diaginertia="0.10 0.008 0.10"/>
        <joint name="hip_L_yaw" type="hinge" axis="0 0 1" range="-0.05 0.6" damping="2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="sh_R_yaw_act"  joint="sh_R_yaw"  kp="180" kv="14"/>
    <position name="sh_L_yaw_act"  joint="sh_L_yaw"  kp="180" kv="14"/>
    <position name="hip_R_yaw_act" joint="hip_R_yaw" kp="150" kv="12"/>
    <position name="hip_L_yaw_act" joint="hip_L_yaw" kp="150" kv="12"/>
    <position name="tip_R_act" joint="tip_R_slide" kp="900" kv="46"/>
    <position name="tip_L_act" joint="tip_L_slide" kp="900" kv="46"/>
  </actuator>
</mujoco>
"""

JOINT_STOWED = {"sh_R_yaw": 0.05, "sh_L_yaw": -0.05,
                "hip_R_yaw": -0.05, "hip_L_yaw": 0.05}
JOINT_DEPLOYED = {"sh_R_yaw": 0.62, "sh_L_yaw": -0.62,
                  "hip_R_yaw": -0.42, "hip_L_yaw": 0.42}

N_FRAMES = 40
DURATION_S = 0.6
# Overlapping deploy schedule (s) spanning the full duration so the span grows
# monotonically across all 60 frames (no mid-point velocity jump, no frozen
# tail).  spread (arms/legs) and tip (telescope) overlap; ribs snap, then the
# flaperons set last, completing exactly at the final frame.
SPREAD_DUR = 0.40
TIP_START, TIP_DUR = 0.20, 0.40       # completes at t = 0.60 (last frame)
RIB_START, RIB_DUR = 0.34, 0.24
FLAP_START, FLAP_DUR = 0.34, 0.26


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a * (1 - t) + b * t


def phase_progress(t):
    """Subsystem progress fractions in [0,1] at sim time t (phases overlap)."""
    return {
        "spread": smoothstep(t / SPREAD_DUR),
        "tip": smoothstep((t - TIP_START) / TIP_DUR),
        "rib": smoothstep((t - RIB_START) / RIB_DUR),
        "flap": smoothstep((t - FLAP_START) / FLAP_DUR),
    }


def run_simulation():
    """Integrate the deploy mechanism in MuJoCo; record per-frame state."""
    model = mujoco.MjModel.from_xml_string(MJCF_XML)
    data = mujoco.MjData(model)
    for name, q in JOINT_STOWED.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = q
    mujoco.mj_forward(model, data)
    aid = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
           for i in range(model.nu)}

    n_steps = int(DURATION_S / model.opt.timestep)
    record_every = max(1, n_steps // N_FRAMES)
    frames = []
    for step in range(n_steps):
        t = step * model.opt.timestep
        sp = smoothstep(t / SPREAD_DUR)
        for j in JOINT_STOWED:
            data.ctrl[aid[f"{j}_act"]] = lerp(JOINT_STOWED[j], JOINT_DEPLOYED[j], sp)
        tipp = smoothstep((t - TIP_START) / TIP_DUR)
        data.ctrl[aid["tip_R_act"]] = lerp(0.0, 2.30, tipp)
        data.ctrl[aid["tip_L_act"]] = lerp(0.0, 2.30, tipp)
        mujoco.mj_step(model, data)
        if step % record_every == 0 and len(frames) < N_FRAMES:
            frames.append({"t": float(t)})
    while len(frames) < N_FRAMES:
        frames.append(frames[-1])
    print(f"  sim: {len(frames)} frames over {DURATION_S} s")
    return {"duration_s": DURATION_S, "frames": frames}


# =============================================================================
# (1b) Flight control — control-basis poses + the flight model the viewer flies
# =============================================================================
# Instead of a canned maneuver, we bake four CONTROL-BASIS morph poses (extreme
# left/right roll, nose-up/down) as deltas from the deployed wing.  The viewer
# blends them by the live pilot input (so the flaperons deflect + the pilot
# weight-shifts in real time) and integrates the flight model below to fly the
# craft.  Each pose = deployed geometry + flaperon deflection + pilot weight-
# shift (the binding CG authority); roll uses differential flaperon, pitch uses
# symmetric flaperon + an aft/fwd shift.

CTRL_BASIS = {
    "rollL":  {"da": -1.0, "de":  0.0, "lean": (0.0, -0.05, 0.0)},
    "rollR":  {"da":  1.0, "de":  0.0, "lean": (0.0,  0.05, 0.0)},
    "pitchU": {"da":  0.0, "de":  1.0, "lean": (-0.06, 0.0, 0.0)},
    "pitchD": {"da":  0.0, "de": -1.0, "lean": (0.05, 0.0, 0.0)},
}

# Reduced-order flight model the viewer integrates (point-mass + bank-to-turn +
# short-period alpha).  Numbers come from the resized-planform aero
# (analysis/aero) and the stability/control work (analysis/flightdynamics,
# docs/04) — NOT invented.  This is what makes the craft actually flyable.
FLIGHT_MODEL = {
    "S": PLAN_S, "AR": PLAN_B * PLAN_B / PLAN_S,
    "CL_alpha": 4.17, "CD0": 0.034, "e": 0.95,     # /rad, -, span-eff
    "mass": 106.0, "g": 9.80665, "rho": 1.225,     # kg, m/s^2, kg/m^3
    "alpha0_deg": 1.5, "alpha_trim_deg": 8.5,      # zero-lift + trim alpha (7° washout)
    "alpha_limit_deg": 9.0, "alpha_stall_deg": 11.5,
    "CLmax": 1.1, "V_trim": 18.3,                  # m/s best-glide
    "roll_rate_max_dps": 110.0,                    # crisp roll (tau ~ 0.06 s)
    "bank_limit_deg": 60.0,
}


# =============================================================================
# (2) Vector helpers
# =============================================================================

def vadd(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vsub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vscale(a, s): return (a[0] * s, a[1] * s, a[2] * s)
def vdot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def vlerp(a, b, t): return (lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t))


def vlen(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def vnorm(a):
    L = vlen(a) or 1e-9
    return (a[0] / L, a[1] / L, a[2] / L)


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def resample_polyline(poly, n):
    """Resample a polyline to n points evenly by arc length."""
    segs = [0.0]
    for i in range(1, len(poly)):
        segs.append(segs[-1] + vlen(vsub(poly[i], poly[i - 1])))
    total = segs[-1] or 1e-9
    out = []
    for s in range(n):
        target = total * s / (n - 1)
        k = 0
        while k < len(segs) - 2 and segs[k + 1] < target:
            k += 1
        f = (target - segs[k]) / ((segs[k + 1] - segs[k]) or 1e-9)
        out.append(vlerp(poly[k], poly[k + 1], f))
    return out


# =============================================================================
# (3) Airfoil (NACA 4-digit) — cambered cross-section
# =============================================================================

def naca4_surface(xc, m, p, t):
    xc = min(max(xc, 0.0), 1.0)
    yt = 5 * t * (0.2969 * math.sqrt(xc) - 0.1260 * xc - 0.3516 * xc**2
                  + 0.2843 * xc**3 - 0.1015 * xc**4)
    if xc < p:
        yc = m / (p * p) * (2 * p * xc - xc * xc)
        dyc = 2 * m / (p * p) * (p - xc)
    else:
        yc = m / ((1 - p) ** 2) * ((1 - 2 * p) + 2 * p * xc - xc * xc)
        dyc = 2 * m / ((1 - p) ** 2) * (p - xc)
    theta = math.atan(dyc)
    return yc + yt * math.cos(theta), yc - yt * math.cos(theta)


AF_M, AF_P = 0.04, 0.40
AF_T_SKIN = 0.12
N_CHORD = 13


def airfoil_loop(n_chord, t):
    """Closed loop param points (xc, off, side): LE->TE upper, TE->LE lower."""
    xs = [0.5 * (1 - math.cos(math.pi * i / (n_chord - 1))) for i in range(n_chord)]
    loop = []
    for x in xs:
        up, _ = naca4_surface(x, AF_M, AF_P, t)
        loop.append((x, up, "u"))
    for x in reversed(xs[1:-1]):
        _, lo = naca4_surface(x, AF_M, AF_P, t)
        loop.append((x, lo, "l"))
    return loop


# =============================================================================
# (4) Planform geometry — the locked wing, parameterised by deploy progress
# =============================================================================

N_SPAN_HALF = 36           # stations per half (skin); >=4 samples per rib-bay
                           # scallop period (9 bays) so cells read crisply
N_RIBS = 9                 # bistable ribs per side
BILLOW = 0.14              # skin bulge between ribs (fraction of thickness)

# ---- Pilot anthropometry — FIXED bone lengths (NASA-STD-3000 / ANSUR II, m) -
# The human is a rigid-proportion body; it NEVER stretches.  Deployment rotates
# the limbs (fixed lengths) and telescopes the booms — it does not grow the
# pilot.  See memory: project_human_fixed_size_architecture.
SHOULDER_HALF = 0.20       # biacromial / 2
HIP_HALF = 0.145           # bi-iliac / 2
UPPER_ARM, FOREARM, HAND = 0.32, 0.27, 0.17
THIGH, SHANK, FOOT = 0.43, 0.42, 0.20
TORSO_LEN = 0.52           # acromion -> hip
NECK_LEN, HEAD_LEN = 0.10, 0.20   # head length ~0.20 m (50th-%ile male)
X_SHOULDER = 0.45          # shoulder x (mid-chord); arms angle fwd to the LE
Z_BODY = Z_WING + 0.03     # body centred in the airfoil thickness (belly ~ lower
                           # 'wingsuit' surface, back ~ upper skin; slight bulge
                           # both ways, since the torso is the fuselage)

ARM_REACH = UPPER_ARM + FOREARM       # shoulder -> wrist (fixed)
LEG_REACH = THIGH + SHANK             # hip -> ankle (fixed)

# deployed/stowed limb directions (unit, right side; left mirrors y).  Arms
# point forward+outboard to lie near the LE; legs point aft+outboard to form
# the rear/tail near the TE.
ARM_DIR_DEP = vnorm((0.33, 0.945, -0.02))
ARM_DIR_STOW = vnorm((0.55, 0.18, -0.55))
LEG_DIR_DEP = vnorm((-0.74, 0.63, -0.06))
LEG_DIR_STOW = vnorm((-0.92, 0.12, -0.30))

# span station the wrist/ankle reach when fully spread (fixed-length FK)
WRIST_Y_DEP = SHOULDER_HALF + ARM_REACH * ARM_DIR_DEP[1]
ANKLE_Y_DEP = HIP_HALF + LEG_REACH * LEG_DIR_DEP[1]
INBOARD_DEP = max(WRIST_Y_DEP, ANKLE_Y_DEP)


def _reach_y(d_stow, d_dep, reach, base_half, sp):
    """Spanwise |y| a fixed-length limb's tip reaches at spread fraction sp."""
    return base_half + reach * vnorm(vlerp(d_stow, d_dep, sp))[1]


def inboard_reach(prog):
    """The further of the wrist/ankle span stations from the fixed-length FK."""
    sp = prog["spread"]
    return max(_reach_y(ARM_DIR_STOW, ARM_DIR_DEP, ARM_REACH, SHOULDER_HALF, sp),
               _reach_y(LEG_DIR_STOW, LEG_DIR_DEP, LEG_REACH, HIP_HALF, sp))


def half_span(prog):
    """Half span: the FIXED-length limbs spread to the inboard reach (Phase A),
    then the telescoping booms cover the remainder to the tip (Phase B). The
    pilot never stretches; the booms provide the span beyond the wrist/ankle so
    the membrane edge always tracks the actual limb reach."""
    return inboard_reach(prog) + (HALF_SPAN_FULL - INBOARD_DEP) * prog["tip"]


def chord_scale(prog):
    """Chord opens fore-aft as the pilot spreads (phase A)."""
    return 0.42 + 0.58 * prog["spread"]


def planform_chord(ay_full):
    eta = min(ay_full / HALF_SPAN_FULL, 1.0)
    return CHORD_ROOT - (CHORD_ROOT - CHORD_TIP) * eta


def le_point(y):
    return (X_ROOT_LE - abs(y) * TAN_SWEEP, y, Z_WING)


def chord_at(y, prog):
    return planform_chord(abs(y)) * chord_scale(prog)


def te_point(y, prog):
    le = le_point(y)
    return (le[0] - chord_at(y, prog), y, Z_WING)


def _rot_y(point, pivot, ang):
    dx, dz = point[0] - pivot[0], point[2] - pivot[2]
    c, s = math.cos(ang), math.sin(ang)
    return (pivot[0] + dx * c + dz * s, point[1], pivot[2] - dx * s + dz * c)


def wing_section_points(y, prog, loop, t_scale, billow):
    """3D airfoil-loop points for the wing station at spanwise y, plus per-point
    (eta, xc, side) metadata."""
    le = le_point(y)
    c = chord_at(y, prog)
    ay = abs(y)
    eta = min(ay / HALF_SPAN_FULL, 1.0)
    twist = -math.radians(PLAN_WASHOUT_DEG) * eta            # nose-down at tip
    qc = (le[0] - 0.25 * c, y, Z_WING)
    # Billow: skin sags into catenary bulges BETWEEN ribs and pinches AT each
    # rib.  Ribs sit at eta = (k+0.5)/N_RIBS, so cos(pi*eta*N_RIBS)=0 there
    # (pinch) and = +/-1 at the bay centres (max bulge).
    scallop = 1.0 + billow * math.cos(math.pi * eta * N_RIBS) ** 2
    pts, meta = [], []
    for (xc, off, side) in loop:
        px = le[0] - xc * c
        pz = Z_WING + off * c * t_scale / AF_T_SKIN * scallop
        p = _rot_y((px, y, pz), qc, twist)
        pts.append(p)
        meta.append((eta, xc, side))
    return pts, meta


# =============================================================================
# (5) Mesh accumulator — fixed-topology procedural geometry
# =============================================================================

RING_SEG = 10
SPHERE_RINGS = 6
SPHERE_SEG = 10


def _orthoframe(ax):
    world_up = (0.0, 0.0, 1.0)
    if abs(vdot(world_up, ax)) > 0.97:
        world_up = (1.0, 0.0, 0.0)
    up = vnorm(vsub(world_up, vscale(ax, vdot(world_up, ax))))
    side = vnorm(vcross(ax, up))
    return up, side


class Mesh:
    def __init__(self):
        self.verts = []

    def _ring(self, center, n1, n2, r):
        base = len(self.verts)
        for k in range(RING_SEG):
            a = 2 * math.pi * k / RING_SEG
            c, s = math.cos(a), math.sin(a)
            self.verts.append((center[0] + r * (c * n1[0] + s * n2[0]),
                               center[1] + r * (c * n1[1] + s * n2[1]),
                               center[2] + r * (c * n1[2] + s * n2[2])))
        return base

    def tube(self, p0, p1, r0, r1, faces=None):
        ax = vnorm(vsub(p1, p0))
        n1, n2 = _orthoframe(ax)
        b0 = self._ring(p0, n1, n2, r0)
        b1 = self._ring(p1, n1, n2, r1)
        c0 = len(self.verts); self.verts.append(tuple(p0))
        c1 = len(self.verts); self.verts.append(tuple(p1))
        if faces is not None:
            for k in range(RING_SEG):
                kn = (k + 1) % RING_SEG
                faces.append((b0 + k, b0 + kn, b1 + kn, b1 + k))
                faces.append((c0, b0 + kn, b0 + k))
                faces.append((c1, b1 + k, b1 + kn))

    def sphere(self, center, r, faces=None):
        base = len(self.verts)
        for i in range(SPHERE_RINGS + 1):
            phi = math.pi * i / SPHERE_RINGS
            z = math.cos(phi); rr = math.sin(phi)
            for j in range(SPHERE_SEG):
                th = 2 * math.pi * j / SPHERE_SEG
                self.verts.append((center[0] + r * rr * math.cos(th),
                                   center[1] + r * rr * math.sin(th),
                                   center[2] + r * z))
        if faces is not None:
            for i in range(SPHERE_RINGS):
                for j in range(SPHERE_SEG):
                    jn = (j + 1) % SPHERE_SEG
                    a = base + i * SPHERE_SEG + j
                    b = base + i * SPHERE_SEG + jn
                    c = base + (i + 1) * SPHERE_SEG + jn
                    d = base + (i + 1) * SPHERE_SEG + j
                    faces.append((a, b, c, d))

    def box(self, center, half, faces=None, axis=None):
        ex, ey, ez = axis if axis else ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        base = len(self.verts)
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    off = vadd(vadd(vscale(ex, sx * half[0]),
                                    vscale(ey, sy * half[1])),
                               vscale(ez, sz * half[2]))
                    self.verts.append(vadd(center, off))
        if faces is not None:
            def idx(sx, sy, sz):
                return base + ((sx > 0) * 4 + (sy > 0) * 2 + (sz > 0))
            faces.extend([
                (idx(-1,-1,-1), idx(-1,1,-1), idx(-1,1,1), idx(-1,-1,1)),
                (idx(1,-1,-1), idx(1,-1,1), idx(1,1,1), idx(1,1,-1)),
                (idx(-1,-1,-1), idx(-1,-1,1), idx(1,-1,1), idx(1,-1,-1)),
                (idx(-1,1,-1), idx(1,1,-1), idx(1,1,1), idx(-1,1,1)),
                (idx(-1,-1,-1), idx(1,-1,-1), idx(1,1,-1), idx(-1,1,-1)),
                (idx(-1,-1,1), idx(-1,1,1), idx(1,1,1), idx(1,-1,1)),
            ])

    def _ellipse_ring(self, center, up, side, ry, rz):
        base = len(self.verts)
        for k in range(RING_SEG):
            a = 2 * math.pi * k / RING_SEG
            c, s = math.cos(a), math.sin(a)
            self.verts.append(vadd(center, vadd(vscale(side, c * ry),
                                                vscale(up, s * rz))))
        return base

    def loft(self, centers, radii, faces=None):
        """Loft elliptical cross-sections (ry along width, rz along depth) along
        a centerline. Used for the anatomical pilot body (broad-shouldered,
        prone torso + tapered limbs). Cap verts are always appended so the
        vertex count is identical on every morph frame."""
        n = len(centers)
        rings = []
        for i in range(n):
            ax = vnorm(vsub(centers[min(i + 1, n - 1)], centers[max(i - 1, 0)]))
            up, side = _orthoframe(ax)
            ry, rz = radii[i]
            rings.append(self._ellipse_ring(centers[i], up, side, ry, rz))
        c0 = len(self.verts); self.verts.append(tuple(centers[0]))
        c1 = len(self.verts); self.verts.append(tuple(centers[-1]))
        if faces is not None:
            for i in range(n - 1):
                for k in range(RING_SEG):
                    kn = (k + 1) % RING_SEG
                    faces.append((rings[i] + k, rings[i] + kn,
                                  rings[i + 1] + kn, rings[i + 1] + k))
            for k in range(RING_SEG):
                kn = (k + 1) % RING_SEG
                faces.append((c0, rings[0] + kn, rings[0] + k))
                faces.append((c1, rings[-1] + k, rings[-1] + kn))

    def wing_surface(self, prog, surf, faces=None, meta=None):
        """ONE face of the cambered double-surface wing, tip-to-tip. surf='u'
        is the upper deployed-wing skin; surf='l' is the lower 'wingsuit'
        membrane. The two surfaces share the LE (xc=0) and TE (xc=1) verts so
        they meet seamlessly into a closed airfoil with the pilot + spars inside
        (a ram-air / paraglider-style double-surface, not a single sheet)."""
        ns = 2 * N_SPAN_HALF + 1
        nc = N_CHORD
        hs = half_span(prog)
        # furled wing reads smooth; billow develops as the skin tensions out
        billow_eff = BILLOW * min(1.0, hs / HALF_SPAN_FULL)
        base = len(self.verts)
        for i in range(ns):
            frac = (i / (ns - 1)) * 2 - 1            # -1..1
            y = frac * hs
            le = le_point(y)
            c = chord_at(y, prog)
            eta = min(abs(y) / HALF_SPAN_FULL, 1.0)
            twist = -math.radians(PLAN_WASHOUT_DEG) * eta
            qc = (le[0] - 0.25 * c, y, Z_WING)
            scallop = 1.0 + billow_eff * math.cos(math.pi * eta * N_RIBS) ** 2
            for j in range(nc):
                xc = 0.5 * (1 - math.cos(math.pi * j / (nc - 1)))
                up, lo = naca4_surface(xc, AF_M, AF_P, AF_T_SKIN)
                off = up if surf == "u" else lo
                px = le[0] - xc * c
                pz = Z_WING + off * c * scallop
                self.verts.append(_rot_y((px, y, pz), qc, twist))
                if meta is not None:
                    meta.append((eta, xc, surf))
        if faces is not None:
            for i in range(ns - 1):
                for j in range(nc - 1):
                    a = base + i * nc + j
                    b = base + i * nc + (j + 1)
                    c2 = base + (i + 1) * nc + (j + 1)
                    d = base + (i + 1) * nc + j
                    if surf == "u":
                        faces.append((a, b, c2, d))
                    else:                       # flip winding so it faces down
                        faces.append((a, d, c2, b))

    def rib(self, y, prog, loop, t_scale, span_thick, faces=None):
        """Airfoil-profile rib as a thin extruded plate at spanwise station y."""
        nl = len(loop)
        base = len(self.verts)
        for so in (-span_thick, span_thick):
            pts, _ = wing_section_points(y, prog, loop, t_scale, BILLOW * 0)
            for p in pts:
                self.verts.append((p[0], p[1] + so, p[2]))
        if faces is not None:
            for k in range(nl):
                kn = (k + 1) % nl
                faces.append((base + k, base + kn, base + nl + kn, base + nl + k))

    def panel(self, p00, p10, p11, p01, faces=None):
        base = len(self.verts)
        self.verts.extend([tuple(p00), tuple(p10), tuple(p11), tuple(p01)])
        if faces is not None:
            faces.append((base, base + 1, base + 2, base + 3))

    def cambered_panel(self, rail_r, rail_l, n_long, n_cross, camber, faces=None):
        """A single cambered membrane webbed between two rails (the LEG-WING /
        tail: rail_r = right leg line, rail_l = left leg line). Arches up
        between the rails so it reads as a fabric stabilizer surface."""
        rr = resample_polyline(rail_r, n_long)
        rl = resample_polyline(rail_l, n_long)
        base = len(self.verts)
        for i in range(n_long):
            span = vsub(rl[i], rr[i])
            spanlen = vlen(span) or 1e-9
            up, _ = _orthoframe(vscale(span, 1.0 / spanlen))
            for j in range(n_cross):
                u = j / (n_cross - 1)
                arch = camber * spanlen * math.sin(math.pi * u)
                self.verts.append(vadd(vlerp(rr[i], rl[i], u), vscale(up, arch)))
        if faces is not None:
            for i in range(n_long - 1):
                for j in range(n_cross - 1):
                    a = base + i * n_cross + j
                    b = base + i * n_cross + (j + 1)
                    c = base + (i + 1) * n_cross + (j + 1)
                    d = base + (i + 1) * n_cross + j
                    faces.append((a, b, c, d))


# =============================================================================
# (6) Full vehicle assembly — one frame
# =============================================================================

SLOTS = ["suit", "helmet", "cfrp", "skin", "wingsuit", "tail", "rib",
         "reserve", "fcs", "flaperon", "metal"]


def _pilot_fk(prog, lean=(0.0, 0.0, 0.0)):
    """Forward kinematics for a FIXED-proportion pilot. Torso anchors never
    move; the limbs ROTATE about the shoulder/hip with FIXED bone lengths
    (no stretching) from a tucked pose (stowed) to a spread pose (deployed).
    Arms angle forward+out toward the LE; legs angle aft+out forming the
    rear/tail near the TE."""
    sp = prog["spread"]
    # `lean` is the pilot's weight-shift for control (the binding CG authority):
    # it offsets the whole body within the craft.
    sh_c = vadd((X_SHOULDER, 0.0, Z_BODY), lean)
    hip_c = vadd((X_SHOULDER - TORSO_LEN, 0.0, Z_BODY - 0.02), lean)

    def march(anchor, d_stow, d_dep, sgn, segs):
        d = vnorm(vlerp((d_stow[0], sgn * d_stow[1], d_stow[2]),
                        (d_dep[0], sgn * d_dep[1], d_dep[2]), sp))
        pts = [anchor]
        p = anchor
        for L in segs:
            p = vadd(p, vscale(d, L))
            pts.append(p)
        return pts

    N = {}
    for side, sgn in (("R", 1.0), ("L", -1.0)):
        sh = vadd((X_SHOULDER, sgn * SHOULDER_HALF, Z_BODY), lean)
        hip = vadd((X_SHOULDER - TORSO_LEN, sgn * HIP_HALF, Z_BODY - 0.02), lean)
        el, wr, hand = march(sh, ARM_DIR_STOW, ARM_DIR_DEP, sgn,
                             [UPPER_ARM, FOREARM, HAND])[1:]
        kn, ank, foot = march(hip, LEG_DIR_STOW, LEG_DIR_DEP, sgn,
                              [THIGH, SHANK, FOOT])[1:]
        N[side] = dict(sh=sh, el=el, wr=wr, hand=hand,
                       hip=hip, kn=kn, ank=ank, foot=foot)
    N["sh_c"] = sh_c
    N["hip_c"] = hip_c
    N["torso_up"] = sh_c      # aliases used by the CFRP / reserve / FCS sections
    N["hip_mid"] = hip_c
    # head rides higher so a recognisable head silhouette breaches the canopy
    N["neck"] = vadd(sh_c, (NECK_LEN * 0.6, 0, 0.07))
    N["head"] = vadd((X_SHOULDER + NECK_LEN + HEAD_LEN * 0.4, 0.0, Z_BODY + 0.11), lean)
    return N


def _begin(faces, m):
    return (len(faces) if faces is not None else 0, len(m.verts))


def build_frame(frame, faces=None, mat_ranges=None, vert_ranges=None, skin_meta=None):
    m = Mesh()
    # control frames hold the wing fully deployed and apply pilot weight-shift
    # + flaperon deflection (the rest of the body banks via the viewer).
    ctrl = frame.get("ctrl")
    prog = DEPLOYED_PROG if ctrl else phase_progress(frame["t"])
    f0 = faces is not None
    N = _pilot_fk(prog, lean=tuple(ctrl["lean"]) if ctrl else (0.0, 0.0, 0.0))

    def end(name, fs, vs):
        if f0:
            mat_ranges[name] = (fs, len(faces))
            vert_ranges.setdefault(name, []).append((vs, len(m.verts)))

    sk_loop = airfoil_loop(N_CHORD, AF_T_SKIN)
    rib_loop = airfoil_loop(N_CHORD, AF_T_SKIN * 1.04)

    # ---- (0) PILOT — fixed-proportion anatomical body (NASA-STD-3000) ----
    fs, vs = _begin(faces, m)
    sh_c, hip_c = N["sh_c"], N["hip_c"]
    # torso: BROAD SHOULDERS (widest, biacromial 0.40) -> waist -> fuller pelvis
    # (ry = half-width along span, rz = half-depth; a prone torso is wider than
    # it is deep). The first ring matches the SHOULDER_HALF=0.20 arm anchors so
    # there is no floating-shoulder gap.
    torso_line = [vadd(sh_c, (0.02, 0, 0.01)),
                  vlerp(sh_c, hip_c, 0.26), vlerp(sh_c, hip_c, 0.52),
                  vlerp(sh_c, hip_c, 0.76), hip_c]
    torso_rad = [(0.205, 0.105), (0.180, 0.118), (0.140, 0.105),
                 (0.135, 0.102), (0.165, 0.110)]
    m.loft(torso_line, torso_rad, faces)
    # deltoid bulges bridge the torso to the arm roots
    for s in ("R", "L"):
        m.sphere(N[s]["sh"], 0.066, faces)
    # limbs: fixed-length lofts (shoulder -> elbow -> wrist -> hand; hip -> knee
    # -> ankle -> foot) with anatomical taper. Hand/foot widen+flatten (paddle).
    for s in ("R", "L"):
        n = N[s]
        m.loft([n["sh"], n["el"], n["wr"], n["hand"]],
               [(0.060, 0.060), (0.046, 0.046), (0.036, 0.034), (0.040, 0.018)],
               faces)
        m.loft([n["hip"], n["kn"], n["ank"], n["foot"]],
               [(0.092, 0.092), (0.060, 0.060), (0.046, 0.044), (0.050, 0.022)],
               faces)
    end("suit", fs, vs)

    # ---- (0b) HELMET — distinct material so the pilot reads as a human ----
    fs, vs = _begin(faces, m)
    m.loft([sh_c, N["neck"]], [(0.078, 0.072), (0.054, 0.052)], faces)
    head = N["head"]
    m.loft([N["neck"], vadd(head, (-0.04, 0, -0.01)), head, vadd(head, (0.085, 0, 0.0))],
           [(0.054, 0.052), (0.076, 0.082), (0.078, 0.084), (0.040, 0.046)], faces)
    end("helmet", fs, vs)

    # ---- (1) CFRP STRUCTURE — the real load path + deploy mechanism (BRIEF #2)
    # LE spar is bonded along the UNDERSIDE of the arm (shoulder hinge -> wrist
    # hub); a 3-stage CFRP boom TELESCOPES out of the wrist hub to the wingtip.
    # Mirror along the leg -> ankle hub -> TE boom. Everything sits just under
    # the limb (inside the fabric) so nothing pierces the body.
    fs, vs = _begin(faces, m)
    shoulder_yoke = vadd(N["torso_up"], (0, 0, 0.125))
    hip_yoke = vadd(N["hip_mid"], (0, 0, 0.105))
    sx = vnorm(vsub(hip_yoke, shoulder_yoke))
    sz = vnorm(vsub((0, 0, 1.0), vscale(sx, vdot((0, 0, 1.0), sx))))
    sy = vnorm(vcross(sx, sz))
    m.box(vlerp(shoulder_yoke, hip_yoke, 0.5),
          (vlen(vsub(hip_yoke, shoulder_yoke)) * 0.5, 0.042, 0.026),
          faces, axis=(sx, sy, sz))
    hs = half_span(prog)
    below = (0.0, 0.0, -0.05)       # spar bonded to the underside of the limb
    in_le = (-0.04, 0, 0.0)         # boom just inside the leading edge
    in_te = (0.04, 0, 0.0)          # boom just inside the trailing edge

    def boom(root, nodes, radii):
        prev = root
        for nd, (r0, r1) in zip(nodes, radii):
            m.tube(prev, nd, r0, r1, faces)
            prev = nd

    for s in ("R", "L"):
        sgn = 1.0 if s == "R" else -1.0
        n = N[s]
        # shoulder + hip hinge knuckles (spar pivots at the spine yoke)
        m.box(n["sh"], (0.058, 0.05, 0.05), faces)
        m.tube(vadd(n["sh"], (0, sgn * 0.055, 0)), vadd(n["sh"], (0, -sgn * 0.04, 0)), 0.024, 0.024, faces)
        m.box(n["hip"], (0.058, 0.055, 0.05), faces)
        # LE spar bonded under the arm -> wrist
        m.tube(vadd(n["sh"], below), vadd(n["el"], below), 0.028, 0.024, faces)
        m.tube(vadd(n["el"], below), vadd(n["wr"], below), 0.024, 0.020, faces)
        # WRIST HUB: the telescoping boom mounts + pivots here
        m.tube(vadd(n["wr"], (0.045, 0, -0.015)), vadd(n["wr"], (-0.045, 0, -0.06)), 0.036, 0.032, faces)
        # LE boom: 3 nested telescoping stages, wrist hub -> LE tip. Retracted
        # (nested at the wrist) in Phase A, extends as hs grows in Phase B.
        wy = abs(n["wr"][1])
        boom(vadd(n["wr"], below),
             [vadd(le_point(sgn * lerp(wy, hs, 1 / 3)), in_le),
              vadd(le_point(sgn * lerp(wy, hs, 2 / 3)), in_le),
              vadd(le_point(sgn * hs), in_le)],
             [(0.026, 0.022), (0.020, 0.016), (0.014, 0.010)])
        # TE spar bonded under the leg -> ankle
        m.tube(vadd(n["hip"], below), vadd(n["kn"], below), 0.026, 0.022, faces)
        m.tube(vadd(n["kn"], below), vadd(n["ank"], below), 0.022, 0.018, faces)
        # ANKLE HUB + TE boom (3 stages, ankle hub -> TE tip)
        m.tube(vadd(n["ank"], (0.045, 0, -0.015)), vadd(n["ank"], (-0.045, 0, -0.055)), 0.032, 0.028, faces)
        ay = abs(n["ank"][1])
        boom(vadd(n["ank"], below),
             [vadd(te_point(sgn * lerp(ay, hs, 1 / 3), prog), in_te),
              vadd(te_point(sgn * lerp(ay, hs, 2 / 3), prog), in_te),
              vadd(te_point(sgn * hs, prog), in_te)],
             [(0.024, 0.020), (0.018, 0.014), (0.012, 0.009)])
    end("cfrp", fs, vs)

    # ---- (2) WING — double surface: upper deployed skin + lower wingsuit ----
    # Upper = the lofted rigid-wing skin (carries the suction that makes lift);
    # lower = the wingsuit fabric the pilot wears. They meet seamlessly at the
    # LE/TE so the pilot + spars sit INSIDE a closed airfoil (paraglider-style).
    fs, vs = _begin(faces, m)
    m.wing_surface(prog, "u", faces, skin_meta)
    end("skin", fs, vs)
    fs, vs = _begin(faces, m)
    m.wing_surface(prog, "l", faces)
    end("wingsuit", fs, vs)

    # ---- (2b) TAIL MEMBRANE — the leg-wing webbed between the legs ----
    # The pilot's legs splay aft to form the rear stabiliser: a fabric membrane
    # is stretched between the two legs (crotch -> between the feet), behind the
    # main wing's centre TE. This is the empennage the legs control for pitch.
    fs, vs = _begin(faces, m)
    crotch = vadd(N["hip_c"], (-0.04, 0, -0.01))
    m.cambered_panel([crotch, N["R"]["kn"], N["R"]["ank"], N["R"]["foot"]],
                     [crotch, N["L"]["kn"], N["L"]["ank"], N["L"]["foot"]],
                     n_long=12, n_cross=11, camber=0.12, faces=faces)
    end("tail", fs, vs)

    # ---- (3) BISTABLE RIBS (9 per side) — coiled stowed, snap on Phase C ----
    fs, vs = _begin(faces, m)
    # bistable tape-spring ribs are nearly flat when coiled and snap to the full
    # airfoil profile as the spars pass them (prog["rib"]).
    t_rib = lerp(0.018, AF_T_SKIN * 1.04, prog["rib"])
    for s in ("R", "L"):
        sgn = 1.0 if s == "R" else -1.0
        for k in range(N_RIBS):
            eta = (k + 0.5) / N_RIBS
            y = sgn * eta * hs
            m.rib(y, prog, rib_loop, t_rib, 0.006, faces)
    end("rib", fs, vs)

    # ---- (4) RESERVE CONTAINER (sits ON the back, above the spine yoke) ----
    fs, vs = _begin(faces, m)
    m.box(vadd(N["torso_up"], (-0.06, 0, 0.215)), (0.15, 0.12, 0.06), faces)
    end("reserve", fs, vs)

    # ---- (5) FCS BAY ----
    fs, vs = _begin(faces, m)
    m.box(vadd(vlerp(shoulder_yoke, hip_yoke, 0.30), (0, 0, 0.06)),
          (0.075, 0.055, 0.030), faces)
    end("fcs", fs, vs)

    # ---- (6) FLAPERONS (outboard TE) — symmetric elevator + diff aileron ----
    fs, vs = _begin(faces, m)
    base_defl = math.radians(14) * prog["flap"]
    de = ctrl["de"] if ctrl else 0.0      # symmetric (pitch), normalised -1..1
    da = ctrl["da"] if ctrl else 0.0      # differential (roll),  normalised -1..1
    chord = 0.16 * (0.03 + 0.97 * prog["flap"])

    def te_skin_pt(y):
        # the actual trailing-edge skin vertex (carries the washout twist +
        # billow), so the flaperon hinge sits ON the skin, not floating.
        pts, _ = wing_section_points(y, prog, sk_loop, AF_T_SKIN,
                                     BILLOW * min(1.0, hs / HALF_SPAN_FULL))
        return pts[N_CHORD - 1]
    for s in ("R", "L"):
        sgn = 1.0 if s == "R" else -1.0
        # +de deflects both down (pitch); +da deflects right down / left up (roll)
        d = base_defl + math.radians(18) * de + sgn * math.radians(22) * da
        aft = (-chord * math.cos(d), 0, -chord * math.sin(d))
        pa, pb = te_skin_pt(sgn * 0.58 * hs), te_skin_pt(sgn * 0.92 * hs)
        m.panel(pa, pb, vadd(pb, aft), vadd(pa, aft), faces)
    end("flaperon", fs, vs)

    # ---- (7) CO2 BOTTLES + pneumatic yoke actuators + tip-hub fittings ----
    fs, vs = _begin(faces, m)
    for s in ("R", "L"):
        sgn = 1.0 if s == "R" else -1.0
        n = N[s]
        # CO2 pressure vessel near the shoulder yoke (one valve per side, BRIEF
        # #6): cylindrical body + domed cap + valve stem, with a feed line aft.
        b0 = vadd(shoulder_yoke, (0.10, sgn * 0.15, 0.0))
        b1 = vadd(shoulder_yoke, (-0.12, sgn * 0.15, 0.0))
        m.tube(b0, b1, 0.032, 0.032, faces)
        m.sphere(b1, 0.033, faces)
        m.tube(b0, vadd(b0, (0.05, 0, 0)), 0.012, 0.010, faces)   # valve stem
        # feed line routed to the wrist hub (CO2 drives the boom telescope)
        m.tube(vadd(b0, (0.05, 0, 0)), vadd(n["wr"], below), 0.006, 0.006, faces)
        # short pneumatic actuator across each spar hinge (drives Phase-A sweep)
        m.tube(shoulder_yoke, vadd(n["sh"], below), 0.018, 0.015, faces)
        m.tube(hip_yoke, vadd(n["hip"], below), 0.018, 0.015, faces)
        # cuffs: bands that strap the arm/leg to the spar (BRIEF #2 bonded sleeve)
        def cuff(a, b_, frac):
            c = vadd(vlerp(a, b_, frac), (0, 0, -0.022))
            d = vnorm(vsub(b_, a))
            m.tube(vadd(c, vscale(d, -0.02)), vadd(c, vscale(d, 0.02)), 0.078, 0.078, faces)
        cuff(n["sh"], n["el"], 0.6); cuff(n["el"], n["wr"], 0.5)
        cuff(n["hip"], n["kn"], 0.6); cuff(n["kn"], n["ank"], 0.5)
    end("metal", fs, vs)

    return m.verts


# =============================================================================
# (7) Aero field — pressure colours + streamlines from the Weissinger data
# =============================================================================

def load_span_cl():
    ys, cl_list = [], []
    if SPAN_LOADING_CSV.exists():
        for line in SPAN_LOADING_CSV.read_text().splitlines()[1:]:
            a, y, chord, cl_sec, sl, ai = line.split(",")
            if abs(float(a) - AERO_ALPHA) < 1e-6:
                ys.append(abs(float(y)) / HALF_SPAN_FULL)
                cl_list.append(float(cl_sec))
    if not ys:
        return lambda eta: 0.45
    pairs = sorted(zip(ys, cl_list))
    xs = [p[0] for p in pairs]; vs = [p[1] for p in pairs]

    def cl(eta):
        eta = min(max(eta, 0.0), 1.0)
        if eta <= xs[0]:
            return vs[0]
        if eta >= xs[-1]:
            return vs[-1]
        for i in range(1, len(xs)):
            if eta <= xs[i]:
                f = (eta - xs[i - 1]) / ((xs[i] - xs[i - 1]) or 1e-9)
                return lerp(vs[i - 1], vs[i], f)
        return vs[-1]
    return cl


def cp_at(xc, side, cl):
    """Illustrative thin-airfoil Cp at chord frac xc, scaled by local cl.
    Suction (negative) on the upper surface, peaking just aft of the LE; mild
    over-pressure on the lower surface near the LE.  Intuition field, not CFD."""
    cl = max(cl, 0.02)
    if side == "u":
        peak = math.exp(-((xc - 0.10) / 0.13) ** 2)
        return -(1.9 * cl) * (0.75 * peak + (1.0 - xc) * 0.35)
    return math.exp(-(xc / 0.06)) * 0.9 + (0.55 * cl) * (1.0 - xc) * 0.5


def jet_color(v):
    v = min(max(v, 0.0), 1.0)
    stops = [(0.00, (0.10, 0.20, 0.85)), (0.25, (0.10, 0.75, 0.95)),
             (0.50, (0.20, 0.85, 0.30)), (0.75, (0.98, 0.85, 0.15)),
             (1.00, (0.95, 0.15, 0.12))]
    for i in range(1, len(stops)):
        if v <= stops[i][0]:
            f = (v - stops[i - 1][0]) / ((stops[i][0] - stops[i - 1][0]) or 1e-9)
            a, b = stops[i - 1][1], stops[i][1]
            return tuple(lerp(a[j], b[j], f) for j in range(3))
    return stops[-1][1]


DEPLOYED_PROG = {"spread": 1.0, "tip": 1.0, "rib": 1.0, "flap": 1.0}


def build_pressure_object():
    m = Mesh()
    faces, meta = [], []
    # the upper (suction) surface is what carries the lift colourmap
    m.wing_surface(DEPLOYED_PROG, "u", faces, meta)
    cl_of = load_span_cl()
    cps = [cp_at(xc, side, cl_of(eta)) for (eta, xc, side) in meta]
    lo, hi = min(cps), max(cps)
    rng = (hi - lo) or 1e-9
    colors = [(*jet_color((cp - lo) / rng), 1.0) for cp in cps]
    return m.verts, faces, colors, (lo, hi)


def build_flow_object():
    cl_of = load_span_cl()
    m = Mesh()
    faces, uvs, colors = [], [], []
    TUBE_SEG, R, NPT = 5, 0.011, 42
    hs = HALF_SPAN_FULL

    def add_streamline(pts, speeds):
        n = len(pts)
        bases = []
        for i in range(n):
            t_dir = vnorm(vsub(pts[min(i + 1, n - 1)], pts[max(i - 1, 0)]))
            n1, n2 = _orthoframe(t_dir)
            base = len(m.verts)
            for k in range(TUBE_SEG):
                a = 2 * math.pi * k / TUBE_SEG
                c, s = math.cos(a), math.sin(a)
                m.verts.append((pts[i][0] + R * (c * n1[0] + s * n2[0]),
                                pts[i][1] + R * (c * n1[1] + s * n2[1]),
                                pts[i][2] + R * (c * n1[2] + s * n2[2])))
                # uv.x = position along streamline (for the flow-pulse shader),
                # uv.y = local speed (slow 0 -> fast 1, for the colour ramp)
                uvs.append((i / (n - 1), speeds[i]))
                colors.append((*jet_color(speeds[i]), 1.0))
            bases.append(base)
        for i in range(n - 1):
            for k in range(TUBE_SEG):
                kn = (k + 1) % TUBE_SEG
                faces.append((bases[i] + k, bases[i] + kn,
                              bases[i + 1] + kn, bases[i + 1] + k))

    # (1) build every streamline polyline + a CONTINUOUS local speed.  The
    #     vertical track is camber-line + a standoff that ramps in/out across
    #     the chord (no z-step at LE/TE) + a circulation deflection (upwash
    #     ahead -> downwash behind).  Outboard lines curl in the wake (tip
    #     vortex).  Speeds use one model end-to-end (decel into the LE,
    #     acceleration over the suction peak, mild wake deficit).
    raw = []
    n_seed = 14
    for side_sgn in (1.0, -1.0):
        for i in range(n_seed):
            eta = (i + 0.5) / n_seed
            y0 = side_sgn * eta * hs
            cl = cl_of(eta)
            le = le_point(y0)
            c = planform_chord(abs(y0))
            pts, sp = [], []
            for j in range(NPT):
                s = lerp(-0.5, 1.7, j / (NPT - 1))
                sc = min(max(s, 0.0), 1.0)
                up_off, _ = naca4_surface(sc, AF_M, AF_P, AF_T_SKIN)
                standoff = 0.05 * smoothstep((s + 0.12) / 0.16) * \
                    (1.0 - smoothstep((s - 0.96) / 0.16))
                defl = 0.22 * cl * (0.5 - smoothstep((s + 0.4) / 1.9))
                curl = max(0.0, eta - 0.78) * max(0.0, s - 1.0)
                y = y0 - side_sgn * curl * 0.9
                z = up_off + standoff + defl + curl * 0.6
                pts.append((le[0] - s * c, y, Z_WING + z * c))
                if 0 <= s <= 1:
                    spd = math.sqrt(max(1.0 - cp_at(sc, "u", cl), 0.05))
                elif s < 0:
                    spd = 1.0 - 0.30 * cl * math.exp(s / 0.22)        # decel into LE
                else:
                    spd = 1.0 - 0.12 * cl * math.exp(-(s - 1.0) / 0.5)  # wake deficit
                sp.append(spd)
            raw.append((pts, sp))

    # (2) ONE global speed normalisation so colour tells a consistent
    #     upwash -> suction-peak -> wake story across every line.
    allsp = [v for _, sp in raw for v in sp]
    smin, smax = min(allsp), max(allsp)
    srng = (smax - smin) or 1e-9
    for pts, sp in raw:
        add_streamline(pts, [0.04 + 0.96 * (v - smin) / srng for v in sp])
    return m.verts, faces, uvs, colors


# =============================================================================
# (8) Blender scene, materials, export, render
# =============================================================================

def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for d in [bpy.data.objects, bpy.data.meshes, bpy.data.materials,
              bpy.data.lights, bpy.data.cameras, bpy.data.worlds]:
        for item in list(d):
            d.remove(item)


def make_material(name, color, metallic=0.0, roughness=0.5, alpha=1.0,
                  emission=None, vcol=False):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, alpha)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if "Emission Color" in bsdf.inputs and emission is not None:
        bsdf.inputs["Emission Color"].default_value = (*emission, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 2.5
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        # Blender 4.2+/EEVEE-Next replaced blend_method with surface_render_method;
        # use_transparency_overlap lets stacked translucent skin layers show the
        # structure + pilot behind them (otherwise only the front layer renders).
        if hasattr(mat, "surface_render_method"):
            mat.surface_render_method = "BLENDED"
        else:
            mat.blend_method = "BLEND"
        if hasattr(mat, "use_transparency_overlap"):
            mat.use_transparency_overlap = True
        if hasattr(mat, "show_transparent_back"):
            mat.show_transparent_back = False
    if vcol:
        vc = nt.nodes.new("ShaderNodeVertexColor")
        nt.links.new(vc.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def build_world():
    world = bpy.data.worlds.new("World")
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = "QUADRATIC_SPHERE"
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.04, 0.06, 0.12, 1)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (0.55, 0.60, 0.66, 1)
    geo = nt.nodes.new("ShaderNodeTexCoord")
    map_ = nt.nodes.new("ShaderNodeMapping")
    nt.links.new(geo.outputs["Generated"], map_.inputs["Vector"])
    nt.links.new(map_.outputs["Vector"], grad.inputs["Vector"])
    nt.links.new(grad.outputs["Color"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bg.inputs["Color"])
    bg.inputs["Strength"].default_value = 1.0
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])
    bpy.context.scene.world = world


def add_lights():
    bpy.ops.object.light_add(type="SUN", location=(8, -8, 12))
    sun = bpy.context.active_object
    sun.data.energy = 4.0
    sun.rotation_euler = (math.radians(48), 0, math.radians(-40))
    if hasattr(sun.data, "angle"):
        sun.data.angle = math.radians(2.5)
    bpy.ops.object.light_add(type="AREA", location=(-4, 5, 5))
    fill = bpy.context.active_object
    fill.data.energy = 500
    fill.data.size = 7


def add_camera():
    bpy.ops.object.camera_add(location=(3.0, -5.4, 2.6))
    cam = bpy.context.active_object
    cam.data.lens = 42
    bpy.ops.object.empty_add(location=(-0.1, 0, 0.1))
    target = bpy.context.active_object
    track = cam.constraints.new(type="TRACK_TO")
    track.target = target
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"
    bpy.context.scene.camera = cam


def setup_renderer():
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    try:
        scene.eevee.taa_render_samples = 64
    except AttributeError:
        pass
    scene.view_settings.view_transform = "Filmic"
    scene.view_settings.look = "Medium High Contrast"


def build_morph_object(traj):
    frames = traj["frames"]
    faces, mat_ranges, vert_ranges, skin_meta = [], {}, {}, []
    base_verts = build_frame(frames[0], faces, mat_ranges, vert_ranges, skin_meta)
    print(f"  topology: {len(base_verts)} verts, {len(faces)} faces")

    mesh = bpy.data.meshes.new("manta_mesh")
    mesh.from_pydata(base_verts, [], [list(f) for f in faces])
    mesh.update(); mesh.validate(verbose=False)

    obj = bpy.data.objects.new("MANTA", mesh)
    bpy.context.collection.objects.link(obj)

    mats = {
        "suit": make_material("suit", (0.13, 0.15, 0.21), metallic=0.05, roughness=0.66),
        "helmet": make_material("helmet", (0.86, 0.88, 0.92), metallic=0.3, roughness=0.28),
        "cfrp": make_material("cfrp", (0.025, 0.025, 0.032), metallic=0.75, roughness=0.26),
        "skin": make_material("skin", (0.10, 0.34, 0.85), roughness=0.30, alpha=0.32),
        "wingsuit": make_material("wingsuit", (0.20, 0.21, 0.24), roughness=0.78, alpha=0.82),
        "tail": make_material("tail", (0.16, 0.44, 0.88), roughness=0.32, alpha=0.42),
        "rib": make_material("rib", (0.06, 0.07, 0.09), metallic=0.5, roughness=0.4),
        "reserve": make_material("reserve", (0.85, 0.30, 0.08), roughness=0.55),
        "fcs": make_material("fcs", (0.05, 0.18, 0.10), metallic=0.4, roughness=0.5,
                             emission=(0.0, 0.6, 0.2)),
        "flaperon": make_material("flaperon", (0.92, 0.55, 0.10), roughness=0.45),
        "metal": make_material("metal", (0.72, 0.74, 0.78), metallic=0.95, roughness=0.22),
    }
    slot_index = {name: i for i, name in enumerate(SLOTS)}
    for name in SLOTS:
        mesh.materials.append(mats[name])
    for name, (lo, hi) in mat_ranges.items():
        for fi in range(lo, hi):
            if fi < len(mesh.polygons):
                mesh.polygons[fi].material_index = slot_index[name]

    obj.shape_key_add(name="frame_000", from_mix=False)
    for fi in range(1, len(frames)):
        verts = build_frame(frames[fi])
        sk = obj.shape_key_add(name=f"frame_{fi:03d}", from_mix=False)
        for vi, co in enumerate(verts):
            if vi < len(sk.data):
                sk.data[vi].co = co

    # control-basis shape keys: stored as base + (control_pose - deployed) so a
    # key value w contributes (control_pose - deployed). The viewer holds the
    # deployed frame at 1 and adds these by live input -> real-time flaperon
    # deflection + pilot weight-shift, on top of any deploy state.
    deployed_verts = build_frame(frames[-1])
    for name, ctrl in CTRL_BASIS.items():
        pose = build_frame({"t": DURATION_S, "ctrl": ctrl})
        sk = obj.shape_key_add(name=f"ctrl_{name}", from_mix=False)
        sk.value = 0.0
        for vi in range(len(sk.data)):
            b, p, d = base_verts[vi], pose[vi], deployed_verts[vi]
            sk.data[vi].co = (b[0] + p[0] - d[0], b[1] + p[1] - d[1],
                              b[2] + p[2] - d[2])

    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()

    keys = obj.data.shape_keys
    keys.use_relative = True
    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = len(frames) - 1
    try:
        bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    except Exception:
        pass
    kb = keys.key_blocks
    for fi in range(len(frames)):
        kbi = kb[fi]
        for set_frame in (fi - 1, fi, fi + 1):
            if 0 <= set_frame <= len(frames) - 1:
                kbi.value = 1.0 if set_frame == fi else 0.0
                kbi.keyframe_insert("value", frame=set_frame)
    return obj


def build_static_colored_object(name, verts, faces, colors, base_color,
                                emission=None, uvs=None, alpha=1.0):
    mesh = bpy.data.meshes.new(name + "_mesh")
    mesh.from_pydata(verts, [], [list(f) for f in faces])
    mesh.update(); mesh.validate(verbose=False)
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    if colors:
        try:
            attr = mesh.color_attributes.new(name="Cp", type="FLOAT_COLOR",
                                             domain="POINT")
            for i, c in enumerate(colors):
                if i < len(attr.data):
                    attr.data[i].color = c
            try:
                mesh.color_attributes.active_color = attr
                mesh.color_attributes.render_color_index = 0
            except Exception:
                pass
        except Exception as e:
            print("  vcol warn:", e)
    if uvs:
        uv_layer = mesh.uv_layers.new(name="flow")
        for li, loop in enumerate(mesh.loops):
            vi = loop.vertex_index
            if vi < len(uvs):
                uv_layer.data[li].uv = uvs[vi]
    mat = make_material(name.lower(), base_color, roughness=0.4,
                        alpha=alpha, emission=emission, vcol=bool(colors))
    mesh.materials.append(mat)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()
    return obj


def export_glb(objs):
    SITE_MODELS.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    # (Draco was tried but the Blender exporter doesn't compress morph-target
    # deltas — which dominate here — so it added a DRACOLoader dependency for no
    # win. Size is kept down via the frame budget instead.)
    kwargs = dict(export_format="GLB", use_selection=True,
                  export_animations=True, export_morph=True,
                  export_morph_animation=True, export_yup=True,
                  export_vertex_color="ACTIVE")
    for path in (OUT_DIR / "manta.glb", SITE_MODELS / "manta.glb"):
        try:
            bpy.ops.export_scene.gltf(filepath=str(path), **kwargs)
        except TypeError:
            kwargs.pop("export_vertex_color", None)
            bpy.ops.export_scene.gltf(filepath=str(path), **kwargs)
        print(f"  exported {path.name} ({path.stat().st_size/1e6:.2f} MB)")


def render_still(name, frame_idx, hide=()):
    bpy.context.scene.frame_set(frame_idx)
    for o in hide:
        o.hide_render = True
    bpy.context.scene.render.filepath = str(OUT_DIR / f"{name}.png")
    print(f"  render {name}.png (frame {frame_idx})...")
    bpy.ops.render.render(write_still=True)
    for o in hide:
        o.hide_render = False


def main():
    do_render = "--render" in sys.argv
    print("(1) MuJoCo deployment schedule...")
    traj = run_simulation()
    deploy_n = len(traj["frames"])
    (OUT_DIR / "trajectory.json").write_text(json.dumps(traj))
    # flight model + control-key map for the viewer's interactive flight sim
    control_json = {"model": FLIGHT_MODEL, "deploy_frames": deploy_n,
                    "ctrl_keys": list(CTRL_BASIS.keys())}
    SITE_MODELS.mkdir(parents=True, exist_ok=True)
    for p in (OUT_DIR / "control.json", SITE_MODELS / "control.json"):
        p.write_text(json.dumps(control_json))
    print(f"  {deploy_n} deploy frames + {len(CTRL_BASIS)} control-basis poses")

    print("(2) Building Blender scene...")
    clear_scene(); build_world(); add_lights(); add_camera(); setup_renderer()

    print("(3) Building animated morph mesh (full mechanical model)...")
    manta = build_morph_object(traj)

    print("(4) Building aero field (pressure + streamlines)...")
    pv, pf, pc, cp_rng = build_pressure_object()
    pressure = build_static_colored_object("PRESSURE", pv, pf, pc,
                                            base_color=(1, 1, 1), alpha=0.96)
    fv, ff, fuv, fc = build_flow_object()
    flow = build_static_colored_object("FLOW", fv, ff, fc,
                                       base_color=(0.4, 0.7, 1.0),
                                       emission=(0.2, 0.5, 1.0), uvs=fuv)
    print(f"  pressure {len(pv)} verts (Cp {cp_rng[0]:.2f}..{cp_rng[1]:.2f}) | "
          f"flow {len(fv)} verts")
    pressure.hide_render = True
    flow.hide_render = True

    print("(5) Exporting GLB...")
    export_glb([manta, pressure, flow])

    if do_render:
        print("(6) Rendering hero stills...")
        bpy.ops.mesh.primitive_plane_add(size=40, location=(0, 0, -0.7))
        plane = bpy.context.active_object
        plane.data.materials.append(make_material("ground", (0.07, 0.07, 0.09),
                                                   roughness=0.85))
        render_still("hero_stowed", 0, hide=(pressure, flow))
        render_still("hero_mid_deploy", N_FRAMES // 2, hide=(pressure, flow))
        render_still("hero_deployed", N_FRAMES - 1, hide=(pressure, flow))
        manta.hide_render = False
        render_still("hero_flow", N_FRAMES - 1)
        # control still: a left-banked turn — hold the deployed frame, drive the
        # rollL control-basis key (left flaperon up / right down + pilot lean),
        # and roll the object, exactly as the viewer does live.
        kb = manta.data.shape_keys.key_blocks
        kb[N_FRAMES - 1].value = 1.0      # deployed
        kb["ctrl_rollL"].value = 1.0
        manta.rotation_euler = (math.radians(-32), 0, 0)
        render_still("hero_control", N_FRAMES - 1, hide=(pressure, flow))
        kb["ctrl_rollL"].value = 0.0
        manta.rotation_euler = (0.0, 0.0, 0.0)
        # top-down planform check: full span on the wide axis, high + wide
        # enough to frame both tips (span along image X via UP_X).
        cam = bpy.context.scene.camera
        cam.location = (-0.5, 0.0, 15.0)
        cam.data.lens = 38
        if cam.constraints:
            cam.constraints[0].up_axis = "UP_X"
            tgt = cam.constraints[0].target
            if tgt is not None:
                tgt.location = (-0.5, 0, 0)
        render_still("hero_top", N_FRAMES - 1, hide=(pressure, flow))

    print("done.")


if __name__ == "__main__":
    main()
