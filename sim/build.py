"""
MANTA — deployment pipeline: MuJoCo kinematics -> animated GLB + renders.

Two physics layers feed this build:

  * sim/flight_dynamics.py  — the rigorous 6-state freefall->deploy->glide
    trajectory (airspeed, load factor, glide ratio).  Verified against the
    BRIEF targets.  Produces telemetry.json for the viewer overlay.

  * this file's run_simulation() — the MuJoCo *mechanism* kinematics: how the
    spars, arms, legs and telescoping tips move through the 0.6 s deployment.
    These joint trajectories drive the geometry.

The geometry is built as ONE mesh with a FIXED topology that is identical on
every frame; the 60 deployment frames are stored as morph targets (shape
keys).  That gives a single GLB with a real, continuously-interpolated
deployment animation — no opacity crossfades, no per-pose static meshes — that
three.js plays directly via morphTargetInfluences.

Run:  PYTHONPATH=. .venv/bin/python sim/build.py
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

      <!-- ============ RIGHT ARM (y > 0) ============ -->
      <body name="shoulder_R" pos="0.30 0.18 0.10">
        <inertial pos="0 0.15 0" mass="2.5" diaginertia="0.05 0.005 0.05"/>
        <joint name="sh_R_yaw"   type="hinge" axis="0 0 1" range="-0.05 1.6" damping="2"/>
        <joint name="sh_R_pitch" type="hinge" axis="1 0 0" range="-1.5 0.05" damping="2"/>
        <body name="elbow_R" pos="0 0.30 0">
          <inertial pos="0 0.135 0" mass="1.6" diaginertia="0.03 0.003 0.03"/>
          <joint name="el_R" type="hinge" axis="1 0 0" range="-1.6 0.05" damping="1.5"/>
          <body name="wrist_R" pos="0 0.27 0">
            <inertial pos="0 0 0" mass="0.4" diaginertia="0.001 0.001 0.001"/>
            <body name="tip1_R" pos="0 0 0.07">
              <joint name="tip1_R_slide" type="slide" axis="0 1 0" range="0 1.10" damping="2"/>
              <inertial pos="0 -0.55 0" mass="0.45" diaginertia="0.015 0.001 0.015"/>
              <body name="tip2_R" pos="0 0 0">
                <joint name="tip2_R_slide" type="slide" axis="0 1 0" range="0 1.10" damping="2"/>
                <inertial pos="0 -0.55 0" mass="0.30" diaginertia="0.010 0.001 0.010"/>
                <body name="tip3_R" pos="0 0 0">
                  <joint name="tip3_R_slide" type="slide" axis="0 1 0" range="0 1.10" damping="2"/>
                  <inertial pos="0 -0.55 0" mass="0.20" diaginertia="0.006 0.001 0.006"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

      <!-- ============ LEFT ARM (y < 0) ============ -->
      <body name="shoulder_L" pos="0.30 -0.18 0.10">
        <inertial pos="0 -0.15 0" mass="2.5" diaginertia="0.05 0.005 0.05"/>
        <joint name="sh_L_yaw"   type="hinge" axis="0 0 1" range="-1.6 0.05" damping="2"/>
        <joint name="sh_L_pitch" type="hinge" axis="1 0 0" range="-1.5 0.05" damping="2"/>
        <body name="elbow_L" pos="0 -0.30 0">
          <inertial pos="0 -0.135 0" mass="1.6" diaginertia="0.03 0.003 0.03"/>
          <joint name="el_L" type="hinge" axis="1 0 0" range="-1.6 0.05" damping="1.5"/>
          <body name="wrist_L" pos="0 -0.27 0">
            <inertial pos="0 0 0" mass="0.4" diaginertia="0.001 0.001 0.001"/>
            <body name="tip1_L" pos="0 0 0.07">
              <joint name="tip1_L_slide" type="slide" axis="0 -1 0" range="0 1.10" damping="2"/>
              <inertial pos="0 0.55 0" mass="0.45" diaginertia="0.015 0.001 0.015"/>
              <body name="tip2_L" pos="0 0 0">
                <joint name="tip2_L_slide" type="slide" axis="0 -1 0" range="0 1.10" damping="2"/>
                <inertial pos="0 0.55 0" mass="0.30" diaginertia="0.010 0.001 0.010"/>
                <body name="tip3_L" pos="0 0 0">
                  <joint name="tip3_L_slide" type="slide" axis="0 -1 0" range="0 1.10" damping="2"/>
                  <inertial pos="0 0.55 0" mass="0.20" diaginertia="0.006 0.001 0.006"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>

      <!-- ============ HIPS / LEGS ============ -->
      <body name="hip_R" pos="-0.20 0.12 0">
        <inertial pos="-0.225 0 0" mass="6.0" diaginertia="0.10 0.008 0.10"/>
        <joint name="hip_R_yaw" type="hinge" axis="0 0 1" range="-0.5 0.05" damping="2"/>
        <body name="knee_R" pos="-0.45 0 0">
          <inertial pos="-0.21 0 0" mass="3.0" diaginertia="0.05 0.005 0.05"/>
          <joint name="kn_R" type="hinge" axis="0 1 0" range="-0.6 0.6" damping="1.5"/>
          <body name="ankle_R" pos="-0.42 0 0">
            <inertial pos="0 0 0" mass="0.3" diaginertia="0.001 0.001 0.001"/>
          </body>
        </body>
      </body>
      <body name="hip_L" pos="-0.20 -0.12 0">
        <inertial pos="-0.225 0 0" mass="6.0" diaginertia="0.10 0.008 0.10"/>
        <joint name="hip_L_yaw" type="hinge" axis="0 0 1" range="-0.05 0.5" damping="2"/>
        <body name="knee_L" pos="-0.45 0 0">
          <inertial pos="-0.21 0 0" mass="3.0" diaginertia="0.05 0.005 0.05"/>
          <joint name="kn_L" type="hinge" axis="0 1 0" range="-0.6 0.6" damping="1.5"/>
          <body name="ankle_L" pos="-0.42 0 0">
            <inertial pos="0 0 0" mass="0.3" diaginertia="0.001 0.001 0.001"/>
          </body>
        </body>
      </body>

      <!-- ============ TE SPAR (body-mounted) ============ -->
      <body name="te_hub" pos="-0.55 0 0.05">
        <body name="te1_R" pos="0 0 0">
          <joint name="te1_R_slide" type="slide" axis="0 1 0" range="0 1.20" damping="2"/>
          <inertial pos="0 -0.6 0" mass="0.40" diaginertia="0.012 0.001 0.012"/>
          <body name="te2_R" pos="0 0 0">
            <joint name="te2_R_slide" type="slide" axis="0 1 0" range="0 1.20" damping="2"/>
            <inertial pos="0 -0.6 0" mass="0.27" diaginertia="0.008 0.001 0.008"/>
            <body name="te3_R" pos="0 0 0">
              <joint name="te3_R_slide" type="slide" axis="0 1 0" range="0 1.20" damping="2"/>
              <inertial pos="0 -0.6 0" mass="0.18" diaginertia="0.005 0.001 0.005"/>
            </body>
          </body>
        </body>
        <body name="te1_L" pos="0 0 0">
          <joint name="te1_L_slide" type="slide" axis="0 -1 0" range="0 1.20" damping="2"/>
          <inertial pos="0 0.6 0" mass="0.40" diaginertia="0.012 0.001 0.012"/>
          <body name="te2_L" pos="0 0 0">
            <joint name="te2_L_slide" type="slide" axis="0 -1 0" range="0 1.20" damping="2"/>
            <inertial pos="0 0.6 0" mass="0.27" diaginertia="0.008 0.001 0.008"/>
            <body name="te3_L" pos="0 0 0">
              <joint name="te3_L_slide" type="slide" axis="0 -1 0" range="0 1.20" damping="2"/>
              <inertial pos="0 0.6 0" mass="0.18" diaginertia="0.005 0.001 0.005"/>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="sh_R_yaw_act"   joint="sh_R_yaw"   kp="180" kv="14"/>
    <position name="sh_L_yaw_act"   joint="sh_L_yaw"   kp="180" kv="14"/>
    <position name="sh_R_pitch_act" joint="sh_R_pitch" kp="180" kv="14"/>
    <position name="sh_L_pitch_act" joint="sh_L_pitch" kp="180" kv="14"/>
    <position name="el_R_act"       joint="el_R"       kp="120" kv="10"/>
    <position name="el_L_act"       joint="el_L"       kp="120" kv="10"/>
    <position name="hip_R_yaw_act"  joint="hip_R_yaw"  kp="120" kv="10"/>
    <position name="hip_L_yaw_act"  joint="hip_L_yaw"  kp="120" kv="10"/>
    <position name="tip1_R_act" joint="tip1_R_slide" kp="800" kv="40"/>
    <position name="tip2_R_act" joint="tip2_R_slide" kp="800" kv="40"/>
    <position name="tip3_R_act" joint="tip3_R_slide" kp="800" kv="40"/>
    <position name="tip1_L_act" joint="tip1_L_slide" kp="800" kv="40"/>
    <position name="tip2_L_act" joint="tip2_L_slide" kp="800" kv="40"/>
    <position name="tip3_L_act" joint="tip3_L_slide" kp="800" kv="40"/>
    <position name="te1_R_act"  joint="te1_R_slide"  kp="600" kv="30"/>
    <position name="te2_R_act"  joint="te2_R_slide"  kp="600" kv="30"/>
    <position name="te3_R_act"  joint="te3_R_slide"  kp="600" kv="30"/>
    <position name="te1_L_act"  joint="te1_L_slide"  kp="600" kv="30"/>
    <position name="te2_L_act"  joint="te2_L_slide"  kp="600" kv="30"/>
    <position name="te3_L_act"  joint="te3_L_slide"  kp="600" kv="30"/>
  </actuator>
</mujoco>
"""

JOINT_STOWED = {
    "sh_R_yaw": 0.00, "sh_L_yaw": 0.00,
    "sh_R_pitch": -1.40, "sh_L_pitch": -1.40,
    "el_R": -1.30, "el_L": -1.30,
    "hip_R_yaw": 0.00, "hip_L_yaw": 0.00,
}
JOINT_DEPLOYED = {
    "sh_R_yaw": +0.44, "sh_L_yaw": -0.44,
    "sh_R_pitch": +0.00, "sh_L_pitch": +0.00,
    "el_R": 0.00, "el_L": 0.00,
    "hip_R_yaw": -0.44, "hip_L_yaw": +0.44,
}

N_FRAMES = 60
DURATION_S = 0.6


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a * (1 - t) + b * t


def run_simulation() -> dict:
    """Run the deployment mechanism forward in MuJoCo; return trajectory."""
    model = mujoco.MjModel.from_xml_string(MJCF_XML)
    data = mujoco.MjData(model)

    for name, q in JOINT_STOWED.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = q
    mujoco.mj_forward(model, data)

    actuator_id = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
                   for i in range(model.nu)}
    body_ids = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i
                for i in range(model.nbody)}

    n_steps = int(DURATION_S / model.opt.timestep)
    record_every = max(1, n_steps // N_FRAMES)
    frames = []

    for step in range(n_steps):
        t = step * model.opt.timestep
        a_progress = smoothstep(t / 0.30)
        for jname in JOINT_STOWED:
            target = lerp(JOINT_STOWED[jname], JOINT_DEPLOYED[jname], a_progress)
            actname = f"{jname}_act"
            if actname in actuator_id:
                data.ctrl[actuator_id[actname]] = target

        if t >= 0.30:
            phase_b_t = t - 0.30
            for i, stage in enumerate(["tip1", "tip2", "tip3"]):
                start = i * 0.04
                progress = smoothstep((phase_b_t - start) / 0.10)
                for side in ("R", "L"):
                    data.ctrl[actuator_id[f"{stage}_{side}_act"]] = lerp(0.0, 1.10, progress)
            for i, stage in enumerate(["te1", "te2", "te3"]):
                start = i * 0.04
                progress = smoothstep((phase_b_t - start) / 0.10)
                for side in ("R", "L"):
                    data.ctrl[actuator_id[f"{stage}_{side}_act"]] = lerp(0.0, 1.20, progress)

        mujoco.mj_step(model, data)

        if step % record_every == 0 and len(frames) < N_FRAMES:
            frame = {"t": float(t), "bodies": {}}
            for name, bid in body_ids.items():
                frame["bodies"][name] = {"pos": data.xpos[bid].tolist()}
            frames.append(frame)

    # ensure exactly N_FRAMES (pad with last)
    while len(frames) < N_FRAMES:
        frames.append(frames[-1])
    print(f"  sim: {len(frames)} frames over {DURATION_S} s")
    return {"duration_s": DURATION_S, "frames": frames}


# =============================================================================
# (2) Fixed-topology procedural geometry
# =============================================================================
#
# Everything below APPENDS into a shared vertex list and a shared face list.
# The face list (topology) is built ONCE from frame 0 and reused; only vertex
# *positions* change frame to frame.  Vertex counts are constant regardless of
# geometry (segments may collapse to zero length when stowed — that is fine,
# the vertices just coincide), which is exactly what morph targets require.

V = list      # type alias for clarity: a list of (x,y,z)

RING_SEG = 12          # radial segments per tube cross-section
SPHERE_RINGS = 6
SPHERE_SEG = 10


def _frame_axes(p0, p1):
    """Orthonormal frame with x along (p1-p0); robust to vertical axes."""
    ax = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    L = math.sqrt(sum(c * c for c in ax)) or 1e-9
    ax = (ax[0] / L, ax[1] / L, ax[2] / L)
    up = (0.0, 0.0, 1.0)
    if abs(ax[2]) > 0.95:
        up = (1.0, 0.0, 0.0)
    # n1 = ax x up
    n1 = (ax[1] * up[2] - ax[2] * up[1],
          ax[2] * up[0] - ax[0] * up[2],
          ax[0] * up[1] - ax[1] * up[0])
    Ln = math.sqrt(sum(c * c for c in n1)) or 1e-9
    n1 = (n1[0] / Ln, n1[1] / Ln, n1[2] / Ln)
    # n2 = ax x n1
    n2 = (ax[1] * n1[2] - ax[2] * n1[1],
          ax[2] * n1[0] - ax[0] * n1[2],
          ax[0] * n1[1] - ax[1] * n1[0])
    return ax, n1, n2, L


class MeshAccumulator:
    """Builds one frame's vertices; topology captured on first frame."""

    def __init__(self):
        self.verts: list = []

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
        """Two-ring tapered tube with end caps. RING_SEG segs each.
        Returns list of quad/tri faces only when `faces` list is given (frame 0)."""
        ax, n1, n2, _ = _frame_axes(p0, p1)
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

    def membrane(self, le_pts, te_pts, n_span, faces=None):
        """Lofted grid between two boundary polylines (LE and TE).
        Sampled at matching normalized arc-length; chordwise resolution 2
        (LE edge + TE edge).  n_span spanwise stations."""
        def resample(poly, n):
            # cumulative arc length
            segs = [0.0]
            for i in range(1, len(poly)):
                d = math.dist(poly[i], poly[i - 1])
                segs.append(segs[-1] + d)
            total = segs[-1] or 1e-9
            out = []
            for s in range(n):
                target = total * s / (n - 1)
                # find segment
                k = 0
                while k < len(segs) - 2 and segs[k + 1] < target:
                    k += 1
                seg_len = (segs[k + 1] - segs[k]) or 1e-9
                f = (target - segs[k]) / seg_len
                p = tuple(lerp(poly[k][d], poly[k + 1][d], f) for d in range(3))
                out.append(p)
            return out

        le = resample(le_pts, n_span)
        te = resample(te_pts, n_span)
        base = len(self.verts)
        for i in range(n_span):
            self.verts.append(le[i])
            self.verts.append(te[i])
        if faces is not None:
            for i in range(n_span - 1):
                a = base + 2 * i        # le i
                b = base + 2 * i + 1    # te i
                c = base + 2 * (i + 1) + 1   # te i+1
                d = base + 2 * (i + 1)       # le i+1
                faces.append((a, d, c, b))


# Geometry layout: order of operations is identical every frame, so the
# topology (face list) captured on frame 0 stays valid for all frames.
# Material slots: 0=suit/skin, 1=cfrp spar, 2=fabric membrane.

def build_frame(positions: dict, faces=None, mat_ranges=None):
    """Assemble one frame's mesh. On frame 0 pass `faces`/`mat_ranges` lists
    to capture topology + per-slot face index ranges."""
    m = MeshAccumulator()

    def g(name):
        return positions.get(name)

    sh_R, sh_L = g("shoulder_R"), g("shoulder_L")
    el_R, el_L = g("elbow_R"), g("elbow_L")
    wr_R, wr_L = g("wrist_R"), g("wrist_L")
    hip_R, hip_L = g("hip_R"), g("hip_L")
    kn_R, kn_L = g("knee_R"), g("knee_L")
    ank_R, ank_L = g("ankle_R"), g("ankle_L")

    torso_up = tuple((sh_R[i] + sh_L[i]) / 2 for i in range(3))
    hip_mid = tuple((hip_R[i] + hip_L[i]) / 2 for i in range(3))
    torso_lo = tuple(lerp(torso_up[i], hip_mid[i], 0.55) for i in range(3))
    neck = (torso_up[0] + 0.16, 0.0, torso_up[2] + 0.06)
    head = (torso_up[0] + 0.34, 0.0, torso_up[2] + 0.10)

    def extend(a, b, length):
        d = [b[i] - a[i] for i in range(3)]
        L = math.sqrt(sum(c * c for c in d)) or 1e-9
        return tuple(b[i] + d[i] / L * length for i in range(3))

    hand_R = extend(el_R, wr_R, 0.18)
    hand_L = extend(el_L, wr_L, 0.18)

    f0 = faces is not None

    # ---- HUMANOID (material slot 0) ----
    start = len(faces) if f0 else 0
    # torso
    m.tube(hip_mid, torso_lo, 0.150, 0.160, faces)
    m.tube(torso_lo, torso_up, 0.160, 0.150, faces)
    m.tube(torso_up, neck, 0.140, 0.075, faces)
    m.sphere(head, 0.115, faces)
    # arms
    for sh, el, wr, hand in [(sh_R, el_R, wr_R, hand_R), (sh_L, el_L, wr_L, hand_L)]:
        m.sphere(sh, 0.090, faces)
        m.tube(sh, el, 0.075, 0.058, faces)
        m.sphere(el, 0.060, faces)
        m.tube(el, wr, 0.056, 0.044, faces)
        m.tube(wr, hand, 0.044, 0.034, faces)
    # legs
    for hip, kn, ank in [(hip_R, kn_R, ank_R), (hip_L, kn_L, ank_L)]:
        m.sphere(hip, 0.110, faces)
        m.tube(hip, kn, 0.105, 0.072, faces)
        m.sphere(kn, 0.072, faces)
        m.tube(kn, ank, 0.070, 0.050, faces)
        m.sphere(ank, 0.052, faces)
    if f0:
        mat_ranges["suit"] = (start, len(faces))

    # Telescoping tip extensions.  The MuJoCo slide joints sit in the rotated
    # wrist/ankle frames, so their world direction tilts as the limbs spread.
    # For a clean planform we use only the *magnitude* of each stage's travel
    # (a translation, so frame-invariant) and lay the boom out horizontally
    # outboard (+/- y).  Wrist boom forms the leading edge, ankle boom the
    # trailing edge; both reach the same span so the wing closes to a tip chord.
    def boom_stations(anchor, sgn, reach, n=3):
        return [(anchor[0], anchor[1] + sgn * reach * (k + 1) / n, anchor[2])
                for k in range(n)]

    le_booms, te_booms = {}, {}
    for side, wr, ank in [("R", wr_R, ank_R), ("L", wr_L, ank_L)]:
        sgn = 1.0 if side == "R" else -1.0
        t3 = g(f"tip3_{side}")
        le_reach = math.dist(t3, wr)                 # cumulative LE boom travel
        te_reach = le_reach * (1.20 / 1.10)          # TE boom ranges a touch further
        le_booms[side] = boom_stations(wr, sgn, le_reach)
        te_booms[side] = boom_stations(ank, sgn, te_reach)

    # ---- CFRP STRUCTURE (material slot 1) ----
    start = len(faces) if f0 else 0
    shoulder_yoke = tuple((sh_R[i] + sh_L[i]) / 2 + (0, 0, 0.12)[i] for i in range(3))
    hip_yoke = tuple((hip_R[i] + hip_L[i]) / 2 + (0, 0, 0.10)[i] for i in range(3))
    m.tube(shoulder_yoke, hip_yoke, 0.024, 0.024, faces)   # spine yoke
    for side, wr, ank in [("R", wr_R, ank_R), ("L", wr_L, ank_L)]:
        l1, l2, l3 = le_booms[side]
        m.tube(wr, l1, 0.022, 0.019, faces)            # wrist tip extension (LE)
        m.tube(l1, l2, 0.019, 0.015, faces)
        m.tube(l2, l3, 0.015, 0.011, faces)
        a1, a2, a3 = te_booms[side]
        m.tube(ank, a1, 0.020, 0.017, faces)           # ankle tip extension (TE)
        m.tube(a1, a2, 0.017, 0.014, faces)
        m.tube(a2, a3, 0.014, 0.011, faces)
    if f0:
        mat_ranges["cfrp"] = (start, len(faces))

    # ---- WING MEMBRANE (material slot 2) ----
    start = len(faces) if f0 else 0
    for side, wr, ank in [("R", wr_R, ank_R), ("L", wr_L, ank_L)]:
        sh = g(f"shoulder_{side}"); el = g(f"elbow_{side}")
        hip = g(f"hip_{side}"); kn = g(f"knee_{side}")
        l1, l2, l3 = le_booms[side]
        a1, a2, a3 = te_booms[side]
        le = [sh, el, wr, l1, l2, l3]               # leading edge: arm + wrist boom
        te = [hip, kn, ank, a1, a2, a3]             # trailing edge: leg + ankle boom
        m.membrane(le, te, n_span=16, faces=faces)
    if f0:
        mat_ranges["fabric"] = (start, len(faces))

    return m.verts


# =============================================================================
# (3) Blender scene, materials, lights, render
# =============================================================================

def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for d in [bpy.data.objects, bpy.data.meshes, bpy.data.materials,
              bpy.data.lights, bpy.data.cameras, bpy.data.worlds]:
        for item in list(d):
            d.remove(item)


def make_material(name, color, metallic=0.0, roughness=0.5, alpha=1.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, alpha)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        mat.blend_method = "BLEND"
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
    bpy.ops.object.camera_add(location=(3.4, -4.6, 2.2))
    cam = bpy.context.active_object
    cam.data.lens = 52
    bpy.ops.object.empty_add(location=(0, 0, 0.1))
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


# =============================================================================
# (4) Build the animated mesh + export
# =============================================================================

def build_animated_object(traj: dict):
    frames = traj["frames"]
    faces: list = []
    mat_ranges: dict = {}

    # frame 0 captures topology + material ranges
    base_verts = build_frame(world_positions(frames[0]), faces, mat_ranges)
    print(f"  topology: {len(base_verts)} verts, {len(faces)} faces")

    # from_pydata preserves vertex AND polygon order exactly — essential so the
    # morph-target index alignment and per-face material ranges stay valid.
    mesh = bpy.data.meshes.new("manta_mesh")
    mesh.from_pydata(base_verts, [], [list(f) for f in faces])
    mesh.update()
    mesh.validate(verbose=False)

    obj = bpy.data.objects.new("MANTA", mesh)
    bpy.context.collection.objects.link(obj)

    # materials + per-face assignment
    suit = make_material("suit", (0.16, 0.18, 0.23), metallic=0.05, roughness=0.62)
    cfrp = make_material("cfrp", (0.03, 0.03, 0.04), metallic=0.7, roughness=0.28)
    fabric = make_material("fabric", (0.10, 0.34, 0.85), roughness=0.35, alpha=0.62)
    mesh.materials.append(suit)
    mesh.materials.append(cfrp)
    mesh.materials.append(fabric)
    slot = {"suit": 0, "cfrp": 1, "fabric": 2}
    for name, (lo, hi) in mat_ranges.items():
        for fi in range(lo, hi):
            poly = mesh.polygons[fi] if fi < len(mesh.polygons) else None
            if poly is not None:
                poly.material_index = slot[name]
    # note: triangulation/ngon may shift indices slightly; assign by nearest
    # is overkill — material ranges are contiguous and ngons preserved 1:1.

    # Basis shape key (frame 0)
    obj.shape_key_add(name="frame_000", from_mix=False)

    # Per-frame morph targets
    for fi in range(1, len(frames)):
        verts = build_frame(world_positions(frames[fi]))
        sk = obj.shape_key_add(name=f"frame_{fi:03d}", from_mix=False)
        for vi, co in enumerate(verts):
            if vi < len(sk.data):
                sk.data[vi].co = co

    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()

    # Animate shape keys: tent profile so adjacent frames blend smoothly.
    keys = obj.data.shape_keys
    keys.use_relative = True
    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = len(frames) - 1
    # LINEAR interpolation between morph keys (so fractional times = a clean
    # blend of the two neighbouring deployment frames).
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


def world_positions(frame: dict) -> dict:
    return {name: body["pos"] for name, body in frame["bodies"].items()}


def export_glb(obj):
    SITE_MODELS.mkdir(parents=True, exist_ok=True)
    for path in (OUT_DIR / "manta.glb", SITE_MODELS / "manta.glb"):
        bpy.ops.export_scene.gltf(
            filepath=str(path), export_format="GLB",
            use_selection=True,
            export_animations=True,
            export_morph=True,
            export_morph_animation=True,
            export_yup=True,
        )
        print(f"  exported {path}")


def render_still(obj, name, frame_idx):
    bpy.context.scene.frame_set(frame_idx)
    out = OUT_DIR / f"{name}.png"
    bpy.context.scene.render.filepath = str(out)
    print(f"  render {out.name} (frame {frame_idx})...")
    bpy.ops.render.render(write_still=True)


def main():
    render_stills = "--render" in sys.argv

    print("(1) MuJoCo deployment kinematics...")
    traj = run_simulation()
    (OUT_DIR / "trajectory.json").write_text(json.dumps(traj))

    print("(2) Building Blender scene...")
    clear_scene()
    build_world()
    add_lights()
    add_camera()
    setup_renderer()

    print("(3) Building animated mesh (morph targets)...")
    obj = build_animated_object(traj)

    print("(4) Exporting animated GLB...")
    export_glb(obj)

    if render_stills:
        print("(5) Rendering hero stills...")
        # ground for stills only
        bpy.ops.mesh.primitive_plane_add(size=40, location=(0, 0, -0.7))
        plane = bpy.context.active_object
        plane.data.materials.append(make_material("ground", (0.07, 0.07, 0.09),
                                                   roughness=0.85))
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        render_still(obj, "hero_stowed", 0)
        render_still(obj, "hero_mid_deploy", N_FRAMES // 2)
        render_still(obj, "hero_deployed", N_FRAMES - 1)

    print("done.")


if __name__ == "__main__":
    main()
