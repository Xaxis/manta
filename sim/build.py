"""
MANTA — full pipeline in one script.

(1) MuJoCo MJCF model of the deployment kinematics + forward sim
(2) Blender scene:
      - Humanoid via Skin + Subsurf modifier on a vertex skeleton
        (smooth organic shape, not capsules)
      - Airfoil-lofted wing skin with NACA-style cross-section
      - Procedural sky world + ground plane with shadow
      - PBR materials (carbon fiber, fabric, skin)
      - Cinematic camera with f/4 depth of field
(3) Render keyframes via Eevee at 1920x1080
(4) Export GLB for the web viewer

Run:  .venv/bin/python sim/build.py
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import bpy
import bmesh
import mujoco

_HERE = Path(__file__).parent
OUT_DIR = _HERE / "out"
OUT_DIR.mkdir(parents=True, exist_ok=True)


# =============================================================================
# (1) MuJoCo physics — embedded MJCF + sim run
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


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return a * (1 - t) + b * t


def run_simulation() -> dict:
    """Run the deployment forward in MuJoCo. Return trajectory dict."""
    model = mujoco.MjModel.from_xml_string(MJCF_XML)
    data = mujoco.MjData(model)

    for name, q in JOINT_STOWED.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = q
    mujoco.mj_forward(model, data)

    actuator_id = {}
    for i in range(model.nu):
        actuator_id[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)] = i

    body_ids = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i
                 for i in range(model.nbody)}

    duration_s = 0.6
    n_steps = int(duration_s / model.opt.timestep)
    record_every = max(1, n_steps // 60)
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

        if step % record_every == 0:
            frame = {"t": float(t), "bodies": {}}
            for name, bid in body_ids.items():
                frame["bodies"][name] = {
                    "pos": data.xpos[bid].tolist(),
                    "quat": data.xquat[bid].tolist(),
                }
            frames.append(frame)

    print(f"  sim: {len(frames)} frames over {duration_s} s")
    return {"duration_s": duration_s, "frames": frames}


# =============================================================================
# (2) Blender scene
# =============================================================================

def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for d in [bpy.data.objects, bpy.data.meshes, bpy.data.materials,
              bpy.data.lights, bpy.data.cameras, bpy.data.worlds]:
        for item in list(d):
            d.remove(item)


def make_material(name, color, metallic=0.0, roughness=0.5, alpha=1.0,
                   subsurface=0.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, alpha)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if "Subsurface Weight" in bsdf.inputs:
        bsdf.inputs["Subsurface Weight"].default_value = subsurface
        bsdf.inputs["Subsurface Radius"].default_value = (0.1, 0.05, 0.025)
    if alpha < 1.0:
        bsdf.inputs["Alpha"].default_value = alpha
        mat.blend_method = "BLEND"
    return mat


def build_world():
    """Procedural sky + soft fill via World shader."""
    world = bpy.data.worlds.new("World")
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()
    # Sky background gradient (zenith deep blue → horizon warm gray)
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    grad = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = "QUADRATIC_SPHERE"
    ramp = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.0
    ramp.color_ramp.elements[0].color = (0.04, 0.05, 0.08, 1)
    ramp.color_ramp.elements[1].position = 1.0
    ramp.color_ramp.elements[1].color = (0.55, 0.55, 0.50, 1)
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
    sun.data.energy = 5.0
    sun.rotation_euler = (math.radians(45), 0, math.radians(-40))
    if hasattr(sun.data, "angle"):
        sun.data.angle = math.radians(3)   # soft sun for nicer shadows

    bpy.ops.object.light_add(type="AREA", location=(-4, 5, 5))
    fill = bpy.context.active_object
    fill.data.energy = 600
    fill.data.size = 6


def add_ground():
    bpy.ops.mesh.primitive_plane_add(size=40, location=(0, 0, -0.55))
    plane = bpy.context.active_object
    plane.name = "ground"
    mat = make_material("ground", (0.10, 0.10, 0.12), metallic=0.0, roughness=0.8)
    plane.data.materials.append(mat)


def add_camera():
    bpy.ops.object.camera_add(location=(3.6, -4.4, 2.5))
    cam = bpy.context.active_object
    cam.data.lens = 50
    if hasattr(cam.data.dof, "use_dof"):
        cam.data.dof.use_dof = True
        cam.data.dof.aperture_fstop = 4.0
        cam.data.dof.focus_distance = 4.5
    # Track to origin
    bpy.ops.object.empty_add(location=(0, 0, 0))
    target = bpy.context.active_object
    target.name = "cam_target"
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
        scene.eevee.use_ssr = True
        scene.eevee.use_bloom = True
    except AttributeError:
        pass
    # Filmic color management for better tonality
    scene.view_settings.view_transform = "Filmic"
    scene.view_settings.look = "Medium High Contrast"


# ----- Humanoid via Skin modifier ----------------------------------------

def build_humanoid_skeleton_mesh(world_positions: dict, name: str = "humanoid"):
    """Build a vertex-skeleton mesh whose vertices are the joint positions;
    add Skin + Subsurf modifiers to convert into smooth organic geometry.

    world_positions: dict of joint_name → (x, y, z) world position from MuJoCo.
    """
    # Joint connectivity — edges of the skeleton graph
    # Each tuple defines: from_joint, to_joint, skin_radius_root, skin_radius_leaf
    EDGES = [
        # Spine: hip_center → torso → head
        ("hip_center", "torso_lower", 0.140, 0.150),
        ("torso_lower", "torso_upper", 0.150, 0.150),
        ("torso_upper", "neck", 0.140, 0.075),
        ("neck", "head", 0.075, 0.085),
        # Right arm
        ("torso_upper", "shoulder_R_skin", 0.130, 0.075),
        ("shoulder_R_skin", "elbow_R_skin", 0.075, 0.060),
        ("elbow_R_skin", "wrist_R_skin", 0.060, 0.045),
        ("wrist_R_skin", "hand_R_skin", 0.045, 0.038),
        # Left arm
        ("torso_upper", "shoulder_L_skin", 0.130, 0.075),
        ("shoulder_L_skin", "elbow_L_skin", 0.075, 0.060),
        ("elbow_L_skin", "wrist_L_skin", 0.060, 0.045),
        ("wrist_L_skin", "hand_L_skin", 0.045, 0.038),
        # Right leg
        ("hip_center", "hip_R_skin", 0.140, 0.115),
        ("hip_R_skin", "knee_R_skin", 0.115, 0.075),
        ("knee_R_skin", "ankle_R_skin", 0.075, 0.055),
        # Left leg
        ("hip_center", "hip_L_skin", 0.140, 0.115),
        ("hip_L_skin", "knee_L_skin", 0.115, 0.075),
        ("knee_L_skin", "ankle_L_skin", 0.075, 0.055),
    ]

    # Compute derived joint positions from MuJoCo body positions
    def get(name):
        return world_positions.get(name)
    sh_R = get("shoulder_R"); sh_L = get("shoulder_L")
    if sh_R is None or sh_L is None:
        return None
    torso_upper = ((sh_R[0] + sh_L[0]) / 2,
                   (sh_R[1] + sh_L[1]) / 2,
                   (sh_R[2] + sh_L[2]) / 2)
    hip_R = get("hip_R"); hip_L = get("hip_L")
    hip_center = ((hip_R[0] + hip_L[0]) / 2,
                   (hip_R[1] + hip_L[1]) / 2,
                   (hip_R[2] + hip_L[2]) / 2)
    torso_lower = (
        (torso_upper[0] + hip_center[0]) / 2 + 0.05,
        0.0,
        (torso_upper[2] + hip_center[2]) / 2,
    )
    neck = (torso_upper[0] + 0.15, 0.0, torso_upper[2] + 0.05)
    head = (torso_upper[0] + 0.35, 0.0, torso_upper[2] + 0.08)

    # Helper: distance from a body name in MuJoCo trajectory
    el_R = get("elbow_R"); el_L = get("elbow_L")
    wr_R = get("wrist_R"); wr_L = get("wrist_L")
    kn_R = get("knee_R"); kn_L = get("knee_L")
    ank_R = get("ankle_R"); ank_L = get("ankle_L")

    # Hand position — extend from wrist along the arm direction by ~0.2
    def extend(p_anchor, p_dir, length):
        dx = p_dir[0] - p_anchor[0]
        dy = p_dir[1] - p_anchor[1]
        dz = p_dir[2] - p_anchor[2]
        L = math.sqrt(dx * dx + dy * dy + dz * dz)
        if L < 1e-6:
            return p_dir
        return (p_dir[0] + dx / L * length,
                p_dir[1] + dy / L * length,
                p_dir[2] + dz / L * length)

    hand_R = extend(el_R, wr_R, 0.20) if el_R and wr_R else wr_R
    hand_L = extend(el_L, wr_L, 0.20) if el_L and wr_L else wr_L

    joints = {
        "hip_center":      hip_center,
        "torso_lower":     torso_lower,
        "torso_upper":     torso_upper,
        "neck":            neck,
        "head":            head,
        "shoulder_R_skin": sh_R,
        "elbow_R_skin":    el_R,
        "wrist_R_skin":    wr_R,
        "hand_R_skin":     hand_R,
        "shoulder_L_skin": sh_L,
        "elbow_L_skin":    el_L,
        "wrist_L_skin":    wr_L,
        "hand_L_skin":     hand_L,
        "hip_R_skin":      hip_R,
        "knee_R_skin":     kn_R,
        "ankle_R_skin":    ank_R,
        "hip_L_skin":      hip_L,
        "knee_L_skin":     kn_L,
        "ankle_L_skin":    ank_L,
    }

    # Build mesh
    mesh = bpy.data.meshes.new(name + "_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new()

    vtx = {}
    for jname, pos in joints.items():
        if pos is None:
            continue
        v = bm.verts.new(pos)
        vtx[jname] = v
    bm.verts.ensure_lookup_table()

    for from_, to_, _r0, _r1 in EDGES:
        if from_ not in vtx or to_ not in vtx:
            continue
        bm.edges.new((vtx[from_], vtx[to_]))

    bm.to_mesh(mesh)
    bm.free()

    # Set per-vertex skin radii
    skin_mod = obj.modifiers.new("skin", "SKIN")
    skin_layer = obj.data.skin_vertices[0].data
    vert_index = {v.co[:]: i for i, v in enumerate(mesh.vertices)}
    for from_, to_, r0, r1 in EDGES:
        if from_ not in joints or to_ not in joints:
            continue
        p_from = tuple(joints[from_])
        p_to = tuple(joints[to_])
        # Find vert indices
        i_from = -1; i_to = -1
        for i, v in enumerate(mesh.vertices):
            if all(abs(v.co[k] - p_from[k]) < 1e-5 for k in range(3)):
                i_from = i
            if all(abs(v.co[k] - p_to[k]) < 1e-5 for k in range(3)):
                i_to = i
        if i_from >= 0:
            skin_layer[i_from].radius = (r0, r0)
        if i_to >= 0:
            skin_layer[i_to].radius = (r1, r1)

    # Mark "torso_upper" as the skin root
    if "torso_upper" in joints:
        p = tuple(joints["torso_upper"])
        for i, v in enumerate(mesh.vertices):
            if all(abs(v.co[k] - p[k]) < 1e-5 for k in range(3)):
                skin_layer[i].use_root = True
                break

    # Subdivide for smoothness
    sub = obj.modifiers.new("subsurf", "SUBSURF")
    sub.levels = 2
    sub.render_levels = 3

    # Material — wingsuit fabric over skin
    mat = make_material("suit", (0.20, 0.22, 0.26), metallic=0.05, roughness=0.65)
    obj.data.materials.append(mat)
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.shade_smooth()
    return obj


# ----- Spine yoke + LE/TE spars + tip extensions -------------------------

def add_tube(name, p_in, p_out, radius, material):
    p_in = list(p_in); p_out = list(p_out)
    dx, dy, dz = p_out[0] - p_in[0], p_out[1] - p_in[1], p_out[2] - p_in[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    if L < 1e-5:
        return None
    mid = ((p_in[0] + p_out[0]) / 2,
           (p_in[1] + p_out[1]) / 2,
           (p_in[2] + p_out[2]) / 2)
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=L, vertices=18, location=mid)
    obj = bpy.context.active_object
    obj.name = name
    ux, uy, uz = dx / L, dy / L, dz / L
    if abs(ux) + abs(uy) < 1e-9:
        obj.rotation_euler = (0, 0, 0) if uz > 0 else (math.pi, 0, 0)
    else:
        ax, ay, az = -uy, ux, 0.0
        ang = math.acos(max(-1.0, min(1.0, uz)))
        anorm = math.sqrt(ax * ax + ay * ay + az * az)
        ax /= anorm; ay /= anorm; az /= anorm
        half = ang / 2
        obj.rotation_mode = "QUATERNION"
        obj.rotation_quaternion = (math.cos(half),
                                     ax * math.sin(half),
                                     ay * math.sin(half),
                                     az * math.sin(half))
    bpy.ops.object.shade_smooth()
    obj.data.materials.append(material)
    return obj


def build_structure(world_positions: dict, mat_cfrp):
    pieces = []
    # Spine yoke: shoulder midpoint → hip midpoint, with TE hub at the back
    sh_R = world_positions.get("shoulder_R")
    sh_L = world_positions.get("shoulder_L")
    hip_R = world_positions.get("hip_R")
    hip_L = world_positions.get("hip_L")
    if not all([sh_R, sh_L, hip_R, hip_L]):
        return pieces
    shoulder_mid = ((sh_R[0] + sh_L[0]) / 2,
                     (sh_R[1] + sh_L[1]) / 2,
                     (sh_R[2] + sh_L[2]) / 2 + 0.12)
    hip_mid = ((hip_R[0] + hip_L[0]) / 2,
                (hip_R[1] + hip_L[1]) / 2,
                (hip_R[2] + hip_L[2]) / 2 + 0.10)
    pieces.append(add_tube("spine_yoke", shoulder_mid, hip_mid, 0.022, mat_cfrp))

    # Telescoping tubes (right + left)
    for side in ("R", "L"):
        wr = world_positions.get(f"wrist_{side}")
        t1 = world_positions.get(f"tip1_{side}")
        t2 = world_positions.get(f"tip2_{side}")
        t3 = world_positions.get(f"tip3_{side}")
        if all([wr, t1, t2, t3]):
            seg = [(wr, t1, 0.022), (t1, t2, 0.018), (t2, t3, 0.013)]
            for i, (a, b, r) in enumerate(seg):
                d2 = sum((a[k] - b[k]) ** 2 for k in range(3))
                if d2 > 1e-5:
                    pieces.append(add_tube(f"tip{i+1}_{side}", a, b, r, mat_cfrp))

    # TE telescoping
    te_hub = world_positions.get("te_hub")
    for side in ("R", "L"):
        e1 = world_positions.get(f"te1_{side}")
        e2 = world_positions.get(f"te2_{side}")
        e3 = world_positions.get(f"te3_{side}")
        if all([te_hub, e1, e2, e3]):
            seg = [(te_hub, e1, 0.020), (e1, e2, 0.016), (e2, e3, 0.013)]
            for i, (a, b, r) in enumerate(seg):
                d2 = sum((a[k] - b[k]) ** 2 for k in range(3))
                if d2 > 1e-5:
                    pieces.append(add_tube(f"te{i+1}_{side}", a, b, r, mat_cfrp))

    return pieces


# ----- Airfoil-lofted wing skin ------------------------------------------

def naca_section(chord: float, n: int = 24, t_c: float = 0.10):
    """Return a list of 3D points (in section-local frame: x=chord, z=thickness, y=0)
    for a NACA-4-digit symmetric airfoil with thickness t/c.
    """
    pts = []
    # Upper surface from LE to TE
    for i in range(n + 1):
        x_c = (1 - math.cos(i * math.pi / n)) / 2   # cosine-spaced
        yt = 5 * t_c * (
            0.2969 * math.sqrt(x_c)
            - 0.1260 * x_c
            - 0.3516 * x_c * x_c
            + 0.2843 * x_c ** 3
            - 0.1015 * x_c ** 4
        )
        pts.append((x_c * chord, 0.0, +yt * chord))
    # Lower surface from TE to LE
    for i in range(n - 1, 0, -1):
        x_c = (1 - math.cos(i * math.pi / n)) / 2
        yt = 5 * t_c * (
            0.2969 * math.sqrt(x_c)
            - 0.1260 * x_c
            - 0.3516 * x_c * x_c
            + 0.2843 * x_c ** 3
            - 0.1015 * x_c ** 4
        )
        pts.append((x_c * chord, 0.0, -yt * chord))
    return pts


def build_wing_skin(world_positions, mat_fabric):
    """Loft airfoil sections from root to tip on both sides."""
    sh_R = world_positions.get("shoulder_R"); sh_L = world_positions.get("shoulder_L")
    tip_R = world_positions.get("tip3_R"); tip_L = world_positions.get("tip3_L")
    te_hub = world_positions.get("te_hub")
    te_R = world_positions.get("te3_R"); te_L = world_positions.get("te3_L")

    if not all([sh_R, sh_L, tip_R, tip_L, te_hub, te_R, te_L]):
        return []

    pieces = []
    for side_sign, sh, tip, te in [(1, sh_R, tip_R, te_R),
                                       (-1, sh_L, tip_L, te_L)]:
        # Need at least 0.4 m extension for skin to render
        if abs(tip[1]) < 0.4:
            continue
        # Define 8 spanwise stations
        n_stations = 10
        sections = []
        for i in range(n_stations + 1):
            eta = i / n_stations
            # LE point at this station (lerp shoulder→tip)
            le_pt = (
                sh[0] * (1 - eta) + tip[0] * eta,
                sh[1] * (1 - eta) + tip[1] * eta,
                sh[2] * (1 - eta) + tip[2] * eta,
            )
            # TE point at this station (lerp te_hub→te tip)
            te_pt = (
                te_hub[0] * (1 - eta) + te[0] * eta,
                te_hub[1] * (1 - eta) + te[1] * eta,
                te_hub[2] * (1 - eta) + te[2] * eta,
            )
            chord = math.sqrt(sum((te_pt[k] - le_pt[k]) ** 2 for k in range(3)))
            if chord < 1e-4:
                continue
            # Build NACA section, then place it in world coords aligned with
            # the local chord direction.
            section_local = naca_section(chord, n=18, t_c=0.10)
            # Compute chord direction (unit vector LE→TE)
            cx = (te_pt[0] - le_pt[0]) / chord
            cy = (te_pt[1] - le_pt[1]) / chord
            cz = (te_pt[2] - le_pt[2]) / chord
            # Build coordinate frame: chord_dir, span_dir (roughly +y for right
            # wing), up_dir
            chord_dir = (cx, cy, cz)
            # Up direction = world +z, but orthogonal to chord
            up = (0, 0, 1)
            # Project up to be perp to chord_dir
            dot = up[0] * cx + up[1] * cy + up[2] * cz
            up_perp = (up[0] - dot * cx, up[1] - dot * cy, up[2] - dot * cz)
            up_norm = math.sqrt(sum(u * u for u in up_perp))
            up_perp = (up_perp[0] / up_norm, up_perp[1] / up_norm, up_perp[2] / up_norm)
            # Span direction = chord × up
            span_dir = (
                chord_dir[1] * up_perp[2] - chord_dir[2] * up_perp[1],
                chord_dir[2] * up_perp[0] - chord_dir[0] * up_perp[2],
                chord_dir[0] * up_perp[1] - chord_dir[1] * up_perp[0],
            )
            # Transform section points: x_local → chord_dir, z_local → up_perp,
            # origin at LE point.
            section_world = []
            for (x_l, _y_l, z_l) in section_local:
                wx = le_pt[0] + x_l * chord_dir[0] + z_l * up_perp[0]
                wy = le_pt[1] + x_l * chord_dir[1] + z_l * up_perp[1]
                wz = le_pt[2] + x_l * chord_dir[2] + z_l * up_perp[2]
                section_world.append((wx, wy, wz))
            sections.append(section_world)

        # Build the lofted surface as a mesh
        mesh = bpy.data.meshes.new(f"wing_{('R' if side_sign > 0 else 'L')}_mesh")
        obj = bpy.data.objects.new(f"wing_{('R' if side_sign > 0 else 'L')}", mesh)
        bpy.context.collection.objects.link(obj)
        bm = bmesh.new()
        verts = []
        for sec in sections:
            verts.append([bm.verts.new(p) for p in sec])
        # Stitch quads between consecutive sections
        for i in range(len(verts) - 1):
            v0 = verts[i]; v1 = verts[i + 1]
            n_per = min(len(v0), len(v1))
            for k in range(n_per - 1):
                bm.faces.new([v0[k], v0[k + 1], v1[k + 1], v1[k]])
        bm.normal_update()
        bm.to_mesh(mesh)
        bm.free()
        obj.data.materials.append(mat_fabric)
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.shade_smooth()
        pieces.append(obj)
    return pieces


# ----- Build scene + render ----------------------------------------------

def world_positions_from_frame(frame: dict) -> dict:
    return {name: body["pos"] for name, body in frame["bodies"].items()}


def render_pose(name: str, frame: dict):
    clear_scene()
    build_world()
    add_lights()
    add_ground()
    add_camera()
    setup_renderer()

    mat_cfrp = make_material("cfrp", (0.04, 0.04, 0.05), metallic=0.6, roughness=0.30)
    mat_fabric = make_material("fabric", (0.20, 0.40, 0.85), roughness=0.40, alpha=0.55)

    positions = world_positions_from_frame(frame)
    build_humanoid_skeleton_mesh(positions)
    build_structure(positions, mat_cfrp)
    build_wing_skin(positions, mat_fabric)

    out_path = OUT_DIR / f"{name}.png"
    bpy.context.scene.render.filepath = str(out_path)
    print(f"  rendering {out_path.name}...")
    bpy.ops.render.render(write_still=True)
    print(f"  done.")

    # Export GLB
    glb_path = OUT_DIR / f"{name}.glb"
    bpy.ops.export_scene.gltf(filepath=str(glb_path), export_format="GLB",
                              export_animations=False)


def main():
    print("(1) Running MuJoCo physics simulation...")
    traj = run_simulation()
    traj_path = OUT_DIR / "trajectory.json"
    with traj_path.open("w") as f:
        json.dump(traj, f)
    print(f"    wrote {traj_path}")

    # Pick three keyframes: stowed, mid-deploy, deployed
    n = len(traj["frames"])
    keyframes = [
        ("stowed", traj["frames"][0]),
        ("mid_deploy", traj["frames"][n // 2 + 5]),
        ("deployed", traj["frames"][-1]),
    ]
    print("(2) Building Blender scenes...")
    for name, frame in keyframes:
        print(f"  pose: {name} (t = {frame['t']:.3f} s)")
        render_pose(name, frame)


if __name__ == "__main__":
    main()
